"""
Module 4 — IMU Preprocessor

Responsibility:
    - Transforms raw normalized IMU events into clean, fusion-ready IMU motion events
    - (Note) Optimize for Adafruit BNO085 running SH-2 Firmware

Input  (from tv2_normalizer — IMU events):
    {
        'sensor':     'IMU',
        'packet_id':  int,
        'sample_idx': int,
        'ts_hw':      int,
        'quat':       (qx, qy, qz, qw),     - Rotation Vector
        'acc':        (ax, ay, az),         - Linear Acceleration (in m/s²)
        'force':      float
    }

Output (processed IMU event):
    {
        'sensor':      'IMU',
        'ts_hw':       int,
        'packet_id':   int,
        'sample_idx':  int,
        'quat':        (qx, qy, qz, qw),    - normalized unit quaternion
        'acc_sensor':  (ax, ay, az),        - raw sensor linear acc (in m/s²)
        'acc_world':   (ax, ay, az),        - rotated & EMA-smoothed (in m/s²)
        'acc_board':   (bx, bz),            - projected to 2D board plane dynamically
        'jerk':        float,               - computed in BODY frame
        'is_static':   bool,                - ZUPT decision
        'contact':     bool,                - pen touching board?
        'force':       float,               - raw force value (for touch sensitivity)
    }
"""

import math
from background.pipelines.config import cfg


# --- vector / quaternion math ---
def _qnorm(q):
    return math.sqrt(sum(v * v for v in q))

def _qnormalize(q):
    n = _qnorm(q)
    if n < cfg.imu.quat_norm_epsilon:
        return (0.0, 0.0, 0.0, 1.0)   # identity fallback
    return tuple(v / n for v in q)

def _quat_rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )

def _vmag(v):
    return math.sqrt(sum(x * x for x in v))

def _vsub(a, b):
    return tuple(x - y for x, y in zip(a, b))

def _project_board_axes(v):
    """Extracts 2D board coordinates based on the config axes map."""
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    a0, a1 = cfg.imu.board_axes
    return (v[axis_map[a0]], v[axis_map[a1]])


# --- IMU preprocessor ---
class IMUPreprocessor:
    def __init__(self):
        # History for Body-Frame Jerk
        self._prev_acc_body = None  
        # History for World-Frame EMA Smoothing
        self._prev_acc_world_clean = None 
        # Time and ZUPT state
        self._prev_ts = None
        self._still_streak = 0
        self._zupt_active = False

        # Minimum samples required for ZUPT_MIN_DURATION_S
        dt_nom_s = 1.0 / cfg.imu.sample_rate_hz
        self._zupt_min_samples = max(1, int(cfg.imu.zupt_min_duration_s / dt_nom_s))

    def feed(self, events: list[dict]) -> list[dict]:
        out = []
        for ev in events:
            if ev.get('sensor') != 'IMU':
                continue
            processed = self.process_one(ev)
            if processed:
                out.append(processed)
        return out

    def process_one(self, ev: dict) -> dict | None:
        ts  = ev.get('ts_hw')
        q   = ev.get('quat')
        acc = ev.get('acc')

        if ts is None or q is None or acc is None:
            return None

        # --- dt (Microsecond Delta) ---
        if self._prev_ts is not None:
            dt_us = ts - self._prev_ts
            if dt_us <= 0:
                return None
            
            if dt_us > (getattr(cfg.pipeline, 'IMU_MAX_DT_MS', 100) * 1000):
                self.reset()
            
            dt_s = dt_us / 1_000_000.0
        else:
            dt_s = 1.0 / cfg.imu.sample_rate_hz

        self._prev_ts = ts

        # --- Linear Acceleration ---
        acc_ms2 = acc  # Note: Hardware outputs m/s^2 natively.

        # --- Body-Frame Jerk ---
        if self._prev_acc_body is not None and dt_s > 0:
            delta_body = _vsub(acc_ms2, self._prev_acc_body)
            jerk = _vmag(delta_body) / dt_s
        else:
            jerk = 0.0
        self._prev_acc_body = acc_ms2

        # --- Rotation ---
        q_norm = _qnormalize(q)
        acc_world_raw = _quat_rotate(q_norm, acc_ms2)

        # --- World-Frame EMA Smoothing ---
        smooth_alpha = cfg.imu.smooth_alpha
        
        if self._prev_acc_world_clean is None:
            acc_world = acc_world_raw
        else:
            # Formula: Clean = Alpha * Previous_Clean + (1 - Alpha) * Current_Raw
            acc_world = (
                smooth_alpha * self._prev_acc_world_clean[0] + (1 - smooth_alpha) * acc_world_raw[0],
                smooth_alpha * self._prev_acc_world_clean[1] + (1 - smooth_alpha) * acc_world_raw[1],
                smooth_alpha * self._prev_acc_world_clean[2] + (1 - smooth_alpha) * acc_world_raw[2]
            )
        self._prev_acc_world_clean = acc_world

        # --- Board Projection ---
        acc_board = _project_board_axes(acc_world)

        # --- ZUPT (motion detector) ---
        lin_mag = _vmag(acc_world)
        still_now = (
            lin_mag < cfg.imu.zupt_acc_threshold and
            jerk    < cfg.imu.zupt_jerk_threshold
        )

        if still_now:
            self._still_streak += 1
        else:
            self._still_streak  = 0
            self._zupt_active   = False

        if self._still_streak >= self._zupt_min_samples:
            self._zupt_active = True

        # --- Force Contact ---
        force = ev.get('force', 0.0)
        contact = force >= cfg.imu.force_contact_threshold

        return {
            'sensor':     'IMU',
            'ts_hw':      ts,
            'packet_id':  ev.get('packet_id'),
            'sample_idx': ev.get('sample_idx'),
            'quat':       q_norm,
            'acc_sensor': acc_ms2,
            'acc_world':  acc_world,
            'acc_board':  acc_board,
            'jerk':       round(jerk, 6),
            'is_static':  self._zupt_active,
            'contact':    contact,
            'force':      force,
        }

    def reset(self):
        """Clear integration history on startup or buffer limit"""
        self._prev_acc_body = None
        self._prev_acc_world_clean = None
        self._prev_ts = None
        self._still_streak = 0
        self._zupt_active = False



# ==============================================================================
# HARDWARE DATA LOGGING & LIVE REPORTING
# ==============================================================================
if __name__ == '__main__':
    import time
    import csv
    import matplotlib.pyplot as plt
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer

    # CONFIGURATION
    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    REPORT_NAME = "imu_stationary"

    # LIVE PLOT CONFIGURATION
    WINDOW_SIZE = 250
    REFRESH_RATE_S = 0.1

    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    prep = IMUPreprocessor()

    print("=" * 60)
    print(f"  Live IMU Dashboard: {SERIAL_PORT}")
    print("  Collecting data and plotting live... Press Ctrl+C to save and exit.")
    print("=" * 60)

    event_log = []

    # --- LIVE PLOT SETUP ---
    plt.ion()
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title("Live IMU Kinematics")
    fig.suptitle('Live IMU Kinematics & State Detection', fontsize=14, fontweight='bold')

    line_wx, = axs[0].plot([], [], label='World X', alpha=0.8)
    line_wy, = axs[0].plot([], [], label='World Y', alpha=0.8)
    line_wz, = axs[0].plot([], [], label='World Z', alpha=0.8)
    axs[0].set_title('World Acceleration (Smoothed, Gravity Free)')
    axs[0].set_ylabel('Accel (m/s²)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True, linestyle='--', alpha=0.6)

    line_jerk, = axs[1].plot([], [], label='Jerk (m/s³)', color='purple')
    axs[1].set_title('Body-Frame Jerk')
    axs[1].set_ylabel('Jerk')
    axs[1].legend(loc='upper right')
    axs[1].grid(True, linestyle='--', alpha=0.6)

    text_zupt = axs[1].text(
        0.02, 0.85, 'STATE: WAITING',
        transform=axs[1].transAxes,
        fontsize=12, fontweight='bold',
        bbox=dict(facecolor='white', alpha=0.8)
    )

    line_force, = axs[2].plot([], [], label='Raw Force', color='orange')
    axs[2].set_title('Force Sensor')
    axs[2].set_ylabel('Force')
    axs[2].set_xlabel('Time (Seconds)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True, linestyle='--', alpha=0.6)

    text_contact = axs[2].text(
        0.02, 0.85, 'PEN: WAITING',
        transform=axs[2].transAxes,
        fontsize=12, fontweight='bold',
        bbox=dict(facecolor='white', alpha=0.8)
    )

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)

    start_ts = None
    last_plot_time = time.time()

    try:
        while True:
            # Ingest and Process Data
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                events = norm.normalize(raw_packets)
                processed_imu = prep.feed(events)

                if processed_imu:
                    if start_ts is None:
                        start_ts = processed_imu[0]['ts_hw']
                    event_log.extend(processed_imu)

            # Update Live Plot
            current_time = time.time()
            if event_log and (current_time - last_plot_time >= REFRESH_RATE_S):
                window_data = event_log[-WINDOW_SIZE:]

                t_sec = [(ev['ts_hw'] - start_ts) / 1_000_000.0 for ev in window_data]

                line_wx.set_data(t_sec, [ev['acc_world'][0] for ev in window_data])
                line_wy.set_data(t_sec, [ev['acc_world'][1] for ev in window_data])
                line_wz.set_data(t_sec, [ev['acc_world'][2] for ev in window_data])
                line_jerk.set_data(t_sec, [ev['jerk'] for ev in window_data])
                line_force.set_data(t_sec, [ev['force'] for ev in window_data])

                x_min, x_max = t_sec[0], t_sec[-1]
                for ax in axs:
                    ax.set_xlim(x_min, max(x_max, x_min + 0.1))

                axs[0].set_ylim(-3, 3)

                # Throttled autoscale (prevents lag spikes)
                if len(window_data) % 5 == 0:
                    axs[1].relim()
                    axs[1].autoscale_view(scalex=False, scaley=True)

                axs[2].set_ylim(0, 5000.0)

                latest = window_data[-1]

                if latest['is_static']:
                    text_zupt.set_text('STATE: STATIC')
                    text_zupt.set_color('green')
                else:
                    text_zupt.set_text('STATE: MOVING')
                    text_zupt.set_color('red')

                if latest['contact']:
                    text_contact.set_text('PEN: DRAWING')
                    text_contact.set_color('blue')
                else:
                    text_contact.set_text('PEN: LIFTED')
                    text_contact.set_color('gray')

                fig.canvas.draw_idle()
                plt.pause(0.001)

                last_plot_time = current_time

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            exit()

        print(f"Captured {len(event_log)} IMU events. Exporting CSV...")

        # --- CSV EXPORT ---
        csv_filename = f"{REPORT_NAME}.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'ts_hw', 'packet_id', 'sample_idx',
                'acc_world_x', 'acc_world_y', 'acc_world_z',
                'acc_board_x', 'acc_board_z',
                'jerk', 'is_static', 'contact', 'force'
            ])
            for ev in event_log:
                writer.writerow([
                    ev['ts_hw'], ev['packet_id'], ev['sample_idx'],
                    ev['acc_world'][0], ev['acc_world'][1], ev['acc_world'][2],
                    ev['acc_board'][0], ev['acc_board'][1],
                    ev['jerk'], int(ev['is_static']), int(ev['contact']), ev['force']
                ])
        print(f"[EXPORT] Data saved to {csv_filename}")


        # --- FULL SESSION PLOT EXPORT ---
        print("[EXPORT] Generating full-session plot...")

        # Build full timeline
        start_ts = event_log[0]['ts_hw']
        t_sec = [(ev['ts_hw'] - start_ts) / 1_000_000.0 for ev in event_log]

        acc_x = [ev['acc_world'][0] for ev in event_log]
        acc_y = [ev['acc_world'][1] for ev in event_log]
        acc_z = [ev['acc_world'][2] for ev in event_log]
        jerk  = [ev['jerk'] for ev in event_log]
        force = [ev['force'] for ev in event_log]

        # Create clean static figure (NOT the live one)
        fig2, axs2 = plt.subplots(3, 1, figsize=(12, 9))

        fig2.suptitle('IMU Full Session Report', fontsize=14, fontweight='bold')

        # --- Acceleration ---
        axs2[0].plot(t_sec, acc_x, label='World X')
        axs2[0].plot(t_sec, acc_y, label='World Y')
        axs2[0].plot(t_sec, acc_z, label='World Z')
        axs2[0].set_title('World Acceleration')
        axs2[0].set_ylabel('m/s²')
        axs2[0].legend()
        axs2[0].grid(True, linestyle='--', alpha=0.6)

        # --- Jerk ---
        axs2[1].plot(t_sec, jerk, label='Jerk')
        axs2[1].set_title('Body-Frame Jerk')
        axs2[1].set_ylabel('m/s³')
        axs2[1].legend()
        axs2[1].grid(True, linestyle='--', alpha=0.6)

        # --- Force ---
        axs2[2].plot(t_sec, force, label='Force')
        axs2[2].set_title('Force Sensor')
        axs2[2].set_ylabel('Force')
        axs2[2].set_xlabel('Time (Seconds)')
        axs2[2].legend()
        axs2[2].grid(True, linestyle='--', alpha=0.6)
        
        # Build static mask
        static_mask = [ev['is_static'] for ev in event_log]

        # Shade static regions in gray
        for ax in axs2:
            in_static = False
            start_static = 0

            for i, is_static in enumerate(static_mask):
                if is_static and not in_static:
                    start_static = t_sec[i]
                    in_static = True
                elif not is_static and in_static:
                    ax.axvspan(start_static, t_sec[i], color='gray', alpha=0.15)
                    in_static = False

            # Handle if session ends while still static
            if in_static:
                ax.axvspan(start_static, t_sec[-1], color='gray', alpha=0.15)

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)

        # Save image
        plot_filename = f"{REPORT_NAME}.png"
        fig2.savefig(plot_filename, dpi=150)

        print(f"[EXPORT] Plot saved to {plot_filename}")
        print("Done.")