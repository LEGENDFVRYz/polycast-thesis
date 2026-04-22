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



        # --- Rotation ---
        q_norm = _qnormalize(q)
        acc_world_raw = _quat_rotate(q_norm, acc_ms2)

        # ==========================================================
        # DUAL-PATH EMA ARCHITECTURE
        # ==========================================================
        
        # PATH A: Raw Data for ESKF (Trust the filter, minimal lag)
        acc_world_eskf = acc_world_raw 
        acc_board_eskf = _project_board_axes(acc_world_eskf)

        # PATH B: Heavy Smoothing exclusively for ZUPT (Kills 110mm Tremor)
        zupt_alpha = 0.95  # 95% old data, 5% new data (Heavy Low-Pass)
        
        if getattr(self, '_prev_acc_world_zupt', None) is None:
            self._prev_acc_world_zupt = acc_world_raw
            self._prev_acc_world_zupt_old = acc_world_raw

        acc_world_zupt = (
            zupt_alpha * self._prev_acc_world_zupt[0] + (1 - zupt_alpha) * acc_world_raw[0],
            zupt_alpha * self._prev_acc_world_zupt[1] + (1 - zupt_alpha) * acc_world_raw[1],
            zupt_alpha * self._prev_acc_world_zupt[2] + (1 - zupt_alpha) * acc_world_raw[2]
        )
        self._prev_acc_world_zupt = acc_world_zupt



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



        # --- Smoothed ZUPT Jerk ---
        # Compute jerk from the heavily smoothed ZUPT acceleration, NOT raw sensor acc.
        # This prevents the 160+ m/s³ micro-tremor spikes from blocking ZUPT.
        if dt_s > 0:
            delta_zupt = _vsub(acc_world_zupt, self._prev_acc_world_zupt_old)
            zupt_jerk = _vmag(delta_zupt) / dt_s
        else:
            zupt_jerk = 0.0
        self._prev_acc_world_zupt_old = acc_world_zupt

        # --- ZUPT (motion detector) ---
        lin_mag_zupt = _vmag(acc_world_zupt)
        
        # Now you can keep your thresholds tight and accurate!
        still_now = (
            lin_mag_zupt < cfg.imu.zupt_acc_threshold and
            zupt_jerk < cfg.imu.zupt_jerk_threshold
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
    import math
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer

    # CONFIGURATION
    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    REPORT_NAME = "imu_stationary"

    # LIVE PLOT CONFIGURATION
    WINDOW_SIZE = 500  # Slightly larger window to see more context
    REFRESH_RATE_S = 0.1

    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    prep = IMUPreprocessor()

    print("=" * 60)
    print(f"  Live IMU Dashboard: {SERIAL_PORT}")
    print("  Collecting data and plotting live... Press Ctrl+C to save and exit.")
    print("=" * 60)

    event_log = []

    # --- KINEMATICS STATE (For Stroke Tracking) ---
    vel_board = [0.0, 0.0]
    pos_board = [0.0, 0.0]
    last_integration_ts = None

    # --- LIVE PLOT SETUP ---
    plt.ion()
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("Live IMU Kinematics & Drawing")
    fig.suptitle('Live IMU Kinematics & 2D Stroke Reconstruction', fontsize=14, fontweight='bold')

    # Create a Grid: Left column for Time Series (3 rows), Right column for 2D Drawing (1 row spanning all)
    gs = gridspec.GridSpec(3, 2, width_ratios=[1.5, 1])
    
    ax_acc = fig.add_subplot(gs[0, 0])
    ax_jerk = fig.add_subplot(gs[1, 0])
    ax_force = fig.add_subplot(gs[2, 0])
    ax_stroke = fig.add_subplot(gs[:, 1])

    # Time Series Lines
    line_wx, = ax_acc.plot([], [], label='World X', alpha=0.8)
    line_wy, = ax_acc.plot([], [], label='World Y', alpha=0.8)
    line_wz, = ax_acc.plot([], [], label='World Z', alpha=0.8)
    ax_acc.set_title('World Acceleration')
    ax_acc.set_ylabel('Accel (m/s²)')
    ax_acc.legend(loc='upper right')
    ax_acc.grid(True, linestyle='--', alpha=0.6)

    line_jerk, = ax_jerk.plot([], [], label='Jerk (m/s³)', color='purple')
    ax_jerk.set_title('Body-Frame Jerk')
    ax_jerk.set_ylabel('Jerk')
    ax_jerk.legend(loc='upper right')
    ax_jerk.grid(True, linestyle='--', alpha=0.6)

    text_zupt = ax_jerk.text(0.02, 0.85, 'STATE: WAITING', transform=ax_jerk.transAxes, fontsize=12, fontweight='bold', bbox=dict(facecolor='white', alpha=0.8))

    line_force, = ax_force.plot([], [], label='Raw Force', color='orange')
    ax_force.set_title('Force Sensor')
    ax_force.set_ylabel('Force')
    ax_force.set_xlabel('Time (Seconds)')
    ax_force.legend(loc='upper right')
    ax_force.grid(True, linestyle='--', alpha=0.6)

    text_contact = ax_force.text(0.02, 0.85, 'PEN: WAITING', transform=ax_force.transAxes, fontsize=12, fontweight='bold', bbox=dict(facecolor='white', alpha=0.8))

    # 2D Stroke Plot Line
    line_stroke, = ax_stroke.plot([], [], color='black', linewidth=2)
    ax_stroke.set_title('2D Board Strokes (Position)')
    ax_stroke.set_xlabel('Board X (m)')
    ax_stroke.set_ylabel('Board Z (m)')
    ax_stroke.axis('equal')  # Critical: Keeps 1cm X equal to 1cm Z visually
    ax_stroke.grid(True, linestyle='--', alpha=0.6)

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
                    
                    # --- INTEGRATE TO POSITION FOR STROKES ---
                    for ev in processed_imu:
                        if last_integration_ts is not None:
                            dt_s = (ev['ts_hw'] - last_integration_ts) / 1_000_000.0
                            if dt_s > 0 and dt_s < 0.1: # Protect against massive time jumps
                                if ev['is_static']:
                                    vel_board = [0.0, 0.0] # ZUPT: Kill drift
                                else:
                                    vel_board[0] += ev['acc_board'][0] * dt_s
                                    vel_board[1] += ev['acc_board'][1] * dt_s
                                
                                pos_board[0] += vel_board[0] * dt_s
                                pos_board[1] += vel_board[1] * dt_s
                        
                        last_integration_ts = ev['ts_hw']
                        ev['pos_x'] = pos_board[0]
                        ev['pos_z'] = pos_board[1]
                        
                        event_log.append(ev)

            # Update Live Plot
            current_time = time.time()
            if event_log and (current_time - last_plot_time >= REFRESH_RATE_S):
                window_data = event_log[-WINDOW_SIZE:]

                t_sec = [(ev['ts_hw'] - start_ts) / 1_000_000.0 for ev in window_data]

                # Update Time Series
                line_wx.set_data(t_sec, [ev['acc_world'][0] for ev in window_data])
                line_wy.set_data(t_sec, [ev['acc_world'][1] for ev in window_data])
                line_wz.set_data(t_sec, [ev['acc_world'][2] for ev in window_data])
                line_jerk.set_data(t_sec, [ev['jerk'] for ev in window_data])
                line_force.set_data(t_sec, [ev['force'] for ev in window_data])

                # Update 2D Strokes (Plotting full history so the drawing doesn't disappear)
                # We inject NaN when contact is False to break the line between strokes
                stroke_x = [ev['pos_x'] if ev['contact'] else float('nan') for ev in event_log]
                stroke_z = [ev['pos_z'] if ev['contact'] else float('nan') for ev in event_log]
                line_stroke.set_data(stroke_x, stroke_z)

                # Rescale Time Series axes
                x_min, x_max = t_sec[0], t_sec[-1]
                for ax in [ax_acc, ax_jerk, ax_force]:
                    ax.set_xlim(x_min, max(x_max, x_min + 0.1))

                ax_acc.set_ylim(-3, 3)
                ax_force.set_ylim(0, 5000.0)

                # Rescale dynamic axes
                if len(window_data) % 5 == 0:
                    ax_jerk.relim()
                    ax_jerk.autoscale_view(scalex=False, scaley=True)
                    ax_stroke.relim()
                    ax_stroke.autoscale_view()

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
                'pos_board_x', 'pos_board_z',
                'jerk', 'is_static', 'contact', 'force'
            ])
            for ev in event_log:
                writer.writerow([
                    ev['ts_hw'], ev['packet_id'], ev['sample_idx'],
                    ev['acc_world'][0], ev['acc_world'][1], ev['acc_world'][2],
                    ev['acc_board'][0], ev['acc_board'][1],
                    ev['pos_x'], ev['pos_z'],
                    ev['jerk'], int(ev['is_static']), int(ev['contact']), ev['force']
                ])
        print(f"[EXPORT] Data saved to {csv_filename}")


        # --- FULL SESSION PLOT EXPORT ---
        print("[EXPORT] Generating full-session plot...")

        # Create clean static figure with GridSpec
        fig2 = plt.figure(figsize=(16, 10))
        fig2.suptitle('IMU Full Session Report', fontsize=16, fontweight='bold')
        gs2 = gridspec.GridSpec(3, 2, width_ratios=[1.5, 1])

        ax2_acc = fig2.add_subplot(gs2[0, 0])
        ax2_jerk = fig2.add_subplot(gs2[1, 0])
        ax2_force = fig2.add_subplot(gs2[2, 0])
        ax2_stroke = fig2.add_subplot(gs2[:, 1])

        t_sec_full = [(ev['ts_hw'] - start_ts) / 1_000_000.0 for ev in event_log]

        # --- Acceleration ---
        ax2_acc.plot(t_sec_full, [ev['acc_world'][0] for ev in event_log], label='World X')
        ax2_acc.plot(t_sec_full, [ev['acc_world'][1] for ev in event_log], label='World Y')
        ax2_acc.plot(t_sec_full, [ev['acc_world'][2] for ev in event_log], label='World Z')
        ax2_acc.set_title('World Acceleration')
        ax2_acc.set_ylabel('m/s²')
        ax2_acc.legend()
        ax2_acc.grid(True, linestyle='--', alpha=0.6)

        # --- Jerk ---
        ax2_jerk.plot(t_sec_full, [ev['jerk'] for ev in event_log], label='Jerk', color='purple')
        ax2_jerk.set_title('Body-Frame Jerk')
        ax2_jerk.set_ylabel('m/s³')
        ax2_jerk.legend()
        ax2_jerk.grid(True, linestyle='--', alpha=0.6)

        # --- Force ---
        ax2_force.plot(t_sec_full, [ev['force'] for ev in event_log], label='Force', color='orange')
        ax2_force.set_title('Force Sensor')
        ax2_force.set_ylabel('Force')
        ax2_force.set_xlabel('Time (Seconds)')
        ax2_force.legend()
        ax2_force.grid(True, linestyle='--', alpha=0.6)

        # --- 2D Strokes ---
        stroke_x_full = [ev['pos_x'] if ev['contact'] else float('nan') for ev in event_log]
        stroke_z_full = [ev['pos_z'] if ev['contact'] else float('nan') for ev in event_log]
        
        ax2_stroke.plot(stroke_x_full, stroke_z_full, color='black', linewidth=1.5)
        ax2_stroke.set_title('Final 2D Drawing Path')
        ax2_stroke.set_xlabel('Board X (m)')
        ax2_stroke.set_ylabel('Board Z (m)')
        ax2_stroke.axis('equal')
        ax2_stroke.grid(True, linestyle='--', alpha=0.6)

        # Shade static regions in gray
        static_mask = [ev['is_static'] for ev in event_log]
        for ax in [ax2_acc, ax2_jerk, ax2_force]:
            in_static = False
            start_static = 0

            for i, is_static in enumerate(static_mask):
                if is_static and not in_static:
                    start_static = t_sec_full[i]
                    in_static = True
                elif not is_static and in_static:
                    ax.axvspan(start_static, t_sec_full[i], color='gray', alpha=0.15)
                    in_static = False

            if in_static:
                ax.axvspan(start_static, t_sec_full[-1], color='gray', alpha=0.15)

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)

        # Save image
        plot_filename = f"{REPORT_NAME}.png"
        fig2.savefig(plot_filename, dpi=150)

        print(f"[EXPORT] Plot saved to {plot_filename}")
        print("Done.")