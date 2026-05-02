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
        'quat':        (qx, qy, qz, qw),      - normalized unit quaternion

        'acc_sensor':  (ax, ay, az),          - raw sensor linear acc (m/s²)

        # Legacy / diagnostic — sensor-point (back of pen), NOT tip-corrected:
        'acc_world':    (ax, ay, az),         - sensor-point, world frame, Path-A EMA
        'acc_board':    (bx, bz),             - sensor-point, 2D board projection
        'acc_board_hp': (bx, bz),             - sensor-point, HPF bias-stripped (Path C)

        # Canonical ESKF inputs — rigid-body tip-corrected:
        'acc_tip_world':    (ax, ay, az),     - tip, world frame, Path-A EMA
        'acc_board_tip':    (bx, bz),         - tip, 2D board projection
        'acc_board_hp_tip': (bx, bz),         - tip, HPF bias-stripped (Path C)

        # Angular kinematics (exposed for ESKF turn detection):
        'omega_world': (wx, wy, wz),          - angular velocity, world frame (rad/s)
        'omega_body':  (wx, wy, wz),          - angular velocity, body frame (rad/s)
        'alpha_world': (ax, ay, az),          - angular accel, world frame, EMA-filtered (rad/s²)

        'jerk':        float,                 - body-frame Δacc magnitude / dt (m/s³)
        'is_static':   bool,                  - ZUPT decision
        'contact':     bool,                  - pen touching board?
        'force':       float,                 - raw force value
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


# --- rigid-body helpers (lever-arm kinematics) ---

def _quat_mul(q1, q2):
    """Hamilton product q1 ⊗ q2, both [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def _quat_conj(q):
    """Conjugate (= inverse for unit quaternion), [x,y,z,w] → [-x,-y,-z,w]."""
    return (-q[0], -q[1], -q[2], q[3])

def _q_to_R(q):
    """Quaternion [x,y,z,w] → 3×3 rotation matrix (body→world), as nested tuples."""
    x, y, z, w = q
    return (
        (1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)),
        (    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)),
        (    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)),
    )

def _matvec3(R, v):
    """Multiply 3×3 matrix R (nested tuples) by 3-vector v (tuple)."""
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

def _vcross(a, b):
    """3-vector cross product a × b."""
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )

def _vadd(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

def _vscale(a, s):
    return (a[0]*s, a[1]*s, a[2]*s)

def _ema_vec(prev, cur, alpha_new: float):
    """Exponential moving average using alpha as current-sample weight."""
    if prev is None:
        return cur
    a = max(0.0, min(1.0, float(alpha_new)))
    return (
        a * cur[0] + (1.0 - a) * prev[0],
        a * cur[1] + (1.0 - a) * prev[1],
        a * cur[2] + (1.0 - a) * prev[2],
    )

def _deadband_vec(v, threshold: float):
    """Snap small vector magnitudes to exact zero."""
    if threshold <= 0.0:
        return v
    return (0.0, 0.0, 0.0) if _vmag(v) < threshold else v


# --- IMU preprocessor ---
class IMUPreprocessor:
    def __init__(self):
        # History for Body-Frame Jerk
        self._prev_acc_body = None  # legacy/raw fallback only
        self._acc_body_lpf = None
        self._prev_acc_body_lpf_for_jerk = None
        # History for World-Frame EMA Smoothing (Path A — ESKF feed)
        self._prev_acc_world_clean = None
        self._prev_acc_tip_clean   = None   # Path A on tip-corrected signal
        # Path B — heavy EMA state for ZUPT only
        self._prev_acc_world_zupt = None
        self._prev_acc_world_zupt_old = None
        # High-pass filter state (sensor-point Path C)
        self._acc_world_hp_prev = None   # last HPF output y[n-1]
        self._acc_world_in_prev = None   # last HPF input  x[n-1]
        # High-pass filter state (tip-corrected Path C)
        self._acc_tip_hp_prev = None
        self._acc_tip_in_prev = None
        # Time and ZUPT state
        self._prev_ts = None
        self._still_streak = 0
        self._zupt_active = False

        # Rigid-body kinematics state (lever-arm tip correction)
        self._prev_quat_rb      = None   # previous normalized quat — for fallback ω_body derivation
        self._omega_body_lpf    = None   # low-pass omega before alpha derivative
        self._prev_omega_body_lpf = None # previous filtered omega for alpha derivative
        self._ema_alpha_body    = None   # EMA-smoothed α_body (noisy 2nd derivative)
        self._prev_omega_world  = None   # legacy diagnostic compatibility
        self._ema_alpha_world   = None   # legacy diagnostic compatibility

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

            if dt_us > (cfg.pipeline.imu_max_dt_ms * 1000):
                self.reset()

            dt_s = dt_us / 1_000_000.0
        else:
            dt_s = 1.0 / cfg.imu.sample_rate_hz

        self._prev_ts = ts

        # Hardware outputs m/s² natively, in the IMU/body frame.
        acc_ms2 = tuple(float(x) for x in acc)

        # --- Normalize quaternion; all derivative-sensitive math below is
        # --- filtered in the body frame BEFORE rotating to world coordinates.
        q_norm = _qnormalize(q)
        R_body_to_world = _q_to_R(q_norm)

        # ==========================================================
        # PRE-DERIVATIVE LOW-PASS + DEADBAND
        # ==========================================================
        # Never compute alpha or jerk from raw frame-to-frame noise.  First
        # low-pass the body-frame angular velocity and acceleration.  If a
        # hardware gyro is present, use it; otherwise fall back to quaternion
        # delta-derived omega so older recordings still work.
        gyro_ev = ev.get('gyro') or ev.get('gyr') or ev.get('omega_body_raw')
        if gyro_ev is not None:
            try:
                omega_body_raw = tuple(float(x) for x in gyro_ev)
            except (TypeError, ValueError):
                omega_body_raw = (0.0, 0.0, 0.0)
        elif self._prev_quat_rb is not None and dt_s > 1e-6:
            q_delta = _quat_mul(q_norm, _quat_conj(self._prev_quat_rb))
            if q_delta[3] < 0:              # choose shorter arc
                q_delta = (-q_delta[0], -q_delta[1], -q_delta[2], -q_delta[3])
            omega_body_raw = _vscale(q_delta[:3], 2.0 / dt_s)
        else:
            omega_body_raw = (0.0, 0.0, 0.0)

        if cfg.imu.pre_derivative_lpf_enabled:
            acc_body_clean = _ema_vec(self._acc_body_lpf, acc_ms2, cfg.imu.acc_lpf_alpha)
            omega_body = _ema_vec(self._omega_body_lpf, omega_body_raw, cfg.imu.gyro_lpf_alpha)
        else:
            acc_body_clean = acc_ms2
            omega_body = omega_body_raw

        # Dynamic deadband: silence near-zero gyro noise before it enters
        # centripetal correction, turn detection, ZUPT, or alpha derivation.
        omega_body = _deadband_vec(omega_body, cfg.imu.omega_deadband_rads)
        omega_world = _matvec3(R_body_to_world, omega_body)

        # Alpha is the derivative of the already-filtered omega.  Then apply the
        # existing alpha EMA as a second guard against derivative amplification.
        if self._prev_omega_body_lpf is not None and dt_s > 1e-6:
            alpha_body_raw = _vscale(_vsub(omega_body, self._prev_omega_body_lpf), 1.0 / dt_s)
        else:
            alpha_body_raw = (0.0, 0.0, 0.0)

        a_ema = cfg.imu.alpha_ema_alpha
        if self._ema_alpha_body is None:
            alpha_body = alpha_body_raw
        else:
            alpha_body = (
                a_ema * self._ema_alpha_body[0] + (1.0 - a_ema) * alpha_body_raw[0],
                a_ema * self._ema_alpha_body[1] + (1.0 - a_ema) * alpha_body_raw[1],
                a_ema * self._ema_alpha_body[2] + (1.0 - a_ema) * alpha_body_raw[2],
            )
        alpha_body = _deadband_vec(alpha_body, cfg.imu.alpha_deadband_rads2)
        alpha_world = _matvec3(R_body_to_world, alpha_body)

        # ==========================================================
        # RIGID-BODY TIP CORRECTION (BODY FRAME FIRST)
        # Converts IMU/sensor acceleration → marker-tip acceleration.
        #
        #   a_tip = a_sensor − α×r − ω×(ω×r)
        #
        # r is the tip→IMU vector in the same body frame as omega/alpha.
        # The body-frame correction is rotated into world coordinates only
        # after tangential and centripetal terms are removed.
        # ==========================================================
        r_imu = cfg.marker.r_imu_body_m
        s = float(cfg.imu.rigid_body_sign)
        # Preserve the previously validated sign convention.  With the current
        # config rigid_body_sign=-1, this resolves to cfg.marker.r_imu_body_m.
        r_body_tip_to_sensor = (-r_imu[0] * s, -r_imu[1] * s, -r_imu[2] * s)

        tangential_body  = _vcross(alpha_body, r_body_tip_to_sensor)
        centripetal_body = _vcross(omega_body, _vcross(omega_body, r_body_tip_to_sensor))

        if cfg.imu.rigid_body_enabled:
            acc_tip_body_raw = (
                acc_body_clean[0] - tangential_body[0] - centripetal_body[0],
                acc_body_clean[1] - tangential_body[1] - centripetal_body[1],
                acc_body_clean[2] - tangential_body[2] - centripetal_body[2],
            )
        else:
            acc_tip_body_raw = acc_body_clean

        # Rotate cleaned sensor and tip acceleration into world frame.
        acc_world_raw = _quat_rotate(q_norm, acc_body_clean)
        acc_tip_world_raw = _quat_rotate(q_norm, acc_tip_body_raw)

        # Update derivative state for next sample.
        self._prev_quat_rb = q_norm
        self._acc_body_lpf = acc_body_clean
        self._omega_body_lpf = omega_body
        self._prev_omega_body_lpf = omega_body
        self._ema_alpha_body = alpha_body
        # Keep legacy names populated for any external diagnostics.
        self._prev_omega_world = omega_world
        self._ema_alpha_world = alpha_world

        # ==========================================================
        # DUAL-PATH EMA ARCHITECTURE
        # ==========================================================

        eskf_alpha = cfg.imu.smooth_alpha_eskf

        # PATH A (legacy / diagnostic) — light EMA on sensor-point world acc.
        if self._prev_acc_world_clean is None:
            acc_world = acc_world_raw
        else:
            acc_world = (
                eskf_alpha * self._prev_acc_world_clean[0] + (1 - eskf_alpha) * acc_world_raw[0],
                eskf_alpha * self._prev_acc_world_clean[1] + (1 - eskf_alpha) * acc_world_raw[1],
                eskf_alpha * self._prev_acc_world_clean[2] + (1 - eskf_alpha) * acc_world_raw[2],
            )
        self._prev_acc_world_clean = acc_world

        # PATH A (tip-corrected) — light EMA on tip-corrected world acc.
        if self._prev_acc_tip_clean is None:
            acc_tip_world = acc_tip_world_raw
        else:
            acc_tip_world = (
                eskf_alpha * self._prev_acc_tip_clean[0] + (1 - eskf_alpha) * acc_tip_world_raw[0],
                eskf_alpha * self._prev_acc_tip_clean[1] + (1 - eskf_alpha) * acc_tip_world_raw[1],
                eskf_alpha * self._prev_acc_tip_clean[2] + (1 - eskf_alpha) * acc_tip_world_raw[2],
            )
        self._prev_acc_tip_clean = acc_tip_world

        # PATH B — Heavy EMA for ZUPT only: kills micro-tremor, never exported.
        zupt_alpha = 0.95
        if self._prev_acc_world_zupt is None:
            self._prev_acc_world_zupt     = acc_world_raw
            self._prev_acc_world_zupt_old = acc_world_raw

        acc_world_zupt = (
            zupt_alpha * self._prev_acc_world_zupt[0] + (1 - zupt_alpha) * acc_world_raw[0],
            zupt_alpha * self._prev_acc_world_zupt[1] + (1 - zupt_alpha) * acc_world_raw[1],
            zupt_alpha * self._prev_acc_world_zupt[2] + (1 - zupt_alpha) * acc_world_raw[2],
        )
        self._prev_acc_world_zupt = acc_world_zupt

        # --- Board Projection ---
        acc_board     = _project_board_axes(acc_world)        # legacy diagnostic
        acc_board_tip = _project_board_axes(acc_tip_world)    # canonical ESKF input

        # PATH C — 1st-order High-Pass Filter on acc_tip_world (bias rejection).
        # Runs on the tip-corrected signal so both rotational whip AND bias are removed.
        # Formula: y[n] = α·(y[n-1] + x[n] − x[n-1]),  α = RC/(RC+dt)
        if cfg.imu.hpf_enabled:
            RC      = 1.0 / (2.0 * math.pi * cfg.imu.hpf_cutoff_hz)
            alpha_h = RC / (RC + dt_s)
            if self._acc_world_in_prev is None:
                acc_world_hp     = (0.0, 0.0, 0.0)
                acc_tip_world_hp = (0.0, 0.0, 0.0)
                self._acc_world_hp_prev     = acc_world_hp
                self._acc_tip_hp_prev       = acc_tip_world_hp
            else:
                acc_world_hp = (
                    alpha_h * (self._acc_world_hp_prev[0] + acc_world[0] - self._acc_world_in_prev[0]),
                    alpha_h * (self._acc_world_hp_prev[1] + acc_world[1] - self._acc_world_in_prev[1]),
                    alpha_h * (self._acc_world_hp_prev[2] + acc_world[2] - self._acc_world_in_prev[2]),
                )
                acc_tip_world_hp = (
                    alpha_h * (self._acc_tip_hp_prev[0] + acc_tip_world[0] - self._acc_tip_in_prev[0]),
                    alpha_h * (self._acc_tip_hp_prev[1] + acc_tip_world[1] - self._acc_tip_in_prev[1]),
                    alpha_h * (self._acc_tip_hp_prev[2] + acc_tip_world[2] - self._acc_tip_in_prev[2]),
                )
                self._acc_world_hp_prev = acc_world_hp
                self._acc_tip_hp_prev   = acc_tip_world_hp
            self._acc_world_in_prev = acc_world
            self._acc_tip_in_prev   = acc_tip_world
            acc_board_hp     = _project_board_axes(acc_world_hp)
            acc_board_hp_tip = _project_board_axes(acc_tip_world_hp)
        else:
            acc_board_hp     = acc_board
            acc_board_hp_tip = acc_board_tip

        # --- Body-Frame Jerk (pre-derivative low-pass) ---
        # Jerk is also a derivative, so use the filtered body acceleration.
        # This prevents micro-tremor/electrical noise from arming turn logic.
        if self._prev_acc_body_lpf_for_jerk is not None and dt_s > 0:
            delta_body = _vsub(acc_body_clean, self._prev_acc_body_lpf_for_jerk)
            jerk = _vmag(delta_body) / dt_s
        else:
            jerk = 0.0
        self._prev_acc_body_lpf_for_jerk = acc_body_clean
        self._prev_acc_body = acc_ms2

        # --- Smoothed ZUPT Jerk (Path B — prevents tremor spikes blocking ZUPT) ---
        if dt_s > 0:
            delta_zupt = _vsub(acc_world_zupt, self._prev_acc_world_zupt_old)
            zupt_jerk  = _vmag(delta_zupt) / dt_s
        else:
            zupt_jerk = 0.0
        self._prev_acc_world_zupt_old = acc_world_zupt

        # --- ZUPT (motion detector — uses Path B only) ---
        # Rule 1: true stillness requires |a| ≈ 0, jerk ≈ 0, AND |ω| ≈ 0.
        # omega_world already computed above; its board-plane magnitude is sufficient.
        lin_mag_zupt = _vmag(acc_world_zupt)
        omega_mag    = _vmag(omega_world) if omega_world is not None else 0.0
        still_now = (
            lin_mag_zupt < cfg.imu.zupt_acc_threshold and
            zupt_jerk    < cfg.imu.zupt_jerk_threshold and
            omega_mag    < cfg.imu.zupt_omega_threshold
        )

        if still_now:
            self._still_streak += 1
        else:
            self._still_streak = 0
            self._zupt_active  = False

        if self._still_streak >= self._zupt_min_samples:
            self._zupt_active = True

        # --- Force Contact ---
        force   = ev.get('force', 0.0)
        contact = force >= cfg.imu.force_contact_threshold

        return {
            'sensor':      'IMU',
            'ts_hw':       ts,
            'packet_id':   ev.get('packet_id'),
            'sample_idx':  ev.get('sample_idx'),
            'quat':        q_norm,
            'acc_sensor':  acc_ms2,
            'acc_body_lpf': acc_body_clean,
            # --- legacy / diagnostic (sensor-point, not tip-corrected) ---
            'acc_world':    acc_world,
            'acc_board':    acc_board,
            'acc_board_hp': acc_board_hp,
            # --- tip-corrected outputs (canonical ESKF inputs) ---
            'acc_tip_world':    acc_tip_world,
            'acc_board_tip':    acc_board_tip,
            'acc_board_hp_tip': acc_board_hp_tip,
            # --- angular kinematics (consumed by ESKF turn detection) ---
            'omega_world': omega_world,
            'omega_body':  omega_body,
            'omega_body_raw': omega_body_raw,
            'alpha_world': alpha_world,
            'alpha_body':  alpha_body,
            # --- contact / motion ---
            'jerk':           round(jerk, 6),
            'omega_mag_world': round(omega_mag, 6),
            'is_static':      self._zupt_active,
            'contact':        contact,
            'force':          force,
        }

    def reset(self):
        """Clear integration history on startup or buffer limit"""
        self._prev_acc_body = None
        self._acc_body_lpf = None
        self._prev_acc_body_lpf_for_jerk = None
        self._prev_acc_world_clean = None
        self._prev_acc_world_zupt = None
        self._prev_acc_world_zupt_old = None
        self._acc_world_hp_prev = None
        self._acc_world_in_prev = None
        self._prev_ts = None
        self._still_streak = 0
        self._zupt_active = False
        self._prev_quat_rb     = None
        self._omega_body_lpf = None
        self._prev_omega_body_lpf = None
        self._ema_alpha_body = None
        self._prev_omega_world = None
        self._ema_alpha_world  = None
        self._prev_acc_tip_clean = None
        self._acc_tip_hp_prev    = None
        self._acc_tip_in_prev    = None



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