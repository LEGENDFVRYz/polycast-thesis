"""
[config] - Centralized Configuration for the "pipelines" of polycast

All of the configuration here are solely intended for main pipeline
modules, only to be used my the module inside this folder (pipelines)

Note: Main Pipelines - Transforming Marker Sensor to Stroke Coordinates

"""

from dataclasses import dataclass, field


# ------------------------------------------------------------------------
# SERIAL / HARDWARE
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM20"
    baud: int = 921600


# ------------------------------------------------------------------------
# IMU (Adafruit BNO085)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUConfig:
    sample_rate_hz: float = 180.0

    # ZUPT
    # Thresholds raised for fast writing regime (abc4/5/6): median jerk is
    # 600–760 m/s³ and ω rarely drops below 0.08 rad/s mid-stroke, so the
    # original values never fired.  The hard reset (N=20 consecutive samples)
    # still guards against spurious mid-stroke v[:]=0.
    zupt_acc_threshold: float = 0.25   # 0.15 → 0.25
    zupt_jerk_threshold: float = 120.0  # 32.5 → 200.0
    zupt_min_duration_s: float = 0.05
    zupt_omega_threshold: float = 0.25  # 0.08 → 0.25 rad/s

    # Contact / force
    force_contact_threshold: float = 100.0

    # Physics
    gravity_ms2: float = 9.81
    quat_norm_epsilon: float = 1e-6

    # Board projection
    board_axes: tuple[str, str] = ("x", "z")
    smooth_alpha: float = 0.75          # legacy heavy EMA — superseded by smooth_alpha_eskf
    smooth_alpha_eskf: float = 0.25     # Path-A light EMA fed to ESKF (near-raw, minimal lag)
    acc_is_linear: bool = True

    # High-pass filter (Path C) — strips DC bias drift before ESKF integration
    hpf_enabled: bool = True
    hpf_cutoff_hz: float = 1.5         # 0.5 Hz: below handwriting (2–8 Hz), kills bias in ~2 s

    # Rigid-body tip correction (lever-arm kinematics)
    rigid_body_enabled: bool = True     # ablation toggle — False reverts to sensor-point acc
    alpha_ema_alpha: float = 0.7        # EMA weight on α_world (angular accel is noisy 2nd deriv)


# ------------------------------------------------------------------------
# CONTACT STATE DETECTOR
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class ContactConfig:
    force_exit_ratio:     float = 0.70   # exit threshold = force_enter * ratio
    pen_down_debounce_ms: float = 10.0   # reject bumps/spikes (ms of sustained force)
    pen_up_debounce_ms:   float = 30.0   # reject tremor dips (ms below exit threshold)
    min_draw_ms:          float = 30.0   # cumulative CONTACT_DRAWING before session opens
    state_debounce_n:     int   = 5      # N consecutive samples to confirm substate change


# ------------------------------------------------------------------------
# UWB (Ai Thinker BU03)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class UWBConfig:
    range_offsets_m: tuple = (-0.1538, -0.0134, -0.1833, -0.0960)
    # range_offsets_m: tuple = (-0.1752, -0.0466, -0.2227, -0.1620)
    # range_offsets_m: tuple = (-0.1752, -0.0466, -0.2227, -0.1220)
    # range_offsets_m: tuple = (-0.1232, -0.0146, -0.1919, -0.0965)

    rate_hz: float = 50.0

    ema_alpha: float = 0.25
    max_range_jump_m: float = 0.40
    median_window: int = 10

    outlier_speed_limit_ms: float = 2.0
    drop_speed_outliers: bool = True
    num_anchors: int = 4
    pos_ema_alpha: float = 0.75             # legacy — kept for reference, superseded by pos_alpha
    trilat_max_residual: float = 0.12

    # Alpha-Beta filter (replaces scalar EMA)
    pos_alpha: float = 0.60                 # position correction gain
    pos_beta: float = 0.05                  # velocity correction gain

    # Trilateration stale-guess recovery
    stale_guess_timeout_us: int = 1_000_000 # 1 second gap triggers centroid re-seed

    # Range filter time-aware EMA (tau = smoothing time constant)
    range_tau_s: float = 0.30               # at 9 Hz (dt≈0.11s): α ≈ 0.31; tune up to slow down

    # Weighted least squares (inverse distance weighting in trilateration)
    wls_power: float   = 2.0                # exponent: 2.0 = inverse-square, 1.0 = inverse-linear
    wls_epsilon: float = 0.01               # numerical guard prevents ÷0 at very-close anchors


# ------------------------------------------------------------------------
# ANCHOR GEOMETRY (Target Whiteboard)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class AnchorConfig:
    board_size_x: float = 1.25
    board_size_y: float = 1.24

    a0: tuple[float, float, float] = (0.00, 0.00, 0.07)
    a1: tuple[float, float, float] = (1.25, 0.00, 0.07)
    a2: tuple[float, float, float] = (1.25, 1.24, 0.07)
    a3: tuple[float, float, float] = (0.00, 1.24, 0.07)

    positions: tuple[tuple[float, float, float], ...] = (
        (0.00, 0.00, 0.07),
        (1.25, 0.00, 0.07),
        (1.25, 1.24, 0.07),
        (0.00, 1.24, 0.07),
    )


# ------------------------------------------------------------------------
# PIPELINE (constraints)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class PipelineConfig:
    time_align_buffer: int = 2000
    imu_max_dt_ms: int = 100

    uwb_min_range_m: float = 0.05
    uwb_max_range_m: float = 6.00


# ------------------------------------------------------------------------
# MARKER (physical layout)
#   Body frame: z_body = pen long axis, tip at origin, sensors stacked above.
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class MarkerConfig:
    # Changed from (0, 0, Z) to (0, Y, 0)
    # This means the IMU's Z-axis is pointing perpendicular to the pen shaft, not along it
    # The pen shaft is actually aligned with the IMU's Y-axis
    r_imu_body_m: tuple[float, float, float] = (0.110, 0.0, 0.0)   # tip → IMU
    r_uwb_body_m: tuple[float, float, float] = (0.200, 0.0, 0.0)   # tip → UWB


# ------------------------------------------------------------------------
# FUSION — per-mode parameter table
# One row per operating mode: DRAWING / AIR / STATIC.
# All conditional `if drawing:` branches in the filter read from here.
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionModeParams:
    # UWB sigma scale applied on top of sigma_uwb.
    # Lower  → trust UWB more (stronger pull toward measured position).
    sigma_scale: float      = 0.55

    # Velocity drag  s⁻¹.  Higher → IMU integration shrinks faster.
    drag_inv_s: float       = 0.4

    # Direction-disagreement penalty multiplier on sigma.
    # Lower during drawing — handwriting has legitimate backward curves.
    dir_penalty: float      = 3.0

    # UWB jump-gate ceiling (m/s implied tip speed).
    # Higher during drawing — fast strokes are physically plausible.
    jump_speed_max: float   = 1.2

    # Position covariance floor (m std).
    # Lower in air so AIR_MOVE doesn't artificially inflate P.
    pos_floor: float        = 0.010

    # IMU acceleration scale applied before integration.
    # Reduce in air to suppress dead-reckoning drift between UWB fixes.
    acc_scale: float        = 1.0


@dataclass(frozen=True)
class FusionModeTable:
    drawing: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale=0.55,      # 0.42→0.55: pulls K off 0.50 ceiling toward 0.12–0.22 target
        drag_inv_s=1.15,
        dir_penalty=1.8,
        jump_speed_max=1.8,
        pos_floor=0.020,       # 0.025→0.020: lets P shrink so K is floor-driven less often
    ))
    air: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale=1.6,       # 1.8→1.6: slightly more UWB authority on re-entry
        drag_inv_s=2.0,        # 1.2→2.0: stronger drag clamps IMU dead-reckoning drift in air
        dir_penalty=3.0,
        jump_speed_max=1.4,    # 1.2→1.4: small/fast strokes legitimately exceed 1.2 m/s briefly
        pos_floor=0.015,       # 0.010→0.015: higher floor in air → more K authority on re-entry
        acc_scale=0.20,        # suppress IMU integration during air to reduce drift
    ))
    static: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale   = 2.0,    # heavily distrust UWB when pen is still
        drag_inv_s    = 0.4,
        dir_penalty   = 3.0,
        jump_speed_max= 1.2,
        pos_floor     = 0.010,
    ))


# ------------------------------------------------------------------------
# FUSION (ESKF)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionESKFConfig:
    # ── Process noise ─────────────────────────────────────────────────────
    sigma_a: float           = 2.2      # m/s²
    sigma_b_a: float         = 0.0001   # m/s²·√Hz
    sigma_zupt: float        = 0.005

    # ── Measurement noise ─────────────────────────────────────────────────
    sigma_uwb: float         = 0.06     # m — base; mode table scales this
    sigma_trilat: float      = 0.09     # m — velocity-pseudo gate

    # NLOS-adaptive R
    k_nlos: float            = 1.5
    r_scale_max: float       = 25.0
    hard_reject_mult: float  = 7.5

    # ── Turn detection ────────────────────────────────────────────────────
    turn_omega_threshold: float = 0.5   # rad/s
    turn_k_q: float             = 8.0
    turn_n_post: int            = 5

    # ── Ring buffer ───────────────────────────────────────────────────────
    state_buffer_size: int      = 75

    # ── ITrackU-style hybrid reset (Phase 3) ─────────────────────────────
    zupt_hard_reset_n: int     = 20

    # 3b: velocity pseudo-measurement noise
    sigma_uwb_vel: float           = 0.05
    sigma_uwb_vel_min_scale: float = 0.2

    # ── Sliding-window safeguard (Rule 3) ─────────────────────────────────
    uwb_window_s: float     = 0.30
    uwb_stale_k: float      = 2.0
    uwb_stale_max_k: float  = 5.0

    # ── Innovation direction gate (mode-independent thresholds) ───────────
    dir_check_v_min: float      = 0.03  # m/s
    dir_check_y_min: float      = 0.02  # m
    dir_check_cos_thresh: float = -0.3  # ≈107°

    # ── UWB jump gate (mode-independent IMU confirmation threshold) ───────
    uwb_jump_imu_speed_min: float = 0.5  # m/s

    # ── Velocity floor ────────────────────────────────────────────────────
    vel_floor: float = 0.08             # m/s std — same across all modes

    # ── Stroke-end hard-reset ─────────────────────────────────────────────
    stroke_end_v_decay: float     = 0.0
    stroke_end_p_vel_scale: float = 0.5

    # ── dt jitter clamp ───────────────────────────────────────────────────
    # IMU timestamps occasionally spike 8–10× nominal (30+ ms at 180 Hz).
    # Clamping to N× nominal bounds dead-reckoning error per spike to O(dt_max²).
    imu_dt_max_mult: float = 3.0

    # ── Turn gate ─────────────────────────────────────────────────────────
    # Raised from hardcoded 800 m/s³ — fast writing has median jerk 600–760 m/s³
    # so the old threshold was effectively always tripped.
    turn_jerk_threshold: float = 1500.0
    turn_arm_n: int            = 3       # consecutive samples before TURN arms

    # ── Position covariance cap ───────────────────────────────────────────
    # Prevents K≈1 snap on first UWB after long pen-up; cap = mult × pos_floor std
    pos_cap_mult: float = 6.0

    # ── Per-mode parameter table ──────────────────────────────────────────
    modes: FusionModeTable = field(default_factory=FusionModeTable)

    # ── Initial covariance ────────────────────────────────────────────────
    p0_pos: float    = 0.35
    p0_vel: float    = 0.25
    p0_bias: float   = 0.05


# ------------------------------------------------------------------------
# ROOT CONFIG (wrapper)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    serial:  SerialConfig  = field(default_factory=SerialConfig)
    imu:     IMUConfig     = field(default_factory=IMUConfig)
    contact: ContactConfig = field(default_factory=ContactConfig)
    uwb:     UWBConfig     = field(default_factory=UWBConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    marker: MarkerConfig = field(default_factory=MarkerConfig)
    fusion_eskf: FusionESKFConfig = field(default_factory=FusionESKFConfig)


cfg = Config()


# ------------------------------------------------------------------------
# VALIDATION TESTING
# ------------------------------------------------------------------------
if __name__ == "__main__":
    print(cfg)

    assert cfg.imu.zupt_acc_threshold == 0.50
    assert cfg.uwb.ema_alpha == 0.25
    assert cfg.anchors.positions[0] == (0.00, 0.00, 0.07)

    try:
        cfg.imu.gravity_ms2 = 99
    except Exception:
        print("✓ Config is immutable")