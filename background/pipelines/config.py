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
    sample_rate_hz: float = 200.0

    # ZUPT
    zupt_acc_threshold: float = 0.15
    zupt_jerk_threshold: float = 32.5
    zupt_min_duration_s: float = 0.05
    zupt_omega_threshold: float = 0.08  # rad/s — gyro stillness gate (~4.6 dps)

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
    pen_up_debounce_ms:   float = 174.0   # reject tremor dips (ms below exit threshold)
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
    trilat_max_residual: float = 0.15

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
# FUSION (ESKF)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionESKFConfig:
    # ── Process noise ─────────────────────────────────────────────────────
    # sigma_a raised: Path-A feeds near-raw 200 Hz acc, so real micro-accels
    # are present — inflate Q so UWB retains authority between updates.
    sigma_a: float           = 1.10      # m/s² (was 0.15)
    # sigma_b_a tightened further: prevents b_a from absorbing IMU/UWB disagreement
    # during CONTACT_DRAWING where ZUPT never fires (was 0.002, was 0.005).
    sigma_b_a: float         = 0.0001    # m/s²·√Hz
    sigma_zupt: float        = 0.005     # unchanged — already aggressive

    # ── Measurement noise ─────────────────────────────────────────────────
    # sigma_uwb tightened: WLS + α-β filter gives much cleaner pos_raw than before.
    # Each UWB update now pulls harder so IMU drift doesn't accumulate between fixes.
    sigma_uwb: float         = 0.035      # m (was 0.12)
    sigma_trilat: float      = 0.09      # m (was 0.04 — widen velocity-pseudo gate to anchor IMU vel more often)

    # NLOS-adaptive R re-enabled: WLS solve_error is now a reliable confidence signal.
    k_nlos: float            = 1.5       # (was 0.00)
    r_scale_max: float       = 25.0      # (was 100.0 — tighter ceiling, avoids completely freezing updates)

    # Hard-reject threshold tightened back to intended value.
    hard_reject_mult: float  = 7.5       # (was 15.0 — comment said 6.0, now actually enforced)

    # ── Turn detection ────────────────────────────────────────────────────
    # Threshold lowered slightly: cleaner acc signal means real corners are
    # detectable earlier without false positives from noise.
    turn_omega_threshold: float = 0.5    # rad/s (was 1.85 — ~86 dps)
    turn_k_q: float             = 5.8   # (was 3.0 — let UWB shape corners harder)
    turn_n_post: int            = 2      # unchanged

    # ── Ring buffer ───────────────────────────────────────────────────────
    # Enlarged for 200 Hz IMU: covers a ~0.3 s window for robust UWB time-interpolation.
    state_buffer_size: int      = 75     # (was 20)

    # ── Velocity drag ─────────────────────────────────────────────────────
    # Reduced: BUG-1 lag was masking the need for heavy drag. With near-raw
    # Path-A acc, lighter drag still suppresses lever-arm runaway without
    # artificially killing real pen velocity.
    velocity_drag_inv_s: float  = 2.5   # s⁻¹ (was 5.0)

    # ── ITrackU-style hybrid reset (Phase 3) ─────────────────────────────
    # 3a: after this many consecutive static IMU samples, hard-zero velocity.
    # At 200 Hz, 20 samples = 100 ms of confirmed stillness.
    zupt_hard_reset_n: int     = 20

    # 3b: velocity pseudo-measurement noise when UWB is pristine.
    # Applied when solve_error < sigma_trilat (reliable trilateration).
    sigma_uwb_vel: float       = 0.05    # m/s (was 0.08 — tighter now that gate is wider)
    # Floor scale for adaptive sigma: at solve_error→0, sigma shrinks to min_scale * sigma_uwb_vel.
    sigma_uwb_vel_min_scale: float = 0.2  # dimensionless (0.2 → 0.01 m/s minimum)

    # 3c: sigma_uwb scale factor while pen is actively drawing.
    # Pen physically constrained to board → trust UWB more during strokes.
    contact_sigma_scale: float = 1.25    # 30 % tighter (multiplicative)

    # ── Sliding-window safeguard (Rule 3) ────────────────────────────────────
    # Bounds how long IMU dead-reckoning runs without a UWB velocity anchor.
    # After uwb_window_s of UWB silence, Q inflates and velocity drag increases.
    uwb_window_s: float     = 0.30   # s — matches paper's sub-cm threshold (t < 0.3 s)
    uwb_stale_k: float      = 2.0    # ramp slope per window-length of over-run
    uwb_stale_max_k: float  = 5.0    # max stale factor (Q inflates ≤25×, drag ≤5×)

    # ── Initial covariance ────────────────────────────────────────────────
    p0_pos: float    = 0.20
    p0_vel: float    = 0.10
    p0_bias: float   = 0.05


# ------------------------------------------------------------------------
# POSTPROCESS (RTS smoother + spline resample)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class PostprocessConfig:
    rts_enabled:            bool  = True
    rts_max_stroke_samples: int   = 5000    # hard cap ~20 s at 200 Hz
    spline_resample_ds_m:   float = 0.001    # 1 mm arc-length step
    spline_min_points:      int   = 5       # skip spline below this count


# ------------------------------------------------------------------------
# ROOT CONFIG (wrapper)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    serial:      SerialConfig      = field(default_factory=SerialConfig)
    imu:         IMUConfig         = field(default_factory=IMUConfig)
    contact:     ContactConfig     = field(default_factory=ContactConfig)
    uwb:         UWBConfig         = field(default_factory=UWBConfig)
    anchors:     AnchorConfig      = field(default_factory=AnchorConfig)
    pipeline:    PipelineConfig    = field(default_factory=PipelineConfig)
    marker:      MarkerConfig      = field(default_factory=MarkerConfig)
    fusion_eskf: FusionESKFConfig  = field(default_factory=FusionESKFConfig)
    postprocess: PostprocessConfig = field(default_factory=PostprocessConfig)


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