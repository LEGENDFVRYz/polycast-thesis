"""
[config] - Centralized Configuration for the "pipelines" of polycast

All of the configuration here are solely intended for main pipeline
modules, only to be used my the module inside this folder (pipelines)

Note: Main Pipelines - Transforming Marker Sensor to Stroke Coordinates

Handwriting Base Config v1:
- Goal: capture small/fast handwritten letters, not geometric shapes.
- Philosophy:
    IMU = short-term stroke detail / fast motion shape
    UWB = long-term absolute anchor / drift correction
    Force = stroke state / drawing mode selector
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
    # Actual observed effective rate is closer to 170–180 Hz than ideal 250 Hz.
    # ESKF dt clamp depends on this value, so keep it near measured reality.
    sample_rate_hz: float = 180.0

    # ZUPT / stillness detector.
    # These values are intentionally permissive enough to catch real pauses,
    # but not so permissive that small handwriting corners get velocity-killed.
    zupt_acc_threshold: float = 0.25
    zupt_jerk_threshold: float = 140.0
    zupt_min_duration_s: float = 0.06
    zupt_omega_threshold: float = 0.25

    # Force sensor threshold for pen-down detection.
    # This controls when contact.py may classify CONTACT_DRAWING.
    force_contact_threshold: float = 100.0

    # Physical constants / quaternion safety.
    gravity_ms2: float = 9.81
    quat_norm_epsilon: float = 1e-6

    # Board projection.
    # The 2D board plane is X-Z in world coordinates.
    board_axes: tuple[str, str] = ("x", "z")

    # Legacy EMA kept for diagnostics.
    smooth_alpha: float = 0.75

    # Light EMA for IMU Path-A. Lower = more responsive, higher = smoother.
    # ESKF now prefers HPF acceleration, but this still affects diagnostics
    # and fallback behavior.
    smooth_alpha_eskf: float = 0.25

    # BNO085 linear acceleration is assumed gravity-compensated.
    acc_is_linear: bool = True

    # High-pass filter used by ESKF for handwriting motion.
    # This removes slow bias/DC drift while preserving fast letter strokes.
    hpf_enabled: bool = True
    hpf_cutoff_hz: float = 1.5

    # Rigid-body tip correction.
    # Converts sensor-end acceleration to estimated tip acceleration.
    # Keep enabled, but verify lever_arm_m is near zero when marker is perpendicular.
    rigid_body_enabled: bool = True
    # A/B diagnostic: +1 = normal sensor→tip vector, -1 = negated (test wrong-sign hypothesis).
    # Only meaningful when rigid_body_enabled=True. Set to 1 for production.
    rigid_body_sign: int = 1

    # EMA on angular acceleration used in rigid-body tip correction.
    # Higher = smoother but more lag; 0.7 is conservative.
    alpha_ema_alpha: float = 0.7


# ------------------------------------------------------------------------
# CONTACT STATE DETECTOR
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class ContactConfig:
    # Hysteresis: pen exits contact at force_enter * ratio.
    force_exit_ratio: float = 0.70

    # Debounce prevents force spikes/dips from fragmenting strokes.
    pen_down_debounce_ms: float = 10.0
    pen_up_debounce_ms: float = 8.0   # Phase-3: reduced from 30 ms — debounce window was wider than FSR noise requires

    # Minimum confirmed drawing time before opening a stroke.
    min_draw_ms: float = 25.0

    # Consecutive samples needed to confirm contact sub-state changes.
    state_debounce_n: int = 4


# ------------------------------------------------------------------------
# UWB (Ai Thinker BU03)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class UWBConfig:
    # Per-anchor range calibration offsets.
    range_offsets_m: tuple = (-0.1538, -0.0134, -0.1833, -0.0960)

    # Nominal UWB rate.
    rate_hz: float = 50.0

    # Legacy range EMA alpha, kept for old modules.
    ema_alpha: float = 0.25

    # Per-anchor range jump threshold.
    # Larger values allow fast movement but risk accepting spikes.
    max_range_jump_m: float = 0.40

    # Median window for per-anchor range filtering.
    median_window: int = 10

    # Position-level speed outlier gate.
    outlier_speed_limit_ms: float = 2.0
    drop_speed_outliers: bool = True

    num_anchors: int = 4

    # Legacy position EMA, kept for compatibility.
    pos_ema_alpha: float = 0.75

    # Hard trilateration RMS rejection threshold.
    # 0.12 rejects the worst geometric failures while keeping difficult motion.
    trilat_max_residual: float = 0.12

    # Alpha-Beta position smoother.
    # Higher alpha follows UWB faster; beta estimates UWB velocity.
    # For handwriting, avoid too much beta because it can create UWB tail drift.
    pos_alpha: float = 0.55
    pos_beta: float = 0.035

    # If UWB pauses too long, reset solver initial guess.
    stale_guess_timeout_us: int = 1_000_000

    # Time-aware EMA smoothing constant for ranges.
    # Lower = more responsive; higher = smoother but laggier.
    range_tau_s: float = 0.25

    # Weighted least squares weighting.
    # Higher power trusts nearer anchors more.
    wls_power: float = 2.0
    wls_epsilon: float = 0.01


# ------------------------------------------------------------------------
# ANCHOR GEOMETRY (Target Whiteboard)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class AnchorConfig:
    board_size_x: float = 1.25
    board_size_y: float = 1.24

    # Anchors are mounted at board corners with slight depth offset.
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
    # Buffer for timestamp sorting across sensors.
    time_align_buffer: int = 2000

    # IMU preprocessor resets if a very long hardware gap appears.
    imu_max_dt_ms: int = 100

    # UWB range sanity limits.
    uwb_min_range_m: float = 0.05
    uwb_max_range_m: float = 6.00


# ------------------------------------------------------------------------
# MARKER (physical layout)
#   Body frame: tip at origin; sensors are mounted near marker end.
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class MarkerConfig:
    # Current prototype body-axis mapping.
    # For perpendicular wall testing, projected board-plane lever arm should be small.
    r_imu_body_m: tuple[float, float, float] = (0.110, 0.0, 0.0)
    r_uwb_body_m: tuple[float, float, float] = (0.200, 0.0, 0.0)


# ------------------------------------------------------------------------
# FUSION — per-mode parameter table
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionModeParams:
    # UWB sigma scale on top of sigma_uwb.
    # Lower  = stronger UWB anchor.
    # Higher = more IMU freedom.
    sigma_scale: float = 0.55

    # Velocity drag in s⁻¹.
    # Higher = velocity dies faster, smoother but less expressive.
    drag_inv_s: float = 0.4

    # Direction-disagreement penalty.
    # Higher penalizes UWB if it pulls opposite IMU velocity.
    dir_penalty: float = 3.0

    # UWB jump speed limit.
    jump_speed_max: float = 1.2

    # Position covariance floor in metres.
    # Higher keeps Kalman gain alive.
    pos_floor: float = 0.010

    # IMU acceleration scale.
    # Higher gives IMU more stroke shape authority.
    acc_scale: float = 1.0

    # Hard ceiling on position Kalman gain K[0:2].
    # Values < 1.0 prevent a momentarily clean UWB from yanking the stroke
    # even if R collapses.  Set to 1.0 to disable.
    pos_gain_cap: float = 1.0


@dataclass(frozen=True)
class FusionModeTable:
    # Normal pen-down drawing.
    # IMU owns letter shape; UWB gives a gentle global nudge only.
    drawing: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.85,   # Phase-1: weakened UWB pull during contact so IMU preserves handwritten curvature
        drag_inv_s     = 0.70,   # keep from Stage-1 tuner — stable on abc_s1/s2
        dir_penalty    = 1.5,    # relaxed — handwriting has legitimate backward curves
        jump_speed_max = 1.8,
        pos_floor      = 0.005,  # keep from Stage-1 tuner — lower floor prevents excessive UWB gain during contact
        acc_scale      = 1.35,   # keep from Stage-1 tuner
        pos_gain_cap   = 0.030,  # Phase-4c: very tight cap — prevents IMU runaway without UWB sculpting the letter
    ))

    # Short high-speed burst mode.
    # IMU authority burst; UWB kept loosely so fast strokes don't explode.
    drawing_fast: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 1.05,   # Phase-1: moderate loosening for fast strokes
        drag_inv_s     = 0.35,   # light damping preserves burst motion without excessive drift
        dir_penalty    = 1.1,
        jump_speed_max = 2.6,
        pos_floor      = 0.030,
        acc_scale      = 1.00,   # keep from Stage-2 tuner
        pos_gain_cap   = 0.060,  # Phase-4c: tighter than before; fast strokes still need slightly more latitude
    ))

    # Pen lifted / air movement.
    # UWB re-anchors aggressively; IMU suppressed to prevent drift carry-over.
    air: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.48,   # Phase-1: strong UWB pull between strokes to re-anchor placement
        drag_inv_s     = 4.0,    # keep from Stage-3 tuner — aggressively kill air dead-reckoning drift
        dir_penalty    = 3.0,
        jump_speed_max = 1.4,
        pos_floor      = 0.015,
        acc_scale      = 0.08,   # suppress IMU integration while pen is lifted
        pos_gain_cap   = 1.0,    # no cap in air — UWB should correct freely
    ))

    # Still/idle mode.
    # UWB re-anchors hard; IMU completely suppressed.
    static: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.31,   # Phase-1: very strong UWB pull when truly idle
        drag_inv_s     = 0.6,
        dir_penalty    = 3.0,
        jump_speed_max = 1.2,
        pos_floor      = 0.010,
        acc_scale      = 0.0,
        pos_gain_cap   = 1.0,    # no cap when idle
    ))


# ------------------------------------------------------------------------
# FUSION (ESKF)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionESKFConfig:
    # Process noise for acceleration.
    # Higher = filter admits IMU prediction uncertainty and lets UWB correct.
    # Too high makes UWB dominate; too low makes IMU drift dominate.
    sigma_a: float = 2.4
    # A/B diagnostic: clamp in-stroke acceleration magnitude to prevent impulse excursions.
    # Disabled by default; enable to test whether spikes are causing loop distortion.
    acc_spike_clamp_enabled: bool = False
    acc_spike_clamp_ms2:     float = 7.0   # threshold in m/s²; 6–8 suggested

    # Acceleration-bias random walk.
    # Keep very small so bias does not absorb UWB/IMU disagreement too quickly.
    sigma_b_a: float = 0.0001

    # ZUPT velocity measurement noise.
    sigma_zupt: float = 0.005

    # Base UWB measurement noise before mode scaling.
    sigma_uwb: float = 0.060

    # Trilateration quality threshold for UWB velocity pseudo-update.
    sigma_trilat: float = 0.09

    # NLOS adaptive R parameters.
    k_nlos: float = 0.5     # tuner Stage-4 winner: less aggressive NLOS scaling on current datasets
    r_scale_max: float = 10.0  # tuner Stage-4 winner: cap R inflation earlier
    hard_reject_mult: float = 3.0  # tighter hard rejection for bad trilateration periods
    innov_hard_reject_m: float = 0.50  # UWB innovation magnitude hard reject (m); >25 cm UWB↔IMU disagreement is non-physical in board writing
    # Recovery: after this many consecutive innovation-gate rejections with clean
    # UWB geometry the filter is assumed lost and _snap_to_uwb() re-localizes.
    # At 50 Hz UWB this is ~120 ms before recovery triggers.
    innov_recovery_n: int = 6

    # Turn detection.
    # For handwriting, avoid making TURN fire too often.
    turn_omega_threshold: float = 0.45
    turn_k_q: float = 6.0
    turn_n_post: int = 5

    # IMU state buffer for UWB timestamp interpolation.
    state_buffer_size: int = 75

    # Static hard reset after sustained ZUPT.
    zupt_hard_reset_n: int = 20

    # UWB velocity pseudo-measurement.
    # This is useful, but can make velocity too UWB-shaped.
    sigma_uwb_vel: float = 0.075
    sigma_uwb_vel_min_scale: float = 0.35
    # A/B diagnostic: if True, skip UWB velocity pseudo-update during any active stroke.
    # Prevents UWB-derived velocity from fighting IMU curved motion.
    uwb_vel_stroke_gate: bool = False

    # If accepted UWB updates are stale, inflate Q and increase drag.
    uwb_window_s: float = 0.30
    uwb_stale_k: float = 2.0
    uwb_stale_max_k: float = 5.0

    # Direction check thresholds.
    dir_check_v_min: float = 0.03
    dir_check_y_min: float = 0.02
    dir_check_cos_thresh: float = -0.3

    # UWB jump gate: if UWB implies fast motion but IMU speed is low, reject.
    # Lowered from 0.5 → 0.15 so the gate fires whenever pen is not genuinely fast,
    # preventing NLOS spikes from sneaking through when IMU has moderate velocity.
    uwb_jump_imu_speed_min: float = 0.15

    # Velocity covariance floor.
    # Lowered from 0.08 → 0.02 so the filter can converge velocity confidence after
    # consistent UWB anchoring.  0.08 kept P_vel ≥ 0.0064 m²/s² permanently, giving
    # the UWB velocity pseudo-update a ~0.9 Kalman gain even on noisy samples.
    vel_floor: float = 0.02

    # UWB velocity deviation gate: skip the velocity pseudo-update when the central-
    # difference UWB velocity disagrees with the current filter velocity by more than
    # this amount.  Prevents ~5 cm UWB noise over 50 ms (≈1 m/s) from injecting a
    # large spurious velocity that integrates into a 40+ cm teleport.
    uwb_vel_dev_max: float = 0.45

    # Stroke-start UWB soft snap.
    # On pen-down rising edge, apply a position pseudo-measurement toward the
    # last known UWB fix so each letter starts at the correct board location.
    # Lower sigma_scale = stronger pull toward UWB at pen-down.
    stroke_start_sigma_scale: float = 0.50    # multiplied onto sigma_uwb
    stroke_start_uwb_max_age_s: float = 0.10  # skip snap if UWB is older than this

    # Phase 4 — in-stroke position bias (pos_bias EMA tracker).
    # During CONTACT_DRAWING or DRAWING_FAST, each accepted UWB fix nudges a
    # parallel bias b_p by alpha*(z_uwb - (p + b_p)).  The visible output is
    # p + b_p, so the letter shape (relative IMU motion in p) is preserved while
    # the global placement slowly drifts toward UWB.
    # alpha = 0.01 → time-constant ~1/( 50 Hz * 0.01) = 2 s; absorbs ~63% of
    # a steady offset over a 2-second stroke.
    bias_uwb_alpha: float = 0.03    # Phase-4c: slightly faster bias absorption (~1.3 s time constant at 50 Hz)
    bias_decay:     float = 0.30     # multiplicative decay applied at stroke falling edge
    bias_max_m:     float = 0.10     # hard clip on |b_p| to prevent runaway (metres)

    # Stroke-age drift guard — adaptive pos_gain_cap ramp.
    # Multiplier is 1.0 for age < age_ramp_start_s, then linearly grows to
    # age_ramp_mult_max at age_ramp_end_s and holds there.  Applied on top of
    # the per-mode pos_gain_cap so handwriting (< 0.8 s) keeps full IMU shape
    # authority while long geometric strokes (> 2.5 s) get stronger UWB pull.
    age_ramp_start_s:  float = 1.20   # handwriting window (IMU shape authority)
    age_ramp_end_s:    float = 2.75   # beyond this → full drift-guard multiplier
    age_ramp_mult_max: float = 1.50   # cap multiplier at full drift-guard

    # Stroke-end reset.
    # Hard zero is good for letters because pen-up should break momentum.
    stroke_end_v_decay: float = 0.0
    stroke_end_p_vel_scale: float = 0.5

    # Clamp IMU dt spikes.
    imu_dt_max_mult: float = 3.0

    # Turn gate.
    # High jerk threshold prevents normal handwriting vibration from always firing TURN.
    turn_jerk_threshold: float = 1200.0
    turn_arm_n: int = 2

    # Covariance cap multiplier.
    pos_cap_mult: float = 6.0

    # DRAWING_FAST gate.
    # 4 frames at 180 Hz ≈ 22 ms.
    # This gives short IMU authority without letting drift dominate.
    drawing_fast_speed_thresh: float = 0.30     # tuner Stage-2 winner: fast mode should trigger only on clear speed bursts
    drawing_fast_min_frames: int = 8            # tuner Stage-2 winner: require sustained fast motion, avoids noisy over-triggering
    drawing_fast_burst_frames: int = 3          # hold fast authority briefly after trigger, then return to UWB anchoring

    # Per-mode parameter table.
    modes: FusionModeTable = field(default_factory=FusionModeTable)

    # Initial covariance.
    p0_pos: float = 0.35
    p0_vel: float = 0.25
    p0_bias: float = 0.05


# ------------------------------------------------------------------------
# ROOT CONFIG (wrapper)
# ------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    serial: SerialConfig = field(default_factory=SerialConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    contact: ContactConfig = field(default_factory=ContactConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    marker: MarkerConfig = field(default_factory=MarkerConfig)
    fusion_eskf: FusionESKFConfig = field(default_factory=FusionESKFConfig)


# declare the config file
cfg = Config()
