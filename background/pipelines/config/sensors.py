"""
Sensor and geometry configuration: serial link, IMU, contact detection, UWB
ranging, anchor placement, marker rendering.
"""

from dataclasses import dataclass

# -----------------------------------------------------------------------------
# Serial / hardware
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM20"
    baud: int = 921600


# -----------------------------------------------------------------------------
# IMU (Adafruit BNO085)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUConfig:
    # Measured median across test/_datasets on the current rig: 203-215 Hz
    # (kuru rate_profile.json reports 219.15 Hz over 109k samples).
    # ESKF dt clamp depends on this value, so keep it near measured reality.
    sample_rate_hz: float = 215.0

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

    # Quaternion safety.
    quat_norm_epsilon: float = 1e-6

    # Board projection.
    # The 2D board plane is X-Z in world coordinates.
    board_axes: tuple[str, str] = ("x", "z")


    # Light EMA for IMU Path-A. Lower = more responsive, higher = smoother.
    # ESKF now prefers HPF acceleration, but this still affects diagnostics
    # and fallback behavior.
    smooth_alpha_eskf: float = 0.25


    # High-pass filter used by ESKF for handwriting motion.
    # This removes slow bias/DC drift while preserving fast letter strokes.
    hpf_enabled: bool = True
    hpf_cutoff_hz: float = 0.75

    # Rigid-body tip correction.
    # Converts sensor-end acceleration to estimated tip acceleration.
    # Keep enabled, but verify lever_arm_m is near zero when marker is perpendicular.
    rigid_body_enabled: bool = True
    # Stage-5 winner: sign=-1 corrects the lever-arm direction for this marker geometry.
    # Confirmed on circle, triangle, hline, abc, w across two recording sessions.
    rigid_body_sign: int = -1

    # Pre-derivative low-pass filtering. Never derive alpha or jerk from raw
    # frame-to-frame sensor deltas; first smooth the gyro/quaternion-derived
    # omega and body acceleration, then take the derivative. The alpha values
    # below use new-sample weight: 0.20 means current = 20%, previous = 80%.
    pre_derivative_lpf_enabled: bool = True
    gyro_lpf_alpha: float = 0.20
    acc_lpf_alpha: float = 0.20

    # Dynamic deadbanding / noise floor for stationary marker behaviour.
    # When the smoothed angular velocity is below this floor, snap it to zero
    # so the rigid-body correction and ZUPT logic stay silent while held still.
    omega_deadband_rads: float = 0.02
    alpha_deadband_rads2: float = 0.10

    # EMA on angular acceleration used in rigid-body tip correction.
    # Higher = smoother but more lag; 0.7 is conservative.
    # This uses previous-sample weight for backward compatibility.
    alpha_ema_alpha: float = 0.7


# -----------------------------------------------------------------------------
# Contact state detector
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class ContactConfig:
    # Hysteresis: pen exits contact at force_enter * ratio.
    force_exit_ratio: float = 0.70

    # Debounce prevents force spikes/dips from fragmenting strokes.
    pen_down_debounce_ms: float = 10.0
    pen_up_debounce_ms: float = 8.0   # Phase-3: reduced from 30 ms - debounce window was wider than FSR noise requires

    # Minimum confirmed drawing time before opening a stroke.
    min_draw_ms: float = 25.0

    # Consecutive samples needed to confirm contact sub-state changes.
    state_debounce_n: int = 4


# -----------------------------------------------------------------------------
# UWB (Ai Thinker BU03)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class UWBConfig:
    # Per-anchor range calibration offsets.
    range_offsets_m: tuple = (-0.1040, 0.0273, -0.1902, -0.1353)

    # Nominal UWB rate.
    rate_hz: float = 50.0

    # Per-anchor range jump threshold.
    # Larger values allow fast movement but risk accepting spikes.
    max_range_jump_m: float = 0.40

    # Median window for per-anchor range filtering.
    median_window: int = 10

    # Per-anchor range Kalman pre-filter (preprocess/uwb/range_kf.py), feeding
    # the trilateration solver. A median or EMA wide enough to suppress UWB noise
    # also spans a stroke and averages the handwriting away; a constant-velocity
    # filter tracks range-rate, so it follows real motion instead.
    #
    # sigma_rate bounds range change - a stroke barely exceeds 0.3 m/s along one
    # anchor bearing. Raise if strokes look rounded off, lower if position stays
    # noisy.
    range_kf_enabled: bool = True
    range_kf_sigma_rate_ms: float = 0.30
    range_kf_sigma_meas_m: float = 0.08

    # Position-level speed outlier gate: a sanity bound, not a motion model.
    # Consecutive raw fixes imply a median 1.4-1.5 m/s here, which is 50 Hz
    # solver jitter rather than pen speed (handwriting is 0.1-0.5 m/s), so a
    # threshold near that median would censor the fastest real fixes.
    #
    # Flags, never drops: a dropped event cannot be weighed by the ESKF at all,
    # whereas speed_flag inflates R for that one update via
    # gates.quality_noise_multiplier - the same call made with covariance and
    # innovation in hand.
    outlier_speed_limit_ms: float = 2.0
    drop_speed_outliers: bool = False

    num_anchors: int = 4


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


# -----------------------------------------------------------------------------
# Anchor geometry (target whiteboard)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class AnchorConfig:
    board_size_x: float = 1.90
    board_size_y: float = 1.20

    # Anchors are mounted at board corners with slight depth offset.
    a0: tuple[float, float, float] = (0.00, 0.00, 0.01)
    a1: tuple[float, float, float] = (1.90, 0.00, 0.01)
    a2: tuple[float, float, float] = (1.90, 1.20, 0.01)
    a3: tuple[float, float, float] = (0.00, 1.20, 0.01)

    # Derived, not restated. The solver reads `positions` while the depth
    # probe reads `a0`, so a hand-edited corner used to take effect in one
    # place and be silently ignored in the other.
    @property
    def positions(self) -> tuple[tuple[float, float, float], ...]:
        return (self.a0, self.a1, self.a2, self.a3)


# -----------------------------------------------------------------------------
# Marker (physical layout)
#
# Body frame: tip at origin; sensors are mounted near marker end.
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class MarkerConfig:
    # Current prototype body-axis mapping, measured along the marker long axis
    # from the tip. Matches kuru_method/asynchronous_stream/config.py, which is
    # the calibrated source for this rig:
    #   IMU_S_FROM_TIP_M      = 0.125  (tip -> IMU)
    #   TIP_OFFSET_FROM_TAG_M = 0.215  (tip -> UWB tag, == MARKER_LENGTH)
    # For perpendicular wall testing, projected board-plane lever arm should be small.
    r_imu_body_m: tuple[float, float, float] = (0.125, 0.0, 0.0)
    r_uwb_body_m: tuple[float, float, float] = (0.215, 0.0, 0.0)
