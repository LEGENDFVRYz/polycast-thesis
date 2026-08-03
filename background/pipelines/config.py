"""
Centralized configuration for the PolyCast pipeline stages.

Every value here is intended solely for the modules under
background/pipelines/ - nothing outside that package should read this file.

Handwriting Base Config:
    
    Philosophy:
        IMU    short-term stroke detail / fast motion shape
        UWB    long-term absolute anchor / drift correction
        Force  stroke state / drawing mode selector
"""

from dataclasses import dataclass, field


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
    # range_offsets_m: tuple = (-0.1538, -0.0134, -0.1833, -0.0960)         # -- old validation
    range_offsets_m: tuple = (-0.1040, 0.0273, -0.1902, -0.1353)

    # Nominal UWB rate.
    rate_hz: float = 50.0

    # Legacy range EMA alpha, kept for old modules.
    ema_alpha: float = 0.25

    # Per-anchor range jump threshold.
    # Larger values allow fast movement but risk accepting spikes.
    max_range_jump_m: float = 0.40

    # Median window for per-anchor range filtering.
    median_window: int = 10

    # Per-anchor range Kalman pre-filter (preprocess/uwb/range_kf.py).
    #
    # This is what the trilateration solver consumes. A median or an EMA wide
    # enough to suppress UWB noise also spans a whole stroke and averages the
    # handwriting away; a constant-velocity filter tracks range-rate instead, so
    # it follows real motion and only pulls hard on samples that disagree with
    # it.
    #
    # sigma_rate bounds how fast a range is allowed to change: a writing stroke
    # barely exceeds 0.3 m/s along any one anchor bearing. Raise it if strokes
    # look rounded off, lower it if the solved position stays noisy.
    range_kf_enabled: bool = True
    range_kf_sigma_rate_ms: float = 0.30
    range_kf_sigma_meas_m: float = 0.08

    # Position-level speed outlier gate.
    #
    # The limit is a sanity bound, not a motion model: consecutive raw fixes
    # imply a median 1.4-1.5 m/s on this rig, which is solver jitter at 50 Hz
    # rather than pen speed (handwriting runs 0.1-0.5 m/s). A threshold near
    # that median cuts into the noise floor and censors the fastest fixes,
    # which are the ones carrying the most motion.
    #
    # Flag, do not drop. Dropping deletes the event before fusion, so the ESKF
    # cannot weigh a fix it never receives. Kept as a flag, speed_flag reaches
    # gates.quality_noise_multiplier and inflates R for that one update, which
    # is the same decision made with covariance and innovation in hand.
    outlier_speed_limit_ms: float = 2.0
    drop_speed_outliers: bool = False

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

    positions: tuple[tuple[float, float, float], ...] = (
        (0.00, 0.00, 0.01),
        (1.90, 0.00, 0.01),
        (1.90, 1.20, 0.01),
        (0.00, 1.20, 0.01),
    )


# -----------------------------------------------------------------------------
# Pipeline (constraints)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class PipelineConfig:
    # Buffer for timestamp sorting across sensors.
    time_align_buffer: int = 2000

    # IMU preprocessor resets if a very long hardware gap appears.
    imu_max_dt_ms: int = 100

    # UWB range sanity limits.
    uwb_min_range_m: float = 0.05
    uwb_max_range_m: float = 6.00


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


# -----------------------------------------------------------------------------
# Fusion - per-mode parameter table
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionModeParams:
    # UWB sigma scale on top of sigma_uwb.
    # Lower  = stronger UWB anchor.
    # Higher = more IMU freedom.
    sigma_scale: float = 0.55

    # Velocity drag in s^-1.
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
    # even if R collapses. Set to 1.0 to disable.
    pos_gain_cap: float = 1.0


@dataclass(frozen=True)
class FusionModeTable:
    # Normal pen-down drawing.
    # IMU owns letter shape; UWB gives a gentle global nudge only.
    drawing: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        # Tier B revised: 0.42 -> 0.85 produced visible tremor, so most of that
        # step is taken back. Our p90 speed already exceeds kuru's; we were never
        # short of high-frequency energy, and weakening the correction that damped
        # it let IMU noise integrate between fixes. 0.55 keeps a mild R inflation
        # during ink without freeing the noise.
        sigma_scale    = 0.55,
        # Tier A: was 1.67. At 215 Hz that is a 0.6 s velocity half-life against a
        # ~1.3 s stroke, so mid-stroke velocity was a fraction of what the IMU asked
        # for. 0.30 gives a 2.3 s half-life - longer than a stroke. kuru has no
        # equivalent drag term at all.
        drag_inv_s     = 0.30,
        dir_penalty    = 1.0,
        jump_speed_max = 1.8,
        pos_floor      = 0.012,
        # Tier A: was 0.50. Halving acceleration at the input discarded half the
        # stroke shape before it could integrate. kuru integrates unscaled.
        acc_scale      = 1.00,
        # Tier B revised: 0.13 -> 0.06 contributed to tremor by letting the state
        # run unchecked between fixes. 0.11 is close to the original.
        pos_gain_cap   = 0.11
    ))

    # Short high-speed burst mode.
    # IMU authority burst; UWB kept loosely so fast strokes don't explode.
    drawing_fast: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.62,   # Tier B revised: 0.52 -> 1.00 was too far; see drawing mode.
        drag_inv_s     = 0.30,   # Tier A: was 1.70. Matches drawing mode.
        dir_penalty    = 1.0,
        jump_speed_max = 2.2,
        pos_floor      = 0.020,
        acc_scale      = 1.00,   # Tier A: was 0.48.
        pos_gain_cap   = 0.075   # Tier B revised: 0.04 contributed to tremor.
    ))

    # Pen lifted / air movement.
    # UWB re-anchors aggressively; IMU suppressed to prevent drift carry-over.
    air: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.48,   # Phase-1: strong UWB pull between strokes to re-anchor placement
        drag_inv_s     = 4.0,    # keep from Stage-3 tuner - aggressively kill air dead-reckoning drift
        dir_penalty    = 3.0,
        jump_speed_max = 1.4,
        pos_floor      = 0.015,
        acc_scale      = 0.08,   # suppress IMU integration while pen is lifted
        pos_gain_cap   = 1.0,    # no cap in air - UWB should correct freely
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


# -----------------------------------------------------------------------------
# Stroke dead reckoner
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class StrokeDeadReckonerConfig:
    # Blend weight for HPF component inside CONTACT_DRAWING.
    # acc_for_pred = (acc_raw - b_a)*(1-detail_weight) + acc_hpf*detail_weight
    # Raised from 0.35 -> 0.55: more HPF reduces slow-bias accumulation during fast strokes.
    detail_weight: float = 0.55

    # IMU scale during AIR_MOVE - limits air drift without blackout.
    air_scale: float = 0.10

    # Snap dead-reckoner origin toward latest UWB tip on pen-down.
    pen_down_snap_to_uwb: bool = True

    # Max distance (m) for snap. Beyond this, origin stays at current p.
    pen_down_snap_max_dist_m: float = 0.08


# -----------------------------------------------------------------------------
# Fusion (ESKF)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionESKFConfig:
    # -------------------------------------------------------------------------
    # DIAGNOSTIC: pure-IMU mode
    # -------------------------------------------------------------------------
    # Set True to cut every path by which UWB reaches the output, leaving the
    # visible tip driven only by double-integrated IMU acceleration. This is a
    # measurement instrument, not a shipping configuration.
    #
    # What it answers: does the IMU carry letter shape at all? Partial tuning
    # cannot separate "IMU has no shape to contribute" from "IMU shape is being
    # overridden downstream", because both look identical in the render.
    #
    # How to read the output: judge SHAPE WITHIN each stroke and ignore where
    # strokes sit relative to each other. Unaided inertial position drifts
    # quadratically with time, so absolute placement WILL be wrong and later
    # strokes will be further off than earlier ones. That is expected physics,
    # not a defect.
    #
    #   letterforms present but drifting -> IMU carries the shape; the problem is
    #                                       how UWB is coupled, and tuning is the
    #                                       right lever
    #   fragments as before              -> the shape is not in the IMU signal;
    #                                       no rebalancing will produce it, and
    #                                       the answer is upstream (preprocessing,
    #                                       calibration) or structural
    #
    # UWB still runs upstream - ranges are solved and events flow - so only the
    # fusion coupling is severed, nothing is starved of data.
    imu_only_mode: bool = False

    # -------------------------------------------------------------------------
    # Shape mode - IMU owns stroke shape, UWB owns stroke placement
    # -------------------------------------------------------------------------
    # The working configuration derived from the pure-IMU diagnostic above. That
    # test showed the IMU produces clearly recognizable letterforms, so the
    # pipeline's problem was never a missing signal - UWB was overriding a good
    # one.
    #
    # This splits the sensors by what each is actually good at, instead of
    # blending them everywhere:
    #
    #   during a stroke   IMU owns position outright. No Kalman correction, no
    #                     bias tracking, no boundary guard. Drift is real but
    #                     bounded by stroke duration, and letters are small
    #                     enough that it stays below the noise it replaces.
    #
    #   at pen-down       one deliberate UWB re-anchor places the stroke. This is
    #                     the only moment a large correction is both correct and
    #                     invisible, because no ink is committed yet.
    #
    #   in air            UWB converges the estimate but does not drag the
    #                     visible tip. Measured on abcde_1, the blended config let
    #                     UWB move the tip a median 19.7 cm between strokes -
    #                     larger than a 14 cm letter - and each stroke then began
    #                     wherever that drag landed.
    #
    # Set False to restore the previous blended behaviour exactly. Ignored when
    # imu_only_mode is True, which is a diagnostic and takes precedence.
    shape_mode: bool = True

    # Pen-down re-anchor distance cap. The existing snap uses
    # dead_reckoner.pen_down_snap_max_dist_m = 0.08, well under the measured
    # 19.7 cm inter-stroke error, so it was skipped exactly when most needed. In
    # shape mode the re-anchor must cover the full error; this bound exists only
    # to reject a wild fix.
    shape_mode_reanchor_max_m: float = 0.35

    # Fraction of the pen-down placement error corrected at stroke start. 1.0
    # places the stroke exactly on UWB; slightly under keeps one noisy fix from
    # throwing the letter, since UWB carries ~2.4 cm of noise on this rig.
    shape_mode_reanchor_alpha: float = 0.85

    # Integrate ink the way the visualizer's blue reference track does.
    #
    # The blue "IMU dead-reck." layer is _IMUTrack in visualizer.py, a plain
    # forward-Euler integrator. Users repeatedly observed it producing better
    # letterforms than the fused trace from the same recording - most visibly the
    # descender of 'g' - and with shape_mode on, UWB is provably not the cause:
    # every UWB constraint measures 0.00 cm of tip movement during ink.
    #
    # The difference is what each integrates:
    #
    #   blue    acc_board_tip, as given
    #   ESKF    (acc_board_tip - bias) * 0.45 + acc_board_hp_tip * 0.55,
    #           then drag (drag_inv_s) and a velocity cap
    #
    # The high-pass path is the problem. At a 0.75 Hz cutoff its time constant is
    # ~1.3 s, about the length of one stroke, so it removes stroke-scale content -
    # which is why fused strokes come out truncated and doubled back while blue
    # keeps the full extent.
    #
    # With this enabled, during ink only: acceleration is acc_board_tip with no
    # HPF blend and no bias subtraction, and neither drag nor the velocity cap is
    # applied. Air movement, placement and every UWB path are unchanged, so
    # pen-down re-anchoring still puts strokes within ~4 cm - UWB's noise level.
    #
    # Requires shape_mode. Set False to restore the blended acceleration.
    ink_from_dead_reckoner: bool = True

    # Process noise for acceleration.
    # Higher = filter admits IMU prediction uncertainty and lets UWB correct.
    # Too high makes UWB dominate; too low makes IMU drift dominate.
    sigma_a: float = 2.8
    # A/B diagnostic: clamp in-stroke acceleration magnitude to prevent impulse excursions.
    # Disabled by default; enable to test whether spikes are causing loop distortion.
    acc_spike_clamp_enabled: bool = True
    # Tier A: was 1.8. Real handwriting acceleration exceeds 1.8 m/s^2 on corners,
    # so the clamp was cutting letterform rather than impulse spikes. 4.0 keeps the
    # runaway guard while leaving normal writing untouched.
    acc_spike_clamp_ms2:     float = 4.0

    # Acceleration-bias random walk.
    # Keep very small so bias does not absorb UWB/IMU disagreement too quickly.
    sigma_b_a: float = 0.0001

    # Hard bound on the magnitude of the estimated accelerometer bias vector.
    #
    # sigma_b_a already says the bias is nearly constant, but nothing enforced a
    # magnitude, so a run of large innovations could walk it arbitrarily far. On
    # abc_extralarge it reached 0.71 m/s^2 - which integrates to 0.36 m/s of
    # position error per second, drove the state off the board, and then failed
    # every UWB fix on the board-margin gate, leaving no way back.
    #
    # A BNO085 does not legitimately exhibit a bias this large. 0.15 leaves room
    # for a real sensor offset while making that runaway impossible. Set 0 to
    # disable the bound.
    accel_bias_max_ms2: float = 0.15

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
    innov_hard_reject_m: float = 0.50  # UWB innovation magnitude hard reject (m); >25 cm UWB<->IMU disagreement is non-physical in board writing
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
    uwb_vel_stroke_gate: bool = True

    # If accepted UWB updates are stale, inflate Q and increase drag.
    uwb_window_s: float = 0.30
    uwb_stale_k: float = 2.0
    uwb_stale_max_k: float = 5.0

    # Direction check thresholds.
    dir_check_v_min: float = 0.03
    dir_check_y_min: float = 0.02
    dir_check_cos_thresh: float = -0.3

    # UWB jump gate: if UWB implies fast motion but IMU speed is low, reject.
    # Lowered from 0.5 -> 0.15 so the gate fires whenever pen is not genuinely fast,
    # preventing NLOS spikes from sneaking through when IMU has moderate velocity.
    uwb_jump_imu_speed_min: float = 0.15

    # Velocity covariance floor.
    # Lowered from 0.08 -> 0.02 so the filter can converge velocity confidence after
    # consistent UWB anchoring. 0.08 kept P_vel >= 0.0064 m^2/s^2 permanently, giving
    # the UWB velocity pseudo-update a ~0.9 Kalman gain even on noisy samples.
    vel_floor: float = 0.02

    # UWB velocity deviation gate: skip the velocity pseudo-update when the central-
    # difference UWB velocity disagrees with the current filter velocity by more than
    # this amount. Prevents ~5 cm UWB noise over 50 ms (~1 m/s) from injecting a
    # large spurious velocity that integrates into a 40+ cm teleport.
    uwb_vel_dev_max: float = 0.45

    # Stroke-start UWB soft snap.
    # On pen-down rising edge, apply a position pseudo-measurement toward the
    # last known UWB fix so each letter starts at the correct board location.
    # Lower sigma_scale = stronger pull toward UWB at pen-down.
    stroke_start_sigma_scale: float = 0.90    # multiplied onto sigma_uwb
    stroke_start_uwb_max_age_s: float = 0.10  # skip snap if UWB is older than this

    # Phase 4 - in-stroke position bias (pos_bias EMA tracker).
    # During CONTACT_DRAWING or DRAWING_FAST, each accepted UWB fix nudges a
    # parallel bias b_p by alpha*(z_uwb - (p + b_p)). The visible output is
    # p + b_p, so the letter shape (relative IMU motion in p) is preserved while
    # the global placement slowly drifts toward UWB.
    # alpha = 0.01 -> time-constant ~1/( 50 Hz * 0.01) = 2 s; absorbs ~63% of
    # a steady offset over a 2-second stroke.
    # Tier B: was 0.055. At 50 Hz that is a ~0.36 s time constant, so the bias
    # tracker was acting as a second position correction *within* a stroke rather
    # than a slow placement fix across strokes. 0.015 gives ~1.3 s - longer than a
    # stroke, so placement still converges but letter shape is left alone.
    bias_uwb_alpha: float = 0.015
    bias_decay:     float = 0.25
    bias_max_m:     float = 0.045

    # Stroke-age drift guard - adaptive pos_gain_cap ramp.
    # Stage-9 result: ramp fires on abc handwriting at all tested start thresholds
    # because abc stroke durations overlap geometric shape durations in this dataset.
    # Neutralised by setting mult_max=1.0 (multiplier stays flat = ramp disabled).
    # Fields preserved so the ramp can be re-enabled per-dataset if needed.
    age_ramp_start_s:  float = 1.20   # (inactive while mult_max=1.0)
    age_ramp_end_s:    float = 2.75   # (inactive while mult_max=1.0)
    age_ramp_mult_max: float = 1.00    # 1.0 = disabled; Stage-9 winner

    # Active-stroke UWB boundary guard.
    # This is a safety net, not a normal correction path: if live IMU integration
    # expands far away from the last accepted tip-corrected UWB point, gently pull
    # the visible output back inside a local radius. It prevents large abc/cat loops
    # while still allowing IMU shape within the UWB neighbourhood.
    # Tier A: was 0.36. Measured pen speed on the kuru reference over abcde_1 is
    # median 0.292 m/s with p90 0.550, so the old cap sat below the 90th percentile
    # of real motion and clipped every fast stroke segment. 1.00 still catches a
    # genuine runaway.
    active_vel_cap_ms: float = 1.00
    active_uwb_guard_enabled: bool = True
    # Tier B revised. The 0.060 attempt caused visible blooming and was based on a
    # bad inference: the radius was sized against a whole 14 cm letter, but the
    # guard measures excursion from the *latest* UWB fix, which updates at 50 Hz
    # and tracks along the stroke. It never needed to be letter-sized - only large
    # enough that normal in-stroke IMU detail is not clipped every frame.
    # 0.030 gives that headroom over the original 0.022 while keeping the guard a
    # real safety net.
    active_uwb_guard_radius_m: float = 0.030
    active_uwb_guard_alpha: float = 0.78
    active_uwb_guard_max_age_s: float = 0.85

    # Fraction of the outward velocity component removed when visible fused ink
    # is already drifting away from the latest tip-corrected UWB neighbourhood.
    # 0.0 = disabled; 1.0 = remove all outward velocity; tangential velocity remains.
    # Tier B revised. 0.30 was too weak once Tier A removed the input-side damping:
    # with little restoring force, strokes bloomed past where they should stop.
    # 0.65 still relaxes the original 0.85 - which was absorbing the velocity Tier A
    # restored - but keeps a real brake on outward excursion.
    active_uwb_outward_velocity_damping: float = 0.65

    # Mode-aware stationary-contact clamp.
    # Goal: if the marker tip is physically on the board but not truly moving,
    # the visible tip should stay put instead of integrating IMU noise. This
    # directly targets start/end hold artefacts and contact micro-pauses.
    contact_static_lock_enabled: bool = True

    # CONTACT_STATIC from contact.py is trusted immediately. The thresholds
    # below are a fallback for older logs or borderline frames where force/contact
    # and IMU stillness are present but the diagnostic substate has not switched.
    contact_static_lock_min_frames: int = 2
    contact_static_lock_speed_thresh_ms: float = 0.035
    contact_static_lock_acc_thresh_ms2: float = 0.65
    contact_static_lock_omega_thresh_rads: float = 0.60

    # UWB anchoring while the tip is locked. Pen-down uses stronger UWB anchoring
    # because no ink has been committed yet; mid-stroke pauses use a much smaller
    # blend to avoid snapping corners/letter pauses away from their drawn shape.
    contact_static_lock_uwb_max_age_s: float = 0.18
    contact_static_lock_pen_down_uwb_blend: float = 0.85
    contact_static_lock_micro_pause_uwb_blend: float = 0.02

    # How hard to hold the visible tip at the lock anchor. Position alpha is
    # applied to p so b_p remains the normal global-placement bias.
    contact_static_lock_pos_alpha: float = 0.92
    contact_static_lock_vel_decay: float = 0.08
    contact_static_lock_vel_zero_thresh_ms: float = 0.015
    contact_static_lock_cov_vel_scale: float = 0.20
    contact_static_lock_cov_pos_scale: float = 0.85

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
    # 4 frames at 180 Hz ~= 22 ms.
    # This gives short IMU authority without letting drift dominate.
    drawing_fast_speed_thresh: float = 0.36     # tuner Stage-2 winner: fast mode should trigger only on clear speed bursts
    drawing_fast_min_frames: int = 10            # tuner Stage-2 winner: require sustained fast motion, avoids noisy over-triggering
    drawing_fast_burst_frames: int = 1          # hold fast authority briefly after trigger, then return to UWB anchoring

    # Per-mode parameter table.
    modes: FusionModeTable = field(default_factory=FusionModeTable)

    # Stroke-local dead reckoner config.
    dead_reckoner: StrokeDeadReckonerConfig = field(default_factory=StrokeDeadReckonerConfig)

    # Initial covariance.
    p0_pos: float = 0.35
    p0_vel: float = 0.25
    p0_bias: float = 0.05


# -----------------------------------------------------------------------------
# Stroke finalization IMU cleaner
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class StrokeCleanerConfig:
    # Batch/offline cleanup applied only after pen-up. Live ESKF output is still
    # emitted immediately, then the finished stroke is corrected before delivery.
    enabled: bool = False

    # Minimum useful stroke size. Shorter strokes are left untouched because
    # double-integrating a tiny segment is usually less reliable than the fused path.
    min_points: int = 6

    # Reference-style acceleration drift removal threshold, matching the role of
    # `threshold` in john2zy/IMU-Position-Tracking.removeAccErr(). Units: m/s^2.
    acc_motion_threshold: float = 0.20

    # Reference-style stationary threshold for ZUPT velocity correction, matching
    # john2zy/IMU-Position-Tracking.zupt(..., threshold=0.2). Units: m/s^2.
    zupt_acc_threshold: float = 0.20

    # Pen-up is treated as the end still phase for whiteboard strokes. This applies
    # the same backward velocity-drift distribution used by the reference ZUPT when
    # a still phase is reached, but forces it at stroke close when no still samples
    # were recorded inside the active ink segment.
    force_zero_velocity_at_end: bool = True

    # Keep placement anchored to the fused/UWB-supported stroke endpoints. 1.0 means
    # force the IMU-cleaned relative trajectory to end at the original fused endpoint.
    endpoint_anchor_blend: float = 1.0

    # Conservative blend between current fused ink and cleaned IMU-relative shape.
    # 0.0 = keep current pipeline output, 1.0 = full reference-style IMU cleanup.
    shape_blend: float = 0.25

    # Guard against a bad re-integration exploding a stroke. If the cleaned bbox is
    # outside this ratio versus the raw fused bbox, keep the raw fused stroke.
    max_bbox_ratio: float = 1.10


# -----------------------------------------------------------------------------
# Post-process (pen-up corrections: centroid alignment + minimum-jerk)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class CentroidAlignConfig:
    # Minimum UWB samples buffered during the stroke to trust the centroid estimate.
    min_uwb_points: int = 3

    # Robust UWB centroid: remove the farthest points before computing the final
    # UWB centre so a small NLOS arc does not drag the whole stroke.
    trim_quantile: float = 0.85

    # Reject the correction if IMU and UWB centroids diverge by more than this (metres).
    # Prevents a bad UWB cluster from teleporting an otherwise good stroke.
    max_translation_m: float = 0.10

    # Optional uniform scale correction. Kept tightly clamped so letter shapes are preserved.
    scale_enabled: bool = True
    scale_percentile: float = 0.80
    scale_min: float = 0.70
    scale_max: float = 1.10

    # Similarity alignment: after centroid translation, estimate one global
    # 2D transform that can rotate and scale the finished stroke as a rigid
    # object. Procrustes uses resampled stroke<->UWB correspondences; PCA is kept
    # as a fallback for older tests.
    rotation_enabled: bool = True
    rotation_max_deg: float = 20.0
    procrustes_enabled: bool = True
    procrustes_samples: int = 48
    # Trim a small fraction only for estimating the transform. The full stroke
    # is still transformed and rendered. This reduces pen-down/pen-up hooks from
    # dominating the rotation estimate.
    procrustes_endpoint_trim: float = 0.04
    # Minimum eigenvalue ratio (lambda_max / lambda_min) required to trust PCA fallback.
    # A ratio < 2 means the distribution is too round to have a reliable direction.
    pca_min_eigenratio: float = 2.0


@dataclass(frozen=True)
class MinJerkConfig:
    # Skip smoothing for very short strokes (too few points to detect waypoints).
    min_points_for_minjerk: int = 8
    # Curvature threshold (1/m) above which a sample is a waypoint candidate.
    # Lower = more waypoints (less smoothing); higher = fewer waypoints (more smoothing).
    curvature_threshold: float = 30.0
    # Minimum physical distance (metres) between consecutive accepted waypoints.
    min_waypoint_spacing_m: float = 0.005
    # Hard cap on waypoint count; top-N by curvature are kept when exceeded.
    max_waypoints: int = 32
    # Blend factor: 0.0 = keep raw aligned points, 1.0 = full min-jerk replacement.
    shape_blend: float = 0.7
    # Reject smoothed result if its bbox grows beyond this ratio vs the aligned bbox.
    max_bbox_ratio: float = 1.10


# -----------------------------------------------------------------------------
# Two-point stroke anchoring (pen-up drift removal)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class TwoPointAnchorConfig:
    # Pins both endpoints of a finished stroke to UWB, removing the drift that
    # ink_from_dead_reckoner reintroduced by dropping the HPF, bias subtraction,
    # drag and velocity cap during ink.
    #
    # A constant acceleration bias displaces position by 0.5*b*t^2, so drift is
    # negligible at pen-down and largest at pen-up - which is exactly where the
    # overshoot appears. Subtracting error*(t/T)^2 removes that parabola and
    # leaves genuine motion, so the letter keeps its shape.
    #
    # Runs at pen-up on a finished stroke, so unlike every mid-stroke UWB
    # correction tried before it, it cannot fight the IMU while the letter is
    # being drawn.
    enabled: bool = True

    # Shorter strokes have not accumulated enough drift for the correction to
    # beat the UWB noise it would introduce.
    min_points: int = 8

    # Half-width of the window around the stroke endpoint whose UWB fixes are
    # medianed into the anchor. One fix carries ~2.4 cm of noise on this rig -
    # the same order as the drift - so a single sample is not a usable target.
    # 150 ms spans roughly 7 fixes at 50 Hz.
    anchor_window_us: int = 150_000

    # Beyond this the endpoint disagreement is not drift, and applying it would
    # move the letter rather than straighten it.
    #
    # Measured per-stroke endpoint error on abcde_1 runs 1.5-18.6 cm, and it
    # tracks stroke duration and span the way t^2 drift predicts: 0.29 s strokes
    # show 1.5-5 cm while 1.1-1.4 s strokes show 14-19 cm. An earlier 0.12 value
    # rejected 4 of 10 strokes - precisely the long ones that had drifted most
    # and needed the correction. 0.25 covers the observed range with headroom
    # while still refusing a fix that disagrees by more than a letter width.
    max_correction_m: float = 0.25

    # Fraction of the measured endpoint error removed. Below 1.0 because the
    # anchor itself is noisy: correcting fully would inject UWB noise into the
    # stroke tail, which is the thing being fixed.
    correction_alpha: float = 0.85


# -----------------------------------------------------------------------------
# Per-stroke IMU reliability gate (UWB shape fallback)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUDegeneracyConfig:
    # For most strokes the IMU carries shape and UWB only places it. For some it
    # does not: the three strokes forming the 'h' in abcde_1 integrate to a
    # nearly one-dimensional path while UWB observes real 2D motion over the same
    # interval. Undamped double integration of the raw acceleration, with no
    # filter in the path, is equally flat - so the weak axis is absent from the
    # inertial signal rather than removed by our processing.
    #
    # Such a stroke is rebuilt from UWB. The result is noisier than a good IMU
    # stroke, but a noisy 'h' is legible where a flat line is not.
    enabled: bool = True

    # Anisotropy is minor/major principal axis: 0 is a line, 1 is round.
    # Measured across abcde_1 and abc_extralarge:
    #   good strokes   0.227 - 0.739
    #   'h' strokes    0.020 - 0.141
    # An order of magnitude apart with nothing between, so 0.18 separates them
    # without being near either group.
    max_anisotropy: float = 0.18

    # Second condition on the same stroke: the minor axis in absolute terms.
    # Good strokes measured 2.4-5.5 cm, degenerate ones 0.10-0.25 cm. This stops
    # a large stroke that happens to be elongated from qualifying.
    max_minor_axis_m: float = 0.012

    # UWB must show meaningfully more structure than the IMU did, or there is
    # nothing to recover - a genuinely straight stroke (a 'T' stem, an underline)
    # is legitimately anisotropic and must not be replaced by UWB noise.
    uwb_structure_ratio: float = 1.5

    # And that structure must be real rather than jitter. UWB carries ~2.4 cm of
    # noise on this rig, so its minor axis has to clear a floor before it counts
    # as observed motion.
    min_uwb_minor_axis_m: float = 0.004

    # Enough samples for the PCA to mean anything.
    min_points: int = 12
    min_uwb_points: int = 6

    # Centred moving average over the rebuilt stroke. UWB noise would otherwise
    # be drawn directly as ink.
    smoothing_window: int = 5


@dataclass(frozen=True)
class PostprocessConfig:
    enabled: bool = False
    # Strokes shorter than this are passed through unchanged.
    min_stroke_points: int = 5
    centroid: CentroidAlignConfig = field(default_factory=CentroidAlignConfig)
    minjerk: MinJerkConfig = field(default_factory=MinJerkConfig)



# -----------------------------------------------------------------------------
# Causal trace filter (display-side smoothing of the fused ink)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class TraceFilterConfig:
    """
    Adaptive-alpha causal EMA applied to the fused position for display.

    This is downstream of the ESKF with no feedback, so it cannot destabilize
    the filter - it changes what is drawn, not what is estimated.

    The problem it solves: 55-60% of consecutive in-stroke fused samples land
    within 0.5 mm of each other while the p90 step is 3.5-5.9 mm. The trace is
    mostly sub-noise dither punctuated by real motion, and the dither shows up
    as direction reversal - a single `circle_medium-` stroke accumulates over
    50 full turns of absolute heading change. A fixed EMA strong enough to
    remove that also rounds off letter corners; selecting alpha per sample from
    the step length, then raising it again when the direction turns sharply,
    removes the dither and keeps the corners.

    Measured on test/_datasets by replaying the fused output through this
    filter: accumulated turning falls 14% (abcde_1), 35% (abc_extralarge),
    69% (hello_world_1) and 90% (circle_medium-), while the stroke bounding-box
    diagonal moves by at most 1%. Extent is preserved, so what is removed is
    not carrying letter shape.

    Ported from kuru_method/asynchronous_stream/main_ekf.py RecognitionTraceFilter.
    Keep the numerics aligned with it - the measurements above assume them.
    """

    enabled: bool = True

    # 'light' follows intentional motion closely and only suppresses dither;
    # 'normal' is markedly heavier. Light is what the measurements above used.
    mode: str = "light"

    # Emitted points closer together than this are dropped. Removes repeated
    # near-identical samples without touching the drawn shape.
    min_step_m: float = 0.0015

    # Step-length band edges that select alpha. Below step_small_m a sample is
    # almost certainly quantisation noise; above step_large_m it is deliberate
    # pen motion and should be followed nearly unfiltered.
    step_small_m: float = 0.003
    step_medium_m: float = 0.010
    step_large_m: float = 0.025

    # New-sample weight per band, ordered (small, medium, large, larger).
    # Higher = follows the raw sample more closely.
    alpha_light: tuple[float, float, float, float] = (0.50, 0.65, 0.78, 0.90)
    alpha_normal: tuple[float, float, float, float] = (0.18, 0.32, 0.52, 0.72)

    # Corner preservation. When the stroke direction turns sharply, alpha is
    # raised to at least the value below so the filtered line does not cut the
    # corner. Steps shorter than corner_min_step_m have too noisy a direction
    # to test. cos is between the previous and current step vectors:
    #   cos < 0.35  -> roughly a >70 degree turn
    #   cos < 0.0   -> a reversal / cusp
    #
    # The cusp arm is the one that does real work. A >70 degree turn taken at a
    # 4-10 mm step already gets a base alpha of 0.65, so corner_alpha_turn is
    # nearly a no-op; a reversal jumps from 0.65 to 0.80, which is what keeps
    # the apex of a 'v' or the bottom of a stem from being cut. trace_filter.py
    # tests the cusp threshold before the turn threshold for that reason - see
    # its docstring for why the reference implementation's ordering disables it.
    corner_min_step_m: float = 0.004
    corner_cos_turn: float = 0.35
    corner_alpha_turn: float = 0.68
    corner_cos_cusp: float = 0.0
    corner_alpha_cusp: float = 0.80


# -----------------------------------------------------------------------------
# Root config (wrapper)
# -----------------------------------------------------------------------------
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
    stroke_cleaner: StrokeCleanerConfig = field(default_factory=StrokeCleanerConfig)
    postprocess: PostprocessConfig = field(default_factory=PostprocessConfig)
    two_point_anchor: TwoPointAnchorConfig = field(default_factory=TwoPointAnchorConfig)
    imu_degeneracy: IMUDegeneracyConfig = field(default_factory=IMUDegeneracyConfig)
    trace_filter: TraceFilterConfig = field(default_factory=TraceFilterConfig)


# Module-level singleton every pipeline stage imports.
cfg = Config()
