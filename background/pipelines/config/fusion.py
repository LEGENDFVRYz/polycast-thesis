"""
Fusion configuration: pipeline constraints, per-mode gain tables, the stroke
dead reckoner, and the error-state Kalman filter.

`FusionESKFConfig` carries the flags that select a pipeline mode
(`shape_mode`, `ink_from_dead_reckoner`, `imu_only_mode`); see modes.py.
"""

from dataclasses import dataclass, field

from background.pipelines.config._meta import inert

# Why a field is unreachable under the shipped default. Named rather than
# repeated inline: the same reason applies to many fields, and a constant keeps
# them from drifting apart. See tools/verify_modes.py --reachability.
_BYPASSED_BY_SHAPE_MODE = (
    'shape_mode: the ink path returns at eskf.py:306 and drag/cap are skipped'
    ' at eskf.py:363, so this is bypassed while drawing'
)
_STATIC_LOCK_NEVER_TRIPS = (
    'contact_static_lock gate never trips on handwriting: the speed/accel/omega'
    ' thresholds are not met during a stroke'
)
_TURN_GATE_NEVER_ARMS = (
    'turn gate never arms: turn_jerk_threshold=1200 vs 481.8 max jerk measured'
    ' on abcde_1'
)
_INNOV_STREAK_NEVER_REACHED = (
    'innovation reject streak never reaches this length on measured data'
)


# -----------------------------------------------------------------------------
# Pipeline (constraints)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class PipelineConfig:

    # IMU preprocessor resets if a very long hardware gap appears.
    imu_max_dt_ms: int = 100

    # UWB range sanity limits.
    uwb_min_range_m: float = 0.05
    uwb_max_range_m: float = 6.00
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
        # 0.85 produced visible tremor - weakening the correction let IMU noise
        # integrate between fixes. 0.55 keeps a mild R inflation during ink.
        sigma_scale    = 0.55,
        # 1.67 was a 0.6 s velocity half-life against a ~1.3 s stroke, so mid-stroke
        # velocity fell well short of the IMU's. 0.30 gives 2.3 s - longer than a
        # stroke. kuru has no drag term at all.
        drag_inv_s     = 0.30,
        dir_penalty    = 1.0,
        jump_speed_max = 1.8,
        pos_floor      = 0.012,
        # 0.50 discarded half the stroke shape before it could integrate; kuru
        # integrates unscaled.
        acc_scale      = 1.00,
        # 0.06 let the state run unchecked between fixes and added tremor.
        pos_gain_cap   = 0.11
    ))

    # Short high-speed burst mode.
    # IMU authority burst; UWB kept loosely so fast strokes don't explode.
    drawing_fast: FusionModeParams = field(default_factory=lambda: FusionModeParams(
        sigma_scale    = 0.62,   # 1.00 was too far; see drawing mode.
        drag_inv_s     = 0.30,   # was 1.70; matches drawing mode.
        dir_penalty    = 1.0,
        jump_speed_max = 2.2,
        pos_floor      = 0.020,
        acc_scale      = 1.00,   # was 0.48.
        pos_gain_cap   = 0.075   # 0.04 contributed to tremor.
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
    # DIAGNOSTIC: pure-IMU mode. Cuts every path by which UWB reaches the
    # output, to answer whether the IMU carries letter shape at all.
    #
    # Read it by judging shape WITHIN a stroke, not placement between strokes:
    # unaided inertial position drifts quadratically, so absolute placement is
    # expected to be wrong. Letterforms present -> the IMU has the shape and
    # the problem is UWB coupling. Fragments -> the shape is not in the signal,
    # and the answer is upstream or structural.
    imu_only_mode: bool = False

    # Shape mode - IMU owns stroke shape, UWB owns stroke placement. Derived
    # from the diagnostic above, which showed the IMU produces recognizable
    # letterforms: the signal was never missing, UWB was overriding a good one.
    #
    #   during a stroke   IMU owns position outright - no Kalman correction,
    #                     bias tracking or boundary guard. Drift is bounded by
    #                     stroke duration and stays under the noise it replaces.
    #   at pen-down       one UWB re-anchor places the stroke, the only moment a
    #                     large correction is both correct and invisible.
    #   in air            UWB converges the estimate but does not drag the tip.
    #                     Blended, it moved the tip a median 19.7 cm between
    #                     strokes on abcde_1 - wider than a 14 cm letter.
    #
    # False restores blended behaviour. Ignored when imu_only_mode is True.
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
    # Clamp in-stroke acceleration magnitude against impulse excursions.
    acc_spike_clamp_enabled: bool = True
    # 1.8 cut letterform, not spikes - handwriting exceeds it on corners. 4.0
    # keeps the runaway guard while leaving normal writing untouched.
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
    innov_recovery_n: int = inert(_INNOV_STREAK_NEVER_REACHED, default=6)

    # Turn detection.
    # For handwriting, avoid making TURN fire too often.
    turn_omega_threshold: float = 0.45
    turn_k_q: float = inert(_TURN_GATE_NEVER_ARMS, default=6.0)
    turn_n_post: int = inert(_TURN_GATE_NEVER_ARMS, default=5)

    # IMU state buffer for UWB timestamp interpolation.
    state_buffer_size: int = 75

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
    # multiplied onto sigma_uwb
    stroke_start_sigma_scale: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.90)
    stroke_start_uwb_max_age_s: float = 0.10  # skip snap if UWB is older than this

    # Phase 4 - in-stroke position bias (pos_bias EMA tracker).
    # During CONTACT_DRAWING or DRAWING_FAST, each accepted UWB fix nudges a
    # parallel bias b_p by alpha*(z_uwb - (p + b_p)). The visible output is
    # p + b_p, so the letter shape (relative IMU motion in p) is preserved while
    # the global placement slowly drifts toward UWB.
    # alpha = 0.01 -> time-constant ~1/( 50 Hz * 0.01) = 2 s; absorbs ~63% of
    # a steady offset over a 2-second stroke.
    # At 50 Hz that is a ~0.36 s time constant, so the bias
    # tracker was acting as a second position correction *within* a stroke rather
    # than a slow placement fix across strokes. 0.015 gives ~1.3 s - longer than a
    # stroke, so placement still converges but letter shape is left alone.
    # Held at 0.055 (a 0.015 trial was reverted).
    bias_uwb_alpha: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.055)
    bias_decay:     float = 0.25
    bias_max_m:     float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.045)

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
    # Held at 0.36 (a 1.00 trial was reverted). Note measured pen speed on
    # abcde_1 is median 0.292 m/s, p90 0.550, so this sits below the 90th
    # percentile of real motion - it would clip fast segments if it were live.
    active_vel_cap_ms: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.36)
    active_uwb_guard_enabled: bool = True
    # Held at 0.022 (0.030 and 0.060 trials were reverted). The guard measures
    # excursion from the latest 50 Hz UWB fix, which tracks along the stroke, so
    # it never needed to be letter-sized - 0.060 bloomed on that bad inference.
    active_uwb_guard_radius_m: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.022)
    active_uwb_guard_alpha: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.78)
    active_uwb_guard_max_age_s: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.85)

    # Fraction of the outward velocity component removed when visible fused ink
    # is already drifting away from the latest tip-corrected UWB neighbourhood.
    # 0.0 = disabled; 1.0 = remove all outward velocity; tangential velocity remains.
    # Held at 0.85 (a 0.65 relaxation was reverted). 0.30 was far too weak -
    # strokes bloomed past where they should stop.
    active_uwb_outward_velocity_damping: float = inert(_BYPASSED_BY_SHAPE_MODE, default=0.85)

    # Mode-aware stationary-contact clamp.
    # Goal: if the marker tip is physically on the board but not truly moving,
    # the visible tip should stay put instead of integrating IMU noise. This
    # directly targets start/end hold artefacts and contact micro-pauses.
    contact_static_lock_enabled: bool = True

    # CONTACT_STATIC from contact.py is trusted immediately. The thresholds
    # below are a fallback for older logs or borderline frames where force/contact
    # and IMU stillness are present but the diagnostic substate has not switched.
    contact_static_lock_min_frames: int = inert(_STATIC_LOCK_NEVER_TRIPS, default=2)
    contact_static_lock_speed_thresh_ms: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.035)
    contact_static_lock_acc_thresh_ms2: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.65)
    contact_static_lock_omega_thresh_rads: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.60)

    # UWB anchoring while the tip is locked. Pen-down uses stronger UWB anchoring
    # because no ink has been committed yet; mid-stroke pauses use a much smaller
    # blend to avoid snapping corners/letter pauses away from their drawn shape.
    contact_static_lock_uwb_max_age_s: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.18)
    contact_static_lock_pen_down_uwb_blend: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.85)
    contact_static_lock_micro_pause_uwb_blend: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.02)

    # How hard to hold the visible tip at the lock anchor. Position alpha is
    # applied to p so b_p remains the normal global-placement bias.
    contact_static_lock_pos_alpha: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.92)
    contact_static_lock_vel_decay: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.08)
    contact_static_lock_vel_zero_thresh_ms: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.015)
    contact_static_lock_cov_vel_scale: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.20)
    contact_static_lock_cov_pos_scale: float = inert(_STATIC_LOCK_NEVER_TRIPS, default=0.85)

    # Stroke-end reset.
    stroke_end_p_vel_scale: float = 0.5

    # Clamp IMU dt spikes.
    imu_dt_max_mult: float = 3.0

    # Turn gate.
    # High jerk threshold prevents normal handwriting vibration from always firing TURN.
    turn_jerk_threshold: float = 1200.0
    turn_arm_n: int = inert(_TURN_GATE_NEVER_ARMS, default=2)

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

    dead_reckoner: StrokeDeadReckonerConfig = field(default_factory=StrokeDeadReckonerConfig)

    # Initial covariance.
    p0_pos: float = 0.35
    p0_vel: float = 0.25
    p0_bias: float = 0.05
