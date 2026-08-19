"""
Pen-up stage configuration. Each stage carries its own `enabled` flag and runs
in the order listed in modes.POSTPROCESS_STAGES.
"""

from dataclasses import dataclass, field

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
    enabled: bool = False

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
    enabled: bool = False

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
# -----------------------------------------------------------------------------
# Per-stroke velocity detrend (pen-up integration closure)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class VelocityDetrendConfig:
    # Closes the velocity integration of a finished stroke.
    #
    # With ink_from_dead_reckoner on, in-stroke geometry is plain double
    # integration of acc_board_tip. Nothing enforces the constraint the physics
    # gives for free: a writing pen starts at rest and ends at rest. The ESKF
    # zeroes velocity at pen-up, but that only fixes the next stroke's starting
    # condition - the ink already drawn keeps its accumulated velocity error.
    #
    # A constant velocity error displaces position linearly in time, so the
    # resulting ramp is about the size of the letter. Measured on abcde_1,
    # stroke #1 ends at 0.168 m/s over 1.42 s, a 23.9 cm ramp across a 25.9 cm
    # letter. That is why letters came out 1.84x and 0.55x their true extent.
    #
    # Subtracting strength * v_end * (t/T) from velocity before re-integrating
    # removes the ramp and leaves genuine motion. Measured effect on mean
    # |our span / kuru span - 1|: abcde_1 0.288 -> 0.197, abc_extralarge
    # 0.633 -> 0.285, with ink length essentially unchanged.
    #
    # Necessarily a pen-up stage: v_end and T are both unknown until the stroke
    # closes. A causal velocity leak was tested as a substitute and rejected -
    # it damps real pen motion along with the error.
    enabled: bool = True

    # Fraction of the terminal velocity removed. 1.0 forces the stroke to end at
    # exactly zero. Lower values suit a rig where the FSR releases slightly
    # before the pen stops, leaving genuine residual motion at pen-up.
    strength: float = 1.0

    # Below this a stroke has not integrated long enough for the ramp to matter.
    min_points: int = 8

    # Guard against re-integrating a near-instantaneous stroke, where dt noise
    # dominates the terminal velocity estimate.
    min_duration_s: float = 0.10

    # A terminal speed above this is not accumulated drift - either the pen was
    # genuinely moving at pen-up or the samples are unreliable. Forcing a stop
    # in that case would distort real motion, so the stroke is left alone.
    # Handwriting peaks around 1 m/s; measured drift terminals here are
    # 0.03-0.17 m/s.
    max_terminal_velocity_ms: float = 0.60


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

    # OG-BASELINE: was True. This filter postdates e50b74b, so it is off for
    # a true baseline render.
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
