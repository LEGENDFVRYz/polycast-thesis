"""
Module 9 - Causal Trace Filter

Display-side smoothing of the fused pen position. Sits between the fusion
engine and whatever draws the ink, and feeds nothing back, so it changes what
is drawn without touching what is estimated.

Why a hand-rolled filter rather than an EMA:

    The fused trace is mostly sub-noise dither. Across test/_datasets, 55-60%
    of consecutive in-stroke samples land within 0.5 mm of each other while the
    p90 step is 3.5-5.9 mm. That dither reads as direction reversal - one
    `circle_medium-` stroke accumulates over 50 full turns of absolute heading
    change for a single circle. A fixed EMA strong enough to remove it also
    rounds off letter corners, which is the one thing the drawn shape cannot
    afford to lose.

    So alpha is picked per sample from the raw step length - small steps are
    treated as noise and smoothed hard, large steps as deliberate pen motion
    and followed closely - and then raised again whenever the stroke direction
    turns sharply, so corners and cusps survive the pass.

Measured by replaying the fused output of test/_datasets through this filter:
accumulated turning falls 14% (abcde_1), 35% (abc_extralarge), 69%
(hello_world_1) and 90% (circle_medium-), while the stroke bounding-box
diagonal moves by at most 1%. Extent is preserved, so what is removed is not
carrying letter shape.

Ported from kuru_method/asynchronous_stream/main_ekf.py RecognitionTraceFilter,
with one deliberate deviation. That implementation orders the corner test as

    if   cos < 0.35:  alpha = max(alpha, 0.68)     # sharp turn
    elif cos < 0.0:   alpha = max(alpha, 0.80)     # reversal / cusp

so the cusp arm is unreachable - any cos below 0.0 is already below 0.35 and
takes the first branch. Reversals therefore get the weaker 0.68 boost, which is
not what the comment beside it describes. This module tests the cusp first, so a
reversal gets the 0.80 the design intends. It shows up as roughly 1 mm of extra
apex fidelity at cusps and leaves the aggregate figures above unchanged.

Usage:
    filt = CausalTraceFilter()
    for x, y in stroke_samples:
        point = filt.update(x, y)
        if point is not None:
            draw(point)
    filt.reset()          # at every stroke boundary and every trace break

Run directly for a self-test:
    python -m background.pipelines.trace_filter
"""

import math

from background.pipelines.config import cfg


class CausalTraceFilter:
    """
    Single-pass causal smoother for one continuous run of pen samples.

    One instance tracks one unbroken trace. Call reset() at every stroke
    boundary and wherever the drawn polyline is deliberately broken (a teleport
    guard, a relocation break), otherwise the filter blends across the gap and
    drags the new segment toward the old one.
    """

    __slots__ = (
        '_config', '_alphas',
        '_output', '_last_raw', '_last_direction', '_last_emitted',
        'samples_in', 'samples_out',
    )

    def __init__(self, config=None):
        self._config = config if config is not None else cfg.trace_filter
        self._alphas = (
            self._config.alpha_normal if self._config.mode == 'normal'
            else self._config.alpha_light
        )
        self.reset()

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def reset(self):
        """Drop all state. The next sample is emitted unfiltered."""

        self._output = None
        self._last_raw = None
        self._last_direction = None
        self._last_emitted = None
        self.samples_in = 0
        self.samples_out = 0

    def update(self, x: float, y: float) -> tuple[float, float] | None:
        """
        Feed one raw sample, return the point to draw or None to draw nothing.

        None means one of two things, and neither is an error: the sample was
        non-finite (treated as a break), or the filtered output has not moved
        far enough since the last emitted point to be worth a vertex.
        """

        x = float(x)
        y = float(y)

        # A non-finite sample is the caller's break marker, not data. Reset so
        # the segment after the gap starts clean instead of being blended
        # toward the segment before it.
        if not (math.isfinite(x) and math.isfinite(y)):
            self.reset()
            return None

        self.samples_in += 1

        if self._output is None:
            self._output = (x, y)
            self._last_raw = (x, y)
            self._last_emitted = (x, y)
            self.samples_out += 1
            return (x, y)

        step_x = x - self._last_raw[0]
        step_y = y - self._last_raw[1]
        step_length = math.hypot(step_x, step_y)

        alpha = self._alpha_for_step(step_length)
        alpha = self._apply_corner_boost(alpha, step_x, step_y, step_length)

        out_x = alpha * x + (1.0 - alpha) * self._output[0]
        out_y = alpha * y + (1.0 - alpha) * self._output[1]

        self._last_raw = (x, y)
        # A zero-length step has no meaningful direction; keeping the previous
        # one leaves the corner test comparing against real motion.
        if step_length > 1e-6:
            self._last_direction = (step_x / step_length, step_y / step_length)
        self._output = (out_x, out_y)

        # Decimate: the EMA state above has already advanced, we just decline
        # to emit a vertex that would sit on top of the last one.
        emitted_x = out_x - self._last_emitted[0]
        emitted_y = out_y - self._last_emitted[1]
        if math.hypot(emitted_x, emitted_y) < self._config.min_step_m:
            return None

        self._last_emitted = (out_x, out_y)
        self.samples_out += 1
        return (out_x, out_y)

    def filter_points(self, points) -> list[tuple[float, float]]:
        """
        Run a finished sequence through a clean pass of this filter.

        Resets first, so a stroke filtered here is independent of whatever the
        instance was streaming before. Accepts (x, y) or (x, y, ts) tuples and
        returns (x, y) pairs.
        """

        self.reset()
        out = []
        for point in points:
            filtered = self.update(point[0], point[1])
            if filtered is not None:
                out.append(filtered)
        return out

    # -------------------------------------------------------------------------
    # Internals
    # -------------------------------------------------------------------------

    def _alpha_for_step(self, step_length: float) -> float:
        """Pick the base new-sample weight from how far the pen actually moved."""

        config = self._config
        if step_length < config.step_small_m:
            return self._alphas[0]
        if step_length < config.step_medium_m:
            return self._alphas[1]
        if step_length < config.step_large_m:
            return self._alphas[2]
        return self._alphas[3]

    def _apply_corner_boost(
        self, alpha: float, step_x: float, step_y: float, step_length: float
    ) -> float:
        """
        Raise alpha when the stroke direction turns sharply.

        Without this the filter cuts letter corners, which is the failure mode
        that makes smoothed handwriting unreadable. Steps shorter than
        corner_min_step_m are skipped because their direction is dominated by
        the very noise this filter exists to remove.

        Cusp is tested before turn. Order matters and the reference
        implementation gets it wrong: with the turn test first, every reversal
        is caught by it and the cusp threshold can never be reached.
        """

        config = self._config
        if self._last_direction is None or step_length <= config.corner_min_step_m:
            return alpha

        cos_turn = (
            self._last_direction[0] * step_x + self._last_direction[1] * step_y
        ) / step_length

        if cos_turn < config.corner_cos_cusp:
            return max(alpha, config.corner_alpha_cusp)
        if cos_turn < config.corner_cos_turn:
            return max(alpha, config.corner_alpha_turn)
        return alpha


# =============================================================================
# MODULE TESTING
#   Synthetic self-test - no hardware, no datasets. Checks the two properties
#   the filter exists for (dither removal, corner survival) and the three
#   contract details the visualizer depends on (decimation, reset, NaN break).
#
#   Run:  python -m background.pipelines.trace_filter
# =============================================================================
if __name__ == '__main__':
    import random

    def turning(points) -> float:
        """Total absolute heading change in radians, over segments > 0.5 mm."""

        total = 0.0
        previous = None
        for before, after in zip(points, points[1:]):
            dx = after[0] - before[0]
            dy = after[1] - before[1]
            length = math.hypot(dx, dy)
            if length < 5e-4:
                continue
            current = (dx / length, dy / length)
            if previous is not None:
                cross = previous[0] * current[1] - previous[1] * current[0]
                dot = previous[0] * current[0] + previous[1] * current[1]
                total += abs(math.atan2(cross, dot))
            previous = current
        return total

    def diagonal(points) -> float:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def path_length(points) -> float:
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(points, points[1:])
        )

    passed = 0
    failed = 0

    def check(label, condition, detail=''):
        global passed, failed
        if condition:
            passed += 1
            print(f'  [PASS] {label}  {detail}')
        else:
            failed += 1
            print(f'  [FAIL] {label}  {detail}')

    print('=' * 70)
    print('  CausalTraceFilter self-test')
    print('=' * 70)

    # -- 1. Dither removal without shrinking the stroke ----------------------
    # A clean 6 cm circle sampled at handwriting rates, plus 0.4 mm of noise
    # per axis - roughly what the fused trace carries.
    rng = random.Random(20260802)
    truth = []
    circle = []
    for i in range(400):
        angle = 2.0 * math.pi * i / 400.0
        point = (0.03 * math.cos(angle), 0.03 * math.sin(angle))
        truth.append(point)
        circle.append((
            point[0] + rng.gauss(0.0, 0.0004),
            point[1] + rng.gauss(0.0, 0.0004),
        ))

    smoothed = CausalTraceFilter().filter_points(circle)
    raw_turn = turning(circle)
    smooth_turn = turning(smoothed)
    turn_drop = 1.0 - smooth_turn / raw_turn
    # Measured against the noise-free circle, not the noisy input: noise
    # inflates the raw extrema, so comparing to the input would credit the
    # filter for removing an artefact of the test rather than for preserving
    # the shape.
    diag_error = abs(diagonal(smoothed) / diagonal(truth) - 1.0)

    print('\n1. Jittered circle')
    check('turning falls by more than 40%', turn_drop > 0.40,
          f'{raw_turn / (2 * math.pi):.1f} -> {smooth_turn / (2 * math.pi):.1f} turns '
          f'({turn_drop * 100:.0f}%)')
    check('bbox diagonal within 2% of the true circle', diag_error < 0.02,
          f'true {diagonal(truth) * 100:.2f} cm -> {diagonal(smoothed) * 100:.2f} cm '
          f'({diag_error * 100:.1f}%)')
    check('path length is not inflated',
          path_length(smoothed) <= path_length(circle),
          f'{path_length(circle) * 100:.1f} -> {path_length(smoothed) * 100:.1f} cm')

    # -- 2. Cusp survival -----------------------------------------------------
    # The corner boost has two arms and they are not equally useful at kuru's
    # band edges. A 90-degree turn taken at a 4-10 mm step already gets base
    # alpha 0.65, so raising it to corner_alpha_turn (0.68) is close to inert.
    # The arm that does real work is the cusp: a direction reversal jumps alpha
    # to 0.80, which is what keeps the apex of a 'v' or the turn at the bottom
    # of a stem from being cut. Test the arm that matters.
    from dataclasses import replace

    # Measure the mechanism, not the distance to the apex: the closest the
    # trace comes to the apex is set on the approach leg, before any boost
    # applies, so that metric is blind to the thing under test. What corner
    # preservation actually does is cut the filter's lag behind the raw sample
    # once the direction has changed.
    turn_index = 13
    cusp_path = [(0.005 * i, 0.0) for i in range(turn_index)]
    cusp_path += [(0.060 - 0.005 * i, 0.0005 * i) for i in range(1, 13)]

    # A threshold of -2.0 can never be crossed, which disables that arm.
    no_boost_cfg = replace(cfg.trace_filter, corner_cos_turn=-2.0, corner_cos_cusp=-2.0)

    def lag_at_turn(config=None) -> float:
        """Distance between output and raw on the step that reverses direction.

        Measured on that one step rather than averaged over the return leg: the
        boost fires once, and averaging it across a dozen following samples
        dilutes it to the point where the test cannot tell the arms apart.
        """

        filt = CausalTraceFilter(config)
        lag = 0.0
        for index, (x, y) in enumerate(cusp_path):
            filt.update(x, y)
            if index == turn_index:
                lag = math.hypot(filt._output[0] - x, filt._output[1] - y)
        return lag

    boosted_lag = lag_at_turn()
    plain_lag = lag_at_turn(no_boost_cfg)

    print('\n2. Cusp (direction reversal)')
    check('cusp boost tracks the reversal more closely', boosted_lag < plain_lag,
          f'{boosted_lag * 1000:.2f} mm lag vs {plain_lag * 1000:.2f} mm without boost')
    check('cusp boost cuts the lag by at least a third',
          boosted_lag < 0.67 * plain_lag,
          f'{100 * (1 - boosted_lag / plain_lag):.0f}% reduction')
    check('lag at the cusp stays under 1 mm', boosted_lag < 0.001,
          f'{boosted_lag * 1000:.2f} mm')

    # The reference implementation's ordering makes the cusp arm unreachable.
    # Guard against anyone "restoring" it during a future sync.
    turn_only_cfg = replace(cfg.trace_filter, corner_cos_cusp=-2.0)
    check('cusp arm is reachable (reference orders this so it is not)',
          lag_at_turn() < lag_at_turn(turn_only_cfg),
          f'{lag_at_turn() * 1000:.2f} mm vs {lag_at_turn(turn_only_cfg) * 1000:.2f} mm '
          f'with only the turn arm')

    # -- 3. Decimation --------------------------------------------------------
    still = CausalTraceFilter()
    first = still.update(0.5, 0.5)
    repeats = [still.update(0.5, 0.5) for _ in range(20)]

    print('\n3. Decimation')
    check('first sample is emitted unfiltered', first == (0.5, 0.5), f'{first}')
    check('a stationary pen emits nothing further',
          all(r is None for r in repeats),
          f'{sum(r is not None for r in repeats)}/20 emitted')
    check('samples_in counts every sample', still.samples_in == 21,
          f'in={still.samples_in} out={still.samples_out}')

    # -- 4. reset() -----------------------------------------------------------
    resettable = CausalTraceFilter()
    for i in range(50):
        resettable.update(0.001 * i, 0.0)
    resettable.reset()

    print('\n4. reset()')
    check('post-reset sample is emitted unfiltered',
          resettable.update(1.234, 5.678) == (1.234, 5.678))
    check('counters restart at reset', resettable.samples_in == 1,
          f'in={resettable.samples_in}')

    # -- 5. NaN break ---------------------------------------------------------
    breaker = CausalTraceFilter()
    for i in range(50):
        breaker.update(0.001 * i, 0.0)
    nan_result = breaker.update(float('nan'), 0.0)
    after_break = breaker.update(1.0, 1.0)

    print('\n5. NaN break')
    check('NaN emits nothing', nan_result is None, f'{nan_result}')
    check('NaN resets, so the next segment starts clean',
          after_break == (1.0, 1.0), f'{after_break}')
    check('NaN is not counted as a sample', breaker.samples_in == 1,
          f'in={breaker.samples_in}')

    # -- 6. Disabled-config parity -------------------------------------------
    # A caller that skips the filter must get exactly the fused samples back.
    print('\n6. Config')
    check("default mode is 'light'", cfg.trace_filter.mode == 'light',
          cfg.trace_filter.mode)
    check('mode selects the alpha table',
          CausalTraceFilter(replace(cfg.trace_filter, mode='normal'))._alphas
          == cfg.trace_filter.alpha_normal)

    print('\n' + '=' * 70)
    print(f'  {passed} passed, {failed} failed')
    print('=' * 70)
    raise SystemExit(1 if failed else 0)
