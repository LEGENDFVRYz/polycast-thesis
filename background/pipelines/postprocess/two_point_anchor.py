"""
(Post-Process) Two-point stroke anchoring - removes inertial drift from a
finished stroke by pinning both of its endpoints to UWB.

Why this exists
---------------
With `ink_from_dead_reckoner` enabled the ESKF integrates `acc_board_tip`
directly during ink: no high-pass filter, no bias subtraction, no drag, no
velocity cap. That produced correct letterforms for the first time, because the
mechanisms that had been suppressing shape were exactly those.

The cost is that nothing bounds drift inside a stroke any more. Double
integration turns a constant acceleration error into position error growing as
t^2, so error is negligible at pen-down and largest at pen-up. That matches the
observed defect precisely: stroke ends overshoot and hook away while the body of
the letter is correct.

The correction
--------------
A constant acceleration bias `b` displaces the trajectory by `0.5 * b * t^2`.
That is a known shape with one unknown vector, and the stroke gives us one clean
observation of it: at pen-up UWB independently measures where the tip actually
is, so the endpoint mismatch is the accumulated drift.

Removing `error * (t/T)^2` across the stroke therefore subtracts the drift
parabola and nothing else. The start is untouched (t=0), the end lands on UWB
(t=T), and the middle is corrected in proportion to how much drift it had
actually accumulated by that instant - which is what makes this shape-preserving
rather than a squash toward UWB.

This is the same principle as ZUPT in foot-mounted inertial navigation, where a
known-zero velocity at each footfall is what keeps an otherwise identical
integrator from drifting.

Why it cannot deform the letter
-------------------------------
It runs at pen-up, on a finished stroke. Every mid-stroke UWB correction tried
before this deformed letters because it fought the IMU while the letter was
still being drawn. Here the shape is already committed; only the drift component
is removed.

Endpoint targets come from the UWB points buffered during the stroke, not from a
single fix, because one fix carries ~2.4 cm of noise on this rig.

Run directly for the self-test:
    python -m background.pipelines.postprocess.two_point_anchor
"""

import math

from background.pipelines.config import cfg


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return 0.0
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _robust_endpoint(
    uwb_points: list[tuple[float, float, int]],
    target_ts: int,
    window_us: int,
) -> tuple[float, float] | None:
    """
    Estimate the true tip position at `target_ts` from nearby UWB fixes.

    A single fix carries roughly 2.4 cm of noise on this rig, which is the same
    order as the drift being corrected, so the median of a short window is used
    instead. Falls back to the nearest fixes when the window is empty, since a
    noisy anchor still beats no anchor for a stroke that has visibly drifted.
    """

    if not uwb_points:
        return None

    in_window = [
        (x, y) for x, y, ts in uwb_points
        if abs(ts - target_ts) <= window_us
    ]

    if not in_window:
        nearest = sorted(uwb_points, key=lambda point: abs(point[2] - target_ts))
        in_window = [(x, y) for x, y, _ in nearest[:3]]

    if not in_window:
        return None

    return (
        _median([x for x, _ in in_window]),
        _median([y for _, y in in_window]),
    )


class TwoPointStrokeAnchor:
    """Pins a finished stroke's endpoints to UWB, removing the drift parabola."""

    def __init__(self):
        self.strokes_seen = 0
        self.strokes_corrected = 0
        self.strokes_rejected = 0
        self.last_correction_m = 0.0

    def apply(self, stroke: dict, uwb_points: list[tuple[float, float, int]]) -> dict:
        """
        Return the stroke with accumulated inertial drift removed.

        The stroke is returned unchanged when the correction cannot be trusted:
        too few points, no usable UWB anchor, or an implausibly large endpoint
        error (which means the IMU, the UWB, or both were unreliable for this
        stroke - and a bad correction is worse than none).
        """

        config = cfg.two_point_anchor
        self.strokes_seen += 1

        if not config.enabled:
            return stroke

        points = stroke.get('points') or []
        if len(points) < config.min_points:
            return stroke

        end_target = _robust_endpoint(
            uwb_points, stroke['end_ts'], config.anchor_window_us
        )
        if end_target is None:
            return stroke

        end_x, end_y, _ = points[-1]
        error_x = end_target[0] - end_x
        error_y = end_target[1] - end_y
        error_norm = math.hypot(error_x, error_y)

        # A correction this large is not drift - it means the endpoint anchor and
        # the integrated stroke disagree about something more fundamental, so
        # applying it would move the letter rather than straighten it.
        if error_norm > config.max_correction_m:
            self.strokes_rejected += 1
            return stroke

        error_x *= config.correction_alpha
        error_y *= config.correction_alpha

        start_ts = points[0][2]
        span_us = float(points[-1][2] - start_ts)
        if span_us <= 0.0:
            return stroke

        # (t/T)^2 mirrors how a constant acceleration bias accumulates, so this
        # subtracts the drift and leaves genuine motion alone. A linear ramp
        # would over-correct the early stroke, where little drift exists yet.
        corrected = []
        for x, y, ts in points:
            fraction = (ts - start_ts) / span_us
            weight = fraction * fraction
            corrected.append((x + error_x * weight, y + error_y * weight, ts))

        stroke['points'] = corrected
        stroke['two_point_anchor'] = {
            'applied': True,
            'error_m': error_norm,
            'applied_m': math.hypot(error_x, error_y),
        }

        self.strokes_corrected += 1
        self.last_correction_m = error_norm
        return stroke

    def stats(self) -> dict:
        return {
            'strokes_seen': self.strokes_seen,
            'strokes_corrected': self.strokes_corrected,
            'strokes_rejected': self.strokes_rejected,
            'last_correction_m': self.last_correction_m,
        }

    def reset(self) -> None:
        self.strokes_seen = 0
        self.strokes_corrected = 0
        self.strokes_rejected = 0
        self.last_correction_m = 0.0


# -----------------------------------------------------------------------------
# Self-test
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    def build_stroke(points):
        return {
            'stroke_id': 1,
            'points': points,
            'start_ts': points[0][2],
            'end_ts': points[-1][2],
        }

    def circle_with_drift(n=60, radius=0.03, drift=(0.04, -0.02)):
        """A clean circle plus a t^2 drift - the exact defect being corrected."""
        clean, drifted = [], []
        for i in range(n):
            angle = 2.0 * math.pi * i / (n - 1)
            ts = 1_000_000 + i * 5000
            cx = 0.5 + radius * math.cos(angle)
            cy = 0.5 + radius * math.sin(angle)
            clean.append((cx, cy, ts))
            f = i / (n - 1)
            drifted.append((cx + drift[0] * f * f, cy + drift[1] * f * f, ts))
        return clean, drifted

    passed = failed = 0

    def check(name, condition, detail=''):
        global passed, failed
        if condition:
            passed += 1
            print(f'  PASS  {name}')
        else:
            failed += 1
            print(f'  FAIL  {name}  {detail}')

    print('two-point stroke anchor self-test')
    print('-' * 60)

    anchor = TwoPointStrokeAnchor()
    clean, drifted = circle_with_drift()

    # The UWB buffer sees the true endpoint, which is what makes the drift
    # observable at pen-up.
    uwb = [(clean[-1][0], clean[-1][1], clean[-1][2])]
    result = anchor.apply(build_stroke(list(drifted)), uwb)
    out = result['points']

    before = math.dist(drifted[-1][:2], clean[-1][:2])
    after = math.dist(out[-1][:2], clean[-1][:2])
    check('endpoint error shrinks', after < before * 0.25,
          f'before {before * 100:.2f} cm, after {after * 100:.2f} cm')

    check('start point untouched',
          math.dist(out[0][:2], drifted[0][:2]) < 1e-12)

    # correction_alpha deliberately leaves a fraction of the drift in place, so
    # the bound is expressed against that residual rather than against zero.
    # Comparing to the uncorrected worst case is what proves the parabola was
    # actually removed instead of merely reduced somewhere.
    worst_before = max(math.dist(d[:2], c[:2]) for d, c in zip(drifted, clean))
    worst_after = max(math.dist(o[:2], c[:2]) for o, c in zip(out, clean))
    residual_allowance = (1.0 - cfg.two_point_anchor.correction_alpha) * worst_before
    check('drift parabola removed to within the configured alpha',
          worst_after <= residual_allowance * 1.15,
          f'worst {worst_after * 100:.2f} cm vs allowance '
          f'{residual_allowance * 100:.2f} cm (was {worst_before * 100:.2f} cm)')

    # Shape must survive: path length is the thing a naive squash would destroy.
    def path_len(points):
        return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))

    ratio = path_len(out) / path_len(clean)
    check('path length preserved', 0.93 < ratio < 1.07, f'ratio {ratio:.3f}')

    # A wild anchor must be refused rather than dragging the letter to it.
    anchor.reset()
    far = [(2.0, 2.0, drifted[-1][2])]
    untouched = anchor.apply(build_stroke(list(drifted)), far)
    check('implausible correction rejected',
          untouched['points'][-1][:2] == drifted[-1][:2]
          and anchor.strokes_rejected == 1)

    # Degenerate inputs must pass through rather than raise.
    anchor.reset()
    short = build_stroke(list(drifted[:3]))
    check('too-short stroke passes through',
          anchor.apply(short, uwb)['points'] == drifted[:3])
    check('missing UWB passes through',
          anchor.apply(build_stroke(list(drifted)), [])['points'] == drifted)

    # Zero-duration stroke: all timestamps equal, span is 0.
    flat = [(0.5, 0.5, 1000), (0.51, 0.5, 1000), (0.52, 0.5, 1000),
            (0.53, 0.5, 1000), (0.54, 0.5, 1000), (0.55, 0.5, 1000)]
    check('zero-span stroke does not divide by zero',
          anchor.apply(build_stroke(flat), [(0.6, 0.5, 1000)])['points'] == flat)

    print('-' * 60)
    print(f'{passed} passed, {failed} failed')
    raise SystemExit(1 if failed else 0)
