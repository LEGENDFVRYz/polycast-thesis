"""
Close a finished stroke's velocity integration.

With `ink_from_dead_reckoner` on, in-stroke geometry comes from plain double
integration of `acc_board_tip` with no bias subtraction, no drag and no
velocity cap. Nothing in that path enforces the one constraint the physics
gives for free: a pen that is writing starts at rest and ends at rest.

The ESKF zeroes velocity at pen-down (eskf.py:412) and again at pen-up
(eskf.py:433), but the pen-up zero only fixes the *next* stroke's starting
condition. The ink already drawn keeps whatever velocity error accumulated
across the stroke, and a constant velocity error displaces position linearly
in time. Measured on abcde_1:

    stroke   duration   |v_end|      v_end * T   letter span
    #1          1.42 s   0.168 m/s      23.9 cm       25.9 cm
    #9          1.17 s   0.159 m/s      18.6 cm       31.0 cm
    #4          1.13 s   0.148 m/s      16.7 cm       16.1 cm

The ramp is about the size of the letter. That is the mechanism behind letters
coming out 1.84x and 0.55x their true extent - not noise, an unclosed integral.

This stage re-integrates the stroke with that ramp removed:

    v'(t) = v(t) - strength * v_end * (t / T)

At strength 1.0 the stroke ends at exactly zero velocity. Lower values apply a
partial correction, for the case where the FSR releases slightly before the pen
physically stops and the true terminal velocity is not quite zero.

Why this is a pen-up stage and not a filter change: `v_end` is unknown until
the last sample and `T` until the stroke closes, so the correction cannot be
computed live. A causal velocity leak was tested as a substitute
(tools/causal_detrend_sweep.py) and rejected - it damps real pen motion along
with the error, shrinking letters and flattening internal structure, and only
matched this stage's size metric by collapsing the trace.

Ordering: this runs before TwoPointStrokeAnchor. The anchor fits a t^2 drift
parabola to the stroke's endpoint error, which assumes the shape it is
correcting is otherwise sound; handing it a stroke with an unclosed linear ramp
would fit the parabola to the wrong error.

Measured effect (mean |our span / kuru span - 1| per stroke):

    abcde_1          0.288 -> 0.197
    abc_extralarge   0.633 -> 0.285

Ink length barely moves (3.17 -> 3.10 m on abcde_1), so extent is corrected
rather than collapsed.

Run directly for a self-test:
    python -m background.pipelines.postprocess.velocity_detrend
"""

import math

import numpy as np

from background.pipelines.config import cfg


class StrokeVelocityDetrend:
    """Removes the linear velocity ramp from a finished stroke's integration."""

    def __init__(self):
        self.strokes_seen = 0
        self.strokes_corrected = 0
        self.last_terminal_velocity_ms = 0.0
        self.last_shift_m = 0.0

    def apply(self, stroke: dict) -> dict:
        """
        Return the stroke with its velocity integration closed.

        The stroke is returned unchanged when the correction cannot be computed
        or cannot be trusted: the stage is disabled, the per-sample record is
        missing, the stroke is too short, or the implied terminal velocity is
        too large to be drift rather than a genuine fast pen-up.

        Requires the per-sample `samples` list, so this must run before
        StrokeFinalizationIMUCleaner, which consumes and discards it.
        """

        settings = cfg.velocity_detrend
        self.strokes_seen += 1

        if not settings.enabled:
            return stroke

        points = stroke.get('points') or []
        samples = stroke.get('samples') or []

        # Without a per-sample record there is no acceleration to re-integrate.
        if len(points) < settings.min_points or len(samples) != len(points):
            stroke['velocity_detrend'] = {'applied': False, 'reason': 'missing_samples'}
            return stroke

        acceleration, deltas = self._extract(samples)
        if acceleration is None:
            stroke['velocity_detrend'] = {'applied': False, 'reason': 'no_usable_dt'}
            return stroke

        total_time = float(deltas.sum())
        if total_time <= settings.min_duration_s:
            stroke['velocity_detrend'] = {'applied': False, 'reason': 'too_brief'}
            return stroke

        # Forward-integrate velocity from rest, exactly as the dead reckoner does.
        velocity = np.cumsum(acceleration * deltas[:, None], axis=0)
        terminal = velocity[-1]
        terminal_speed = float(np.linalg.norm(terminal))
        self.last_terminal_velocity_ms = terminal_speed

        # A terminal speed this high is not accumulated drift. Either the pen was
        # genuinely still moving at pen-up or the samples are unreliable; in both
        # cases forcing a stop would distort real motion.
        if terminal_speed > settings.max_terminal_velocity_ms:
            stroke['velocity_detrend'] = {
                'applied': False,
                'reason': 'terminal_velocity_too_large',
                'terminal_velocity_ms': round(terminal_speed, 4),
            }
            return stroke

        strength = float(np.clip(settings.strength, 0.0, 1.0))
        elapsed = np.cumsum(deltas)
        detrended = velocity - strength * terminal * (elapsed / total_time)[:, None]

        # Re-integrate position from the stroke's own first point, so placement
        # is untouched and only shape changes.
        origin = np.array([float(points[0][0]), float(points[0][1])])
        positions = origin + np.cumsum(detrended * deltas[:, None], axis=0)

        corrected = [points[0]]
        for index in range(1, len(points)):
            corrected.append(
                (float(positions[index - 1][0]),
                 float(positions[index - 1][1]),
                 points[index][2])
            )

        shift = math.hypot(
            corrected[-1][0] - points[-1][0],
            corrected[-1][1] - points[-1][1],
        )
        self.last_shift_m = shift
        self.strokes_corrected += 1

        stroke['points'] = corrected
        stroke['velocity_detrend'] = {
            'applied': True,
            'strength': strength,
            'terminal_velocity_ms': round(terminal_speed, 4),
            'duration_s': round(total_time, 4),
            'endpoint_shift_m': round(shift, 4),
        }
        return stroke

    @staticmethod
    def _extract(samples: list) -> tuple:
        """
        Pull per-sample board acceleration and dt out of the stroke record.

        Returns (None, None) when no sample carries a usable dt, since every
        term below divides by or multiplies through it.
        """

        acceleration = []
        deltas = []

        for sample in samples[1:]:
            # Deliberately no fallback to acc_board_hp_tip. The high-pass variant
            # has a ~1.3 s time constant, about one stroke, so re-integrating it
            # reconstructs a different trajectory than the one drawn and warps the
            # letter. Falling back silently produced exactly that: terminal
            # velocities read 0.0375 m/s where the true value was 0.168 m/s.
            accel = sample.get('acc_board_tip')
            if accel is None:
                return None, None

            ax, ay = float(accel[0]), float(accel[1])
            if not (math.isfinite(ax) and math.isfinite(ay)):
                ax = ay = 0.0

            dt = sample.get('dt_s')
            if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0.0:
                continue

            acceleration.append((ax, ay))
            deltas.append(float(dt))

        if len(acceleration) < 2:
            return None, None

        return np.asarray(acceleration, dtype=float), np.asarray(deltas, dtype=float)

    def stats(self) -> dict:
        return {
            'strokes_seen': self.strokes_seen,
            'strokes_corrected': self.strokes_corrected,
            'last_terminal_velocity_ms': self.last_terminal_velocity_ms,
            'last_shift_m': self.last_shift_m,
        }


# -----------------------------------------------------------------------------
# Self-test
# -----------------------------------------------------------------------------

def _synthetic_stroke(bias: float, count: int = 100, dt: float = 0.005) -> dict:
    """
    A stroke that should close at rest, plus a constant acceleration bias.

    The motion is one full sine period per axis, whose velocity integral over
    the stroke is exactly zero -- so any terminal velocity is the injected bias
    and nothing else.
    """

    points = []
    samples = []
    velocity = np.zeros(2)
    position = np.zeros(2)

    for index in range(count):
        phase = 2.0 * math.pi * index / count
        accel = np.array([math.cos(phase), math.sin(phase)]) + bias

        position = position + velocity * dt + 0.5 * accel * dt * dt
        velocity = velocity + accel * dt

        points.append((float(position[0]), float(position[1]), 1000 * index))
        samples.append({'acc_board_tip': (float(accel[0]), float(accel[1])), 'dt_s': dt})

    return {'points': points, 'samples': samples, 'start_ts': 0, 'end_ts': 1000 * count}


def _run_self_test() -> bool:
    settings = cfg.velocity_detrend
    was_enabled = settings.enabled
    object.__setattr__(settings, 'enabled', True)

    passed = True

    def check(name: str, condition: bool, detail: str = ''):
        nonlocal passed
        status = 'PASS' if condition else 'FAIL'
        if not condition:
            passed = False
        print(f'  [{status}] {name}{("  -- " + detail) if detail else ""}')

    # 1. A biased stroke is corrected, and its terminal velocity is the bias.
    stroke = _synthetic_stroke(bias=0.5)
    detrend = StrokeVelocityDetrend()
    result = detrend.apply(dict(stroke))
    info = result['velocity_detrend']
    expected = 0.5 * math.sqrt(2.0) * (99 * 0.005)
    check('biased stroke corrected', info['applied'])
    check(
        'terminal velocity matches injected bias',
        abs(info['terminal_velocity_ms'] - expected) < 0.02,
        f"got {info['terminal_velocity_ms']:.4f}, expected ~{expected:.4f}",
    )
    check('endpoint moved', info['endpoint_shift_m'] > 0.001)

    # 2. An unbiased stroke is left essentially alone.
    #
    # "Essentially" rather than "exactly": a sine sampled at 100 points does not
    # integrate to exactly zero, and that truncation leaves ~0.005 m/s of
    # terminal velocity which the stage correctly removes. The bar is therefore
    # the discretisation residual, not zero - and it must stay far below the
    # correction applied to the biased stroke, which is the property that
    # matters.
    clean = _synthetic_stroke(bias=0.0)
    baseline = StrokeVelocityDetrend().apply(dict(clean))
    clean_shift = baseline['velocity_detrend']['endpoint_shift_m']
    check(
        'unbiased stroke barely moves',
        clean_shift < 0.005,
        f'shift {clean_shift:.6f} m (discretisation residual)',
    )
    check(
        'biased correction dwarfs the unbiased one',
        info['endpoint_shift_m'] > 20 * clean_shift,
        f"{info['endpoint_shift_m']:.4f} m vs {clean_shift:.6f} m",
    )

    # 3. Placement is preserved: only shape may change.
    check(
        'first point unchanged',
        result['points'][0] == stroke['points'][0],
    )

    # 4. Timestamps are carried through untouched.
    check(
        'timestamps preserved',
        [p[2] for p in result['points']] == [p[2] for p in stroke['points']],
    )

    # 5. Guards.
    short = {'points': stroke['points'][:3], 'samples': stroke['samples'][:3]}
    check(
        'short stroke skipped',
        not StrokeVelocityDetrend().apply(short)['velocity_detrend']['applied'],
    )

    no_samples = {'points': stroke['points'], 'samples': []}
    check(
        'missing samples skipped',
        not StrokeVelocityDetrend().apply(no_samples)['velocity_detrend']['applied'],
    )

    wild = _synthetic_stroke(bias=40.0)
    check(
        'implausible terminal velocity rejected',
        not StrokeVelocityDetrend().apply(wild)['velocity_detrend']['applied'],
    )

    object.__setattr__(settings, 'enabled', False)
    disabled = StrokeVelocityDetrend().apply(dict(stroke))
    check('disabled is a no-op', 'velocity_detrend' not in disabled)

    object.__setattr__(settings, 'enabled', was_enabled)
    return passed


if __name__ == '__main__':
    print('=' * 60)
    print('  StrokeVelocityDetrend self-test')
    print('=' * 60)
    ok = _run_self_test()
    print('=' * 60)
    print(f'  {"ALL PASS" if ok else "FAILURES PRESENT"}')
    print('=' * 60)
    raise SystemExit(0 if ok else 1)
