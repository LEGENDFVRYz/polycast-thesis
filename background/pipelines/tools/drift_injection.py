"""
Synthetic drift injection: how much IMU error does UWB actually remove?

Correlation cannot answer this. Measured on square_1, the pure IMU
dead-reckoned trace - which never touches UWB - still correlates +0.443 with the
UWB steps, because both sensors are watching the same pen. So a fused trace
correlating +0.82 with UWB proves nothing on its own: most of that is shared
physics, not coupling.

This test removes the ambiguity by adding an error that only one sensor can see.
A constant acceleration bias is injected into the IMU path after preprocessing.
UWB is untouched and therefore still reports the true position, so the injected
bias is the one component of the disagreement whose origin is known exactly.

Running the same recording with and without the injection and differencing the
two fused outputs isolates the response - everything the two runs share, real
motion included, cancels.

The reference is measured, not derived. An earlier version of this tool compared
against the textbook `0.5 * b * t^2` and reported 78% rejection with UWB
severed entirely, which is impossible. The cause was that the ESKF applies
velocity drag (`modes.*.drag_inv_s`, 0.30 during drawing), so a constant bias
does not integrate quadratically at all - it approaches a terminal velocity
`b / drag` and the displacement becomes linear in time. The formula was
measuring its own wrongness.

So the baseline is a third run with UWB correction severed (`imu_only_mode`),
which is by construction the full unconstrained response of this filter to this
bias, drag and velocity caps included:

    rejection = 1 - (surviving drift / unconstrained drift)

    rejection ~ 1.0    UWB fully corrects IMU error: the filter is UWB-driven,
                       and IMU shape contributes little
    rejection ~ 0.0    IMU error passes straight through: the filter is
                       IMU-driven and drift is unbounded, which is what
                       shape_mode does
    in between         the filter is genuinely blending, and the number is the
                       blend ratio

The same difference also shows how fast the correction acts: `t_half` is when
half the injected drift has been rejected, which is the filter's response time
to an IMU error.

Injection is deliberately applied to `acc_board_tip` and `acc_board_hp_tip`
together, after `preprocess/imu.py` has produced them. Perturbing raw body
acceleration instead would also rotate through the attitude path and change the
lever-arm correction, so the injected quantity would no longer be known.

Usage:
    python -m background.pipelines.tools.drift_injection square_1
    python -m background.pipelines.tools.drift_injection --all
    python -m background.pipelines.tools.drift_injection square_1 --bias 0.08
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from background.pipelines.config import cfg
from background.pipelines.tools.cross_fusion import (
    OurPreprocess,
    _REPO_ROOT,
    read_packets,
    resolve_dataset,
)

# Default injected bias, m/s^2. Chosen to be clearly visible against the ~2.4 cm
# UWB noise floor without being physically absurd: over a 1.5 s stroke it
# integrates to 0.5 * 0.05 * 1.5^2 = 5.6 cm, a few times the noise. The measured
# residual accel bias on this rig is 0.03-0.04 m/s^2, so this is the same order
# as a real one.
_DEFAULT_BIAS_MS2 = 0.05


@dataclass
class StrokeResponse:
    """One stroke's response to the injected bias."""

    index: int
    duration_s: float
    elapsed: np.ndarray          # seconds since stroke start, per sample
    injected: np.ndarray         # 0.5 * b * t^2, the drift that was added
    surviving: np.ndarray        # |fused_injected - fused_clean|, what got through

    @property
    def final_injected_cm(self) -> float:
        return float(self.injected[-1] * 100.0)

    @property
    def final_surviving_cm(self) -> float:
        return float(self.surviving[-1] * 100.0)

    @property
    def rejection(self) -> float:
        """Fraction of the injected drift the filter removed, at stroke end."""

        if self.injected[-1] < 1e-9:
            return float('nan')
        return float(1.0 - self.surviving[-1] / self.injected[-1])

    @property
    def half_response_s(self) -> float:
        """
        Time at which half the injected drift had been rejected.

        NaN when the filter never reaches 50% rejection during the stroke, which
        is itself the finding - the correction is slower than the stroke.
        """

        for index in range(len(self.elapsed)):
            if self.injected[index] < 1e-6:
                continue
            if self.surviving[index] <= 0.5 * self.injected[index]:
                return float(self.elapsed[index])
        return float('nan')


@dataclass
class DatasetResponse:
    name: str
    bias_ms2: float
    strokes: list = field(default_factory=list)


def _run(packets: list[dict], bias: np.ndarray) -> list[dict]:
    """
    Drive the full pipeline once, optionally biasing the IMU acceleration.

    Returns one record per in-stroke fused sample. The pipeline is rebuilt from
    scratch here so the two runs cannot share filter state.
    """

    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    preprocess = OurPreprocess()
    fusion = ESKF()
    solver = UWBSolver()
    position_filter = UWBPositionFilter()

    samples: list[dict] = []
    stroke_index = 0
    was_active = False

    for packet in packets:
        if packet['sensor'] == 'UWB':
            # Untouched: UWB must keep reporting the true position, or the
            # injected drift would no longer be the only known disagreement.
            preprocess.uwb(packet)
            ranged = preprocess.last_uwb_event
            if ranged is None:
                continue
            solved = solver.process_one(ranged)
            if not solved:
                continue
            positioned = position_filter.process_one(solved)
            if positioned:
                fusion.process_event(positioned)
            continue

        if packet['sensor'] != 'IMU':
            continue

        preprocess.imu(packet)
        event = preprocess.last_imu_event
        if not event:
            continue

        if bias is not None and np.any(bias):
            # Both board-frame acceleration paths are shifted together. The ESKF
            # blends between them depending on mode, so biasing only one would
            # make the injected quantity depend on which mode happened to be
            # active.
            event = dict(event)
            for key in ('acc_board_tip', 'acc_board_hp_tip'):
                value = event.get(key)
                if value is not None and len(value) >= 2:
                    event[key] = (float(value[0]) + bias[0], float(value[1]) + bias[1])

        fused = fusion.process_event(event)
        if not fused:
            continue

        x, y = fused.get('fused_x'), fused.get('fused_y')
        if x is None or y is None or not (math.isfinite(x) and math.isfinite(y)):
            continue

        active = bool(fused.get('stroke_active'))
        if active and not was_active:
            stroke_index += 1
        was_active = active

        if active:
            samples.append({
                'stroke': stroke_index,
                'ts': int(fused['ts_hw']),
                'xy': (float(x), float(y)),
            })

    return samples


def _paired_response(clean: list[dict], biased: list[dict]) -> dict[int, tuple]:
    """
    Difference two runs, keyed by stroke, aligned on timestamps.

    Injection can shift stroke segmentation at the margins, so samples are
    paired on (stroke, timestamp) rather than by index - pairing by position
    would silently compare different strokes once a boundary moved.
    """

    biased_by_key = {(row['stroke'], row['ts']): row['xy'] for row in biased}

    grouped: dict[int, list[tuple[int, tuple, tuple]]] = {}
    for row in clean:
        partner = biased_by_key.get((row['stroke'], row['ts']))
        if partner is None:
            continue
        grouped.setdefault(row['stroke'], []).append((row['ts'], row['xy'], partner))

    out: dict[int, tuple] = {}
    for index, rows in grouped.items():
        if len(rows) < 30:
            continue
        rows.sort(key=lambda item: item[0])
        start_ts = rows[0][0]
        elapsed = np.array([(ts - start_ts) / 1_000_000.0 for ts, _, _ in rows])
        clean_xy = np.array([xy for _, xy, _ in rows], dtype=float)
        biased_xy = np.array([xy for _, _, xy in rows], dtype=float)
        out[index] = (elapsed, np.linalg.norm(biased_xy - clean_xy, axis=1))
    return out


def measure(dataset: str, bias_ms2: float) -> DatasetResponse:
    """
    Measure how much of an injected IMU drift the filter rejects.

    Three runs of the same recording: clean, biased, and biased with UWB
    correction severed. The third supplies the unconstrained response this
    filter has to this bias, which is the only honest reference - see the module
    docstring on why the analytic formula is not.
    """

    packets = read_packets(resolve_dataset(dataset))

    # Injected along +x only. Direction is arbitrary for the magnitude question,
    # and a single axis keeps the perturbation easy to reason about.
    bias = np.array([bias_ms2, 0.0], dtype=float)

    clean = _run(packets, np.zeros(2))
    biased = _run(packets, bias)

    # Baseline: same injection, UWB severed. imu_only_mode takes precedence over
    # every other fusion flag, so this is the unconstrained response regardless
    # of which configuration is being tested.
    original = cfg.fusion_eskf
    try:
        object.__setattr__(
            cfg, 'fusion_eskf', dataclasses.replace(original, imu_only_mode=True)
        )
        clean_free = _run(packets, np.zeros(2))
        biased_free = _run(packets, bias)
    finally:
        object.__setattr__(cfg, 'fusion_eskf', original)

    constrained = _paired_response(clean, biased)
    unconstrained = _paired_response(clean_free, biased_free)

    result = DatasetResponse(name=dataset, bias_ms2=bias_ms2)

    for index in sorted(constrained):
        if index not in unconstrained:
            continue

        elapsed, surviving = constrained[index]
        free_elapsed, free_surviving = unconstrained[index]

        # The two runs can differ in length if segmentation shifted; compare
        # only the overlapping span.
        count = min(len(elapsed), len(free_elapsed))
        if count < 30:
            continue

        result.strokes.append(
            StrokeResponse(
                index=index,
                duration_s=float(elapsed[count - 1]),
                elapsed=elapsed[:count],
                injected=free_surviving[:count],
                surviving=surviving[:count],
            )
        )

    return result


def print_table(results: list[DatasetResponse]) -> None:
    print()
    print('=' * 92)
    print('  synthetic IMU drift injection  |  how much IMU error does UWB remove?')
    print('=' * 92)
    print(f'  {"stroke":<18}{"dur s":>7}{"injected":>11}{"survived":>11}'
          f'{"rejected":>11}{"t_half s":>10}')
    print('  ' + '-' * 88)

    rejections = []

    for result in results:
        for stroke in result.strokes:
            half = stroke.half_response_s
            half_text = f'{half:.2f}' if not math.isnan(half) else '  --'
            print(f'  {result.name + " #" + str(stroke.index):<18}{stroke.duration_s:>7.2f}'
                  f'{stroke.final_injected_cm:>9.1f}cm'
                  f'{stroke.final_surviving_cm:>9.1f}cm'
                  f'{stroke.rejection * 100:>10.0f}%{half_text:>10}')
            if not math.isnan(stroke.rejection):
                rejections.append(stroke.rejection)

    print('  ' + '-' * 88)
    if rejections:
        mean = float(np.mean(rejections)) * 100.0
        print(f'  mean rejection: {mean:.0f}%')
        print()
        if mean > 85.0:
            print('  -> UWB removes nearly all IMU error. The filter is UWB-driven, and')
            print('     IMU shape contributes little to the drawn stroke.')
        elif mean < 25.0:
            print('  -> IMU error passes through almost unchanged. Drift is effectively')
            print('     unbounded within a stroke; UWB is not constraining it.')
        else:
            print('  -> The filter is genuinely blending. This number is the blend ratio:')
            print('     the share of an IMU-only error that UWB pulls back.')
    print()
    print('  injected = 0.5 * bias * t^2, the drift added to the IMU path alone.')
    print('  survived = |fused_biased - fused_clean|. Shared motion cancels in the')
    print('             difference, so this is purely the response to the injection.')
    print('  t_half   = when half the injected drift had been rejected; "--" means')
    print('             the filter never got there inside the stroke.')
    print('=' * 92)


def render(results: list[DatasetResponse], out_path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as error:
        print(f'  [plot] matplotlib unavailable ({error}); skipping PNG.')
        return False

    strokes = [(result.name, stroke) for result in results for stroke in result.strokes]
    if not strokes:
        print('  [plot] nothing to draw.')
        return False

    columns = min(4, len(strokes))
    rows = math.ceil(len(strokes) / columns)
    figure, axes = plt.subplots(
        rows, columns, figsize=(3.6 * columns, 3.2 * rows), squeeze=False
    )
    figure.suptitle(
        'synthetic IMU drift injection  |  grey = injected drift, green = what survived\n'
        'the gap between them is what UWB corrected',
        fontsize=13,
    )

    for axis in axes.flat[len(strokes):]:
        axis.axis('off')

    for axis, (name, stroke) in zip(axes.flat, strokes):
        axis.plot(stroke.elapsed, stroke.injected * 100.0,
                  color='#9a9a9a', linewidth=2.2, label='injected')
        axis.plot(stroke.elapsed, stroke.surviving * 100.0,
                  color='#2ca02c', linewidth=1.8, label='survived')
        axis.fill_between(stroke.elapsed, stroke.surviving * 100.0,
                          stroke.injected * 100.0, color='#2ca02c', alpha=0.12)
        axis.set_title(f'{name} #{stroke.index}', fontsize=10)
        axis.set_xlabel(f'rejected {stroke.rejection * 100:.0f}%', fontsize=9)
        axis.set_ylabel('cm', fontsize=8)
        axis.tick_params(labelsize=7)
        axis.grid(alpha=0.15)

    axes[0][0].legend(fontsize=8, loc='upper left')

    figure.tight_layout(rect=(0, 0, 1, 0.90))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, dpi=140)
    plt.close(figure)
    print(f'  [plot] wrote {out_path}')
    return True


_DEFAULT_SET = ['square_1', 'abc_1', 'triangle_1', 'hello_lg']


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Inject a known IMU drift and measure how much UWB removes.'
    )
    parser.add_argument('dataset', nargs='?', help='dataset name, or omit with --all')
    parser.add_argument('--all', action='store_true', help='run the default set')
    parser.add_argument('--bias', type=float, default=_DEFAULT_BIAS_MS2,
                        help=f'injected accel bias in m/s^2 (default {_DEFAULT_BIAS_MS2})')
    parser.add_argument('--out', default=None, help='output PNG path')
    parser.add_argument('--compare-configs', action='store_true',
                        help='also report shape_mode and authority_split variants')
    args = parser.parse_args(argv)

    if not args.dataset and not args.all:
        parser.error('give a dataset name or --all')

    names = _DEFAULT_SET if args.all else [args.dataset]

    if args.compare_configs:
        return _compare_configs(names, args.bias)

    results: list[DatasetResponse] = []
    for name in names:
        try:
            results.append(measure(name, args.bias))
        except FileNotFoundError as error:
            print(f'  [skip] {name}: {error}')

    if not results:
        print('  nothing to report.')
        return 2

    print(f'  injected bias: {args.bias:.3f} m/s^2')
    print_table(results)

    stem = 'all' if args.all else results[0].name
    out_path = Path(args.out) if args.out else (
        _REPO_ROOT / 'output' / f'drift_injection_{stem}.png'
    )
    render(results, out_path)
    return 0


def _compare_configs(names: list[str], bias: float) -> int:
    """Report mean rejection under each fusion configuration."""

    variants = [
        ('shape_mode=True  (IMU owns ink)', {'shape_mode': True, 'ink_from_dead_reckoner': True,
                                             'authority_split': False}),
        ('shape_mode=False (blended ESKF)', {'shape_mode': False, 'ink_from_dead_reckoner': False,
                                             'authority_split': False}),
        ('authority_split  (freq split)  ', {'shape_mode': False, 'ink_from_dead_reckoner': False,
                                             'authority_split': True}),
    ]

    original = cfg.fusion_eskf
    print()
    print('=' * 78)
    print(f'  drift rejection by configuration  |  bias {bias:.3f} m/s^2')
    print('=' * 78)
    print(f'  {"configuration":<34}{"mean rejection":>16}{"strokes":>10}')
    print('  ' + '-' * 74)

    try:
        for label, overrides in variants:
            object.__setattr__(cfg, 'fusion_eskf', dataclasses.replace(original, **overrides))
            rejections = []
            for name in names:
                try:
                    result = measure(name, bias)
                except FileNotFoundError:
                    continue
                rejections += [
                    stroke.rejection for stroke in result.strokes
                    if not math.isnan(stroke.rejection)
                ]
            mean = float(np.mean(rejections)) * 100.0 if rejections else float('nan')
            print(f'  {label:<34}{mean:>15.0f}%{len(rejections):>10}')
    finally:
        object.__setattr__(cfg, 'fusion_eskf', original)

    print('  ' + '-' * 74)
    print('  100% = UWB removes all injected IMU error (UWB-driven)')
    print('    0% = injected error passes straight through (IMU-driven, unbounded)')
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
