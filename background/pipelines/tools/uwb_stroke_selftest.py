"""
Per-stroke UWB-only comparison: our range stage vs kuru's, same solver question.

The IMU counterpart (`imu_stroke_selftest`) showed our IMU front end is the
better of the two. This is the other half: with the IMU removed entirely, how
much letter shape does each pipeline's UWB path carry?

UWB measures distances, not position, so "UWB-only" necessarily means
range-preprocessing plus a position solve. Both sides are run end to end in
their own idiom rather than forced through a shared solver, because the range
stage and the solver are co-designed on each side and splitting them would
test a pipeline neither project actually ships:

    ours  UWBRangePreprocessor -> UWBSolver          (weighted least squares)
    kuru  UWBPreprocessor      -> IRLSTrilateration  (IRLS + Huber weights)

Both consume the identical raw packets and identical anchor geometry, and both
use the same per-anchor range offsets (verified equal in the two configs), so
the comparison is fair even though the internals differ.

Which range series feeds each solver matters, and each side's own choice is
honoured:

    ours  `solver_dists` -- the unsmoothed series our solver is built for.
          `clean_dists` adds display smoothing that averages stroke motion away.
    kuru  `filtered`     -- kuru's own visualisation/IRLS path, which is what
          batch_main_ekf_plot.py:66 feeds its UWB-only baseline.

Stroke boundaries come from our fused pipeline for both curves, so panel #N is
the same pen-down segment on both sides. UWB runs ~50 Hz against ~216 Hz IMU,
so a stroke holds far fewer UWB fixes than IMU samples -- expect coarse,
angular traces. That is the sample rate, not a defect.

Each panel is normalised to its own bounding box:

    grey   our fused output   the reference shape for that stroke
    blue   our UWB-only       UWBRangePreprocessor -> UWBSolver
    red    kuru UWB-only      UWBPreprocessor -> IRLSTrilateration

Reading the panels:

  * both roughly trace grey
        -> UWB alone carries that letter; either range stage is adequate.
  * one tracks and the other scatters
        -> that range stage is the weaker one on this data.
  * both scatter while grey is a clean letter
        -> the IMU is carrying that stroke, and UWB is only anchoring it.

Counts are descriptive. No legibility score is computed, for the same reason
as the other tools here: every metric proxy tried on this problem rewards a
stalled pen.

Usage:
    python -m background.pipelines.tools.uwb_stroke_selftest abcde_1
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from background.pipelines.tools.cross_fusion import (
    OurPreprocess,
    _REPO_ROOT,
    read_packets,
    resolve_dataset,
)
from background.pipelines.tools.imu_stroke_selftest import _normalise


@dataclass
class UWBStroke:
    """One pen-down segment with each pipeline's UWB-only position fixes."""

    index: int
    fused: list = field(default_factory=list)
    ours: list = field(default_factory=list)
    kuru: list = field(default_factory=list)
    duration_s: float = 0.0

    def spread_cm(self, points: list) -> float:
        """Bounding-box diagonal of a trace, in centimetres."""

        if len(points) < 2:
            return 0.0
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return 100.0 * math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def collect(packets: list[dict]) -> list[UWBStroke]:
    """
    Single pass: drive our fused pipeline for stroke boundaries while solving
    both UWB-only positions from the same packets.

    The fused pipeline is what decides pen-down/pen-up, so both UWB traces are
    cut at identical boundaries and no stroke-pairing assumption is needed.
    """

    from kuru_method.asynchronous_stream.config import (
        ANCHORS,
        MARKER_LENGTH,
        UWB_OFFSETS,
    )
    from kuru_method.asynchronous_stream.fusion_engine import IRLSTrilateration
    from kuru_method.asynchronous_stream.preprocessor import UWBPreprocessor

    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    preprocess = OurPreprocess()
    fusion = ESKF()
    solver = UWBSolver()
    position_filter = UWBPositionFilter()

    # kuru's UWB-only baseline, constructed exactly as batch_main_ekf_plot.py
    # does at lines 51-55.
    kuru_range = UWBPreprocessor(offsets=UWB_OFFSETS)
    bounds_min = [-0.30, -0.30, -0.50]
    bounds_max = [float(np.max(ANCHORS[:, 0])) + 0.30,
                  float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    kuru_irls = IRLSTrilateration(ANCHORS, bounds_min, bounds_max, tag_z=MARKER_LENGTH)

    strokes: list[UWBStroke] = []
    current: UWBStroke | None = None
    active = False
    start_ts: int | None = None

    for packet in packets:
        if packet['sensor'] == 'UWB':
            # Solve both UWB-only positions regardless of pen state; they are
            # only recorded while a stroke is open.
            ours_xy = kuru_xy = None

            preprocess.uwb(packet)
            ranged = preprocess.last_uwb_event
            if ranged is not None:
                solved = solver.process_one(ranged)
                if solved and solved.get('pos_raw'):
                    ours_xy = (float(solved['pos_raw'][0]), float(solved['pos_raw'][1]))
                    positioned = position_filter.process_one(solved)
                    if positioned:
                        fusion.process_event(positioned)

            try:
                filtered, _weights, _ekf_dists = kuru_range.process(*packet['dists'])
                position, _ = kuru_irls.solve(filtered)
                if position is not None and np.all(np.isfinite(position[:2])):
                    kuru_xy = (float(position[0]), float(position[1]))
            except Exception:
                kuru_xy = None

            if active and current is not None:
                if ours_xy is not None:
                    current.ours.append(ours_xy)
                if kuru_xy is not None:
                    current.kuru.append(kuru_xy)
            continue

        if packet['sensor'] != 'IMU':
            continue

        preprocess.imu(packet)
        event = preprocess.last_imu_event
        if not event:
            continue

        fused = fusion.process_event(event)
        if not fused:
            continue

        now_active = bool(fused.get('stroke_active'))
        x, y = fused.get('fused_x'), fused.get('fused_y')
        if x is None or y is None or not (math.isfinite(x) and math.isfinite(y)):
            continue

        if not now_active:
            active = False
            current = None
            continue

        if not active:
            current = UWBStroke(index=len(strokes) + 1)
            strokes.append(current)
            active = True
            start_ts = event.get('ts_hw')

        current.fused.append((float(x), float(y)))
        ts = event.get('ts_hw')
        if start_ts is not None and ts is not None:
            current.duration_s = (int(ts) - int(start_ts)) / 1_000_000.0

    return [s for s in strokes if len(s.fused) > 1]


def print_table(dataset: str, strokes: list[UWBStroke]):
    print()
    print('=' * 76)
    print(f'  per-stroke UWB-only comparison  |  dataset: {dataset}')
    print('=' * 76)
    print(f'  {"stroke":<8}{"dur s":>8}{"fused cm":>10}'
          f'{"ours fix":>10}{"ours cm":>10}{"kuru fix":>10}{"kuru cm":>10}')
    print('  ' + '-' * 72)

    for stroke in strokes:
        print(f'  #{stroke.index:<7}{stroke.duration_s:>8.2f}'
              f'{stroke.spread_cm(stroke.fused):>10.1f}'
              f'{len(stroke.ours):>10}{stroke.spread_cm(stroke.ours):>10.1f}'
              f'{len(stroke.kuru):>10}{stroke.spread_cm(stroke.kuru):>10.1f}')

    print('  ' + '-' * 72)
    print('  "fix" = UWB position solves inside that stroke. UWB runs ~50 Hz')
    print('  against ~216 Hz IMU, so these counts are small by design.')
    print('  "cm" = bounding-box diagonal. Descriptive only; the grid decides.')
    print('=' * 76)


def render(dataset: str, strokes: list[UWBStroke], out_path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as error:
        print(f'  [plot] matplotlib unavailable ({error}); skipping PNG.')
        return False

    if not strokes:
        print('  [plot] no strokes to draw.')
        return False

    columns = min(5, len(strokes))
    rows = math.ceil(len(strokes) / columns)
    figure, axes = plt.subplots(
        rows, columns, figsize=(3.0 * columns, 3.3 * rows), squeeze=False
    )
    figure.suptitle(
        f'per-stroke UWB-only comparison  |  {dataset}\n'
        'grey = our fused output      blue = our UWB-only      red = kuru UWB-only',
        fontsize=13,
    )

    for axis in axes.flat[len(strokes):]:
        axis.axis('off')

    for axis, stroke in zip(axes.flat, strokes):
        fused = _normalise(stroke.fused)
        ours = _normalise(stroke.ours)
        kuru = _normalise(stroke.kuru)

        if fused:
            axis.plot([p[0] for p in fused], [p[1] for p in fused],
                      color='#9a9a9a', linewidth=2.6, solid_capstyle='round')
        if ours:
            axis.plot([p[0] for p in ours], [p[1] for p in ours],
                      color='#1f77d0', linewidth=1.5, marker='o', markersize=2.2,
                      solid_capstyle='round')
        if kuru:
            axis.plot([p[0] for p in kuru], [p[1] for p in kuru],
                      color='#d62728', linewidth=1.4, marker='o', markersize=2.2,
                      alpha=0.85, solid_capstyle='round')

        axis.set_title(f'#{stroke.index}', fontsize=11)
        axis.set_aspect('equal', adjustable='box')
        axis.set_xticks([])
        axis.set_yticks([])
        axis.set_xlabel(
            f'{stroke.duration_s:.2f}s\n'
            f'{len(stroke.ours)} ours / {len(stroke.kuru)} kuru fixes',
            fontsize=8,
        )

    figure.tight_layout(rect=(0, 0, 1, 0.93))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, dpi=140)
    plt.close(figure)
    print(f'  [plot] wrote {out_path}')
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Per-stroke UWB-only comparison.')
    parser.add_argument('dataset', help='dataset name (e.g. abcde_1) or path to a CSV')
    parser.add_argument('--out', default=None, help='output PNG path')
    args = parser.parse_args(argv)

    try:
        csv_path = resolve_dataset(args.dataset)
    except FileNotFoundError as error:
        print(f'  {error}')
        return 2

    packets = read_packets(csv_path)
    print(f'  [read] {csv_path.name}: {len(packets)} packets')

    strokes = collect(packets)
    if not strokes:
        print('  no strokes found.')
        return 2

    print_table(csv_path.stem, strokes)

    out_path = Path(args.out) if args.out else (
        _REPO_ROOT / 'output' / f'uwb_selftest_{csv_path.stem}.png'
    )
    render(csv_path.stem, strokes, out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
