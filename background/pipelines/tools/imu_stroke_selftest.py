"""
Per-stroke IMU self-test: how much letter shape does the IMU alone carry?

Every experiment so far judged the pipeline on a whole dataset, where a single
bad letter is easy to miss and easy to blame on the wrong stage. This tool
splits the run into individual strokes -- one letter, or one pen-down segment --
and scores each one on its own.

For each stroke it dead-reckons the IMU independently:

    p(t) = p(0) + integral of ( integral of acc_board_tip )

starting from rest at the stroke's own first point. Nothing else is applied:
no UWB, no bias subtraction, no drag, no velocity cap, no post-processing. That
is deliberately the rawest possible reading of "what did the IMU see", and it
matches what `ink_from_dead_reckoner` integrates during ink (eskf.py, via
`_ink_from_dead_reckoner`), so a stroke that dead-reckons well is one the
current in-stroke path can in principle reproduce.

Each stroke is drawn normalised to its own bounding box, so letters of
different sizes stay comparable:

    grey   fused output   what our pipeline actually produced
    blue   IMU-only       double-integrated acc_board_tip
    red    kuru           the same stroke through kuru's pipeline

The red reference is load-bearing, and the tool is misleading without it.
`drift` measures how far the IMU-only trace ends from the fused trace -- how
much those two agree with *each other*. It says nothing about whether either
resembles a letter, and both can agree while both are wrong. kuru renders this
dataset legibly from the same recording, so its per-stroke output is the
closest available stand-in for "what this letter should look like".

Reading the panels:

  * blue tracks red, grey does not
        -> the IMU carried the shape and our fusion lost it.
  * grey tracks red, blue does not
        -> UWB is carrying that letter; IMU alone is not sufficient there.
  * blue and grey agree with each other but neither tracks red
        -> our pipeline is consistently wrong; low drift here means agreement,
           not correctness.
  * none of the three resembles a letter
        -> that stroke's recording is genuinely poor; do not tune against it.

Reported per stroke, all descriptive rather than scored:

    pts        sample count
    dur        stroke duration in seconds
    drift      end-point distance between IMU-only and fused, in cm. Grows as
               t^2 under constant accel bias, so it is expected to be larger on
               long strokes -- compare against `dur`, not across strokes.
    span       bounding-box diagonal of the fused stroke, in cm.

No pass/fail verdict is printed. Tortuosity, turning and path/span ratio were
each tried on this problem and each rewards a stalled pen; the numbers here
size the drift, they do not rank legibility. The grid is the output that
decides.

Usage:
    python -m background.pipelines.tools.imu_stroke_selftest abcde_1
    python -m background.pipelines.tools.imu_stroke_selftest abcde_1 --out g.png
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


@dataclass
class StrokeTrace:
    """One pen-down segment, with the fused output and the IMU-only integration."""

    index: int
    fused: list = field(default_factory=list)      # [(x, y), ...]
    imu: list = field(default_factory=list)        # [(x, y), ...]
    kuru: list = field(default_factory=list)       # [(x, y), ...] reference
    duration_s: float = 0.0

    @property
    def point_count(self) -> int:
        return len(self.fused)

    def drift_cm(self) -> float:
        """End-point separation between the IMU-only and fused traces."""

        if not self.fused or not self.imu:
            return float('nan')
        fx, fy = self.fused[-1]
        ix, iy = self.imu[-1]
        return 100.0 * math.hypot(ix - fx, iy - fy)

    def span_cm(self) -> float:
        """Bounding-box diagonal of the fused stroke."""

        return 100.0 * _bbox_diagonal(self.fused)


def _bbox_diagonal(points: list) -> float:
    if len(points) < 2:
        return 0.0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _normalise(points: list) -> list:
    """
    Scale a stroke into a unit box about its own centre.

    Each panel is drawn in its own normalised frame so a large letter and a
    small one are equally readable, and so the IMU/fused comparison is about
    shape rather than absolute placement.
    """

    if len(points) < 2:
        return points

    xs = np.array([p[0] for p in points], dtype=float)
    ys = np.array([p[1] for p in points], dtype=float)

    extent = max(xs.max() - xs.min(), ys.max() - ys.min())
    if extent < 1e-9:
        return points

    cx = 0.5 * (xs.max() + xs.min())
    cy = 0.5 * (ys.max() + ys.min())
    return list(zip((xs - cx) / extent, (ys - cy) / extent))


def collect_strokes(packets: list[dict]) -> list[StrokeTrace]:
    """
    Run the pipeline once, capturing each stroke's fused output alongside an
    independent IMU-only integration of the same samples.

    The IMU integration is reset at every pen-down, so drift never carries from
    one letter into the next -- each stroke is judged on its own recording.
    """

    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    preprocess = OurPreprocess()
    fusion = ESKF()
    solver = UWBSolver()
    position_filter = UWBPositionFilter()

    strokes: list[StrokeTrace] = []
    current: StrokeTrace | None = None
    velocity = np.zeros(2)
    position = np.zeros(2)
    start_ts: int | None = None

    for packet in packets:
        if packet['sensor'] == 'UWB':
            # UWB still drives the fused trace; it simply contributes nothing
            # to the IMU-only integration.
            adapted = preprocess.uwb(packet)
            if adapted is None or preprocess.last_uwb_event is None:
                continue
            solved = solver.process_one(preprocess.last_uwb_event)
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

        fused = fusion.process_event(event)
        if not fused:
            continue

        active = bool(fused.get('stroke_active'))
        x, y = fused.get('fused_x'), fused.get('fused_y')
        if x is None or y is None or not (math.isfinite(x) and math.isfinite(y)):
            continue

        if not active:
            current = None
            continue

        if current is None:
            # Pen-down: start a fresh stroke and restart the integrator from
            # rest at this point. Starting from rest is the same assumption the
            # in-stroke dead reckoner makes at every pen-down.
            current = StrokeTrace(index=len(strokes) + 1)
            strokes.append(current)
            velocity = np.zeros(2)
            position = np.array([float(x), float(y)])
            start_ts = event.get('ts_hw')
            current.fused.append((float(x), float(y)))
            current.imu.append((position[0], position[1]))
            continue

        dt = event.get('dt_s')
        if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
            continue

        accel = event.get('acc_board_tip') or (0.0, 0.0)
        ax, ay = float(accel[0]), float(accel[1])
        if not (math.isfinite(ax) and math.isfinite(ay)):
            ax = ay = 0.0

        # Plain double integration. No bias term, no damping, no clamp -- any
        # of those would be the pipeline's opinion rather than the IMU's.
        accel_vec = np.array([ax, ay])
        position = position + velocity * dt + 0.5 * accel_vec * dt * dt
        velocity = velocity + accel_vec * dt

        current.fused.append((float(x), float(y)))
        current.imu.append((float(position[0]), float(position[1])))

        ts = event.get('ts_hw')
        if start_ts is not None and ts is not None:
            current.duration_s = (int(ts) - int(start_ts)) / 1_000_000.0

    # A 1-point stroke has no shape to judge.
    return [s for s in strokes if s.point_count > 1]


def attach_kuru_reference(packets: list[dict], strokes: list[StrokeTrace]) -> bool:
    """
    Overlay kuru's per-stroke output as a known-legible reference.

    Why this matters: `drift` measures how far the IMU-only trace ends from the
    fused trace -- how much the two agree with each other. It says nothing
    about whether either one looks like a letter, and the two can agree while
    both being wrong. kuru renders this dataset legibly from the same file, so
    its per-stroke trace is the closest thing available to ground truth for
    "what should this letter look like".

    Both pipelines segment on the same FSR contact signal and produce the same
    stroke count in the same order, so stroke #N is the same pen-down segment
    in both. That is asserted rather than assumed: on a count mismatch the
    reference is dropped instead of silently pairing the wrong letters.
    """

    from background.pipelines.tools.cross_fusion import KuruPreprocess, run_kuru_fusion

    result = run_kuru_fusion(packets, KuruPreprocess(), 'ref', 'kuru reference')
    if result.error:
        print(f'  [kuru] reference unavailable: {result.error}')
        return False

    if len(result.strokes) != len(strokes):
        print(f'  [kuru] stroke count mismatch '
              f'(kuru {len(result.strokes)} vs ours {len(strokes)}); '
              f'reference dropped rather than risk pairing different letters.')
        return False

    for stroke, (xs, ys) in zip(strokes, result.strokes):
        stroke.kuru = list(zip(xs, ys))
    return True


def print_table(dataset: str, strokes: list[StrokeTrace]):
    print()
    print('=' * 72)
    print(f'  per-stroke IMU self-test  |  dataset: {dataset}')
    print('=' * 72)
    print(f'  {"stroke":<8}{"pts":>7}{"dur s":>9}{"drift cm":>11}{"span cm":>10}')
    print('  ' + '-' * 68)

    for stroke in strokes:
        print(f'  #{stroke.index:<7}{stroke.point_count:>7}{stroke.duration_s:>9.2f}'
              f'{stroke.drift_cm():>11.1f}{stroke.span_cm():>10.1f}')

    print('  ' + '-' * 68)
    print('  drift = IMU-only vs fused end-point gap. Grows as t^2 under a')
    print('  constant accel bias, so read it against dur, not across strokes.')
    print('  No score is computed -- the grid below is what decides.')
    print('=' * 72)


def render(dataset: str, strokes: list[StrokeTrace], out_path: Path) -> bool:
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
    has_kuru = any(stroke.kuru for stroke in strokes)
    legend = 'grey = fused output      blue = IMU-only (double-integrated)'
    if has_kuru:
        legend += '      red = kuru (reference)'
    figure.suptitle(f'per-stroke IMU self-test  |  {dataset}\n{legend}', fontsize=13)

    for axis in axes.flat[len(strokes):]:
        axis.axis('off')

    for axis, stroke in zip(axes.flat, strokes):
        fused = _normalise(stroke.fused)
        imu = _normalise(stroke.imu)
        kuru = _normalise(stroke.kuru)

        if fused:
            axis.plot([p[0] for p in fused], [p[1] for p in fused],
                      color='#9a9a9a', linewidth=2.6, solid_capstyle='round')
        if imu:
            axis.plot([p[0] for p in imu], [p[1] for p in imu],
                      color='#1f77d0', linewidth=1.6, solid_capstyle='round')
        if kuru:
            axis.plot([p[0] for p in kuru], [p[1] for p in kuru],
                      color='#d62728', linewidth=1.5, alpha=0.85,
                      solid_capstyle='round')

        axis.set_title(f'#{stroke.index}', fontsize=11)
        axis.set_aspect('equal', adjustable='box')
        axis.set_xticks([])
        axis.set_yticks([])
        axis.set_xlabel(
            f'{stroke.point_count} pts / {stroke.duration_s:.2f}s\n'
            f'drift {stroke.drift_cm():.1f} cm',
            fontsize=8,
        )

    figure.tight_layout(rect=(0, 0, 1, 0.93))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, dpi=140)
    plt.close(figure)
    print(f'  [plot] wrote {out_path}')
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Per-stroke IMU dead-reckoning self-test.')
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

    strokes = collect_strokes(packets)
    if not strokes:
        print('  no strokes found.')
        return 2

    attach_kuru_reference(packets, strokes)
    print_table(csv_path.stem, strokes)

    out_path = Path(args.out) if args.out else (
        _REPO_ROOT / 'output' / f'imu_selftest_{csv_path.stem}.png'
    )
    render(csv_path.stem, strokes, out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
