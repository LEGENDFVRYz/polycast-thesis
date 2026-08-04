"""
Cross-wiring harness: which stage causes the readability gap?

`background/pipelines/` (our ESKF) and `kuru_method/asynchronous_stream/`
(kuru's EKF) render the same dataset with very different legibility. Each
pipeline is preprocess + fusion, so the gap could live in either stage. Six
consecutive fusion-side changes were neutral or worse, which is consistent with
the defect living upstream in preprocessing.

This script swaps only the preprocessing stage and holds fusion fixed, running
a 2x2:

    A  kuru preprocess -> kuru fusion    control (kuru baseline)
    B  ours preprocess -> kuru fusion    the cell under test
    C  ours preprocess -> ours fusion    control (our baseline)
    D  kuru preprocess -> ours fusion    completes the square

A and C reproduce existing behaviour. If they do not look like what the
visualizer already produces for a dataset, this harness is wrong and nothing
may be concluded from B.

Reading the result:
    B looks like A  ->  our preprocessing is fine; the gap is in our ESKF.
    B looks like C  ->  our preprocessing carries the defect.
    B never inits   ->  our range gate is too aggressive for kuru's >=3-anchor
                        seeding, which is itself a concrete finding.

Two honest limits, both structural rather than fixable here:

  1. `cleaner/` is only incidentally exercised. `unpacker.py` is serial framing
     and `time_alignment.py` largely no-ops on a pre-sorted CSV, so what is
     really under test is `preprocess/`.

  2. The IMU arm is weak. kuru's `IMUPreprocessor.process_sample()` only
     normalises the quaternion and passes acceleration through untouched, so
     feeding it our `acc_sensor` (which is the unmodified hardware sample)
     reproduces kuru's own input almost exactly. Everything our IMU stage
     actually does -- HPF, pre-derivative LPF, rigid-body tip correction, board
     projection -- lands in derived fields that kuru's integrator structurally
     cannot accept, because it does its own projection and lever-arm correction
     internally. Passing those would double-apply both.

     So UWB is the informative arm: our `solver_dists` (outlier gate +
     per-anchor range Kalman) and kuru's `ekf_dists` (offset + validity gate,
     deliberately no median/EMA) are genuinely different pipelines feeding the
     same slot, over identical anchor geometry and identical range offsets.

No readability score is computed. Tortuosity, turning and path/span ratio were
each tried on this problem and each rewards a stalled pen, having produced a
wrong conclusion at least once. The printed numbers are descriptive only: they
detect a broken run (never initialised, zero strokes, collapsed extent). They do
not rank quality. The eye check on the PNG decides.

Usage:
    python -m background.pipelines.tools.cross_fusion abcde_1
    python -m background.pipelines.tools.cross_fusion abc_extralarge --out run.png
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from background.pipelines.cleaner.unpacker import parse_packet_line

# Repo root: background/pipelines/tools/cross_fusion.py -> up 4.
_REPO_ROOT = Path(__file__).resolve().parents[3]
_DATASET_DIR = _REPO_ROOT / 'test' / '_datasets'

# Weight handed to kuru for an anchor our range stage flagged. kuru's own
# AnchorQualityTracker floors its weights at 0.05; matching that keeps a flagged
# anchor influential in the same degree kuru would allow, rather than inventing
# a harsher penalty that would confound the comparison.
_FLAGGED_ANCHOR_WEIGHT = 0.05


# -----------------------------------------------------------------------------
# Dataset reading
# -----------------------------------------------------------------------------

def resolve_dataset(name: str) -> Path:
    """Accept a bare dataset name, a filename, or a full path."""

    candidate = Path(name)
    if candidate.is_file():
        return candidate

    for stem in (name, f'{name}.csv'):
        path = _DATASET_DIR / stem
        if path.is_file():
            return path

    raise FileNotFoundError(f'dataset not found: {name} (looked in {_DATASET_DIR})')


def read_packets(csv_path: Path) -> list[dict]:
    """
    Parse the dataset once into raw packets, shared by all four configurations.

    Uses our own `parse_packet_line` rather than kuru's `AsyncDataParser`
    because both read the identical on-disk format and a single reader removes
    any chance that a parsing difference is mistaken for a pipeline difference.
    """

    packets = []
    with open(csv_path, 'r', encoding='utf-8', errors='replace') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            packet = parse_packet_line(line)
            if packet is not None:
                packets.append(packet)
    return packets


# -----------------------------------------------------------------------------
# Result container
# -----------------------------------------------------------------------------

@dataclass
class RunResult:
    """One configuration's trace. `strokes` is a list of (xs, ys) pen-down runs."""

    label: str
    detail: str
    strokes: list = field(default_factory=list)
    imu_samples: int = 0
    uwb_samples: int = 0
    init_index: int | None = None   # IMU sample at which fusion first produced a fix
    error: str = ''

    @property
    def point_count(self) -> int:
        return sum(len(xs) for xs, _ in self.strokes)

    def ink_length_m(self) -> float:
        total = 0.0
        for xs, ys in self.strokes:
            for i in range(1, len(xs)):
                total += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        return total

    def bbox_diagonal_m(self) -> float:
        xs = [x for stroke_x, _ in self.strokes for x in stroke_x]
        ys = [y for _, stroke_y in self.strokes for y in stroke_y]
        if len(xs) < 2:
            return 0.0
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


class _StrokeCollector:
    """Accumulates pen-down points into discrete strokes."""

    def __init__(self):
        self.strokes: list = []
        self._x: list[float] = []
        self._y: list[float] = []

    def add(self, x: float, y: float, writing: bool):
        if writing:
            if math.isfinite(x) and math.isfinite(y):
                self._x.append(float(x))
                self._y.append(float(y))
        else:
            self.close()

    def close(self):
        # A 1-point stroke carries no shape; drop it so it cannot inflate counts.
        if len(self._x) > 1:
            self.strokes.append((self._x, self._y))
        self._x = []
        self._y = []


# -----------------------------------------------------------------------------
# Preprocessing front ends
# -----------------------------------------------------------------------------

class KuruPreprocess:
    """kuru's own preprocessors, producing exactly what kuru's fusion expects."""

    name = 'kuru'

    def __init__(self):
        from kuru_method.asynchronous_stream.preprocessor import (
            IMUPreprocessor,
            UWBPreprocessor,
        )
        from kuru_method.asynchronous_stream.config import UWB_OFFSETS

        self._imu = IMUPreprocessor()
        self._uwb = UWBPreprocessor(offsets=UWB_OFFSETS)

    def imu(self, packet: dict) -> dict | None:
        quat = packet['quat']
        acc = packet['acc']
        clean = self._imu.process_sample(*quat, *acc)
        return {
            'quat': clean[0:4],
            'acc': clean[4:7],
            'force': packet.get('force', 0.0),
            'ts': packet.get('ts_hw'),
        }

    def uwb(self, packet: dict) -> tuple | None:
        _filtered, weights, ekf_dists = self._uwb.process(*packet['dists'])
        return ekf_dists, weights, packet.get('ts_hw')

    def feedback(self, engine, tag_pos):
        """
        Feed the EKF posterior back into kuru's anchor quality tracker.

        main_ekf.py:1520 does this with the tag position returned by
        process_uwb and the tilt-aware tag z. Skipping it would leave kuru's
        NLOS weighting blind and handicap it against its own baseline.
        """
        if tag_pos is None:
            return
        try:
            self._uwb.update_predictions(
                np.append(tag_pos, engine.tag_z_wb), engine.anchors
            )
        except Exception:
            pass


class OurPreprocess:
    """
    Our `preprocess/` stage, adapted to kuru's fusion input contract.

    The adaptation is a rename, not a conversion. Verified by reading both
    sides:

      * kuru's `IMUPreprocessor.process_sample()` (preprocessor.py:267-271)
        only normalises the quaternion; acceleration passes through untouched.
      * kuru's `IMUIntegrator.get_wb_acceleration()` (imu_integrator.py:169)
        expects raw body-frame linear accel and performs its own clamp,
        deadband, heading lock, rotation and lever-arm correction.
      * our `preprocess/imu.py:260` stores the unmodified hardware sample as
        `acc_sensor`.

    So `acc_sensor` -> `acc` is like-for-like. We deliberately do NOT pass
    `acc_board_tip` or any `acc_board_*` variant: those are already projected
    into the board frame and already tip-corrected, and handing them to kuru's
    integrator would double-apply both the projection and the lever arm.
    """

    name = 'ours'

    def __init__(self):
        from background.pipelines.config import cfg
        from background.pipelines.preprocess.contact import ContactStateDetector
        from background.pipelines.preprocess.imu import IMUPreprocessor
        from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor

        self._imu = IMUPreprocessor()
        self._contact = ContactStateDetector()
        offsets = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
        self._range = UWBRangePreprocessor(offsets=offsets)

        # Kept so the ours->ours path can reuse the identical preprocessed
        # events rather than running the stage twice with different state.
        self.last_imu_event: dict | None = None
        self.last_uwb_event: dict | None = None

    def imu(self, packet: dict) -> dict | None:
        preprocessed = self._imu.process_one(packet)
        if not preprocessed:
            self.last_imu_event = None
            return None

        with_contact = self._contact.process_one(preprocessed)
        self.last_imu_event = with_contact

        return {
            'quat': with_contact['quat'],
            'acc': with_contact['acc_sensor'],
            'force': with_contact.get('force', 0.0),
            'ts': with_contact.get('ts_hw'),
        }

    def uwb(self, packet: dict) -> tuple | None:
        ranged_events = list(self._range.feed([packet]))
        if not ranged_events:
            self.last_uwb_event = None
            return None

        ranged = ranged_events[-1]
        self.last_uwb_event = ranged

        # solver_dists is the path our own trilateration consumes (steps 1-6).
        # clean_dists adds display-oriented smoothing, which would hand kuru a
        # laggier signal than our own solver sees and bias the comparison.
        dists = ranged.get('solver_dists') or ranged.get('clean_dists')

        valid = ranged.get('valid_mask', (True,) * 4)
        outlier = ranged.get('outlier_flags', (False,) * 4)
        weights = tuple(
            1.0 if (valid[i] and not outlier[i]) else _FLAGGED_ANCHOR_WEIGHT
            for i in range(len(dists))
        )
        return tuple(dists), weights, ranged.get('ts_hw')

    def feedback(self, engine, tag_pos):
        # Our range stage has no posterior-feedback hook; nothing to do.
        return


# -----------------------------------------------------------------------------
# Fusion back ends
# -----------------------------------------------------------------------------

def run_kuru_fusion(packets: list[dict], preprocess, label: str, detail: str) -> RunResult:
    """
    Drive kuru's AsyncEKFFusionEngine.

    Mirrors batch_main_ekf_plot.py:76-120, including the pen-tip readout via
    `engine.tip_position` and (for the kuru front end) the posterior feedback
    into the anchor quality tracker that tracker.py:217-222 performs. Omitting
    that feedback would handicap kuru relative to its own baseline.
    """

    result = RunResult(label=label, detail=detail)

    try:
        from kuru_method.asynchronous_stream.ekf_fusion import AsyncEKFFusionEngine
        engine = AsyncEKFFusionEngine()
    except Exception as error:                     # pragma: no cover - import guard
        result.error = f'engine construction failed: {error}'
        return result

    collector = _StrokeCollector()

    for packet in packets:
        try:
            if packet['sensor'] == 'IMU':
                adapted = preprocess.imu(packet)
                if adapted is None or adapted['ts'] is None:
                    continue

                result.imu_samples += 1
                pos, _vel, is_writing = engine.process_imu(adapted)
                if pos is None:
                    continue

                if result.init_index is None:
                    result.init_index = result.imu_samples

                tip = engine.tip_position
                point = tip if tip is not None else pos
                collector.add(float(point[0]), float(point[1]), bool(is_writing))

            elif packet['sensor'] == 'UWB':
                adapted = preprocess.uwb(packet)
                if adapted is None:
                    continue

                dists, weights, ts = adapted
                result.uwb_samples += 1
                tag_pos, _vel, _accepted, _rejected = engine.process_uwb(
                    dists, weights, ts=ts
                )
                preprocess.feedback(engine, tag_pos)

        except Exception as error:                 # pragma: no cover - diagnostic
            result.error = f'{type(error).__name__}: {error}'
            break

    collector.close()
    result.strokes = collector.strokes
    return result


def run_our_fusion(packets: list[dict], preprocess, label: str, detail: str) -> RunResult:
    """
    Drive our ESKF + StrokeReconstructor.

    Mirrors the event ordering of reconstruct.py's `_run_live()`: IMU events go
    through fusion directly, UWB events go through trilateration and the
    position filter first. Strokes are taken from the reconstructor's closed
    output, which is the same geometry the visualizer draws.
    """

    result = RunResult(label=label, detail=detail)

    try:
        from background.pipelines.fusion.eskf import ESKF
        from background.pipelines.preprocess.uwb.position import UWBPositionFilter
        from background.pipelines.preprocess.uwb.trilateration import UWBSolver
        from background.pipelines.reconstruct import StrokeReconstructor

        fusion = ESKF()
        solver = UWBSolver()
        position_filter = UWBPositionFilter()
        reconstructor = StrokeReconstructor()
    except Exception as error:                     # pragma: no cover - import guard
        result.error = f'pipeline construction failed: {error}'
        return result

    kuru_front = preprocess.name == 'kuru'
    if kuru_front:
        # Our ESKF consumes the rich preprocessed event (board-frame tip accel,
        # contact substate, stillness). kuru's preprocessor emits none of that,
        # so configuration D cannot be driven end-to-end without reimplementing
        # our IMU stage on top of kuru's output -- which would no longer be
        # kuru's preprocessing. Reported rather than faked.
        result.error = (
            'not runnable: our ESKF requires board-frame tip acceleration and '
            'contact substate, which kuru preprocessing does not produce'
        )
        return result

    strokes: list = []

    def _collect(stroke: dict | None):
        if not stroke:
            return
        points = stroke.get('points') or []
        xs = [float(p[0]) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
        ys = [float(p[1]) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
        if len(xs) > 1:
            strokes.append((xs, ys))

    for packet in packets:
        try:
            if packet['sensor'] == 'IMU':
                adapted = preprocess.imu(packet)
                if adapted is None:
                    continue
                event = preprocess.last_imu_event
                if event is None:
                    continue

                result.imu_samples += 1
                fused = fusion.process_event(event)
                if fused:
                    if result.init_index is None:
                        result.init_index = result.imu_samples
                    _collect(reconstructor.process_event(fused))

            elif packet['sensor'] == 'UWB':
                adapted = preprocess.uwb(packet)
                if adapted is None:
                    continue
                ranged = preprocess.last_uwb_event
                if ranged is None:
                    continue

                result.uwb_samples += 1
                solved = solver.process_one(ranged)
                if not solved:
                    continue
                positioned = position_filter.process_one(solved)
                if not positioned:
                    continue
                fused = fusion.process_event(positioned)
                if fused:
                    _collect(reconstructor.process_event(fused))

        except Exception as error:                 # pragma: no cover - diagnostic
            result.error = f'{type(error).__name__}: {error}'
            break

    _collect(reconstructor.flush())
    result.strokes = strokes
    return result


# -----------------------------------------------------------------------------
# Reporting
# -----------------------------------------------------------------------------

def print_table(dataset: str, results: list[RunResult]):
    print()
    print('=' * 78)
    print(f'  cross-fusion  |  dataset: {dataset}')
    print('=' * 78)
    print(f'  {"cfg":<4}{"preprocess -> fusion":<26}{"strokes":>8}{"points":>8}'
          f'{"init":>7}{"ink m":>9}{"bbox m":>9}')
    print('  ' + '-' * 74)

    for item in results:
        if item.error:
            print(f'  {item.label:<4}{item.detail:<26}{"--":>8}{"--":>8}{"--":>7}'
                  f'{"--":>9}{"--":>9}')
            continue

        init = '-' if item.init_index is None else str(item.init_index)
        print(f'  {item.label:<4}{item.detail:<26}{len(item.strokes):>8}'
              f'{item.point_count:>8}{init:>7}'
              f'{item.ink_length_m():>9.2f}{item.bbox_diagonal_m():>9.3f}')

    print('  ' + '-' * 74)

    for item in results:
        if item.error:
            print(f'  {item.label}: {item.error}')
        elif item.init_index is None:
            print(f'  {item.label}: fusion never initialised '
                  f'({item.imu_samples} IMU / {item.uwb_samples} UWB samples consumed)')

    print()
    print('  Counts are descriptive: they detect a broken run, not a better one.')
    print('  A and C are controls -- if they do not match the known output for')
    print('  this dataset, the harness is wrong and B proves nothing.')
    print('=' * 78)


def render(dataset: str, results: list[RunResult], out_path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as error:
        print(f'  [plot] matplotlib unavailable ({error}); skipping PNG.')
        return False

    figure, axes = plt.subplots(2, 2, figsize=(15, 10))
    figure.suptitle(f'cross-fusion 2x2  |  {dataset}', fontsize=15)

    # Shared limits so the four panels are visually comparable at a glance.
    all_x = [x for item in results for xs, _ in item.strokes for x in xs]
    all_y = [y for item in results for _, ys in item.strokes for y in ys]
    if all_x and all_y:
        pad = 0.05
        xlim = (min(all_x) - pad, max(all_x) + pad)
        ylim = (min(all_y) - pad, max(all_y) + pad)
    else:
        xlim = ylim = None

    for axis, item in zip(axes.flat, results):
        axis.set_title(f'{item.label}.  {item.detail}', fontsize=11)
        axis.set_aspect('equal', adjustable='box')
        axis.grid(True, alpha=0.25, linewidth=0.5)

        if item.error:
            axis.text(0.5, 0.5, item.error, ha='center', va='center',
                      transform=axis.transAxes, fontsize=8, wrap=True, color='#b00020')
            axis.set_xticks([])
            axis.set_yticks([])
            continue

        for xs, ys in item.strokes:
            axis.plot(xs, ys, linewidth=1.4, color='#111111', solid_capstyle='round')

        if not item.strokes:
            axis.text(0.5, 0.5, 'no strokes', ha='center', va='center',
                      transform=axis.transAxes, fontsize=11, color='#b00020')

        if xlim and ylim:
            axis.set_xlim(*xlim)
            axis.set_ylim(*ylim)

        axis.set_xlabel(f'{len(item.strokes)} strokes / {item.point_count} pts', fontsize=9)

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out_path, dpi=140)
    plt.close(figure)
    print(f'  [plot] wrote {out_path}')
    return True


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Swap preprocessing between our pipeline and kuru, holding fusion fixed.'
    )
    parser.add_argument('dataset', help='dataset name (e.g. abcde_1) or path to a CSV')
    parser.add_argument('--out', default=None, help='output PNG path')
    args = parser.parse_args(argv)

    try:
        csv_path = resolve_dataset(args.dataset)
    except FileNotFoundError as error:
        print(f'  {error}')
        return 2

    packets = read_packets(csv_path)
    imu_count = sum(1 for p in packets if p['sensor'] == 'IMU')
    uwb_count = sum(1 for p in packets if p['sensor'] == 'UWB')
    print(f'  [read] {csv_path.name}: {len(packets)} packets '
          f'({imu_count} IMU, {uwb_count} UWB)')

    if not packets:
        print('  no parsable packets; nothing to do.')
        return 2

    results = [
        run_kuru_fusion(packets, KuruPreprocess(), 'A', 'kuru -> kuru  (control)'),
        run_kuru_fusion(packets, OurPreprocess(), 'B', 'ours -> kuru  (TEST)'),
        run_our_fusion(packets, OurPreprocess(), 'C', 'ours -> ours  (control)'),
        run_our_fusion(packets, KuruPreprocess(), 'D', 'kuru -> ours'),
    ]

    print_table(csv_path.stem, results)

    out_path = Path(args.out) if args.out else (
        _REPO_ROOT / 'output' / f'cross_fusion_{csv_path.stem}.png'
    )
    render(csv_path.stem, results, out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
