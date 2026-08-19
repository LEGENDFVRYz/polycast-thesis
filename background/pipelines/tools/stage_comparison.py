"""
Side-by-side comparison of the fusion configurations this pipeline has shipped.

Each stage below is a configuration of the *current* code, not a checkout of an
old commit. That is deliberate and it is also what makes the comparison fair:
the flags fully determine which paths execute, so reproducing a stage by
configuration isolates the design decision from every unrelated change that
happened to land in the same period. Checking out `e50b74b` would also drag in a
different range pre-filter, a different renderer and a different contact
detector, and the comparison would no longer be about fusion.

The stages, in the order they were adopted:

    OG          blended ESKF. UWB corrects continuously through the Kalman
                update while ink is being drawn. Predates `shape_mode`
                entirely - the flag was introduced in b7b6aec.

    SHAPE       `shape_mode` + `ink_from_dead_reckoner`, both introduced
                together in b7b6aec and the configuration the branch ships
                today. UWB is severed during ink and places the stroke once at
                pen-down; the letter itself is plain double integration of
                acc_board_tip.

    SHAPE+PP    the same, with the pen-up correction stages enabled
                (velocity detrend, trace filter). This is what "final output
                looks good" referred to - the geometry is repaired after the
                pen lifts.

    LIVE-ONLY   `shape_mode` with every pen-up stage disabled. What the
                pipeline looks like when nothing may wait for the stroke to
                close.

What is measured, and why these three:

    end gap     distance from the last ink sample to the lever-arm corrected
                UWB tip. Measured against `z_uwb_tip`, never `uwb_x`/`uwb_y` -
                the latter is the raw tag position, 21.5 cm behind the tip, and
                comparing against it reports pen tilt as if it were drift.

    drift growth  end gap minus start gap. Positive means error accumulated
                *during* the stroke, which is the failure that scales with
                pen-down duration rather than with letter complexity.

    turning     total absolute heading change. The jitter proxy: a clean letter
                needs a few hundred degrees and anything far above that is
                dither reading as direction reversal.

No legibility score is computed. Every proxy tried on this problem rewards a
stalled pen, so extent is reported as a guard rail and the grid decides.

Usage:
    python -m background.pipelines.tools.stage_comparison abc_1
    python -m background.pipelines.tools.stage_comparison --all
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
from background.pipelines.tools.board_overlay import OverlayLayer, render_overlay
from background.pipelines.tools.cross_fusion import (
    OurPreprocess,
    _REPO_ROOT,
    read_packets,
    resolve_dataset,
)


@dataclass(frozen=True)
class Stage:
    """One historical configuration, expressed as flag overrides."""

    key: str
    label: str
    colour: str
    fusion: dict = field(default_factory=dict)
    postprocess_enabled: bool = False
    trace_filter_enabled: bool = False
    velocity_detrend_enabled: bool = False


STAGES = [
    Stage(
        key='og',
        label='OG blended ESKF',
        colour='#1f77d0',
        fusion={'shape_mode': False, 'ink_from_dead_reckoner': False},
    ),
    Stage(
        key='shape',
        label='SHAPE (UWB place + IMU path)',
        colour='#2ca02c',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True},
    ),
    Stage(
        key='shape_pp',
        label='SHAPE + pen-up correction',
        colour='#ff7f0e',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True},
        postprocess_enabled=True,
        trace_filter_enabled=True,
        velocity_detrend_enabled=True,
    ),
    Stage(
        key='live',
        label='LIVE-ONLY (no pen-up stage)',
        colour='#d62728',
        fusion={'shape_mode': True, 'ink_from_dead_reckoner': True},
    ),
]


@dataclass
class StrokeTrace:
    """One stroke as produced by one stage."""

    points: np.ndarray
    uwb_tip: np.ndarray
    duration_s: float

    @property
    def start_gap_cm(self) -> float:
        return float(np.linalg.norm(self.points[0] - self.uwb_tip[0]) * 100.0)

    @property
    def end_gap_cm(self) -> float:
        return float(np.linalg.norm(self.points[-1] - self.uwb_tip[-1]) * 100.0)

    @property
    def drift_growth_cm(self) -> float:
        """How much the gap widened over the stroke."""

        return self.end_gap_cm - self.start_gap_cm

    @property
    def extent_cm(self) -> float:
        if len(self.points) < 2:
            return 0.0
        span = self.points.max(axis=0) - self.points.min(axis=0)
        return float(np.hypot(span[0], span[1]) * 100.0)

    @property
    def turning_deg(self) -> float:
        if len(self.points) < 3:
            return 0.0
        deltas = np.diff(self.points, axis=0)
        lengths = np.linalg.norm(deltas, axis=1)
        keep = lengths > 5e-4
        deltas, lengths = deltas[keep], lengths[keep]
        if len(deltas) < 2:
            return 0.0
        units = deltas / lengths[:, None]
        cosines = np.clip((units[:-1] * units[1:]).sum(axis=1), -1.0, 1.0)
        return float(np.degrees(np.arccos(cosines)).sum())


def _override(stage: Stage):
    """
    Apply a stage's flags to the frozen config, returning a restore callback.

    `object.__setattr__` is the only way past the frozen dataclass, and every
    pipeline module reads `cfg` at call time, so the override takes effect
    without reimporting anything.
    """

    saved = {
        'fusion_eskf': cfg.fusion_eskf,
        'postprocess': cfg.postprocess,
        'trace_filter': cfg.trace_filter,
    }
    detrend = getattr(cfg, 'velocity_detrend', None)
    if detrend is not None:
        saved['velocity_detrend'] = detrend

    object.__setattr__(cfg, 'fusion_eskf', dataclasses.replace(cfg.fusion_eskf, **stage.fusion))
    object.__setattr__(
        cfg, 'postprocess',
        dataclasses.replace(cfg.postprocess, enabled=stage.postprocess_enabled),
    )
    object.__setattr__(
        cfg, 'trace_filter',
        dataclasses.replace(cfg.trace_filter, enabled=stage.trace_filter_enabled),
    )
    if detrend is not None:
        object.__setattr__(
            cfg, 'velocity_detrend',
            dataclasses.replace(detrend, enabled=stage.velocity_detrend_enabled),
        )

    def restore():
        for name, value in saved.items():
            object.__setattr__(cfg, name, value)

    return restore


def run_stage(packets: list[dict], stage: Stage) -> list[StrokeTrace]:
    """
    Drive the whole pipeline once under one stage's configuration.

    Strokes are taken from the reconstructor rather than from the raw fused
    stream, so the pen-up stages actually appear in the output - reading
    `fused_x` directly would compare the stages before their corrections ran.
    """

    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.reconstruct import StrokeReconstructor

    restore = _override(stage)
    try:
        preprocess = OurPreprocess()
        fusion = ESKF()
        solver = UWBSolver()
        position_filter = UWBPositionFilter()
        reconstructor = StrokeReconstructor()

        closed: list[dict] = []
        # UWB tip per timestamp, so a closed stroke can be scored against the
        # measurement that was current when each sample was emitted.
        tip_by_ts: dict[int, tuple] = {}
        latest_tip: tuple | None = None

        for packet in packets:
            if packet['sensor'] == 'UWB':
                preprocess.uwb(packet)
                ranged = preprocess.last_uwb_event
                if ranged is None:
                    continue
                solved = solver.process_one(ranged)
                if not solved:
                    continue
                positioned = position_filter.process_one(solved)
                if not positioned:
                    continue
                fused = fusion.process_event(positioned)
                if fused:
                    tip = (fused.get('eskf') or {}).get('z_uwb_tip')
                    if tip and all(v is not None for v in tip[:2]):
                        latest_tip = (float(tip[0]), float(tip[1]))
                    stroke = reconstructor.process_event(fused)
                    if stroke:
                        closed.append(stroke)
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

            if latest_tip is not None:
                tip_by_ts[int(fused['ts_hw'])] = latest_tip

            stroke = reconstructor.process_event(fused)
            if stroke:
                closed.append(stroke)

        tail = reconstructor.flush()
        if tail:
            closed.append(tail)

        traces: list[StrokeTrace] = []
        for stroke in closed:
            points = stroke.get('points') or []
            if len(points) < 25:
                continue

            # Point tuples are (x, y, ts) out of the reconstructor but only
            # (x, y) once the causal trace filter has run - it emits filtered
            # coordinates without carrying the timestamp through. So a stroke
            # that went through the pen-up filter cannot be joined to the UWB
            # tip by timestamp, and is matched by sample position instead.
            has_timestamps = len(points[0]) >= 3

            if has_timestamps:
                xy, tips = [], []
                for point in points:
                    tip = tip_by_ts.get(int(point[2]))
                    if tip is None:
                        continue
                    xy.append((float(point[0]), float(point[1])))
                    tips.append(tip)
                duration = (points[-1][2] - points[0][2]) / 1_000_000.0
            else:
                # Scoped to this stroke's own time span - the map covers the
                # whole recording, and resampling against all of it would align
                # the stroke to tips from other strokes.
                start_ts = int(stroke.get('start_ts', 0))
                end_ts = int(stroke.get('end_ts', 0))
                ordered = [
                    tip_by_ts[key] for key in sorted(tip_by_ts)
                    if start_ts <= key <= end_ts
                ]
                if not ordered:
                    continue
                xy = [(float(p[0]), float(p[1])) for p in points]
                # Resample the tip series onto the filtered point count so the
                # two sequences stay aligned end to end despite decimation.
                index = np.linspace(0, len(ordered) - 1, len(xy)).round().astype(int)
                tips = [ordered[i] for i in index]
                duration = float(stroke.get('end_ts', 0) - stroke.get('start_ts', 0)) / 1_000_000.0

            if len(xy) < 25:
                continue
            traces.append(
                StrokeTrace(
                    points=np.asarray(xy, dtype=float),
                    uwb_tip=np.asarray(tips, dtype=float),
                    duration_s=float(duration),
                )
            )
        return traces

    finally:
        restore()


def compare(dataset: str) -> dict[str, list[StrokeTrace]]:
    """Run every stage over one recording."""

    packets = read_packets(resolve_dataset(dataset))
    return {stage.key: run_stage(packets, stage) for stage in STAGES}


def print_table(dataset: str, results: dict[str, list[StrokeTrace]]) -> None:
    print()
    print('=' * 96)
    print(f'  pipeline stage comparison  |  {dataset}')
    print('=' * 96)
    print(f'  {"stage":<32}{"strokes":>9}{"end gap":>11}{"drift growth":>15}'
          f'{"turning":>11}{"extent":>10}')
    print('  ' + '-' * 92)

    for stage in STAGES:
        traces = results.get(stage.key) or []
        if not traces:
            print(f'  {stage.label:<32}{"-":>9}')
            continue
        print(f'  {stage.label:<32}{len(traces):>9}'
              f'{np.mean([t.end_gap_cm for t in traces]):>9.1f}cm'
              f'{np.mean([t.drift_growth_cm for t in traces]):>+13.1f}cm'
              f'{np.mean([t.turning_deg for t in traces]):>10.0f}d'
              f'{np.mean([t.extent_cm for t in traces]):>8.1f}cm')

    print('  ' + '-' * 92)
    print('  end gap      = last ink sample to the lever-arm corrected UWB tip.')
    print('  drift growth = end gap minus start gap. Positive means error accumulated')
    print('                 during the stroke - the failure that scales with duration.')
    print('  turning      = total heading change; the jitter proxy.')
    print('  extent       = bounding-box diagonal, a guard rail rather than a score.')
    print('=' * 96)


def print_duration_split(results_by_dataset: dict[str, dict[str, list[StrokeTrace]]]) -> None:
    """Short versus long strokes, which is where the stages actually differ."""

    print()
    print('=' * 96)
    print('  drift growth by stroke duration  |  the long-stroke weakness, isolated')
    print('=' * 96)
    print(f'  {"stage":<32}{"short <1.5s":>16}{"long >=1.5s":>16}{"degradation":>16}')
    print('  ' + '-' * 92)

    for stage in STAGES:
        short, long = [], []
        for results in results_by_dataset.values():
            for trace in results.get(stage.key) or []:
                (short if trace.duration_s < 1.5 else long).append(trace.drift_growth_cm)

        if not short and not long:
            continue
        short_mean = float(np.mean(short)) if short else float('nan')
        long_mean = float(np.mean(long)) if long else float('nan')
        delta = long_mean - short_mean if short and long else float('nan')
        print(f'  {stage.label:<32}{short_mean:>14.1f}cm{long_mean:>14.1f}cm{delta:>+14.1f}cm')

    print('  ' + '-' * 92)
    print('  degradation = long-stroke drift minus short-stroke drift. A large positive')
    print('  value is the signature of unbounded in-stroke integration.')
    print('=' * 96)


def render(dataset: str, results: dict[str, list[StrokeTrace]], out_path: Path) -> bool:
    """
    Overlay every stage's ink on one cropped, board-coordinate axis.

    Overlaying rather than tiling is the point: separate panels show what each
    stage produced, but only a shared axis answers whether two stages disagree
    about a letter's shape or merely about where it sits. On abcdefgh_low they
    place every letter identically and differ entirely within the letters.
    """

    layers = []
    for stage in STAGES:
        traces = results.get(stage.key) or []
        if not traces:
            continue
        layers.append(OverlayLayer(
            label=stage.label,
            colour=stage.colour,
            strokes=[trace.points for trace in traces],
            summary=(
                f'gap {np.mean([t.end_gap_cm for t in traces]):.1f} cm, '
                f'drift {np.mean([t.drift_growth_cm for t in traces]):+.1f} cm, '
                f'turning {np.mean([t.turning_deg for t in traces]):.0f} deg'
            ),
            # Turning is the noisiness measure here, so the jitteriest stage
            # ends up underneath instead of burying the others.
            weight=float(np.mean([t.turning_deg for t in traces])),
        ))

    return render_overlay(
        layers=layers,
        title=f'pipeline stage comparison  |  {dataset}',
        subtitle='same recording through each configuration the pipeline has shipped',
        out_path=out_path,
    )


_DEFAULT_SET = ['abc_1', 'square_1', 'triangle_1', 'hello_lg']


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Compare pipeline fusion stages.')
    parser.add_argument('dataset', nargs='?', help='dataset name, or omit with --all')
    parser.add_argument('--all', action='store_true', help='run the default set')
    parser.add_argument('--out', default=None, help='output PNG path')
    args = parser.parse_args(argv)

    if not args.dataset and not args.all:
        parser.error('give a dataset name or --all')

    names = _DEFAULT_SET if args.all else [args.dataset]

    everything: dict[str, dict[str, list[StrokeTrace]]] = {}
    for name in names:
        try:
            results = compare(name)
        except FileNotFoundError as error:
            print(f'  [skip] {name}: {error}')
            continue
        everything[name] = results
        print_table(name, results)

        out_path = Path(args.out) if args.out and len(names) == 1 else (
            _REPO_ROOT / 'output' / f'stage_comparison_{name}.png'
        )
        render(name, results, out_path)

    if len(everything) > 1:
        print_duration_split(everything)

    return 0 if everything else 2


if __name__ == '__main__':
    sys.exit(main())
