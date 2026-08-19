"""
Sweep the two measured weaknesses of the OG ESKF configuration.

Both target the same symptom -- truncated letterforms -- from opposite ends of
the prediction path, and both are live in the OG config (shape_mode off,
ink_from_dead_reckoner off), unlike the knobs that turned out to be bypassed.

1. Velocity cap.  `active_vel_cap_ms = 0.36` clips the nominal velocity norm
   during ink (eskf.py:368).  Measured in-stroke speed on this branch:

       abcdefgh_low  p90 0.330  p99 0.360  max 0.360   6.3% of samples at cap
       abc_1         p90 0.289  p99 0.360  max 0.360   4.0%
       b_lg          p90 0.313  p99 0.360  max 0.360   1.5%

   Max equals the cap exactly on every dataset, so the distribution is being
   truncated rather than merely bounded.  kuru puts natural pen-on-board writing
   at 0.5-1.0 m/s (ekf_fusion.py MAX_WRITING_SPEED), so 0.36 sits below the
   physical range and clips the ballistic part of each stroke.  Clipped velocity
   integrates straight into missing displacement.

2. The bias-rejection double-pay.  The filter removes accelerometer bias twice:
   through the `accel_bias` error state (norm-clamped, frozen during ink) and
   through the 0.75 Hz high-pass.  `detail_weight = 0.55` takes the majority of
   the prediction from the high-pass path, whose ~1.3 s time constant is about
   one stroke long, so it also removes stroke-scale motion.  That cost is real
   and the benefit is redundant with the bias state.

   Two independent ways to unwind it: lower the cutoff so the filter passes
   stroke-scale content, or lower the blend weight so it leans on the bias state
   instead.

Configurations:

    V0  shipped                      cap 0.36, hpf 0.75, weight 0.55
    V1  cap 0.50
    V2  cap 0.80                     kuru's lower bound
    V3  cap 1.20
    H1  hpf cutoff 0.20              tau 1.3 s -> 5 s
    H2  detail_weight 0.25
    H3  hpf 0.20 + weight 0.25
    C1  cap 0.80 + hpf 0.20 + weight 0.25

Counts are descriptive only.  Extent has repeatedly moved the right way while
the eye check moved the wrong way on this problem -- a noisier integrand also
covers more ground -- so the PNG is the deliverable.

Usage:
    python -m background.pipelines.tools.og_eskf_sweep abcdefgh_low
    python -m background.pipelines.tools.og_eskf_sweep --all
"""

from __future__ import annotations

import argparse
import math
import sys
from contextlib import contextmanager
from pathlib import Path

from background.pipelines.cleaner.unpacker import parse_packet_line
from background.pipelines.config import cfg

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DATASET_DIR = _REPO_ROOT / 'test' / '_datasets'

# Representative spread: cursive letters, block letters, one large glyph,
# connected words, long text, and a fast sharp-cornered shape.
_DEFAULT_DATASETS = (
    'abcdefgh_low', 'abc_1', 'b_lg', 'hello_lg', 'qckbrwnfx_1', 'triangle_vfast',
)

# (label, detail, eskf overrides, imu overrides)
_CONFIGS = (
    ('V0', 'shipped', {}, {}),
    ('V1', 'vel cap 0.50', {'active_vel_cap_ms': 0.50}, {}),
    ('V2', 'vel cap 0.80', {'active_vel_cap_ms': 0.80}, {}),
    ('V3', 'vel cap 1.20', {'active_vel_cap_ms': 1.20}, {}),
    ('H1', 'hpf 0.20 Hz', {}, {'hpf_cutoff_hz': 0.20}),
    ('H2', 'detail_weight 0.25', {}, {}),
    ('H3', 'hpf 0.20 + dw 0.25', {}, {'hpf_cutoff_hz': 0.20}),
    ('C1', 'cap 0.80 + hpf + dw', {'active_vel_cap_ms': 0.80}, {'hpf_cutoff_hz': 0.20}),
)

# detail_weight lives on the nested dead_reckoner config, so it is handled
# separately from the flat eskf overrides above.
_DETAIL_WEIGHT = {'H2': 0.25, 'H3': 0.25, 'C1': 0.25}


@contextmanager
def overrides(eskf_values: dict, imu_values: dict, detail_weight: float | None):
    """Temporarily apply config overrides, restoring every one afterwards."""

    eskf_cfg, imu_cfg, dr_cfg = cfg.fusion_eskf, cfg.imu, cfg.fusion_eskf.dead_reckoner
    saved = [
        (eskf_cfg, k, getattr(eskf_cfg, k)) for k in eskf_values
    ] + [
        (imu_cfg, k, getattr(imu_cfg, k)) for k in imu_values
    ]
    if detail_weight is not None:
        saved.append((dr_cfg, 'detail_weight', dr_cfg.detail_weight))

    for k, v in eskf_values.items():
        object.__setattr__(eskf_cfg, k, v)
    for k, v in imu_values.items():
        object.__setattr__(imu_cfg, k, v)
    if detail_weight is not None:
        object.__setattr__(dr_cfg, 'detail_weight', detail_weight)

    try:
        yield
    finally:
        for obj, key, value in saved:
            object.__setattr__(obj, key, value)


def read_packets(csv_path: Path) -> list[dict]:
    packets = []
    with open(csv_path, 'r', encoding='utf-8', errors='replace') as handle:
        for line in handle:
            packet = parse_packet_line(line.strip())
            if packet is not None:
                packets.append(packet)
    return packets


def run_pipeline(packets: list[dict]) -> tuple[list, int]:
    """
    Drive the full OG pipeline and return (strokes, samples_at_velocity_cap).

    Mirrors reconstruct.py's `_run_live()` event ordering: IMU events go through
    fusion directly, UWB events through trilateration and the position filter
    first.  Constructed fresh per call so no filter state leaks between configs.
    """

    import numpy as np
    from background.pipelines.fusion.eskf import ESKF
    from background.pipelines.preprocess.contact import ContactStateDetector
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.reconstruct import StrokeReconstructor

    fusion = ESKF()
    imu_prep = IMUPreprocessor()
    contact = ContactStateDetector()
    range_prep = UWBRangePreprocessor(
        offsets=getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    )
    solver = UWBSolver()
    position_filter = UWBPositionFilter()
    reconstructor = StrokeReconstructor()

    cap = cfg.fusion_eskf.active_vel_cap_ms
    strokes: list = []
    at_cap = 0

    def collect(stroke):
        if not stroke:
            return
        points = stroke.get('points') or []
        xs = [float(p[0]) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
        ys = [float(p[1]) for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
        if len(xs) > 1:
            strokes.append((xs, ys))

    for packet in packets:
        if packet['sensor'] == 'IMU':
            preprocessed = imu_prep.process_one(packet)
            if not preprocessed:
                continue
            event = contact.process_one(preprocessed)
            fused = fusion.process_event(event)
            if event.get('stroke_active'):
                if float(np.linalg.norm(fusion.state.velocity)) >= cap * 0.999:
                    at_cap += 1
            if fused:
                collect(reconstructor.process_event(fused))
        else:
            for ranged in range_prep.feed([packet]):
                solved = solver.process_one(ranged)
                if not solved:
                    continue
                positioned = position_filter.process_one(solved)
                if not positioned:
                    continue
                fused = fusion.process_event(positioned)
                if fused:
                    collect(reconstructor.process_event(fused))

    collect(reconstructor.flush())
    return strokes, at_cap


def ink_length(strokes) -> float:
    return sum(
        math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        for xs, ys in strokes for i in range(1, len(xs))
    )


def bbox_diagonal(strokes) -> float:
    xs = [x for sx, _ in strokes for x in sx]
    ys = [y for _, sy in strokes for y in sy]
    if len(xs) < 2:
        return 0.0
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def sweep_dataset(name: str) -> int:
    csv_path = _DATASET_DIR / f'{name}.csv'
    if not csv_path.is_file():
        print(f'  dataset not found: {csv_path}')
        return 2

    packets = read_packets(csv_path)
    results = []

    for label, detail, eskf_values, imu_values in _CONFIGS:
        with overrides(eskf_values, imu_values, _DETAIL_WEIGHT.get(label)):
            strokes, at_cap = run_pipeline(packets)
        results.append((label, detail, strokes, at_cap))

    print()
    print('=' * 78)
    print(f'  OG ESKF sweep  |  {name}')
    print('=' * 78)
    print(f'  {"cfg":<4}{"change":<24}{"strokes":>8}{"points":>8}'
          f'{"ink m":>9}{"bbox m":>9}{"at cap":>9}')
    print('  ' + '-' * 74)
    for label, detail, strokes, at_cap in results:
        points = sum(len(xs) for xs, _ in strokes)
        print(f'  {label:<4}{detail:<24}{len(strokes):>8}{points:>8}'
              f'{ink_length(strokes):>9.2f}{bbox_diagonal(strokes):>9.3f}{at_cap:>9}')
    print('  ' + '-' * 74)
    print('  V0 is the shipped OG config; compare everything against it.')
    print('  "at cap" counts in-stroke samples pinned at the velocity cap.')
    print('=' * 78)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as error:
        print(f'  [plot] matplotlib unavailable ({error})')
        return 0

    all_x = [x for _, _, s, _ in results for xs, _ in s for x in xs]
    all_y = [y for _, _, s, _ in results for _, ys in s for y in ys]
    lim = None
    if all_x and all_y:
        pad = 0.05
        lim = ((min(all_x) - pad, max(all_x) + pad), (min(all_y) - pad, max(all_y) + pad))

    columns = 4
    rows = math.ceil(len(results) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(4.0 * columns, 3.5 * rows), squeeze=False)
    figure.suptitle(f'OG ESKF sweep  |  {name}', fontsize=14)

    for axis in axes.flat[len(results):]:
        axis.axis('off')

    for axis, (label, detail, strokes, _) in zip(axes.flat, results):
        for xs, ys in strokes:
            axis.plot(xs, ys, lw=1.4, color='#111111', solid_capstyle='round')
        axis.set_aspect('equal')
        axis.grid(alpha=0.25, lw=0.5)
        title = f'{label}.  {detail}'
        axis.set_title(title + ('  (shipped)' if label == 'V0' else ''), fontsize=10)
        points = sum(len(xs) for xs, _ in strokes)
        axis.set_xlabel(f'{len(strokes)} strokes / {points} pts / {ink_length(strokes):.2f} m',
                        fontsize=8)
        if lim:
            axis.set_xlim(*lim[0])
            axis.set_ylim(*lim[1])

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    out = _REPO_ROOT / 'output' / f'og_eskf_sweep_{name}.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=135)
    plt.close(figure)
    print(f'  [plot] wrote {out}')
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Sweep the OG ESKF weak points.')
    parser.add_argument('dataset', nargs='?', help='dataset name')
    parser.add_argument('--all', action='store_true', help='run the default six datasets')
    args = parser.parse_args(argv)

    if args.all:
        for name in _DEFAULT_DATASETS:
            sweep_dataset(name)
        return 0
    if not args.dataset:
        parser.error('give a dataset name or --all')
    return sweep_dataset(args.dataset)


if __name__ == '__main__':
    sys.exit(main())
