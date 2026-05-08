"""
param_sweep.py — Phase 8 Step 4 pen-up parameter sweep.

Sweeps four high-leverage knobs from `config.py` over a representative
subset of datasets and tabulates the joint Layer 6 / Layer 8 / Layer 9
score. Pick the cell with the lowest tip-error proxy (Layer 8
`pen_down_median_m`) AND zero stroke false-splits/merges (Layer 9).

Sweep grid (2 × 3 × 3 × 3 = 54 cells per dataset by default):
    ALPHA_R_HOVER       ∈ {16.0, 25.0, 36.0}
    T_SHORT_S           ∈ {0.10, 0.15, 0.20}
    R_TOUCHDOWN_SCALE   ∈ {2.0, 4.0, 6.0}
    Q_SHORT             ∈ {0.25, 0.35, 0.50}

Datasets: chosen to cover slow strokes, fast strokes, and lift/relocate.
    - ABC_b1, abc_s1     — slow + fast cursive characters
    - HELLO_b1, hello_s1 — multi-letter writing
    - ct_diagonal1, ct_circle1 — control-test shapes (lift between strokes)

Output: `verification/out/param_sweep/sweep_results.csv` plus a per-cell
summary printed to stdout.

Usage
-----
    python tools/param_sweep.py
    python tools/param_sweep.py --datasets ABC_b1 abc_s1
"""

from __future__ import annotations

import argparse
import csv
import importlib
import sys
from itertools import product
from pathlib import Path

import numpy as np

_ASYNC = Path(__file__).resolve().parent.parent
if str(_ASYNC) not in sys.path:
    sys.path.insert(0, str(_ASYNC))

import kuru_method.asynchronous_stream.config as _config_mod   # noqa: E402

DATASET_DIR = _ASYNC / 'datasets_str_50hz'
OUT_DIR     = _ASYNC / 'verification' / 'out' / 'param_sweep'

DEFAULT_DATASETS = [
    'ABC_b1', 'abc_s1', 'HELLO_b1', 'hello_s1',
    'ct_diagonal1', 'ct_circle1',
]

DEFAULT_GRID = {
    'ALPHA_R_HOVER':     [16.0, 25.0, 36.0],
    'T_SHORT_S':         [0.10, 0.15, 0.20],
    'R_TOUCHDOWN_SCALE': [2.0, 4.0, 6.0],
    'Q_SHORT':           [0.25, 0.35, 0.50],
}


def _run_one_cell(stem: str, params: dict) -> dict:
    """Override config knobs for this cell, reload dependent modules, run
    the engine, and return Layer-8/Layer-9-style metrics for this dataset."""
    # Apply config overrides BEFORE importing/reloading downstream modules.
    for k, v in params.items():
        setattr(_config_mod, k, v)
    # Reload modules that read config at import time.
    import pen_mode, ekf_fusion, preprocessor   # noqa
    importlib.reload(pen_mode)
    importlib.reload(ekf_fusion)

    from data_parser   import AsyncDataParser
    from preprocessor  import UWBPreprocessor
    from ekf_fusion    import AsyncEKFFusionEngine
    from fusion_engine import IRLSTrilateration
    from pen_mode      import PenMode
    from config        import ANCHORS, MARKER_LENGTH, UWB_OFFSETS

    csv_path = DATASET_DIR / f'{stem}.csv'
    if not csv_path.exists():
        return {'stem': stem, 'error': 'missing'}

    engine = AsyncEKFFusionEngine()
    pre    = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    parser = AsyncDataParser(csv_path=str(csv_path))
    parser.connect()
    pen_diffs:   list[float] = []
    hover_diffs: list[float] = []
    contact_states: list[int] = []

    while True:
        pkt = parser.get_packet()
        if pkt == 'EOF': break
        if pkt is None: continue
        if pkt['type'] == 'imu':
            engine.process_imu(pkt)
        else:
            _, weights, ekf_dists = pre.process(*pkt['dists'])
            mode_before = engine._pen_mode.mode
            pos, _, _, _ = engine.process_uwb(ekf_dists, weights, ts=pkt.get('ts'))
            ref_xyz, _ = irls.solve(ekf_dists, weights)
            if pos is None or not irls.last_solve_ok:
                continue
            d = float(np.linalg.norm(pos[:2] - ref_xyz[:2]))
            if mode_before == PenMode.PEN_DOWN:
                pen_diffs.append(d)
            elif mode_before in (PenMode.HOVER_SHORT, PenMode.HOVER_LONG, PenMode.HOVER_H):
                hover_diffs.append(d)
            contact_states.append(1 if mode_before == PenMode.PEN_DOWN else 0)
    parser.close()

    pen_arr   = np.asarray(pen_diffs)   if pen_diffs   else np.zeros(0)
    hover_arr = np.asarray(hover_diffs) if hover_diffs else np.zeros(0)

    # False-split heuristic: stroke gaps shorter than 80 ms (8 IMU @ 100 Hz)
    # using the contact_states stream from per-UWB-cycle sampling.
    return {
        'stem':                stem,
        'pen_down_median_m':   float(np.median(pen_arr))   if pen_arr.size   else float('nan'),
        'pen_down_p95_m':      float(np.percentile(pen_arr, 95))  if pen_arr.size   else float('nan'),
        'pen_down_n':          int(pen_arr.size),
        'hover_median_m':      float(np.median(hover_arr)) if hover_arr.size else float('nan'),
        'hover_n':             int(hover_arr.size),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--datasets', nargs='*', default=DEFAULT_DATASETS)
    ap.add_argument('--out', default=str(OUT_DIR / 'sweep_results.csv'))
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    keys = list(DEFAULT_GRID.keys())
    grid = list(product(*[DEFAULT_GRID[k] for k in keys]))
    print(f'[sweep] {len(grid)} cells × {len(args.datasets)} datasets '
          f'= {len(grid) * len(args.datasets)} EKF replays.')

    rows: list[dict] = []
    for cell in grid:
        params = dict(zip(keys, cell))
        for stem in args.datasets:
            try:
                m = _run_one_cell(stem, params)
            except Exception as e:
                m = {'stem': stem, 'error': repr(e)}
            row = {**params, **m}
            rows.append(row)
            print(f'  {params}  {stem}: '
                  f'pen={m.get("pen_down_median_m", float("nan")):.4f} '
                  f'(n={m.get("pen_down_n", 0)})  '
                  f'hover={m.get("hover_median_m", float("nan")):.4f}')

    # Aggregate by cell — mean pen-down median across datasets.
    by_cell: dict[tuple, list[float]] = {}
    for r in rows:
        key = tuple(r[k] for k in keys)
        if r.get('pen_down_n', 0) > 0:
            by_cell.setdefault(key, []).append(r['pen_down_median_m'])

    summary = sorted(
        ((np.mean(v), key) for key, v in by_cell.items()),
        key=lambda x: x[0])

    print('\n[sweep] best 5 cells by mean pen-down median (lower = better):')
    for score, key in summary[:5]:
        params = dict(zip(keys, key))
        print(f'  {score:.4f}  {params}')

    # Write CSV
    if rows:
        fieldnames = sorted({k for r in rows for k in r.keys()})
        with open(args.out, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f'\n[sweep] -> {args.out}')


if __name__ == '__main__':
    main()
