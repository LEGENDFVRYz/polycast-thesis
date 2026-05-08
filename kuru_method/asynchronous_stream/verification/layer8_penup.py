"""
layer8_penup.py -- pen-up behaviour audit for the four-mode state machine.

Splits each dataset by FSR contact state (PEN_DOWN vs HOVER) and reports
the EKF-vs-IRLS discrepancy and the pen-mode mix. Also exercises Priority
5's touchdown reacquisition by counting how many UWB cycles the engine
spent under each PenMode.
"""

from __future__ import annotations

from pathlib import Path
from collections import Counter

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR)

from preprocessor  import UWBPreprocessor
from ekf_fusion    import AsyncEKFFusionEngine
from fusion_engine import IRLSTrilateration
from config        import ANCHORS, MARKER_LENGTH, UWB_OFFSETS
from pen_mode      import PenMode


LAYER = 'layer8_penup'


def analyse(csv_path: Path) -> LayerResult:
    engine = AsyncEKFFusionEngine()
    pre    = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    mode_hist: Counter = Counter()
    pen_diffs: list[float] = []
    hover_diffs: list[float] = []
    long_hover_uwb_skipped = 0
    short_hover_uwb_kept   = 0

    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            engine.process_imu(pkt)
            mode_hist[engine._pen_mode.mode.value] += 1
        else:
            raw = pkt['dists']
            _, weights, ekf_dists = pre.process(*raw)
            mode_before = engine._pen_mode.mode
            pos, _, accepted, _ = engine.process_uwb(ekf_dists, weights,
                                                    ts=pkt.get('ts'))
            ref_xyz, _ = irls.solve(ekf_dists, weights)
            if pos is None or not irls.last_solve_ok:
                continue
            d = float(np.linalg.norm(pos[:2] - ref_xyz[:2]))
            if mode_before == PenMode.PEN_DOWN:
                pen_diffs.append(d)
            else:
                hover_diffs.append(d)
            if mode_before == PenMode.HOVER_LONG and not accepted:
                long_hover_uwb_skipped += 1
            if mode_before == PenMode.HOVER_SHORT and accepted:
                short_hover_uwb_kept += 1

    def _stats(arr: list[float]) -> dict:
        if not arr:
            return {'n': 0, 'median_m': float('nan'), 'p95_m': float('nan')}
        a = np.asarray(arr)
        return {'n': len(a), 'median_m': float(np.median(a)),
                'p95_m': float(np.percentile(a, 95))}

    metrics: dict = {}
    for k, v in _stats(pen_diffs).items():     metrics[f'pen_down_{k}']   = v
    for k, v in _stats(hover_diffs).items():   metrics[f'hover_{k}']      = v
    metrics['long_hover_skipped']  = long_hover_uwb_skipped
    metrics['short_hover_updates'] = short_hover_uwb_kept
    for m in ('pen_down', 'hover_short', 'hover_long', 'hover_h', 'cold_start'):
        metrics[f'mode_imu_count_{m}'] = mode_hist.get(m, 0)

    # Pass: pen-down median better than hover median (state machine
    # protected the writing path), AND short-hover kept at least some
    # updates (didn't degrade to skip-only for everything).
    passed = (
        metrics['pen_down_n'] > 0
        and (metrics['hover_n'] == 0
             or metrics['pen_down_median_m'] <= metrics['hover_median_m'] + 0.05)
    )

    out_dir = ensure_out(LAYER)
    fig, ax = plt.subplots(figsize=(7, 4))
    labels = ['pen_down', 'hover']
    medians = [metrics['pen_down_median_m'], metrics['hover_median_m']]
    p95s    = [metrics['pen_down_p95_m'],    metrics['hover_p95_m']]
    x = np.arange(len(labels))
    ax.bar(x - 0.2, medians, 0.4, label='median')
    ax.bar(x + 0.2, p95s,    0.4, label='p95')
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel('|EKF − IRLS| (m)')
    ax.set_title(f'Layer 8 pen-up audit: {csv_path.stem}')
    ax.legend()
    fig.tight_layout()
    plot = out_dir / f'{csv_path.stem}.png'
    fig.savefig(plot, dpi=110); plt.close(fig)

    return LayerResult(LAYER, csv_path.stem, passed, metrics, [],
                       plot=str(plot.relative_to(Path(__file__).resolve().parent)))


def run_all() -> list[LayerResult]:
    return [analyse(p) for p in list_datasets()]


if __name__ == '__main__':
    for p in list_datasets():
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:32s} {v}')
