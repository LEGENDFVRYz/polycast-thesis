"""
layer9_strokes.py -- stroke continuity audit (Tier C).

Counts how many strokes the FSR contact state generated, plus duration
distribution and false-split / false-merge proxies (very-short strokes
or very-short gaps).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, collect, LayerResult, list_datasets,
                     DATASET_DIR)
from kuru_method.asynchronous_stream.force_detector import ForceContactDetector


LAYER = 'layer9_strokes'

# Heuristic thresholds — keep tunable, but tied to wall-clock so they
# stay consistent across capture rates.
_FALSE_SPLIT_GAP_S   = 0.080   # gap < 80 ms looks like a debounce blip
_FALSE_MERGE_LEN_S   = 0.030   # stroke < 30 ms is below human writing


def analyse(csv_path: Path) -> LayerResult:
    imu_pkts, _ = collect(csv_path)
    if not imu_pkts:
        return LayerResult(LAYER, csv_path.stem, False,
                           {'imu_count': 0}, ['no IMU packets'])

    det = ForceContactDetector()
    states: list[int] = []
    tss:    list[int] = []
    for p in imu_pkts:
        s, _ = det.process(p['force'], ts=p.get('ts'))
        states.append(int(s)); tss.append(int(p['ts']))

    states = np.asarray(states); tss = np.asarray(tss, dtype=np.int64)

    # Run-length encode stroke episodes
    strokes:  list[tuple[int,int]] = []   # (start_ts, end_ts) inclusive
    i = 0; n = len(states)
    while i < n:
        if states[i] == 1:
            j = i
            while j < n and states[j] == 1:
                j += 1
            strokes.append((int(tss[i]), int(tss[j-1])))
            i = j
        else:
            i += 1

    # Gaps between strokes
    gaps_s = [(strokes[k+1][0] - strokes[k][1]) / 1e6
              for k in range(len(strokes) - 1)]
    durations_s = [(s[1] - s[0]) / 1e6 for s in strokes]

    false_splits = sum(1 for g in gaps_s     if g < _FALSE_SPLIT_GAP_S)
    false_merges = sum(1 for d in durations_s if d < _FALSE_MERGE_LEN_S)

    metrics = {
        'stroke_count':    len(strokes),
        'mean_duration_s': float(np.mean(durations_s)) if durations_s else 0.0,
        'min_duration_s':  float(np.min(durations_s))  if durations_s else 0.0,
        'mean_gap_s':      float(np.mean(gaps_s))      if gaps_s else 0.0,
        'min_gap_s':       float(np.min(gaps_s))       if gaps_s else 0.0,
        'false_splits':    false_splits,
        'false_merges':    false_merges,
        'contact_duty':    float(states.mean()),
    }

    passed = false_splits == 0 and false_merges == 0

    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    if durations_s:
        axes[0].hist(durations_s, bins=20, color='tab:blue')
    axes[0].set_title('stroke durations (s)')
    if gaps_s:
        axes[1].hist(gaps_s, bins=20, color='tab:green')
    axes[1].set_title('inter-stroke gaps (s)')
    fig.suptitle(f'Layer 9 strokes: {csv_path.stem}')
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
            print(f'    {k:24s} {v}')
