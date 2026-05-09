"""
layer11_latency.py -- end-to-end latency proxy (Tier C).

Without a real-time player we cannot measure render_ts − sender_ts
directly, so this layer reports the fundamental components that drive
end-to-end latency:
  * IMU→UWB pairing lag      : for each UWB packet, how old (in seconds)
                                is the freshest IMU packet whose state
                                feeds the EKF predict at update time?
  * UWB→display lag (proxy)  : per-cycle UWB processing time measured
                                with time.perf_counter (a generous upper
                                bound on the live render delay).
  * histogram percentiles    : median, p95, p99.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR)
from kuru_method.asynchronous_stream.preprocessor  import UWBPreprocessor
from kuru_method.asynchronous_stream.ekf_fusion    import AsyncEKFFusionEngine
from kuru_method.asynchronous_stream.config        import UWB_OFFSETS


LAYER = 'layer11_latency'


def analyse(csv_path: Path) -> LayerResult:
    engine = AsyncEKFFusionEngine()
    pre    = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))

    pairing_lag_ms: list[float] = []
    proc_ms:        list[float] = []
    last_imu_ts: int | None = None

    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            engine.process_imu(pkt)
            last_imu_ts = int(pkt.get('ts')) if pkt.get('ts') is not None else last_imu_ts
        else:
            raw = pkt['dists']
            _, weights, ekf_dists = pre.process(*raw)
            t0 = time.perf_counter()
            engine.process_uwb(ekf_dists, weights, ts=pkt.get('ts'))
            t1 = time.perf_counter()
            proc_ms.append((t1 - t0) * 1000.0)
            uwb_ts = pkt.get('ts')
            if last_imu_ts is not None and uwb_ts is not None:
                pairing_lag_ms.append((int(uwb_ts) - last_imu_ts) / 1000.0)

    def _pct(a: list[float]) -> dict:
        if not a:
            return {'n': 0, 'median_ms': 0.0, 'p95_ms': 0.0, 'p99_ms': 0.0}
        a = np.asarray(a)
        return {'n': len(a),
                'median_ms': float(np.median(a)),
                'p95_ms':    float(np.percentile(a, 95)),
                'p99_ms':    float(np.percentile(a, 99))}

    metrics: dict = {}
    for k, v in _pct(pairing_lag_ms).items(): metrics[f'pairing_{k}'] = v
    for k, v in _pct(proc_ms).items():        metrics[f'proc_{k}']    = v

    # Pass: median per-cycle UWB processing under 5 ms (well within
    # 50 Hz cycle budget) and p99 pairing lag under 30 ms.
    passed = (metrics.get('proc_median_ms', 1e9) < 5.0
              and abs(metrics.get('pairing_p99_ms', 1e9)) < 30.0)

    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    if pairing_lag_ms:
        axes[0].hist(pairing_lag_ms, bins=40, color='tab:purple')
    axes[0].set_title('IMU↔UWB pairing lag (ms)')
    if proc_ms:
        axes[1].hist(proc_ms, bins=40, color='tab:orange')
    axes[1].set_title('UWB cycle processing (ms)')
    fig.suptitle(f'Layer 11 latency: {csv_path.stem}')
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
