"""
layer0_stream_health.py -- stream-level sanity check.

Metrics per dataset
-------------------
    imu_count, uwb_count, duration_s
    imu_rate_hz, uwb_rate_hz            (median dt -> rate)
    imu_loss_pct, uwb_loss_pct          (sequence-number gaps)
    imu_dt_spike_ms, uwb_dt_spike_ms    (worst dt gap)

Pass criteria (from plan)
-------------------------
    loss < 2 % per stream
    80 Hz <= IMU rate <= 120 Hz
    8 Hz  <= UWB rate <= 12 Hz
    no dt spike > 200 ms

Plot: two histograms of dt (IMU and UWB) for the dataset.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR)


LAYER = 'layer0_stream_health'


def analyse(csv_path: Path) -> LayerResult:
    imu_ts: list[int] = []
    uwb_ts: list[int] = []
    imu_seq: list[int] = []
    uwb_seq: list[int] = []

    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            imu_ts.append(pkt['ts'])
            imu_seq.append(pkt['seq'])
        else:
            uwb_ts.append(pkt['ts'])
            uwb_seq.append(pkt['seq'])

    metrics: dict = {
        'imu_count': len(imu_ts),
        'uwb_count': len(uwb_ts),
    }
    notes: list[str] = []

    def stream_stats(ts: list[int], seq: list[int], label: str) -> dict:
        if len(ts) < 2:
            notes.append(f'{label}: fewer than 2 packets -- cannot compute rates')
            return {'rate_hz': 0.0, 'loss_pct': 100.0, 'dt_spike_ms': 0.0,
                    'dts_ms': np.array([])}
        dts_us = np.diff(np.asarray(ts, dtype=np.int64))
        dts_ms = dts_us / 1000.0
        median_dt_ms = float(np.median(dts_ms))
        rate = 1000.0 / median_dt_ms if median_dt_ms > 0 else 0.0
        spike_ms = float(np.max(dts_ms))
        # Sequence-gap loss
        gaps = np.diff(np.asarray(seq, dtype=np.int64)) - 1
        gaps = gaps[gaps > 0]
        lost = int(gaps.sum())
        received = len(seq)
        total = received + lost
        loss_pct = (100.0 * lost / total) if total > 0 else 0.0
        return {'rate_hz': rate, 'loss_pct': loss_pct,
                'dt_spike_ms': spike_ms, 'dts_ms': dts_ms}

    imu_stats = stream_stats(imu_ts, imu_seq, 'IMU')
    uwb_stats = stream_stats(uwb_ts, uwb_seq, 'UWB')

    duration_s = 0.0
    if imu_ts and uwb_ts:
        t0 = min(imu_ts[0], uwb_ts[0])
        t1 = max(imu_ts[-1], uwb_ts[-1])
        duration_s = (t1 - t0) / 1e6

    metrics.update({
        'duration_s':       duration_s,
        'imu_rate_hz':      imu_stats['rate_hz'],
        'uwb_rate_hz':      uwb_stats['rate_hz'],
        'imu_loss_pct':     imu_stats['loss_pct'],
        'uwb_loss_pct':     uwb_stats['loss_pct'],
        'imu_dt_spike_ms':  imu_stats['dt_spike_ms'],
        'uwb_dt_spike_ms':  uwb_stats['dt_spike_ms'],
    })

    # Pass criteria
    passed = (
        imu_stats['loss_pct']    < 2.0  and
        uwb_stats['loss_pct']    < 2.0  and
        80.0 <= imu_stats['rate_hz'] <= 120.0 and
        8.0  <= uwb_stats['rate_hz'] <= 12.0  and
        imu_stats['dt_spike_ms'] < 200.0 and
        uwb_stats['dt_spike_ms'] < 500.0
    )

    # Plot dt histograms
    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    if len(imu_stats['dts_ms']):
        axes[0].hist(imu_stats['dts_ms'], bins=40, color='steelblue')
    axes[0].set_title(f'IMU dt  (rate={imu_stats["rate_hz"]:.1f} Hz)')
    axes[0].set_xlabel('dt (ms)'); axes[0].set_ylabel('count')
    axes[0].axvline(10.0, color='grey', ls='--', lw=1)

    if len(uwb_stats['dts_ms']):
        axes[1].hist(uwb_stats['dts_ms'], bins=40, color='seagreen')
    axes[1].set_title(f'UWB dt  (rate={uwb_stats["rate_hz"]:.1f} Hz)')
    axes[1].set_xlabel('dt (ms)'); axes[1].set_ylabel('count')
    axes[1].axvline(100.0, color='grey', ls='--', lw=1)

    fig.suptitle(f'Layer 0 -- Stream health: {csv_path.stem}')
    fig.tight_layout()
    plot_path = out_dir / f'{csv_path.stem}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER,
        dataset=csv_path.stem,
        passed=passed,
        metrics=metrics,
        notes=notes,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


def run_all() -> list[LayerResult]:
    return [analyse(p) for p in list_datasets()]


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*',
                    help='optional dataset stems; defaults to every CSV in datasets_a3_ls/')
    args = ap.parse_args()
    if args.datasets:
        files = [DATASET_DIR / f'{d}.csv' for d in args.datasets]
    else:
        files = list_datasets()
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            if isinstance(v, float):
                print(f'    {k:20s} {v:10.3f}')
            else:
                print(f'    {k:20s} {v}')
