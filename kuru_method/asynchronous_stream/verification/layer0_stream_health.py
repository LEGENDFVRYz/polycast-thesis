"""
layer0_stream_health.py -- stream-level sanity check (Priority 1).

Metrics per dataset
-------------------
    imu_count, uwb_count, duration_s
    imu_rate_hz, uwb_rate_hz                (1 / median dt)
    imu_loss_pct, uwb_loss_pct              (sequence-number gaps)
    imu_dt_{mean,median,p05,p95,p99,iqr}_ms (full distribution)
    uwb_dt_{mean,median,p05,p95,p99,iqr}_ms (full distribution)
    imu_dt_spike_ms, uwb_dt_spike_ms        (worst dt gap)

Side effect
-----------
After all datasets complete, run_all() merges the per-dataset distributions
and writes `rate_profile.json` next to config.py. Every other module that
needs a "rate" (force_detector debounce, EKF ZUPT/bias-freeze windows,
preprocessor median window, etc.) reads from there. No more 50/100/200 Hz
hardcoded constants.

Pass criteria
-------------
    loss < 2 % per stream
    IMU rate in [80, 120] Hz (legacy capture) OR [190, 225] Hz (BNO085 actual)
    45 Hz <= UWB rate <= 55 Hz
    p99 IMU dt < 2x nominal IMU dt
    p99 UWB dt < 2x nominal UWB dt
    no dt spike > 200 ms (IMU) / 500 ms (UWB)

Note on paired-loss / CAL round-trip
------------------------------------
The receiver flattens BNO085 ROTATION_VECTOR and LINEAR_ACCELERATION
into a single CSV row (`I,seq,qx,qy,qz,qw,ax,ay,az,force,ts`), so
paired-arrival loss is implicit -- the sender only emits a CSV row
when both reports were available within the pairing window.

Plot: two histograms of dt (IMU and UWB) for the dataset.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR, ASYNC_DIR)


LAYER = 'layer0_stream_health'

# Aggregated per-dataset dt arrays (filled by analyse(), drained by run_all()).
_GLOBAL_DTS_MS: dict[str, list[np.ndarray]] = {'imu': [], 'uwb': []}


def _percentiles(dts_ms: np.ndarray) -> dict:
    if len(dts_ms) == 0:
        return {k: 0.0 for k in
                ('mean_ms', 'median_ms', 'p05_ms', 'p95_ms', 'p99_ms',
                 'iqr_ms')}
    p05, p25, p50, p75, p95, p99 = np.percentile(
        dts_ms, [5, 25, 50, 75, 95, 99])
    return {
        'mean_ms':   float(np.mean(dts_ms)),
        'median_ms': float(p50),
        'p05_ms':    float(p05),
        'p95_ms':    float(p95),
        'p99_ms':    float(p99),
        'iqr_ms':    float(p75 - p25),
    }


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
                    'dts_ms': np.array([]), 'pct': _percentiles(np.array([]))}
        dts_us = np.diff(np.asarray(ts, dtype=np.int64))
        dts_ms = dts_us / 1000.0
        median_dt_ms = float(np.median(dts_ms))
        rate = 1000.0 / median_dt_ms if median_dt_ms > 0 else 0.0
        spike_ms = float(np.max(dts_ms))
        gaps = np.diff(np.asarray(seq, dtype=np.int64)) - 1
        gaps = gaps[gaps > 0]
        lost = int(gaps.sum())
        received = len(seq)
        total = received + lost
        loss_pct = (100.0 * lost / total) if total > 0 else 0.0
        return {'rate_hz': rate, 'loss_pct': loss_pct,
                'dt_spike_ms': spike_ms, 'dts_ms': dts_ms,
                'pct': _percentiles(dts_ms)}

    imu_stats = stream_stats(imu_ts, imu_seq, 'IMU')
    uwb_stats = stream_stats(uwb_ts, uwb_seq, 'UWB')

    # Stash dts for the run_all() aggregation that writes rate_profile.json.
    if len(imu_stats['dts_ms']):
        _GLOBAL_DTS_MS['imu'].append(imu_stats['dts_ms'])
    if len(uwb_stats['dts_ms']):
        _GLOBAL_DTS_MS['uwb'].append(uwb_stats['dts_ms'])

    duration_s = 0.0
    if imu_ts and uwb_ts:
        t0 = min(imu_ts[0], uwb_ts[0])
        t1 = max(imu_ts[-1], uwb_ts[-1])
        duration_s = (t1 - t0) / 1e6

    imu_rate = imu_stats['rate_hz']
    in_legacy_band = 80.0 <= imu_rate <= 120.0
    in_200hz_band  = 190.0 <= imu_rate <= 225.0
    metrics.update({
        'duration_s':       duration_s,
        'imu_rate_hz':      imu_stats['rate_hz'],
        'uwb_rate_hz':      uwb_stats['rate_hz'],
        'imu_loss_pct':     imu_stats['loss_pct'],
        'uwb_loss_pct':     uwb_stats['loss_pct'],
        'imu_dt_spike_ms':  imu_stats['dt_spike_ms'],
        'uwb_dt_spike_ms':  uwb_stats['dt_spike_ms'],
        'imu_rate_at_200hz_target': bool(in_200hz_band),
    })
    for k, v in imu_stats['pct'].items():
        metrics[f'imu_dt_{k}'] = v
    for k, v in uwb_stats['pct'].items():
        metrics[f'uwb_dt_{k}'] = v

    if in_legacy_band and not in_200hz_band:
        notes.append('IMU stream at legacy 100 Hz -- pre-Item-D capture. '
                     'Re-record with sender/imu_module.cpp at 5000us interval '
                     'to exercise the 200 Hz path.')

    # p99 dt should not exceed 2x nominal — Priority-1 spike threshold.
    imu_p99 = imu_stats['pct']['p99_ms']
    uwb_p99 = uwb_stats['pct']['p99_ms']
    imu_nom_ms = 1000.0 / max(imu_rate, 1e-6)
    uwb_nom_ms = 1000.0 / max(uwb_stats['rate_hz'], 1e-6)
    imu_p99_ok = imu_p99 < 2.0 * imu_nom_ms if imu_p99 > 0 else True
    uwb_p99_ok = uwb_p99 < 2.0 * uwb_nom_ms if uwb_p99 > 0 else True

    passed = (
        imu_stats['loss_pct']    < 2.0  and
        uwb_stats['loss_pct']    < 2.0  and
        (in_legacy_band or in_200hz_band) and
        45.0 <= uwb_stats['rate_hz'] <= 55.0  and
        imu_stats['dt_spike_ms'] < 200.0 and
        uwb_stats['dt_spike_ms'] < 500.0 and
        imu_p99_ok and uwb_p99_ok
    )

    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    if len(imu_stats['dts_ms']):
        axes[0].hist(imu_stats['dts_ms'], bins=40, color='steelblue')
    axes[0].set_title(f'IMU dt  (rate={imu_stats["rate_hz"]:.1f} Hz, '
                      f'p99={imu_p99:.1f} ms)')
    axes[0].set_xlabel('dt (ms)'); axes[0].set_ylabel('count')
    axes[0].axvline(imu_nom_ms, color='grey', ls='--', lw=1)

    if len(uwb_stats['dts_ms']):
        axes[1].hist(uwb_stats['dts_ms'], bins=40, color='seagreen')
    axes[1].set_title(f'UWB dt  (rate={uwb_stats["rate_hz"]:.1f} Hz, '
                      f'p99={uwb_p99:.1f} ms)')
    axes[1].set_xlabel('dt (ms)'); axes[1].set_ylabel('count')
    axes[1].axvline(uwb_nom_ms, color='grey', ls='--', lw=1)

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


def _write_rate_profile() -> Path:
    """
    Aggregate the per-dataset dt arrays collected during this run and
    write `rate_profile.json` next to config.py. Downstream modules read
    it via `config.RATE_PROFILE` at import time.
    """
    out_path = ASYNC_DIR / 'rate_profile.json'
    profile: dict = {}

    def fold(label: str) -> dict:
        bufs = _GLOBAL_DTS_MS.get(label, [])
        if not bufs:
            return {}
        all_dts_ms = np.concatenate(bufs)
        median_ms = float(np.median(all_dts_ms))
        rate_hz = 1000.0 / median_ms if median_ms > 0 else 0.0
        p99_ms = float(np.percentile(all_dts_ms, 99))
        return {'rate_hz': rate_hz, 'dt_nom_s': median_ms / 1000.0,
                'dt_p99_s': p99_ms / 1000.0,
                'n_samples': int(len(all_dts_ms))}

    imu = fold('imu')
    uwb = fold('uwb')
    if imu:
        profile['imu_hz']        = imu['rate_hz']
        profile['imu_dt_nom_s']  = imu['dt_nom_s']
        profile['imu_dt_p99_s']  = imu['dt_p99_s']
        profile['imu_n_samples'] = imu['n_samples']
        # FSR rides on the IMU stream — same cadence.
        profile['fsr_hz']        = imu['rate_hz']
        profile['fsr_dt_nom_s']  = imu['dt_nom_s']
        profile['fsr_dt_p99_s']  = imu['dt_p99_s']
    if uwb:
        profile['uwb_hz']        = uwb['rate_hz']
        profile['uwb_dt_nom_s']  = uwb['dt_nom_s']
        profile['uwb_dt_p99_s']  = uwb['dt_p99_s']
        profile['uwb_n_samples'] = uwb['n_samples']
    profile['source_dir'] = str(DATASET_DIR)

    if profile:
        out_path.write_text(json.dumps(profile, indent=2), encoding='utf-8')
    return out_path


def run_all() -> list[LayerResult]:
    results = [analyse(p) for p in list_datasets()]
    _write_rate_profile()
    return results


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*',
                    help=f'optional dataset stems; defaults to every CSV in {DATASET_DIR.name}/')
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
                print(f'    {k:24s} {v:10.3f}')
            else:
                print(f'    {k:24s} {v}')
    profile_path = _write_rate_profile()
    print(f'\nrate_profile.json -> {profile_path}')
