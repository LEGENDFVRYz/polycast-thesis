"""
layer1_uwb_raw.py -- per-anchor raw + calibrated UWB distance accuracy.

For stationary datasets with known ground truth (from ground_truth.py),
computes for each of the 4 anchors:
    raw_mean, raw_std, raw_bias
    calibrated_mean, calibrated_std, calibrated_bias
    despiked_mean, despiked_std
    kf_mean,  kf_std       (Item B per-anchor 1D range KF, raw input)

The 'calibrated' column applies UWB_OFFSETS; the 'despiked' column is
what the EKF actually sees through UWBPreprocessor.  We compare both
against the *expected* 3D Euclidean distance from the true tip position
to each anchor (anchor_z and MARKER_LENGTH factored in).

Pass criteria (per-anchor, per-dataset)
---------------------------------------
    |calibrated_bias| < 0.08 m   (realistic for DWM1000 + ~13 cm tag-antenna height)
    calibrated_std     < 0.05 m
    No anchor is an obvious outlier (bias > 2x median across anchors).

Item B add-on (stationary datasets only)
----------------------------------------
    min_var_reduction_pct > 30 %   — the worst-anchor variance reduction
    achieved by feeding raw ranges through PerAnchorRangeKFBank with
    sigma_meas = SIGMA_UWB.  Skipped on datasets without ground truth.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, collect, LayerResult, DATASET_DIR,
                     stems_matching, list_datasets)

# Truly stationary stems (marker held still). `pt_a`/`pt_aa` are multi-stroke
# character writes, not static holds. Rotation tests (rt_*, pt_middle_r*)
# move the UWB tag in a circle around the still tip via the 0.21 m lever arm,
# so they violate L1's static-range assumption by physical geometry.
STATIONARY_STEMS = {'pt_middle', 'pt_middle-', 'middle-'}
from kuru_method.asynchronous_stream.config    import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from kuru_method.asynchronous_stream.preprocessor import UWBPreprocessor
from ground_truth import get_truth
from kuru_method.asynchronous_stream.range_kf     import PerAnchorRangeKFBank
from kuru_method.asynchronous_stream.ekf_fusion   import TightlyCoupledEKF


LAYER = 'layer1_uwb_raw'


def expected_distance_3d(tip_xy: tuple[float, float], anchor_xyz) -> float:
    """True Euclidean tag-antenna -> anchor-antenna distance.

    Tag antenna sits at (tip_x, tip_y, MARKER_LENGTH) -- assumes the marker
    is perpendicular to the board (layer 4 tests whether that holds)."""
    tag = np.array([tip_xy[0], tip_xy[1], MARKER_LENGTH])
    return float(np.linalg.norm(tag - np.asarray(anchor_xyz, dtype=float)))


def analyse(csv_path: Path) -> LayerResult:
    stem = csv_path.stem
    truth = get_truth(stem)

    _, uwb_pkts = collect(csv_path)
    if not uwb_pkts:
        return LayerResult(LAYER, stem, passed=False,
                           metrics={'uwb_count': 0},
                           notes=['no UWB packets'])

    raws  = np.array([p['dists'] for p in uwb_pkts], dtype=float)  # (N, 4)

    # Calibrated = raw + offset (per anchor)
    offsets = np.asarray(UWB_OFFSETS, dtype=float)
    caled   = raws + offsets

    # Despiked via UWBPreprocessor (what the EKF sees)
    pre = UWBPreprocessor(offsets=tuple(offsets))
    despiked_rows = []
    for d0, d1, d2, d3 in raws:
        _, _, des = pre.process(float(d0), float(d1), float(d2), float(d3))
        despiked_rows.append(des)
    despiked = np.asarray(despiked_rows, dtype=float)

    # Item B: per-anchor 1D range KF on raw input.  Mirrors what
    # AsyncEKFFusionEngine does inside update_uwb() so the variance-reduction
    # metric here reflects the actual filter the EKF consumes.
    bank = PerAnchorRangeKFBank(
        n_anchors=4,
        sigma_meas=TightlyCoupledEKF.SIGMA_UWB,
    )
    kf_rows = []
    prev_ts_us = None
    for pkt, raw_row in zip(uwb_pkts, raws):
        ts = pkt.get('ts')
        if prev_ts_us is None or ts is None:
            dt = 0.1
        else:
            dt_us = ts - prev_ts_us
            dt = (dt_us / 1e6) if 0 < dt_us < 2_000_000 else 0.1
        if ts is not None:
            prev_ts_us = ts
        kf_rows.append(bank.step_all(dt, raw_row))
    kf_filtered = np.asarray(kf_rows, dtype=float)

    # Per-anchor stats
    raw_mean, raw_std = raws.mean(axis=0),   raws.std(axis=0)
    cal_mean, cal_std = caled.mean(axis=0),  caled.std(axis=0)
    des_mean, des_std = despiked.mean(axis=0), despiked.std(axis=0)
    kf_mean,  kf_std  = (np.nanmean(kf_filtered, axis=0),
                         np.nanstd(kf_filtered, axis=0))

    metrics: dict = {
        'uwb_count': len(uwb_pkts),
        'raw_mean':  [float(x) for x in raw_mean],
        'raw_std':   [float(x) for x in raw_std],
        'cal_mean':  [float(x) for x in cal_mean],
        'cal_std':   [float(x) for x in cal_std],
        'des_mean':  [float(x) for x in des_mean],
        'des_std':   [float(x) for x in des_std],
        'kf_mean':   [float(x) for x in kf_mean],
        'kf_std':    [float(x) for x in kf_std],
    }

    # Variance reduction (raw -> KF) per anchor.  Skip anchors with
    # vanishingly small raw variance (avoid division blow-up on a flat
    # channel), and report worst-anchor reduction as the gate witness.
    raw_var = raw_std ** 2
    kf_var  = kf_std ** 2
    var_red_pct = np.full(4, np.nan)
    for i in range(4):
        if raw_var[i] > 1e-8:
            var_red_pct[i] = 100.0 * (1.0 - kf_var[i] / raw_var[i])
    metrics['var_reduction_pct'] = [float(x) for x in var_red_pct]
    finite = var_red_pct[np.isfinite(var_red_pct)]
    if finite.size:
        metrics['min_var_reduction_pct'] = float(finite.min())

    # Bias vs ground truth (only meaningful for stationary + known truth)
    if truth is not None:
        expected = np.array([expected_distance_3d(truth, a) for a in ANCHORS])
        raw_bias = raw_mean - expected
        cal_bias = cal_mean - expected
        des_bias = des_mean - expected
        kf_bias  = kf_mean - expected

        metrics['expected'] = [float(x) for x in expected]
        metrics['raw_bias'] = [float(x) for x in raw_bias]
        metrics['cal_bias'] = [float(x) for x in cal_bias]
        metrics['des_bias'] = [float(x) for x in des_bias]
        metrics['kf_bias']  = [float(x) for x in kf_bias]
        metrics['truth']    = truth

        max_cal_abs_bias = float(np.max(np.abs(cal_bias)))
        max_cal_std      = float(np.max(cal_std))
        metrics['max_cal_abs_bias'] = max_cal_abs_bias
        metrics['max_cal_std']      = max_cal_std

        # Item B gate: per-anchor KF must shave ≥ 30% off raw variance on
        # stationary datasets (Zou §3.1 sanity).  Existing thresholds are
        # left untouched.
        item_b_ok = (
            metrics.get('min_var_reduction_pct') is not None
            and metrics['min_var_reduction_pct'] > 30.0
        )

        passed = (max_cal_abs_bias < 0.08
                  and max_cal_std < 0.05
                  and item_b_ok)
    else:
        # No truth -- only sanity-check noise; pass if std is physically sane.
        max_cal_std = float(np.max(cal_std))
        metrics['max_cal_std'] = max_cal_std
        passed = max_cal_std < 0.10

    # ---- Plot: 4-panel time series per anchor ----
    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    axes = axes.flatten()
    t_s = (np.arange(len(raws))) * 0.02   # ~50 Hz UWB
    for i in range(4):
        ax = axes[i]
        ax.plot(t_s, raws[:, i],    color='grey',   lw=0.6, label='raw')
        ax.plot(t_s, caled[:, i],   color='tab:blue', lw=1.0, label='calibrated')
        ax.plot(t_s, despiked[:, i], color='tab:orange', lw=1.0, label='despiked')
        ax.plot(t_s, kf_filtered[:, i], color='tab:green', lw=1.0, ls='--',
                label='range-KF')
        if truth is not None:
            exp_i = expected_distance_3d(truth, ANCHORS[i])
            ax.axhline(exp_i, color='green', lw=1.2, ls='--',
                       label=f'expected={exp_i:.3f}')
        title = (f'A{i}   cal_mean={cal_mean[i]:.3f}  '
                 f'cal_std={cal_std[i]:.3f}  kf_std={kf_std[i]:.3f}')
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')
    fig.supxlabel('time (s)')
    fig.supylabel('distance (m)')
    fig.suptitle(f'Layer 1 -- UWB per-anchor distances: {stem}'
                 + (f'  truth={truth}' if truth else '  (no truth)'))
    fig.tight_layout()
    plot_path = out_dir / f'{stem}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER, dataset=stem, passed=passed,
        metrics=metrics,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


def run_all() -> list[LayerResult]:
    """Layer 1 runs on all stationary datasets (known ground truth)."""
    results = []
    for p in list_datasets(lambda s: s in STATIONARY_STEMS):
        results.append(analyse(p))
    return results


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*')
    args = ap.parse_args()
    if args.datasets:
        files = [DATASET_DIR / f'{d}.csv' for d in args.datasets]
    else:
        files = list_datasets(lambda s: s in STATIONARY_STEMS)
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            if isinstance(v, list):
                print(f'    {k:20s} {[round(x, 4) for x in v]}')
            elif isinstance(v, float):
                print(f'    {k:20s} {v:10.4f}')
            else:
                print(f'    {k:20s} {v}')
