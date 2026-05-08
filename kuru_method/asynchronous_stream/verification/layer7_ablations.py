"""
layer7_ablations.py -- ablation sweep for the proposed pipeline (Priority 5/Tier B).

Each dataset is replayed through several pipeline configurations and the
same Tier-B end-to-end metric (median tip-error proxy) is recorded. The
"full" column corresponds to this branch's hardened pipeline; every other
column disables one component to attribute its contribution.

Configurations
--------------
    full              : the current (post-Priority-1..6) pipeline
    no_range_kf       : disable per-anchor 1D range KF
    no_quality_weight : pass weights=1 to the EKF
    no_contact_gate   : monkey-patch contact detector to always =1
    irls_only         : use IRLS positions (Layer 2) without EKF

Metric
------
    median_irls_dist : median distance between EKF position and the IRLS
                       solution at every UWB packet. Acts as a ground-
                       truth-free proxy: when EKF and IRLS agree under
                       low-NLOS LOS, both are correct; large discrepancy
                       means the EKF has drifted.

Pass criterion
--------------
    "full" must be the **best** (lowest median_irls_dist) configuration
    on every dataset. If any ablation beats it, the corresponding feature
    is failing rather than helping — flag it.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common import (ensure_out, replay, LayerResult, list_datasets,
                     DATASET_DIR)

from preprocessor    import UWBPreprocessor, IMUPreprocessor
from ekf_fusion      import AsyncEKFFusionEngine
from fusion_engine   import IRLSTrilateration
from config          import ANCHORS, MARKER_LENGTH, UWB_OFFSETS


LAYER = 'layer7_ablations'


def _run_config(csv_path: Path, cfg: str) -> dict:
    engine = AsyncEKFFusionEngine()
    pre    = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    if cfg == 'no_range_kf':
        # Bypass the per-anchor range KF by zeroing its bank
        engine._ekf._range_bank.reset()
        # And patch step_all to be identity-pass-through
        engine._ekf._range_bank.step_all = lambda dt, raw: np.asarray(raw, float)
    elif cfg == 'no_contact_gate':
        orig = engine._contact.process
        engine._contact.process = lambda f, ts=None, dt=None: (1, orig(f, ts=ts, dt=dt)[1])

    diffs: list[float] = []
    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            engine.process_imu(pkt)
        else:
            raw = pkt['dists']
            filt, weights, ekf_dists = pre.process(*raw)
            if cfg == 'no_quality_weight':
                weights = (1.0, 1.0, 1.0, 1.0)
            if cfg == 'irls_only':
                pos_xyz, _ = irls.solve(ekf_dists, weights)
                continue   # no EKF
            pos, _, _, _ = engine.process_uwb(ekf_dists, weights,
                                              ts=pkt.get('ts'))
            ref_xyz, _ = irls.solve(ekf_dists, weights)
            if pos is not None and irls.last_solve_ok:
                d = float(np.linalg.norm(pos[:2] - ref_xyz[:2]))
                diffs.append(d)

    if cfg == 'irls_only':
        return {'median_irls_dist': 0.0, 'n': 0}
    return {'median_irls_dist': float(np.median(diffs)) if diffs else float('nan'),
            'n': len(diffs)}


CONFIGS = ['full', 'no_range_kf', 'no_quality_weight',
           'no_contact_gate', 'irls_only']


def analyse(csv_path: Path) -> LayerResult:
    metrics: dict = {}
    for cfg in CONFIGS:
        try:
            r = _run_config(csv_path, cfg)
        except Exception as e:
            r = {'median_irls_dist': float('nan'), 'n': 0, 'error': repr(e)}
        for k, v in r.items():
            metrics[f'{cfg}_{k}'] = v

    # Pass criterion: "full" must be the lowest non-IRLS-only median.
    full   = metrics.get('full_median_irls_dist', float('nan'))
    others = [metrics[f'{c}_median_irls_dist']
              for c in CONFIGS
              if c not in ('full', 'irls_only')
              and not np.isnan(metrics.get(f'{c}_median_irls_dist', float('nan')))]
    passed = bool(others) and not np.isnan(full) and full <= min(others) + 0.005

    out_dir = ensure_out(LAYER)
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = [c for c in CONFIGS if not np.isnan(metrics.get(f'{c}_median_irls_dist', float('nan')))]
    vals   = [metrics[f'{c}_median_irls_dist'] for c in labels]
    ax.bar(labels, vals, color=['tab:blue' if c == 'full' else 'tab:gray'
                                for c in labels])
    ax.set_ylabel('median |EKF − IRLS| (m)')
    ax.set_title(f'Layer 7 ablations: {csv_path.stem}')
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
