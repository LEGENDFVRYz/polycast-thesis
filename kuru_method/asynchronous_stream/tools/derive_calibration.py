"""
derive_calibration.py — Phase 8 Step 2 calibration profile generator.

Consumes stationary-hold + slow-stroke datasets and writes
`calibration_profile.json` next to `config.py`. The profile replaces
hardcoded constants in `ekf_fusion.py` and `force_detector.py`:

    sigma_los_per_anchor : per-anchor LOS noise σ (m). Used as
                           `R_i,0 = max(σ_LOS_i², 0.05²)`.
    sigma_acc_mps2       : whiteboard-frame acceleration noise (m/s²).
                           Replaces `TightlyCoupledEKF.SIGMA_ACC`.
    sigma_bias_mps2_rt_s : bias drift rate (m/s² / √s). Replaces
                           `TightlyCoupledEKF.SIGMA_BIAS`.
    zupt_var_thresh      : p99 of stationary 30 ms-window acc variance.
                           Replaces `TightlyCoupledEKF.ZUPT_VAR_THRESH`.
    fsr_thresh_enter / fsr_thresh_exit : percentile-driven from idle
                           vs touch ADC histograms. Replaces
                           `ForceContactDetector.THRESH_ENTER` / `THRESH_EXIT`.

`config.py` loads this JSON the same way it loads `rate_profile.json`.
The hardcoded constants stay as fallbacks if the file is missing.

Usage
-----
    python tools/derive_calibration.py

By default uses the stationary middle-hold dataset (`middle-`) for σ_LOS
and ZUPT, the rotation-in-place datasets (`middleCW_rot-` /
`middleCCW_rot-`) for σ_acc, and the writing datasets (`ABC_b1`,
`HELLO_b1`) for FSR thresholds. Override with --static / --rot / --write.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_ASYNC = Path(__file__).resolve().parent.parent
if str(_ASYNC) not in sys.path:
    sys.path.insert(0, str(_ASYNC))

from data_parser    import AsyncDataParser     # noqa: E402
from imu_integrator import IMUIntegrator        # noqa: E402
from force_detector import ForceContactDetector # noqa: E402
from config         import ANCHORS, UWB_OFFSETS, IMU_HZ  # noqa: E402
from verification.ground_truth import get_truth as _gt   # noqa: E402

DATASET_DIR    = _ASYNC / 'datasets_str_50hz'
OUTPUT_PATH    = _ASYNC / 'calibration_profile.json'

# Defaults — picked because they have ground-truth XY in
# verification/ground_truth.py.
DEFAULT_STATIC = ['middle-']
DEFAULT_ROT    = ['middleCW_rot-', 'middleCCW_rot-']
DEFAULT_WRITE  = ['ABC_b1', 'HELLO_b1', 'abc_s1', 'hello_s1']

# 30 ms ZUPT window, expressed in samples via the measured IMU rate.
_ZUPT_WIN_S = 0.030


def _replay_imu_uwb(csv_path: Path):
    """Return (imu_pkts, uwb_pkts) — both as lists of dicts in file order."""
    parser = AsyncDataParser(csv_path=str(csv_path))
    if not parser.connect():
        raise RuntimeError(f'cannot open {csv_path}')
    imu, uwb = [], []
    try:
        while True:
            pkt = parser.get_packet()
            if pkt == 'EOF':
                break
            if pkt is None:
                continue
            (imu if pkt['type'] == 'imu' else uwb).append(pkt)
    finally:
        parser.close()
    return imu, uwb


# --------------------------------------------------------------------------
# Per-anchor LOS noise (σ_LOS_i) from a stationary hold.
# --------------------------------------------------------------------------
def derive_sigma_los_per_anchor(stems: list[str]) -> tuple[list[float], int]:
    """For each anchor i, compute σ over (raw + offset) − ‖anchor − tip_truth‖
    using all UWB samples from datasets at known stationary positions.

    Returns (sigmas[4], n_samples).
    """
    residuals: list[list[float]] = [[] for _ in range(4)]
    n_total = 0
    for stem in stems:
        truth_xy = _gt(stem)
        if truth_xy is None:
            print(f'  [skip] {stem}: no ground truth')
            continue
        csv = DATASET_DIR / f'{stem}.csv'
        if not csv.exists():
            print(f'  [skip] {csv} not found'); continue
        _, uwb = _replay_imu_uwb(csv)
        # Stationary hold → tip ≈ tag (z=0 on board); use truth_xy for tag.
        tag_xyz = np.array([truth_xy[0], truth_xy[1], 0.0])
        # Drop the first 1.0 s and last 0.5 s of the file to avoid the
        # warm-up / cool-down where the marker is being placed/removed.
        if len(uwb) < 30:
            continue
        for pkt in uwb[20:-10]:
            for i, raw in enumerate(pkt['dists']):
                if not np.isfinite(raw):
                    continue
                d_offset = float(raw) + float(UWB_OFFSETS[i])
                if not (0.05 < d_offset < 6.0):
                    continue
                d_geom = float(np.linalg.norm(ANCHORS[i] - tag_xyz))
                residuals[i].append(d_offset - d_geom)
                n_total += 1
    sigmas = []
    for i, r in enumerate(residuals):
        if len(r) < 20:
            sigmas.append(float('nan'))
        else:
            sigmas.append(float(np.std(r)))
    return sigmas, n_total


# --------------------------------------------------------------------------
# σ_acc and ZUPT variance threshold from a stationary hold.
# --------------------------------------------------------------------------
def derive_imu_noise(stems: list[str]) -> dict:
    """Compute whiteboard-frame acceleration σ over the *stationary* portion
    plus the p99 of the 30 ms-window acc variance (used as the ZUPT
    threshold floor)."""
    imu = IMUIntegrator()
    a_wb_seq: list[tuple[float, float]] = []

    for stem in stems:
        csv = DATASET_DIR / f'{stem}.csv'
        if not csv.exists():
            print(f'  [skip] {csv} not found'); continue
        imu_pkts, _ = _replay_imu_uwb(csv)
        if len(imu_pkts) < 200:
            continue
        # Skip the first ~1 s (settling) and last 0.5 s.
        head = int(round(IMU_HZ * 1.0))
        tail = int(round(IMU_HZ * 0.5))
        for p in imu_pkts[head:-tail] if tail else imu_pkts[head:]:
            try:
                a_wb = imu.get_wb_acceleration(
                    p['quat'], p['acc'], ts=p.get('ts'))
            except Exception:
                continue
            a_wb_seq.append((float(a_wb[0]), float(a_wb[1])))

    if len(a_wb_seq) < 100:
        return {'sigma_acc_mps2': float('nan'),
                'zupt_var_thresh': float('nan'),
                'sigma_bias_mps2_rt_s': float('nan'),
                'n_samples': len(a_wb_seq)}

    a = np.asarray(a_wb_seq, dtype=float)

    # σ_acc — combined per-axis stdev (assumes isotropic xy noise).
    sigma_x = float(np.std(a[:, 0]))
    sigma_y = float(np.std(a[:, 1]))
    sigma_acc = float(np.sqrt(0.5 * (sigma_x ** 2 + sigma_y ** 2)))

    # ZUPT variance threshold: p99 of var(window) on a 30 ms window.
    win = max(2, int(round(_ZUPT_WIN_S * IMU_HZ)))
    if len(a) >= win + 1:
        win_var = []
        for k in range(0, len(a) - win, max(1, win // 2)):
            seg = a[k:k + win]
            v = float(np.var(seg[:, 0]) + np.var(seg[:, 1]))
            win_var.append(v)
        zupt_thr = float(np.percentile(win_var, 99))
    else:
        zupt_thr = float('nan')

    # σ_bias (random-walk rate, m/s² / √s) — Allan-variance-style estimate
    # from the long-window drift of the 1 s mean. Conservative; clamp so
    # the EKF Q stays within sane bounds.
    one_sec = max(50, int(round(IMU_HZ)))
    if len(a) >= one_sec * 5:
        means = np.array([np.mean(a[k:k + one_sec, 0])
                          for k in range(0, len(a) - one_sec, one_sec)])
        # σ of successive 1-s mean differences ≈ random walk rate per √s.
        diffs = np.diff(means)
        sigma_bias = float(np.std(diffs))
        sigma_bias = max(min(sigma_bias, 0.005), 0.0001)  # clamp [1e-4, 5e-3]
    else:
        sigma_bias = float('nan')

    return {'sigma_acc_mps2':       sigma_acc,
            'zupt_var_thresh':      zupt_thr,
            'sigma_bias_mps2_rt_s': sigma_bias,
            'n_samples':            len(a)}


# --------------------------------------------------------------------------
# FSR enter / exit thresholds from idle vs touch histograms.
# --------------------------------------------------------------------------
def _otsu_threshold(values: np.ndarray, bins: int = 128) -> float:
    """Otsu binarisation on a 1-D histogram. Returns the bin centre that
    maximises between-class variance."""
    hist, edges = np.histogram(values, bins=bins)
    centres = 0.5 * (edges[:-1] + edges[1:])
    total = hist.sum()
    if total == 0:
        return float(np.median(values))
    p = hist.astype(float) / total
    cum_w  = np.cumsum(p)
    cum_mu = np.cumsum(p * centres)
    mu_t   = cum_mu[-1]
    # Between-class variance σ_b²(t)
    eps = 1e-12
    sigma_b2 = (mu_t * cum_w - cum_mu) ** 2 / (cum_w * (1.0 - cum_w) + eps)
    # Mask invalid (cum_w = 0 or 1) bins
    sigma_b2[~np.isfinite(sigma_b2)] = -1.0
    k = int(np.argmax(sigma_b2))
    return float(centres[k])


def derive_fsr_thresholds(stems: list[str]) -> dict:
    """Label-free FSR threshold derivation. Uses bimodal histogram
    analysis (Otsu) on the EMA-smoothed force value across writing
    datasets, then sets:

      enter ← otsu_split + 0.5 σ_touch (above split, into touch mode)
      exit  ← otsu_split − 0.5 σ_idle  (below split, into idle mode)

    The ForceContactDetector EMA is reused only to smooth the signal
    (so the histogram has clean modes); the detector's *state* output is
    deliberately ignored — using it would make the result depend on
    existing hardcoded thresholds via hysteresis decay.
    """
    raw_vals: list[float] = []
    for stem in stems:
        csv = DATASET_DIR / f'{stem}.csv'
        if not csv.exists():
            print(f'  [skip] {csv} not found'); continue
        imu_pkts, _ = _replay_imu_uwb(csv)
        if len(imu_pkts) < 200:
            continue
        for p in imu_pkts:
            raw_vals.append(float(p['force']))

    if len(raw_vals) < 200:
        return {'fsr_thresh_enter': float('nan'),
                'fsr_thresh_exit':  float('nan'),
                'fsr_otsu_split':   float('nan'),
                'n_samples':        len(raw_vals)}

    arr = np.asarray(raw_vals)
    # Raw force is sharply bimodal (idle ≈ 0, touch ≈ 2100 ADC). Otsu on
    # the raw histogram finds the natural valley between the two modes.
    split = _otsu_threshold(arr, bins=64)
    idle_mask  = arr < split
    touch_mask = arr >= split
    idle_med   = float(np.median(arr[idle_mask]))  if idle_mask.any()  else 0.0
    touch_med  = float(np.median(arr[touch_mask])) if touch_mask.any() else split
    sigma_idle  = float(np.std(arr[idle_mask]))  if idle_mask.any()  else 1.0
    sigma_touch = float(np.std(arr[touch_mask])) if touch_mask.any() else 1.0

    # Place enter close to the touch-mode lower edge and exit near the
    # idle-mode upper edge. With a sharp bimodal distribution, idle_med +
    # 5σ_idle is far below touch_med − 5σ_touch, so we get a wide hysteresis
    # band that matches the hardware's natural separation.
    enter = max(split, idle_med + 5.0 * sigma_idle + 50.0)
    exit_ = max(0.0, min(split * 0.5, idle_med + 2.0 * sigma_idle + 20.0))
    if enter <= exit_ + 30:
        enter = split
        exit_ = max(0.0, split - 100.0)
    return {'fsr_thresh_enter': float(enter),
            'fsr_thresh_exit':  float(exit_),
            'fsr_otsu_split':   float(split),
            'fsr_idle_median':  idle_med,
            'fsr_touch_median': touch_med,
            'sigma_idle':       sigma_idle,
            'sigma_touch':      sigma_touch,
            'n_samples':        len(arr)}


# --------------------------------------------------------------------------
# Orchestration
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--static', nargs='*', default=DEFAULT_STATIC,
                    help='dataset stems for stationary holds (UWB σ + ZUPT)')
    ap.add_argument('--rot', nargs='*', default=DEFAULT_ROT,
                    help='dataset stems for rotation-in-place IMU σ')
    ap.add_argument('--write', nargs='*', default=DEFAULT_WRITE,
                    help='dataset stems for FSR threshold derivation')
    ap.add_argument('--out', default=str(OUTPUT_PATH),
                    help='output JSON path')
    args = ap.parse_args()

    profile: dict = {'source': 'derive_calibration.py'}

    print('[derive] Per-anchor sigma_LOS from:', args.static)
    sigmas, n_los = derive_sigma_los_per_anchor(args.static)
    profile['sigma_los_per_anchor'] = sigmas
    profile['sigma_los_n_samples']  = n_los

    # ZUPT threshold strictly from the stationary middle-hold (rotation
    # datasets are NOT stationary — including them inflates the threshold
    # and would cause spurious zero-velocity updates during real motion).
    print('[derive] ZUPT threshold from stationary holds:', args.static)
    zupt_stats = derive_imu_noise(args.static)
    profile['zupt_var_thresh'] = zupt_stats['zupt_var_thresh']
    profile['zupt_n_samples']  = zupt_stats['n_samples']
    profile['sigma_bias_mps2_rt_s'] = zupt_stats['sigma_bias_mps2_rt_s']

    # σ_acc represents the EKF process noise on accel — see ekf_fusion.py
    # for why it's intentionally inflated above raw IMU stationary noise.
    # We record the *measured* stationary σ as a diagnostic but DO NOT
    # write it into the profile; the EKF default (2.0 m/s²) is preserved.
    profile['sigma_acc_mps2_diagnostic'] = zupt_stats['sigma_acc_mps2']

    print('[derive] FSR enter/exit from:', args.write)
    fsr_stats = derive_fsr_thresholds(args.write)
    profile.update(fsr_stats)

    Path(args.out).write_text(json.dumps(profile, indent=2), encoding='utf-8')
    print()
    print(f'[derive] -> {args.out}')
    for k, v in profile.items():
        if isinstance(v, list):
            v_repr = '[' + ', '.join(f'{x:.4f}' for x in v) + ']'
        elif isinstance(v, float):
            v_repr = f'{v:.6g}'
        else:
            v_repr = str(v)
        print(f'   {k:28s} {v_repr}')


if __name__ == '__main__':
    main()
