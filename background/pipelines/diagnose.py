"""
Pipeline Diagnostic Tool — reads visualizer_log.csv and checks every
layer of the pipeline for problems.

Usage:
    python -m background.pipelines.diagnose
    python -m background.pipelines.diagnose --csv visualizer_log.csv
    python -m background.pipelines.diagnose --csv mylog.csv --stroke 3

Reports:
    1. IMU acceleration axis health
    2. IMU drift rate
    3. UWB position health
    4. ESKF fusion quality
    5. Stroke quality (straightness, position accuracy)
    6. Recommended config changes
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def load(path: str) -> list[dict]:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def _f(row: dict, key: str, default=0.0) -> float:
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return default


def _ink(rows: list) -> list:
    return [r for r in rows if r.get('ink_written') == '1']


def _stroke(rows: list, sid: int) -> list:
    return [r for r in rows if _f(r, 'stroke_id') == sid and r.get('ink_written') == '1']


# ── 1. IMU axis health ────────────────────────────────────────────────────────
def check_imu_axes(ink: list) -> dict:
    if not ink:
        return {'status': 'NO_INK'}

    hp_xs = [abs(_f(r, 'acc_hp_tip_x')) for r in ink]
    hp_ys = [abs(_f(r, 'acc_hp_tip_y')) for r in ink]
    tip_xs = [abs(_f(r, 'acc_tip_x')) for r in ink]
    tip_ys = [abs(_f(r, 'acc_tip_y')) for r in ink]

    mean_hpx = np.mean(hp_xs)
    mean_hpy = np.mean(hp_ys)
    mean_tipx = np.mean(tip_xs)
    mean_tipy = np.mean(tip_ys)

    ratio_hp  = mean_hpy / max(mean_hpx, 1e-6)
    ratio_tip = mean_tipy / max(mean_tipx, 1e-6)

    # Check gravity leakage — tip acc should be gravity-free
    # If mean > 0.5 m/s2 when static it means gravity is leaking
    static_rows = [r for r in ink if r.get('is_static') == '1']
    static_mag = np.mean([_f(r, 'acc_tip_mag') for r in static_rows]) if static_rows else 0.0

    return {
        'mean_hp_x':      mean_hpx,
        'mean_hp_y':      mean_hpy,
        'hp_y_x_ratio':   ratio_hp,
        'mean_tip_x':     mean_tipx,
        'mean_tip_y':     mean_tipy,
        'tip_y_x_ratio':  ratio_tip,
        'static_acc_mag': static_mag,
        'status': (
            'AXIS_SWAPPED'   if ratio_hp > 3.0  else
            'GRAVITY_LEAK'   if static_mag > 0.3 else
            'MARGINAL'       if ratio_hp > 1.5  else
            'OK'
        )
    }


# ── 2. IMU drift ──────────────────────────────────────────────────────────────
def check_imu_drift(ink: list) -> dict:
    if not ink:
        return {'status': 'NO_INK'}

    imu_xs = [_f(r, 'imu_p_x') for r in ink]
    imu_ys = [_f(r, 'imu_p_y') for r in ink]

    x_range = max(imu_xs) - min(imu_xs)
    y_range = max(imu_ys) - min(imu_ys)

    # Fit line to IMU path
    if x_range > 0.01:
        coeffs = np.polyfit(imu_xs, imu_ys, 1)
        y_fit  = np.polyval(coeffs, imu_xs)
        rmse   = float(np.sqrt(np.mean((np.array(imu_ys) - y_fit)**2)))
    else:
        rmse = y_range

    drift_ratio = y_range / max(x_range, 0.001)

    return {
        'x_range_m':   x_range,
        'y_range_m':   y_range,
        'drift_ratio': drift_ratio,
        'rmse_mm':     rmse * 1000,
        'status': (
            'SEVERE'   if drift_ratio > 5.0  else
            'BAD'      if drift_ratio > 2.0  else
            'MARGINAL' if drift_ratio > 0.5  else
            'OK'
        )
    }


# ── 3. UWB health ─────────────────────────────────────────────────────────────
def check_uwb(rows: list, ink: list) -> dict:
    uwb_rows = [r for r in rows if _f(r, 'uwb_p_x') > 0 or _f(r, 'uwb_p_y') > 0]
    if not uwb_rows:
        return {'status': 'NO_UWB'}

    # Check for zero dropouts
    zero_y = sum(1 for r in rows if _f(r, 'uwb_p_y') == 0.0)
    zero_pct = 100 * zero_y / len(rows)

    # UWB range during stroke
    if ink:
        uwb_xs = [_f(r, 'uwb_p_x') for r in ink]
        uwb_ys = [_f(r, 'uwb_p_y') for r in ink]
        uwb_x_range = max(uwb_xs) - min(uwb_xs)
        uwb_y_range = max(uwb_ys) - min(uwb_ys)
    else:
        uwb_x_range = uwb_y_range = 0.0

    # Innovation during stroke
    innovs = [_f(r, 'innovation_norm') for r in ink if _f(r, 'innovation_norm') > 0]
    mean_innov = np.mean(innovs) * 100 if innovs else 0.0

    return {
        'zero_dropout_pct': zero_pct,
        'uwb_x_range_m':   uwb_x_range,
        'uwb_y_range_m':   uwb_y_range,
        'mean_innov_cm':   mean_innov,
        'status': (
            'DROPOUT'  if zero_pct > 10     else
            'BAD'      if mean_innov > 15.0 else
            'MARGINAL' if mean_innov > 5.0  else
            'OK'
        )
    }


# ── 4. ESKF fusion quality ────────────────────────────────────────────────────
def check_eskf(ink: list) -> dict:
    if not ink:
        return {'status': 'NO_INK'}

    kpos   = [_f(r, 'K_pos_diag') for r in ink if _f(r, 'K_pos_diag') > 0]
    innovs = [_f(r, 'innovation_norm') for r in ink]
    ba     = [_f(r, 'b_a_norm') for r in ink]
    p_tr   = [_f(r, 'P_pos_trace') for r in ink]

    mean_k    = np.mean(kpos) if kpos else 0.0
    mean_innov= np.mean(innovs) * 100
    mean_ba   = np.mean(ba)
    mean_p    = np.mean(p_tr)

    # Count how many times K_pos hit 1.0 (hard reject then snap)
    k_max_hits = sum(1 for k in kpos if k > 0.99)

    return {
        'mean_K_pos':      mean_k,
        'mean_innov_cm':   mean_innov,
        'mean_b_a':        mean_ba,
        'mean_P_trace':    mean_p,
        'k_max_hits':      k_max_hits,
        'status': (
            'UWB_DOMINANT' if mean_k > 0.3      else
            'HIGH_INNOV'   if mean_innov > 10.0  else
            'HIGH_BIAS'    if mean_ba > 0.08     else
            'OK'
        )
    }


# ── 5. Stroke quality ─────────────────────────────────────────────────────────
def check_stroke(ink: list, true_start=None, true_end=None) -> dict:
    if len(ink) < 10:
        return {'status': 'TOO_SHORT'}

    fus_xs = [_f(r, 'fused_x') for r in ink]
    fus_ys = [_f(r, 'fused_y') for r in ink]

    coeffs = np.polyfit(fus_xs, fus_ys, 1)
    y_fit  = np.polyval(coeffs, fus_xs)
    rmse   = float(np.sqrt(np.mean((np.array(fus_ys) - y_fit)**2)))

    x_range = max(fus_xs) - min(fus_xs)
    y_range = max(fus_ys) - min(fus_ys)

    # Position error vs known ground truth
    pos_err = None
    if true_start and true_end:
        start_err = math.sqrt((_f(ink[0], 'fused_x') - true_start[0])**2 +
                              (_f(ink[0], 'fused_y') - true_start[1])**2)
        end_err   = math.sqrt((_f(ink[-1], 'fused_x') - true_end[0])**2 +
                              (_f(ink[-1], 'fused_y') - true_end[1])**2)
        pos_err = (start_err, end_err)

    return {
        'rmse_mm':   rmse * 1000,
        'x_range_m': x_range,
        'y_range_m': y_range,
        'slope':     coeffs[0],
        'pos_err':   pos_err,
        'status': (
            'GOOD'     if rmse < 0.010 else
            'MARGINAL' if rmse < 0.030 else
            'BAD'      if rmse < 0.100 else
            'TERRIBLE'
        )
    }


# ── 6. Recommendations ────────────────────────────────────────────────────────
def recommend(imu_ax, imu_drift, uwb, eskf, stroke) -> list[str]:
    recs = []

    if imu_ax.get('status') == 'AXIS_SWAPPED':
        recs.append(
            "IMU AXES SWAPPED: acc_hp_y >> acc_hp_x during horizontal stroke. "
            "Try swapping board_axes in config: board_axes: ('z', 'x') instead of ('x', 'z'). "
            "Or rotate the BNO085 mount 90°."
        )
    if imu_ax.get('status') == 'GRAVITY_LEAK':
        recs.append(
            "GRAVITY LEAK: acc_tip_mag is large when static. "
            "The BNO085 gravity compensation is not working correctly. "
            "Check acc_is_linear=True in config and verify BNO085 is in linear accel mode."
        )
    if imu_drift.get('status') in ('SEVERE', 'BAD'):
        recs.append(
            f"IMU DRIFT SEVERE: Y drifts {imu_drift.get('y_range_m',0)*100:.1f}cm "
            f"vs X travel {imu_drift.get('x_range_m',0)*100:.1f}cm. "
            "Try increasing hpf_cutoff_hz from 0.75 to 1.5 to remove more bias. "
            "Also increase sigma_b_a to 0.005 for faster bias correction."
        )
    if uwb.get('status') == 'DROPOUT':
        recs.append(
            f"UWB DROPOUTS: {uwb.get('zero_dropout_pct',0):.1f}% of rows have Y=0. "
            "A2/A3 simultaneous dropout. Check anchor wiring and TDMA timing."
        )
    if uwb.get('status') == 'BAD':
        recs.append(
            f"HIGH UWB INNOVATION: {uwb.get('mean_innov_cm',0):.1f}cm mean. "
            "Recalibrate range offsets. Run calibrate_offsets.py."
        )
    if eskf.get('status') == 'UWB_DOMINANT':
        recs.append(
            f"UWB DOMINATING INK: K_pos mean={eskf.get('mean_K_pos',0):.3f}. "
            "Raise sigma_uwb to 0.12 and lower pos_gain_cap to 0.030."
        )
    if eskf.get('status') == 'HIGH_BIAS':
        recs.append(
            f"HIGH IMU BIAS: b_a_norm={eskf.get('mean_b_a',0):.4f} m/s2. "
            "Raise sigma_b_a to 0.005 for faster bias correction by UWB."
        )
    if eskf.get('k_max_hits', 0) > 10:
        recs.append(
            f"FREQUENT INNOVATION REJECTS: K_pos hit 1.0 {eskf['k_max_hits']} times. "
            "Raise innov_hard_reject_m to 0.70 so UWB can re-localize after IMU drift."
        )
    if not recs:
        recs.append("No critical issues found. System looks healthy.")

    return recs


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Pipeline diagnostic tool")
    parser.add_argument("--csv",    default="visualizer_log.csv")
    parser.add_argument("--stroke", type=int, default=None,
                        help="Specific stroke ID to analyze (default: all ink rows)")
    parser.add_argument("--true-start", nargs=2, type=float, default=None,
                        metavar=("X", "Y"),
                        help="Known true start position in metres")
    parser.add_argument("--true-end", nargs=2, type=float, default=None,
                        metavar=("X", "Y"),
                        help="Known true end position in metres")
    args = parser.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"[Error] File not found: {path}")
        return

    rows = load(str(path))
    ink  = (_stroke(rows, args.stroke) if args.stroke is not None
            else _ink(rows))

    true_start = tuple(args.true_start) if args.true_start else None
    true_end   = tuple(args.true_end)   if args.true_end   else None

    print("=" * 62)
    print("  POLYCAST PIPELINE DIAGNOSTIC")
    print(f"  File: {path}  ({len(rows)} rows, {len(ink)} ink rows)")
    print("=" * 62)

    D = "─" * 62

    # ── IMU axes ──────────────────────────────────────────────────
    ax = check_imu_axes(ink)
    print(f"\n{D}\n  1. IMU AXIS HEALTH  [{ax['status']}]")
    print(f"  Mean |acc_hp_x|:  {ax.get('mean_hp_x',0):.4f} m/s²")
    print(f"  Mean |acc_hp_y|:  {ax.get('mean_hp_y',0):.4f} m/s²")
    print(f"  Y/X ratio:        {ax.get('hp_y_x_ratio',0):.2f}x  (target <1.5 for horizontal stroke)")
    print(f"  Static acc mag:   {ax.get('static_acc_mag',0):.4f} m/s²  (target <0.1)")

    # ── IMU drift ──────────────────────────────────────────────────
    dr = check_imu_drift(ink)
    print(f"\n{D}\n  2. IMU DRIFT  [{dr['status']}]")
    print(f"  X travel:    {dr.get('x_range_m',0)*100:.1f}cm")
    print(f"  Y drift:     {dr.get('y_range_m',0)*100:.1f}cm  (target <20% of X travel)")
    print(f"  Drift ratio: {dr.get('drift_ratio',0):.2f}  (target <0.2)")
    print(f"  RMSE:        {dr.get('rmse_mm',0):.1f}mm")

    # ── UWB ────────────────────────────────────────────────────────
    uw = check_uwb(rows, ink)
    print(f"\n{D}\n  3. UWB HEALTH  [{uw['status']}]")
    print(f"  Zero dropouts:   {uw.get('zero_dropout_pct',0):.1f}%  (target <5%)")
    print(f"  UWB X range:     {uw.get('uwb_x_range_m',0)*100:.1f}cm")
    print(f"  UWB Y range:     {uw.get('uwb_y_range_m',0)*100:.1f}cm")
    print(f"  Mean innovation: {uw.get('mean_innov_cm',0):.1f}cm  (target <5cm)")

    # ── ESKF ───────────────────────────────────────────────────────
    es = check_eskf(ink)
    print(f"\n{D}\n  4. ESKF FUSION  [{es['status']}]")
    print(f"  Mean K_pos:    {es.get('mean_K_pos',0):.4f}  (target 0.03-0.10)")
    print(f"  Mean |innov|:  {es.get('mean_innov_cm',0):.1f}cm  (target <5cm)")
    print(f"  Mean b_a:      {es.get('mean_b_a',0):.4f} m/s²  (target <0.05)")
    print(f"  Mean P trace:  {es.get('mean_P_trace',0):.5f}m")
    print(f"  K=1.0 hits:    {es.get('k_max_hits',0)}  (snap/relocalize events)")

    # ── Stroke quality ─────────────────────────────────────────────
    sq = check_stroke(ink, true_start, true_end)
    print(f"\n{D}\n  5. STROKE QUALITY  [{sq['status']}]")
    print(f"  Fused X range:  {sq.get('x_range_m',0)*100:.1f}cm")
    print(f"  Fused Y range:  {sq.get('y_range_m',0)*100:.1f}cm")
    print(f"  Straightness:   {sq.get('rmse_mm',0):.1f}mm RMSE  (target <10mm)")
    print(f"  Line slope:     {sq.get('slope',0):.3f}  (0=horizontal, ±inf=vertical)")
    if sq.get('pos_err'):
        s, e = sq['pos_err']
        print(f"  Start pos err:  {s*1000:.1f}mm vs true {true_start}")
        print(f"  End   pos err:  {e*1000:.1f}mm vs true {true_end}")

    # ── Recommendations ────────────────────────────────────────────
    recs = recommend(ax, dr, uw, es, sq)
    print(f"\n{D}\n  6. RECOMMENDATIONS")
    for i, r in enumerate(recs, 1):
        # Word wrap at 58 chars
        words = r.split()
        line = f"  {i}. "
        for w in words:
            if len(line) + len(w) + 1 > 60:
                print(line)
                line = "     " + w + " "
            else:
                line += w + " "
        print(line)

    print(f"\n{'=' * 62}")


if __name__ == "__main__":
    main()
