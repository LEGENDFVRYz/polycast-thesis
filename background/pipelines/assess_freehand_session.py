"""
Freehand Session Quality Assessment
=====================================
Run after each freehand collection session to verify data quality
before merging and retraining.

Usage:
    python -m background.pipelines.assess_freehand_session data/odometry/freehand_1.npz
    python -m background.pipelines.assess_freehand_session  (auto-finds latest freehand_*.npz)
"""

from __future__ import annotations
import argparse
import glob
import os
import sys
import numpy as np


def assess(path: str) -> None:
    if not os.path.exists(path):
        sys.exit(f'[ERROR] File not found: {path}')

    d       = np.load(path)
    windows = d['windows']   # (N, W, 7)
    deltas  = d['deltas']    # (N, 2)
    N, W, _ = windows.shape

    dx = deltas[:, 0]
    dy = deltas[:, 1]

    print('=' * 62)
    print(f'  Assessment: {os.path.basename(path)}')
    print('=' * 62)

    # ── 1. Sample count ─────────────────────────────────────────────
    print()
    print('[ 1 ] SAMPLE COUNT')
    grade_n = 'GOOD' if N >= 300 else ('OK' if N >= 150 else 'LOW')
    print(f'  Samples collected : {N}  [{grade_n}]')
    if N < 150:
        print('  TIP: Strokes too short or too many pen lifts.')
        print('       Each stroke must be ≥ 2.84s without lifting.')
    elif N < 300:
        print('  TIP: Acceptable but more is better. Aim for 300+.')

    # ── 2. Label diversity ───────────────────────────────────────────
    print()
    print('[ 2 ] LABEL DIVERSITY  (should cover all directions)')
    dx_std = float(dx.std())
    dy_std = float(dy.std())
    dx_range = float(dx.max() - dx.min())
    dy_range = float(dy.max() - dy.min())

    grade_div = 'GOOD' if (dx_std > 0.15 and dy_std > 0.15) else \
                'OK'   if (dx_std > 0.08 and dy_std > 0.08) else 'POOR'

    print(f'  Dx : min={dx.min():+.3f}m  max={dx.max():+.3f}m'
          f'  std={dx_std:.3f}m  range={dx_range:.3f}m')
    print(f'  Dy : min={dy.min():+.3f}m  max={dy.max():+.3f}m'
          f'  std={dy_std:.3f}m  range={dy_range:.3f}m')
    print(f'  Diversity grade : [{grade_div}]')

    if dx_std < 0.08:
        print('  TIP: Dx std is low — you drew mostly vertical strokes.')
        print('       Add more horizontal and diagonal movement.')
    if dy_std < 0.08:
        print('  TIP: Dy std is low — you drew mostly horizontal strokes.')
        print('       Add more vertical and diagonal movement.')

    # ── 3. Quadrant coverage ─────────────────────────────────────────
    print()
    print('[ 3 ] QUADRANT COVERAGE  (need all 4 directions)')
    q_pp = int(((dx > 0.02) & (dy > 0.02)).sum())    # right + up
    q_pn = int(((dx > 0.02) & (dy < -0.02)).sum())   # right + down
    q_np = int(((dx < -0.02) & (dy > 0.02)).sum())   # left  + up
    q_nn = int(((dx < -0.02) & (dy < -0.02)).sum())  # left  + down
    q_h  = int(((np.abs(dx) > 0.05) & (np.abs(dy) < 0.02)).sum())  # pure horizontal
    q_v  = int(((np.abs(dy) > 0.05) & (np.abs(dx) < 0.02)).sum())  # pure vertical

    total = N
    def pct(n): return f'{n:>4} ({100*n/total:4.1f}%)'

    print(f'  Right+Up   : {pct(q_pp)}')
    print(f'  Right+Down : {pct(q_pn)}')
    print(f'  Left+Up    : {pct(q_np)}')
    print(f'  Left+Down  : {pct(q_nn)}')
    print(f'  Horizontal : {pct(q_h)}')
    print(f'  Vertical   : {pct(q_v)}')

    missing = []
    if q_pp < 5:  missing.append('Right+Up')
    if q_pn < 5:  missing.append('Right+Down')
    if q_np < 5:  missing.append('Left+Up')
    if q_nn < 5:  missing.append('Left+Down')
    if missing:
        print(f'  [WARN] Missing directions: {", ".join(missing)}')
        print('  TIP: Draw diagonal strokes in all 4 diagonal directions.')
    else:
        print('  All 4 quadrants covered. [GOOD]')

    # ── 4. Displacement magnitude distribution ───────────────────────
    print()
    print('[ 4 ] DISPLACEMENT MAGNITUDES  (want short + long strokes)')
    mag = np.sqrt(dx**2 + dy**2)
    short  = int((mag < 0.10).sum())   # < 10cm
    medium = int(((mag >= 0.10) & (mag < 0.50)).sum())
    long_  = int((mag >= 0.50).sum())  # > 50cm

    print(f'  Short  (< 10cm) : {pct(short)}')
    print(f'  Medium (10-50cm): {pct(medium)}')
    print(f'  Long   (> 50cm) : {pct(long_)}')

    if short < N * 0.1:
        print('  TIP: Very few short-displacement samples.')
        print('       Add sessions with slow small strokes (letter-sized).')
    if long_ < N * 0.1:
        print('  TIP: Very few long-displacement samples.')
        print('       Add sessions with slow sweeping strokes across the board.')

    # ── 5. IMU signal health ─────────────────────────────────────────
    print()
    print('[ 5 ] IMU SIGNAL HEALTH')
    acc   = windows[:, :, :3]
    quat  = windows[:, :, 3:]
    qnorm = np.sqrt((quat**2).sum(axis=-1))

    nan_count  = int(np.isnan(windows).sum())
    zero_count = int((np.abs(acc).sum(axis=-1) < 0.001).sum())
    qnorm_ok   = bool(np.allclose(qnorm, 1.0, atol=1e-3))

    print(f'  NaN frames        : {nan_count}   {"[OK]" if nan_count == 0 else "[WARN]"}')
    print(f'  Zero-acc frames   : {zero_count}  {"[OK]" if zero_count == 0 else "[WARN]"}')
    print(f'  Quaternion norms  : {"all ~1.0 [OK]" if qnorm_ok else "[WARN] some not normalised"}')
    print(f'  Acc magnitude     : mean={float(np.sqrt((acc**2).sum(axis=-1)).mean()):.3f} m/s2')

    # ── 6. Compare label diversity to Option 2 ───────────────────────
    print()
    print('[ 6 ] COMPARISON TO OPTION 2')
    opt2_unique = 24   # unique label vectors in option 2
    freehand_unique = len(set((round(x,2), round(y,2))
                              for x,y in zip(dx.tolist(), dy.tolist())))
    print(f'  Option 2 unique labels : {opt2_unique}  (fixed directions)')
    print(f'  This session unique    : {freehand_unique}  (~continuous coverage)')
    if freehand_unique > opt2_unique * 5:
        print('  Label diversity is significantly better than Option 2. [GOOD]')
    else:
        print('  TIP: Low unique label count — try more varied stroke directions.')

    # ── Overall verdict ───────────────────────────────────────────────
    print()
    print('[ VERDICT ]')
    issues = []
    if N < 150:           issues.append('too few samples')
    if dx_std < 0.08:     issues.append('missing horizontal coverage')
    if dy_std < 0.08:     issues.append('missing vertical coverage')
    if missing:           issues.append(f'missing quadrants: {missing}')
    if nan_count > 0:     issues.append('NaN in IMU data')
    if zero_count > 100:  issues.append('many zero-acc frames')

    if not issues:
        print('  PASS - session quality is good. Safe to merge and retrain.')
    else:
        print('  NEEDS IMPROVEMENT:')
        for issue in issues:
            print(f'    - {issue}')
        print()
        print('  Run another session addressing the issues above,')
        print('  then merge both sessions together.')

    print('=' * 62)


def main() -> None:
    p = argparse.ArgumentParser(description='Assess freehand odometry session quality.')
    p.add_argument('path', nargs='?', default=None,
                   help='Path to freehand .npz file (auto-finds latest if omitted)')
    args = p.parse_args()

    if args.path:
        assess(args.path)
    else:
        base    = os.path.join('data', 'odometry')
        matches = sorted(glob.glob(os.path.join(base, 'freehand_*.npz')))
        if not matches:
            sys.exit('[ERROR] No freehand_*.npz files found in data/odometry/')
        latest = matches[-1]
        print(f'Auto-selected: {latest}\n')
        assess(latest)


if __name__ == '__main__':
    main()
