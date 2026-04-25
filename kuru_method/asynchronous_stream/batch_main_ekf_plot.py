"""
batch_main_ekf_plot.py -- headless batch generator of main_ekf-style trail PNGs.

Runs the same engine pipeline as main_ekf.py on every CSV in the dataset
folder and saves a 2D whiteboard plot showing:

    blue line  - EKF pen-tip trail (post TrailSmoother), only during writing
    gray dots  - lifted positions (tracked but not drawn)
    orange x   - anchor positions where gate rejected
    blue dot   - UWB-only preprocessed IRLS trail (writing only) for comparison
    red sq     - anchors

Usage (from kuru_method/asynchronous_stream/):
    ASYNC_DATASETS=datasets_str_50hz python batch_main_ekf_plot.py
    # outputs in out_main_ekf/
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

from config         import ANCHORS, MARKER_LENGTH, UWB_OFFSETS
from data_parser    import AsyncDataParser
from preprocessor   import UWBPreprocessor, IMUPreprocessor
from ekf_fusion     import AsyncEKFFusionEngine
from trail_smoother import TrailSmoother
from fusion_engine  import IRLSTrilateration


_TRAIL_SUBSAMPLE = 3
_MAX_TRAIL       = 5000   # generous; CSVs are short


def analyse(csv_path: Path, out_dir: Path):
    parser = AsyncDataParser(csv_path=str(csv_path))
    if not parser.connect():
        return None

    engine      = AsyncEKFFusionEngine()
    uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
    imu_cleaner = IMUPreprocessor()
    smoother    = TrailSmoother()

    # UWB-only baseline (preprocessed IRLS) for direct comparison.
    uwb_only_pre = UWBPreprocessor(offsets=UWB_OFFSETS)
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    irls_only = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    draw_x: list[float] = []
    draw_y: list[float] = []
    lift_x: list[float] = []
    lift_y: list[float] = []
    rej_x:  list[float] = []
    rej_y:  list[float] = []
    uwb_only_x: list[float] = []
    uwb_only_y: list[float] = []

    prev_writing = False
    imu_subsample = 0
    is_writing = False

    try:
        while True:
            pkt = parser.get_packet()
            if pkt == 'EOF':
                break
            if not pkt:
                continue

            if pkt['type'] == 'imu':
                clean = imu_cleaner.process_sample(*pkt['quat'], *pkt['acc'])
                pkt['quat'] = clean[0:4]
                pkt['acc']  = clean[4:7]

                pos, vel, is_writing = engine.process_imu(pkt)
                if pos is None:
                    continue

                tip = engine.tip_position
                draw_pt = tip if tip is not None else pos

                imu_subsample += 1
                if imu_subsample >= _TRAIL_SUBSAMPLE:
                    imu_subsample = 0
                    if is_writing:
                        smoother.push(float(draw_pt[0]), float(draw_pt[1]))
                        sx, sy = smoother.get()
                        draw_x.append(sx)
                        draw_y.append(sy)
                    else:
                        if prev_writing:
                            draw_x.append(float('nan'))
                            draw_y.append(float('nan'))
                            smoother.reset()
                        lift_x.append(float(draw_pt[0]))
                        lift_y.append(float(draw_pt[1]))
                    prev_writing = bool(is_writing)

            elif pkt['type'] == 'uwb':
                _, w, ekf_dists = uwb_cleaner.process(*pkt['dists'])
                pos, vel, accepted, rejected = engine.process_uwb(
                    ekf_dists, w, ts=pkt.get('ts'))
                for i in rejected:
                    rej_x.append(float(engine.anchors[i, 0]))
                    rej_y.append(float(engine.anchors[i, 1]))

                # UWB-only baseline trail (only during writing — same gating as main_ekf draw).
                if is_writing:
                    filtered, _, _ = uwb_only_pre.process(*pkt['dists'])
                    pos_only, _ = irls_only.solve(filtered)
                    uwb_only_x.append(float(pos_only[0]))
                    uwb_only_y.append(float(pos_only[1]))

    finally:
        parser.close()

    # --- plot ---
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1], c='red', marker='s', s=120, zorder=5)
    for i, a in enumerate(ANCHORS):
        ax.text(a[0], a[1] + 0.07, f'A{i}',
                color='red', fontsize=11, ha='center', fontweight='bold')

    if uwb_only_x:
        ax.plot(uwb_only_x, uwb_only_y, '-', lw=1.0, color='orange', alpha=0.55,
                label='UWB-only IRLS (preprocessed)', zorder=2)
    if draw_x:
        ax.plot(draw_x, draw_y, '-', lw=1.8, color='royalblue',
                label='EKF pen tip (writing)', zorder=4)
    if lift_x:
        ax.plot(lift_x, lift_y, '.', color='#888888', ms=3, alpha=0.45,
                label='Lifted (tracked, not drawn)', zorder=3)
    if rej_x:
        ax.plot(rej_x, rej_y, 'x', color='orange', ms=8, mew=1.6,
                alpha=0.6, label='Anchor gate reject', zorder=4)

    ax.set_xlim(-0.3, 1.55)
    ax.set_ylim(-0.3, 1.55)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    n_uwb = len(uwb_only_x)
    n_ekf = sum(1 for x in draw_x if not np.isnan(x))
    n_rej = len(rej_x)
    ax.set_title(f'main_ekf style — {csv_path.stem}\n'
                 f'EKF tip pts={n_ekf}  UWB-only pts={n_uwb}  rej={n_rej}')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f'{csv_path.stem}.png'
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def main():
    here = Path(__file__).resolve().parent
    dataset_dir = here / os.environ.get('ASYNC_DATASETS', 'datasets_str_50hz')
    out_dir = here / os.environ.get('OUT_DIR', 'out_main_ekf')
    csvs = sorted(dataset_dir.glob('*.csv'))
    print(f'Found {len(csvs)} CSVs under {dataset_dir.name}/')
    for p in csvs:
        out = analyse(p, out_dir)
        if out is not None:
            print(f'  saved {out.name}')


if __name__ == '__main__':
    main()
