"""
layer3_imu_raw.py -- raw IMU sanity: quaternion stability + linear-accel noise floor.

Metrics
-------
    quat_drift_deg        -- max angular distance from median orientation
    acc_mean_xyz          -- mean body-frame linear accel (should be ~0 on static)
    acc_std_xyz           -- per-axis std (noise floor)
    acc_noise_max_std     -- max of the three axis stds
    deadband_ratio        -- acc_noise_max_std / ACC_DEADBAND_MS2
    above_deadband_pct    -- % of samples exceeding ACC_DEADBAND_MS2 magnitude

Pass criteria (stationary)
--------------------------
    quat_drift_deg < 10.0    (hardware-realistic; BNO085 tail deviations are 2-7°)
    max |acc_mean_xyz| < 0.05
    acc_noise_max_std  < 0.25
    deadband_ratio <= 3.0    (BNO085 LINEAR_ACCEL has ~0.12-0.15 m/s^2 residual noise)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common    import (ensure_out, collect, LayerResult, DATASET_DIR,
                        stems_matching)
from imu_integrator import IMUIntegrator


LAYER = 'layer3_imu_raw'


def _quat_conjugate(q):
    qx, qy, qz, qw = q
    return np.array([-qx, -qy, -qz, qw])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_angle_between(q1, q2) -> float:
    """Angle (radians) between two unit quaternions representing orientations."""
    qr = _quat_mul(q1, _quat_conjugate(q2))
    qw = max(-1.0, min(1.0, float(qr[3])))
    return 2.0 * np.arccos(abs(qw))


def analyse(csv_path: Path, is_stationary: bool) -> LayerResult:
    stem = csv_path.stem
    imu_pkts, _ = collect(csv_path)
    if not imu_pkts:
        return LayerResult(LAYER, stem, passed=False,
                           metrics={'imu_count': 0}, notes=['no IMU packets'])

    q = np.array([p['quat'] for p in imu_pkts], dtype=float)
    a = np.array([p['acc']  for p in imu_pkts], dtype=float)
    ts = np.array([p['ts']  for p in imu_pkts], dtype=np.int64)
    t_s = (ts - ts[0]) / 1e6

    # Reference orientation = median component then renormalised
    q_ref = np.median(q, axis=0)
    q_ref = q_ref / (np.linalg.norm(q_ref) + 1e-12)
    drifts = np.array([_quat_angle_between(q_ref, qi) for qi in q])
    drift_deg = np.degrees(np.max(drifts))

    acc_mean = a.mean(axis=0)
    acc_std  = a.std(axis=0)
    acc_std_max = float(acc_std.max())

    deadband = IMUIntegrator.ACC_DEADBAND_MS2
    above_pct = 100.0 * float(np.mean(np.linalg.norm(a, axis=1) > deadband))

    # Item D witness: the post-Item-D capture path runs at 5 ms (200 Hz);
    # legacy captures sit at 10 ms.  Reported but not gated -- Layer 0
    # already enforces the rate band.
    dts_s = np.diff(ts) / 1e6
    mean_dt_s = float(np.mean(dts_s)) if dts_s.size else 0.0

    metrics: dict = {
        'imu_count':         len(imu_pkts),
        'duration_s':        float(t_s[-1]),
        'quat_drift_deg':    float(drift_deg),
        'acc_mean_xyz':      [float(x) for x in acc_mean],
        'acc_std_xyz':       [float(x) for x in acc_std],
        'acc_noise_max_std': acc_std_max,
        'deadband_ratio':    float(acc_std_max / max(deadband, 1e-9)),
        'above_deadband_pct': above_pct,
        'mean_imu_dt_s':      mean_dt_s,
    }

    if is_stationary:
        passed = (
            drift_deg < 10.0                    and
            float(np.max(np.abs(acc_mean))) < 0.05 and
            acc_std_max < 0.25                  and
            metrics['deadband_ratio'] <= 3.0
        )
    else:
        # Rotation datasets: pen tip stationary, body rotating. quat_drift is
        # naturally large (up to 180 deg) because orientation sweeps through
        # the rotation range. above_deadband_pct is near zero because there's
        # no translation. Gate on gravity-compensation quality instead:
        # acc_mean should stay near zero (no phantom translation), and acc_std
        # should be bounded (rotation noise + sensor floor).
        passed = (
            float(np.max(np.abs(acc_mean))) < 0.05 and
            acc_std_max < 0.50
        )

    # ---- Plot ----
    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t_s, a[:, 0], 'tab:red',   lw=0.7, label='ax')
    axes[0].plot(t_s, a[:, 1], 'tab:green', lw=0.7, label='ay')
    axes[0].plot(t_s, a[:, 2], 'tab:blue',  lw=0.7, label='az')
    axes[0].axhline( deadband, color='grey', ls='--', lw=0.8)
    axes[0].axhline(-deadband, color='grey', ls='--', lw=0.8)
    axes[0].set_ylabel('body accel (m/s^2)')
    axes[0].legend(fontsize=8, loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Layer 3 -- IMU raw: {stem}  drift={drift_deg:.2f} deg')

    axes[1].plot(t_s, q[:, 0], label='qx')
    axes[1].plot(t_s, q[:, 1], label='qy')
    axes[1].plot(t_s, q[:, 2], label='qz')
    axes[1].plot(t_s, q[:, 3], label='qw')
    axes[1].set_ylabel('quaternion')
    axes[1].legend(fontsize=8, loc='upper right')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_s, np.degrees(drifts), color='purple', lw=0.8)
    axes[2].set_ylabel('drift from median (deg)')
    axes[2].set_xlabel('time (s)')
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    plot_path = out_dir / f'{stem}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER, dataset=stem, passed=passed, metrics=metrics,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


def run_all() -> list[LayerResult]:
    out = []
    # Stationary position tests
    for s in stems_matching('0s', '1s', '2s', '3s', '4s'):
        out.append(analyse(DATASET_DIR / f'{s}.csv', is_stationary=True))
    # Orientation tests (stationary at center, different marker angles)
    for s in ['NorthS', 'SouthS', 'EastS', 'WestS']:
        p = DATASET_DIR / f'{s}.csv'
        if p.exists():
            out.append(analyse(p, is_stationary=True))
    # Rotation tests (stationary position, body rotating — accel should be ~0)
    for base in ['clockwiseM', 'revclockwiseM', 'mix-mix_method']:
        for s in (base, f'{base}-'):
            p = DATASET_DIR / f'{s}.csv'
            if p.exists():
                out.append(analyse(p, is_stationary=False))
    return out


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*')
    ap.add_argument('--moving', action='store_true',
                    help='treat as moving (relaxed pass criteria)')
    args = ap.parse_args()
    files = ([DATASET_DIR / f'{d}.csv' for d in args.datasets]
             if args.datasets else [DATASET_DIR / '0s.csv'])
    for p in files:
        r = analyse(p, is_stationary=not args.moving)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
