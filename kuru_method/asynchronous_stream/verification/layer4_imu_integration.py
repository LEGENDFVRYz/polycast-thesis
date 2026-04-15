"""
layer4_imu_integration.py -- body->whiteboard transform, heading lock, tag_z.

Uses the production IMUIntegrator and UWBPreprocessor + IRLSTrilateration so
we measure the same pipeline the EKF uses.

Tests per dataset
-----------------
    1. Heading lock convergence: samples-until-lock, locked vector,
       post-lock drift (max angle from lock direction).
    2. Heading vs IRLS-motion direction: compare locked heading vector to
       the principal-axis direction of the IRLS trace.  A ~90 deg mismatch
       means the body-axis mapping is wrong.
    3. Dead-reckoning drift between UWB resets (every 2.0 s) -- max
       distance from IRLS anchor position during the resetting window.
    4. Tag-z correctness: for every sample, compute the assumed tag_z
       (fixed = MARKER_LENGTH) vs the actual tag_z = MARKER_LENGTH * cos(tilt)
       derived from the quaternion's body-X world direction.

Pass criteria
-------------
    samples_until_lock <= 80
    post_lock_drift_deg < 3.0
    heading_vs_motion_deg < 20.0  (only for cardinal datasets)
    dr_max_drift_m < 0.10
    tag_z_error_mean_m < 0.01
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common        import (ensure_out, replay, LayerResult, DATASET_DIR,
                            draw_board)
from config         import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from imu_integrator import IMUIntegrator, quat_to_rotmat
from preprocessor   import UWBPreprocessor
from fusion_engine  import IRLSTrilateration


LAYER = 'layer4_imu_integration'

DR_RESET_S = 2.0   # seconds between UWB-anchored DR resets


def _principal_angle_deg(xy: np.ndarray) -> float:
    c = xy.mean(axis=0)
    M = xy - c
    _, _, vt = np.linalg.svd(M, full_matrices=False)
    d = vt[0]
    return math.degrees(math.atan2(d[1], d[0]))


def analyse(csv_path: Path, expected_heading_deg: float | None = None) -> LayerResult:
    stem = csv_path.stem

    integ    = IMUIntegrator()
    pre      = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    samples_until_lock = None
    locked_vec = None
    post_lock_drifts: list[float] = []
    tag_z_errors: list[float] = []
    heading_trace: list[np.ndarray] = []

    # Dead-reckoning vs IRLS anchoring
    dr_pos = None
    dr_vel = np.zeros(2)
    last_imu_ts: int | None = None
    last_reset_ts: int | None = None
    dr_errors: list[float] = []

    irls_xys: list[np.ndarray] = []
    dr_xys:  list[np.ndarray] = []

    n_imu = 0
    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            n_imu += 1
            q = pkt['quat']
            a = pkt['acc']

            # Tag-z from quaternion: body-X world direction (board normal).
            R = quat_to_rotmat(*q)
            body_x_world = R[:, 0]
            # Component along world Z (up) -- if marker is vertical with tip
            # on board and tag above, this would be 0.  The actual tag-height
            # above the board depends on the marker angle from the board
            # surface.  Here we approximate tag_z using body_x's vertical
            # projection: tag_z = MARKER_LENGTH * |cos(tilt_from_board_normal)|.
            # Since body-X maps to board normal in the ideal pose, the
            # board-normal component of body-X equals the horizontal-plane
            # projection length.
            horiz = float(np.sqrt(body_x_world[0] ** 2 + body_x_world[1] ** 2))
            assumed = MARKER_LENGTH
            # Tag height above whiteboard (surface) when tip is on the board:
            # If the marker axis makes angle theta with the board surface,
            # tag_z = MARKER_LENGTH * sin(theta).  Board is vertical (wall),
            # so "height above board" corresponds to horizontal distance from
            # the wall in world-frame -- equal to |body_X . n_board|.
            # With no known board-normal in the capture, we use the fact that
            # at the neutral pose body_X points away from the board, so the
            # magnitude of body_X's horizontal component equals the
            # board-normal projection.  Perfect pose: horiz = 1.
            # Tag_z assumed by the 2-D projection in ekf_fusion.update_uwb
            # is the constant MARKER_LENGTH.  The actual equivalent is
            # MARKER_LENGTH * horiz.
            actual = MARKER_LENGTH * horiz
            tag_z_errors.append(abs(actual - assumed))

            _ = integ.get_wb_acceleration(q, a)
            if integ.heading_locked and samples_until_lock is None:
                samples_until_lock = n_imu
                locked_vec = integ.heading_vec.copy()
            if integ.heading_locked:
                heading_trace.append(integ.heading_vec.copy())

            # Dead-reckoning integration (only once EKF warm)
            ts = pkt['ts']
            if dr_pos is not None and last_imu_ts is not None:
                dt = max(0.0, min(0.1, (ts - last_imu_ts) / 1e6))
                a_wb = integ.get_wb_acceleration(q, a)
                dr_vel += a_wb * dt
                dr_pos += dr_vel * dt
            last_imu_ts = ts

        elif pkt['type'] == 'uwb':
            _, w, des = pre.process(*pkt['dists'])
            pos, _ = irls.solve(des, w)
            irls_xy = np.array([pos[0], pos[1]], dtype=float)
            irls_xys.append(irls_xy)
            if dr_pos is None:
                dr_pos = irls_xy.copy()
                dr_vel[:] = 0.0
                last_reset_ts = pkt['ts']
            else:
                dr_errors.append(float(np.linalg.norm(dr_pos - irls_xy)))
                if last_reset_ts is None or (pkt['ts'] - last_reset_ts) / 1e6 > DR_RESET_S:
                    dr_pos = irls_xy.copy()
                    dr_vel[:] = 0.0
                    last_reset_ts = pkt['ts']
            if dr_pos is not None:
                dr_xys.append(dr_pos.copy())

    # -- Post-processing metrics --
    metrics: dict = {
        'n_imu': n_imu,
        'samples_until_lock': samples_until_lock,
    }
    notes: list[str] = []

    if locked_vec is None:
        passed = False
        notes.append('heading never locked')
        heading_angle = None
    else:
        metrics['locked_vec'] = [float(locked_vec[0]), float(locked_vec[1])]
        heading_angle = math.degrees(math.atan2(locked_vec[1], locked_vec[0]))
        metrics['locked_heading_deg'] = heading_angle

        # Drift: max angle between any post-lock vec and the initial lock vec
        if heading_trace:
            h0 = heading_trace[0]
            max_drift = max(math.degrees(math.acos(
                float(np.clip(np.dot(h0, h), -1.0, 1.0))))
                for h in heading_trace)
        else:
            max_drift = 0.0
        metrics['post_lock_drift_deg'] = max_drift

    # Tag-z error stats
    if tag_z_errors:
        metrics['tag_z_error_mean_m'] = float(np.mean(tag_z_errors))
        metrics['tag_z_error_max_m']  = float(np.max(tag_z_errors))

    # Heading vs motion direction (only if we have enough IRLS points)
    hvsm = None
    if len(irls_xys) > 10 and heading_angle is not None:
        motion_angle = _principal_angle_deg(np.asarray(irls_xys))
        diff = abs(motion_angle - heading_angle) % 180.0
        hvsm = float(min(diff, 180.0 - diff))
        metrics['motion_angle_deg']      = motion_angle
        metrics['heading_vs_motion_deg'] = hvsm

    # DR drift stat
    if dr_errors:
        metrics['dr_drift_mean_m'] = float(np.mean(dr_errors))
        metrics['dr_drift_max_m']  = float(np.max(dr_errors))

    # -- Pass criteria --
    passed_checks = []
    if samples_until_lock is not None:
        passed_checks.append(samples_until_lock <= 80)
    else:
        passed_checks.append(False)
    if 'post_lock_drift_deg' in metrics:
        passed_checks.append(metrics['post_lock_drift_deg'] < 3.0)
    if 'tag_z_error_mean_m' in metrics:
        passed_checks.append(metrics['tag_z_error_mean_m'] < 0.01)
    if 'dr_drift_max_m' in metrics:
        passed_checks.append(metrics['dr_drift_max_m'] < 0.10)
    if expected_heading_deg is not None and hvsm is not None:
        passed_checks.append(hvsm < 20.0)
    passed = all(passed_checks) if passed_checks else False

    # -- Plot --
    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    ax = axes[0]
    draw_board(ax)
    if irls_xys:
        ir = np.asarray(irls_xys)
        ax.plot(ir[:, 0], ir[:, 1], 'b.-', lw=0.6, ms=3, alpha=0.6,
                label='IRLS')
    if dr_xys:
        dr = np.asarray(dr_xys)
        ax.plot(dr[:, 0], dr[:, 1], 'r.-', lw=0.6, ms=3, alpha=0.6,
                label='Dead-reckoning (2s resets)')
    if locked_vec is not None:
        c = np.array([0.6, 0.1])
        ax.annotate('', xy=(c[0] + 0.3 * locked_vec[0], c[1] + 0.3 * locked_vec[1]),
                    xytext=(c[0], c[1]),
                    arrowprops=dict(arrowstyle='->', color='orange', lw=2))
        ax.text(c[0], c[1] - 0.04, f'heading {heading_angle:.1f} deg',
                fontsize=8, color='orange')
    ax.set_title(f'Layer 4 -- body->wb + heading: {stem}')
    ax.legend(loc='upper right', fontsize=8)

    ax2 = axes[1]
    if tag_z_errors:
        ax2.hist(np.asarray(tag_z_errors) * 100.0, bins=40, color='slateblue')
    ax2.set_xlabel('|tag_z - MARKER_LENGTH|  (cm)')
    ax2.set_ylabel('count')
    ax2.set_title('Tag-z projection error')
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    plot_path = out_dir / f'{stem}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER, dataset=stem, passed=passed,
        metrics=metrics, notes=notes,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


CARDINAL_HEADING = {
    'NorthS':  90.0,
    'SouthS': -90.0,
    'EastS':    0.0,
    'WestS':  180.0,
}


def run_all() -> list[LayerResult]:
    out = []
    for stem, heading in CARDINAL_HEADING.items():
        p = DATASET_DIR / f'{stem}.csv'
        if p.exists():
            out.append(analyse(p, expected_heading_deg=heading))
    for stem in ['hline', 'vline']:
        p = DATASET_DIR / f'{stem}.csv'
        if p.exists():
            out.append(analyse(p))
    return out


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*')
    args = ap.parse_args()
    files = ([DATASET_DIR / f'{d}.csv' for d in args.datasets]
             if args.datasets else
             [DATASET_DIR / f'{s}.csv'
              for s in ['NorthS', 'EastS', 'hline', 'vline']])
    for p in files:
        exp = CARDINAL_HEADING.get(p.stem)
        r = analyse(p, expected_heading_deg=exp)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
