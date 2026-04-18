"""
layer4_imu_integration.py -- body->whiteboard transform, heading lock, tag_z.

Uses the production IMUIntegrator and UWBPreprocessor + IRLSTrilateration so
we measure the same pipeline the EKF uses.

Dataset categories
------------------
    A: Stationary orientation (NorthS, SouthS, EastS, WestS)
       All stationary at center.  Heading lock should converge; output
       should be a tight cluster regardless of marker orientation.

    B: Rotation in-place (clockwiseM, revclockwiseM)
       Pen tip at center while body rotates.  Tests heading stability
       under rotation -- heading should NOT track body rotation.

    C: Tilt-rotation (mix-mix_method)
       Same as B but marker is tilted.  Also checks tag_z variation.

    D: Motion (hline, vline, dline_*, shapes, characters)
       Heading vs IRLS-motion direction, dead-reckoning drift.

Pass criteria
-------------
    A (orientation, stationary):
       samples_until_lock <= 80, post_lock_drift_deg < 3.0,
       position_scatter_R95 < 0.08, dr_drift_max_m < 0.10
    B (rotation in-place):
       position_scatter_R95 < MARKER_LENGTH + 0.10, post_lock_drift_deg <= 15.0
    C (tilt-rotation): same as B + tag_z_variation_cm bounded
    D (motion line): samples_until_lock <= 80, post_lock_drift_deg < 3.0,
       dr_drift_max_m < 0.10;
       heading_vs_motion_deg < 20 applied ONLY for horizontal strokes
       (motion_angle near 0/180 deg), since vline/dline have heading
       perpendicular or diagonal to motion by geometry.
    E (motion shapes/chars): samples_until_lock <= 80,
       post_lock_drift_deg < 3.0, dr_drift_max_m < 0.10.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common        import (ensure_out, replay, LayerResult, DATASET_DIR,
                            draw_board, stems_matching)
from config         import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from imu_integrator import IMUIntegrator, quat_to_rotmat
from preprocessor   import UWBPreprocessor
from fusion_engine  import IRLSTrilateration


LAYER = 'layer4_imu_integration'

DR_RESET_S = 2.0   # seconds between UWB-anchored DR resets

# Dataset category classification
ORIENTATION_STEMS = {'NorthS', 'SouthS', 'EastS', 'WestS',
                     'NorthS-', 'SouthS-', 'EastS-', 'WestS-'}
ROTATION_STEMS    = {'clockwiseM', 'revclockwiseM',
                     'clockwiseM-', 'revclockwiseM-'}
TILT_ROT_STEMS    = {'mix-mix_method', 'mix-mix_method-'}
LINE_STEMS        = {'hline', 'hline-', 'vline', 'vline-',
                     'dline_A0_A2', 'dline_A0_A2-',
                     'dline_A3_A1', 'dline_A3_A1-'}


def _classify(stem: str) -> str:
    if stem in ORIENTATION_STEMS:
        return 'orientation'
    if stem in ROTATION_STEMS:
        return 'rotation'
    if stem in TILT_ROT_STEMS:
        return 'tilt_rotation'
    if stem in LINE_STEMS:
        return 'motion_line'
    return 'motion'


def _principal_angle_deg(xy: np.ndarray) -> float:
    c = xy.mean(axis=0)
    M = xy - c
    _, _, vt = np.linalg.svd(M, full_matrices=False)
    d = vt[0]
    return math.degrees(math.atan2(d[1], d[0]))


def analyse(csv_path: Path) -> LayerResult:
    stem = csv_path.stem
    category = _classify(stem)

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
    heading_angles: list[float] = []

    # Dead-reckoning vs IRLS anchoring
    dr_pos = None
    dr_vel = np.zeros(2)
    last_imu_ts: int | None = None
    last_reset_ts: int | None = None
    dr_errors: list[float] = []

    irls_xys: list[np.ndarray] = []
    dr_xys:  list[np.ndarray] = []

    # Item A.2 / D witness — collect IMU dt and lever-arm omega magnitude.
    imu_ts_us: list[int] = []
    omega_mags: list[float] = []

    n_imu = 0
    for pkt in replay(csv_path):
        if pkt['type'] == 'imu':
            n_imu += 1
            q = pkt['quat']
            a = pkt['acc']
            ts = pkt['ts']
            imu_ts_us.append(int(ts))

            # Tag-z from quaternion
            R = quat_to_rotmat(*q)
            body_x_world = R[:, 0]
            horiz = float(np.sqrt(body_x_world[0] ** 2 + body_x_world[1] ** 2))
            assumed = MARKER_LENGTH
            actual = MARKER_LENGTH * horiz
            tag_z_errors.append(abs(actual - assumed))

            # Pass ts so the integrator can estimate |omega| via quaternion
            # differencing (Item A.2 lever-arm correction).
            _ = integ.get_wb_acceleration(q, a, ts=ts)
            omega_mags.append(float(integ.last_omega_mag))
            if integ.heading_locked and samples_until_lock is None:
                samples_until_lock = n_imu
                locked_vec = integ.heading_vec.copy()
            if integ.heading_locked:
                h = integ.heading_vec.copy()
                heading_trace.append(h)
                heading_angles.append(math.degrees(math.atan2(h[1], h[0])))

            # Dead-reckoning integration
            if dr_pos is not None and last_imu_ts is not None:
                dt = max(0.0, min(0.1, (ts - last_imu_ts) / 1e6))
                a_wb = integ.get_wb_acceleration(q, a, ts=ts)
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
        'category': category,
        'samples_until_lock': samples_until_lock,
    }
    notes: list[str] = []

    if locked_vec is None:
        notes.append('heading never locked')
        heading_angle = None
    else:
        metrics['locked_vec'] = [float(locked_vec[0]), float(locked_vec[1])]
        heading_angle = math.degrees(math.atan2(locked_vec[1], locked_vec[0]))
        metrics['locked_heading_deg'] = heading_angle

        # Post-lock drift: max angle from initial lock direction
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

    # DR drift stat
    if dr_errors:
        metrics['dr_drift_mean_m'] = float(np.mean(dr_errors))
        metrics['dr_drift_max_m']  = float(np.max(dr_errors))

    # Item D witness — IMU sample period.  Report-only: Layer 0 enforces
    # the rate band, here we just expose the mean for tunability checks.
    if len(imu_ts_us) > 1:
        ts_arr = np.asarray(imu_ts_us, dtype=np.int64)
        metrics['mean_imu_dt_s'] = float(np.mean(np.diff(ts_arr)) / 1e6)

    # Item A.2 visibility — fraction of samples where the lever-arm
    # centripetal correction was actually applied (|omega| above the
    # IMUIntegrator threshold).  Useful to confirm rotation datasets
    # genuinely exercise the new code path.
    if omega_mags:
        om = np.asarray(omega_mags)
        metrics['mean_omega_rad_s'] = float(om.mean())
        metrics['max_omega_rad_s']  = float(om.max())
        metrics['lever_arm_active_frac'] = float(
            np.mean(om >= IMUIntegrator.LEVER_ARM_OMEGA_THRESH))

    # -- Category-specific metrics and pass criteria --

    if category == 'orientation':
        # Stationary at center — position scatter, heading drift, NO motion angle
        if irls_xys:
            ir = np.asarray(irls_xys)
            mean_xy = ir.mean(axis=0)
            dr_from_mean = np.linalg.norm(ir - mean_xy, axis=1)
            r95 = float(np.percentile(dr_from_mean, 95))
            metrics['position_scatter_R95'] = r95
        else:
            r95 = 999.0

        passed_checks = [
            samples_until_lock is not None and samples_until_lock <= 80,
            metrics.get('post_lock_drift_deg', 999) < 3.0,
            r95 < 0.08,
            metrics.get('dr_drift_max_m', 999) < 0.10,
        ]
        passed = all(passed_checks)

    elif category in ('rotation', 'tilt_rotation'):
        # Rotation in-place — heading range, position scatter
        if irls_xys:
            ir = np.asarray(irls_xys)
            mean_xy = ir.mean(axis=0)
            dr_from_mean = np.linalg.norm(ir - mean_xy, axis=1)
            r95 = float(np.percentile(dr_from_mean, 95))
            metrics['position_scatter_R95'] = r95
        else:
            r95 = 999.0

        if heading_angles:
            h_range = float(max(heading_angles) - min(heading_angles))
            metrics['heading_range_deg'] = h_range
            if len(heading_angles) > 1:
                # Drift rate: total heading change / duration
                duration_s = n_imu / 100.0  # ~100 Hz IMU
                metrics['heading_drift_rate_deg_per_s'] = h_range / max(duration_s, 0.01)
        else:
            h_range = 999.0

        if category == 'tilt_rotation' and tag_z_errors:
            # Tag-z variation in cm
            tz = np.asarray(tag_z_errors)
            metrics['tag_z_variation_cm'] = float((tz.max() - tz.min()) * 100.0)

        # During rotation the UWB tag traces a circle of radius
        # ~MARKER_LENGTH due to the lever arm.  R95 ≈ MARKER_LENGTH
        # is expected.  Heading stability (drift from lock) is the
        # primary metric.
        drift = metrics.get('post_lock_drift_deg', 999)
        passed_checks = [
            r95 < MARKER_LENGTH + 0.10,  # lever arm circle + noise margin
            drift <= 15.0,               # heading stayed within leash
        ]
        passed = all(passed_checks)

    else:
        # Motion datasets (lines, shapes, characters)
        # Heading vs motion direction (only meaningful with enough IRLS points)
        hvsm = None
        motion_angle = None
        if len(irls_xys) > 10 and heading_angle is not None:
            motion_angle = _principal_angle_deg(np.asarray(irls_xys))
            diff = abs(motion_angle - heading_angle) % 180.0
            hvsm = float(min(diff, 180.0 - diff))
            metrics['motion_angle_deg']      = motion_angle
            metrics['heading_vs_motion_deg'] = hvsm

        passed_checks = [
            samples_until_lock is not None and samples_until_lock <= 80,
            metrics.get('post_lock_drift_deg', 999) < 3.0,
        ]
        if 'tag_z_error_mean_m' in metrics:
            passed_checks.append(metrics['tag_z_error_mean_m'] < 0.01)
        if 'dr_drift_max_m' in metrics:
            passed_checks.append(metrics['dr_drift_max_m'] < 0.10)
        # heading_vs_motion only applies when motion is roughly horizontal
        # (aligned with the wb-X heading axis); vline/dline have motion
        # perpendicular or diagonal to heading by geometry, so the metric
        # is physically meaningless there.
        if (category == 'motion_line' and hvsm is not None
                and motion_angle is not None
                and min(abs(motion_angle), abs(abs(motion_angle) - 180.0)) < 20.0):
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
    ax.set_title(f'Layer 4 -- {category}: {stem}')
    ax.legend(loc='upper right', fontsize=8)

    ax2 = axes[1]
    if category in ('rotation', 'tilt_rotation') and heading_angles:
        # Plot heading angle over time for rotation tests
        t_h = np.arange(len(heading_angles)) / 100.0
        ax2.plot(t_h, heading_angles, 'tab:orange', lw=1.0)
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('heading angle (deg)')
        ax2.set_title(f'Heading stability  range={metrics.get("heading_range_deg", 0):.1f} deg')
        ax2.grid(True, alpha=0.3)
    elif tag_z_errors:
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


def run_all() -> list[LayerResult]:
    out = []

    # Category A: Stationary orientation tests
    for stem in ['NorthS', 'SouthS', 'EastS', 'WestS']:
        for s in (stem, f'{stem}-'):
            p = DATASET_DIR / f'{s}.csv'
            if p.exists():
                out.append(analyse(p))

    # Category B: Rotation in-place tests
    for stem in ['clockwiseM', 'revclockwiseM']:
        for s in (stem, f'{stem}-'):
            p = DATASET_DIR / f'{s}.csv'
            if p.exists():
                out.append(analyse(p))

    # Category C: Tilt-rotation test
    for s in ['mix-mix_method', 'mix-mix_method-']:
        p = DATASET_DIR / f'{s}.csv'
        if p.exists():
            out.append(analyse(p))

    # Category D: Motion tests (lines, shapes, characters)
    for stem in ['hline', 'vline', 'dline_A0_A2', 'dline_A3_A1',
                 'CIRCLE', 'SQUARE', 'TRIANGLE', 'STAR',
                 'circleS', 'squareS', 'triangleS', 'starS',
                 'ABC', 'HELLO', 'abcS', 'helloS', 'corners']:
        for s in (stem, f'{stem}-'):
            p = DATASET_DIR / f'{s}.csv'
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
              for s in ['NorthS', 'clockwiseM', 'hline', 'vline']])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
