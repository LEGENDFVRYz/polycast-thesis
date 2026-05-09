"""
layer2_uwb_position.py -- UWB-only IRLS trilateration accuracy.

For each dataset the script replays every UWB packet through the same
UWBPreprocessor + IRLSTrilateration pair the EKF cold-starts with,
then evaluates the resulting (x, y) track.

Dataset classes
---------------
    stationary  ('Ns' / 'Ns-')     -> scatter + bias vs ground truth
    line        ('hline', 'vline', 'dline_*')
                                    -> straightness, slope, length
    shape       ('CIRCLE', 'SQUARE', 'TRIANGLE')
                                    -> geometric fit quality
    other                            -> summary stats only

All results go into a shared LayerResult.  The class is classified from the
stem name so you can throw arbitrary datasets at it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common       import (ensure_out, collect, LayerResult, DATASET_DIR,
                           draw_board, stems_matching, list_datasets)
from kuru_method.asynchronous_stream.config        import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from kuru_method.asynchronous_stream.preprocessor  import UWBPreprocessor
from kuru_method.asynchronous_stream.fusion_engine import IRLSTrilateration
from ground_truth  import get_truth
from kuru_method.asynchronous_stream.range_kf      import PerAnchorRangeKFBank
from kuru_method.asynchronous_stream.ekf_fusion    import TightlyCoupledEKF


LAYER = 'layer2_uwb_position'


_POSITION_STEMS    = {'pt_middle', 'pt_middle-', 'middle-',
                      'pt_a', 'pt_aa', 'pt_a1', 'pt_a2'}
_ROTATION_STEMS    = {'rt_circle', 'rt_square', 'rt_triangle',
                      'pt_middle_r', 'pt_middle_rotation', 'pt_middle_rr',
                      'middleCCW_rot-', 'middleCW_rot-'}


def _classify(stem: str) -> str:
    if stem in _POSITION_STEMS:
        return 'stationary'
    if stem in _ROTATION_STEMS:
        return 'rotation'
    s = stem.lower()
    # ct_ = contour-trace, pt_ = point-trace; both produce the same shape kind.
    if s.startswith('ct_hline') or s == 'hline' or s == 'hline-':
        return 'line_h'
    if s.startswith('ct_vline') or s == 'vline' or s == 'vline-':
        return 'line_v'
    if s.startswith('ct_diagonal'):
        return 'line_d02'
    if s.startswith('ct_circle') or s.startswith('pt_circle'):
        return 'shape_circle'
    if s.startswith('ct_square') or s.startswith('pt_square'):
        return 'shape_square'
    if s.startswith('ct_triangle') or s.startswith('pt_triangle'):
        return 'shape_triangle'
    return 'other'


def _solve_all(uwb_pkts) -> np.ndarray:
    pre  = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30,
            1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    out = []
    for pkt in uwb_pkts:
        _, w, des = pre.process(*pkt['dists'])
        pos, _    = irls.solve(des, w)
        out.append([float(pos[0]), float(pos[1])])
    return np.asarray(out, dtype=float)


def _solve_all_kf(uwb_pkts) -> np.ndarray:
    """Item-B witness: route raw ranges through PerAnchorRangeKFBank
    BEFORE the despike/preprocessor stage, then trilaterate.  The
    despiker still runs (mirrors what the EKF sees through Item B)."""
    pre  = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30,
            1.00]
    irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)
    bank = PerAnchorRangeKFBank(
        n_anchors=4,
        sigma_meas=TightlyCoupledEKF.SIGMA_UWB,
    )

    out = []
    prev_ts_us = None
    for pkt in uwb_pkts:
        ts = pkt.get('ts')
        if prev_ts_us is None or ts is None:
            dt = 0.1
        else:
            dt_us = ts - prev_ts_us
            dt = (dt_us / 1e6) if 0 < dt_us < 2_000_000 else 0.1
        if ts is not None:
            prev_ts_us = ts
        # Pre-filter raw ranges with the per-anchor 1D KF.
        kf_ranges = bank.step_all(dt, np.asarray(pkt['dists'], dtype=float))
        d0, d1, d2, d3 = (float(x) for x in kf_ranges)
        _, w, des = pre.process(d0, d1, d2, d3)
        pos, _    = irls.solve(des, w)
        out.append([float(pos[0]), float(pos[1])])
    return np.asarray(out, dtype=float)


# -- Geometric helpers ---------------------------------------------------

def _fit_line(xs: np.ndarray, ys: np.ndarray):
    """Total-least-squares line fit.  Returns (unit_dir, centroid, rms_perp)."""
    c = np.array([xs.mean(), ys.mean()])
    M = np.stack([xs - c[0], ys - c[1]], axis=1)
    _, _, vt = np.linalg.svd(M, full_matrices=False)
    direction = vt[0]
    # RMS perpendicular distance = smallest singular value component
    normal = np.array([-direction[1], direction[0]])
    perp   = M @ normal
    rms    = float(np.sqrt(np.mean(perp ** 2)))
    return direction, c, rms


def _fit_circle(xs: np.ndarray, ys: np.ndarray):
    """Algebraic Kasa circle fit.  Returns (cx, cy, r, rms_residual)."""
    A = np.column_stack([2 * xs, 2 * ys, np.ones_like(xs)])
    b = xs ** 2 + ys ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = math.sqrt(max(0.0, c + cx * cx + cy * cy))
    d = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    return float(cx), float(cy), r, rms


def analyse(csv_path: Path) -> LayerResult:
    stem = csv_path.stem
    _, uwb_pkts = collect(csv_path)
    if not uwb_pkts:
        return LayerResult(LAYER, stem, passed=False,
                           metrics={'uwb_count': 0},
                           notes=['no UWB packets'])

    xy = _solve_all(uwb_pkts)
    xs, ys = xy[:, 0], xy[:, 1]

    # Item B witness — same UWB packets, but raw ranges go through the
    # per-anchor KF bank first.  Report-only RMS-vs-truth and bias.
    xy_kf = _solve_all_kf(uwb_pkts)

    kind = _classify(stem)
    metrics: dict = {'n_points': int(len(xy)), 'kind': kind}
    notes: list[str] = []
    passed = True

    if kind in ('stationary', 'orientation', 'rotation'):
        truth = get_truth(stem)
        mean_xy = xy.mean(axis=0)
        std_xy  = xy.std(axis=0)
        dr      = np.linalg.norm(xy - mean_xy, axis=1)
        r95     = float(np.percentile(dr, 95)) if len(dr) > 5 else 0.0
        metrics.update({
            'mean':  [float(mean_xy[0]), float(mean_xy[1])],
            'std':   [float(std_xy[0]),  float(std_xy[1])],
            'r95_m': r95,
        })

        # KF-routed companion stats (report-only).
        mean_xy_kf = xy_kf.mean(axis=0)
        dr_kf      = np.linalg.norm(xy_kf - mean_xy_kf, axis=1)
        r95_kf     = float(np.percentile(dr_kf, 95)) if len(dr_kf) > 5 else 0.0
        metrics['kf_r95_m'] = r95_kf

        bias_lim, r95_lim = 0.05, 0.08
        if truth is not None:
            bias = float(np.linalg.norm(mean_xy - np.array(truth)))
            metrics['truth'] = truth
            metrics['bias_m'] = bias
            metrics['kf_irls_rms_m'] = float(
                np.sqrt(np.mean(np.sum(
                    (xy_kf - np.asarray(truth)) ** 2, axis=1))))
            metrics['irls_rms_m'] = float(
                np.sqrt(np.mean(np.sum(
                    (xy - np.asarray(truth)) ** 2, axis=1))))
            passed = bias < bias_lim and r95 < r95_lim
        else:
            notes.append('no ground truth -- passed-criterion uses R95 only')
            passed = r95 < r95_lim

    elif kind.startswith('line'):
        direction, centroid, rms_perp = _fit_line(xs, ys)
        span = float(np.linalg.norm(
            xy[np.argmax(xy @ direction)] - xy[np.argmin(xy @ direction)]))
        angle_deg = math.degrees(math.atan2(direction[1], direction[0]))
        expected_deg = {'line_h':   0.0,
                        'line_v':   90.0,
                        'line_d02': math.degrees(math.atan2(1.24,  1.25)),
                        'line_d31': math.degrees(math.atan2(-1.24, 1.25))}
        exp = expected_deg.get(kind, 0.0)
        # Lines are direction-agnostic mod 180.  Err is in [0, 90].
        diff = abs(angle_deg - exp) % 180.0
        err  = min(diff, 180.0 - diff)
        metrics.update({
            'angle_deg':      angle_deg,
            'expected_deg':   exp,
            'angle_err_deg':  err,
            'span_m':         span,
            'rms_perp_m':     rms_perp,
        })
        passed = rms_perp < 0.05 and err < 10.0

    elif kind == 'shape_circle':
        cx, cy, r, rms = _fit_circle(xs, ys)
        metrics.update({
            'center':  [cx, cy],
            'radius_m': r,
            'rms_radial_m': rms,
        })
        passed = rms < 0.05 and 0.05 < r < 0.70

    elif kind in ('shape_square', 'shape_triangle', 'shape_star'):
        # Convex hull area as a rough proxy
        try:
            from scipy.spatial import ConvexHull
            hull = ConvexHull(xy)
            area = float(hull.volume)
        except Exception:
            area = float('nan')
        metrics.update({
            'hull_area_m2': area,
            'bbox_w':       float(xs.max() - xs.min()),
            'bbox_h':       float(ys.max() - ys.min()),
        })
        passed = math.isfinite(area) and area > 0.02
    else:
        metrics.update({
            'bbox_w': float(xs.max() - xs.min()),
            'bbox_h': float(ys.max() - ys.min()),
        })

    # ---- Plot ----
    out_dir = ensure_out(LAYER)
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_board(ax)
    ax.scatter(xs, ys, s=8, c='royalblue', alpha=0.55,
               label='IRLS position')

    if kind == 'stationary' and 'truth' in metrics:
        tx, ty = metrics['truth']
        ax.plot(tx, ty, '+', ms=18, mew=3, color='limegreen',
                label='truth')
    if kind.startswith('line'):
        direction, centroid, _ = _fit_line(xs, ys)
        t = np.linspace(-0.8, 0.8, 2)
        line = centroid + t[:, None] * direction
        ax.plot(line[:, 0], line[:, 1], 'g--', lw=1.5, label='fit')
    if kind == 'shape_circle':
        cx, cy, r, _ = _fit_circle(xs, ys)
        theta = np.linspace(0, 2 * math.pi, 120)
        ax.plot(cx + r * np.cos(theta), cy + r * np.sin(theta),
                'g--', lw=1.5, label='fit')

    ax.set_title(f'Layer 2 -- IRLS position: {stem}  [{kind}]')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(loc='upper right', fontsize=8)
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
    # Stationary (point-tap and middle-hold)
    stationary = [p.stem for p in list_datasets(lambda s: s in _POSITION_STEMS)]
    # Rotation in place
    rotation = list(_ROTATION_STEMS)
    # Lines (with repeat-count variants -e/-ee/-eee and -l/-ll/-lll)
    lines = stems_matching('ct_hline', 'ct_vline', 'ct_diagonal')
    # Shapes (contour-trace and point-trace variants -e/-ee/-eee)
    shapes = stems_matching('ct_circle', 'ct_square', 'ct_triangle',
                            'pt_circle', 'pt_square', 'pt_triangle')
    interesting = stationary + rotation + lines + shapes

    seen = set()
    out = []
    for s in interesting:
        if s in seen:
            continue
        seen.add(s)
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
              for s in ['pt_middle', 'ct_hline', 'ct_vline', 'ct_circle']])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
