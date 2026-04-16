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
                           draw_board, stems_matching)
from config        import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from preprocessor  import UWBPreprocessor
from fusion_engine import IRLSTrilateration
from ground_truth  import get_truth


LAYER = 'layer2_uwb_position'


_STATIONARY_STEMS = {'0s', '0s-', '1s', '1s-', '2s', '2s-', '3s', '3s-', '4s', '4s-',
                     'NorthS', 'NorthS-', 'SouthS', 'SouthS-',
                     'EastS', 'EastS-', 'WestS', 'WestS-',
                     'clockwiseM', 'clockwiseM-', 'revclockwiseM', 'revclockwiseM-',
                     'mix-mix_method', 'mix-mix_method-'}


def _classify(stem: str) -> str:
    if stem in _STATIONARY_STEMS:
        return 'stationary'
    s = stem.lower()
    if s.startswith('hline'):
        return 'line_h'
    if s.startswith('vline'):
        return 'line_v'
    if s.startswith('dline_a0_a2'):
        return 'line_d02'
    if s.startswith('dline_a3_a1'):
        return 'line_d31'
    if s.startswith('circle'):
        return 'shape_circle'
    if s.startswith('square'):
        return 'shape_square'
    if s.startswith('triangle'):
        return 'shape_triangle'
    if s.startswith('star'):
        return 'shape_star'
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

    kind = _classify(stem)
    metrics: dict = {'n_points': int(len(xy)), 'kind': kind}
    notes: list[str] = []
    passed = True

    if kind == 'stationary':
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
        if truth is not None:
            bias = float(np.linalg.norm(mean_xy - np.array(truth)))
            metrics['truth'] = truth
            metrics['bias_m'] = bias
            passed = bias < 0.05 and r95 < 0.08
        else:
            notes.append('no ground truth -- passed-criterion uses R95 only')
            passed = r95 < 0.08

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
    interesting = (
        stems_matching('0s', '1s', '2s', '3s', '4s') +
        # Orientation tests (stationary at center)
        ['NorthS', 'NorthS-', 'SouthS', 'SouthS-',
         'EastS', 'EastS-', 'WestS', 'WestS-',
         # Rotation tests (stationary at center)
         'clockwiseM', 'clockwiseM-', 'revclockwiseM', 'revclockwiseM-',
         'mix-mix_method', 'mix-mix_method-',
         # Lines
         'hline', 'hline-', 'vline', 'vline-',
         'dline_A0_A2', 'dline_A0_A2-', 'dline_A3_A1', 'dline_A3_A1-',
         # Shapes (large + small)
         'CIRCLE', 'CIRCLE-', 'SQUARE', 'SQUARE-',
         'TRIANGLE', 'TRIANGLE-', 'STAR', 'STAR-',
         'circleS', 'circleS-', 'squareS', 'squareS-',
         'triangleS', 'triangleS-', 'starS', 'starS-',
         # Characters
         'ABC', 'ABC-', 'HELLO', 'HELLO-',
         'abcS', 'abcS-', 'helloS', 'helloS-',
         # Corner visit
         'corners', 'corners-']
    )
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
              for s in ['0s', 'hline', 'vline', 'CIRCLE']])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
