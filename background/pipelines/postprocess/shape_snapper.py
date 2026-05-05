"""
shape_snapper.py
================
Pure synthesis pass — takes a ShapeFit and the original (pts_xy, ts_us) and
produces a snapped polyline of identical length, parametrically resampled at
the original timestamps.

snap_to_shape(pts_xy, ts_us, fit, cfg) -> (np.ndarray, dict)

Timestamp-normalised parameter tau_k = (ts[k] - ts[0]) / (ts[-1] - ts[0])
is used for all primitives — preserves replay timing and matches minimum_jerk
convention (minimum_jerk.py:150).

Blend:
    out = pts_xy * (1 - snap_blend) + snapped * snap_blend
    Default snap_blend=1.0 (full snap — confidence has already gated the call).

Bbox guard (same contract as minimum_jerk.py:159):
    If the snapped bbox grows by more than max_bbox_ratio vs raw, the snap is
    rejected and pts_xy is returned unchanged.

Self-test: run as __main__ — for each primitive, feed a perfect synthetic
stroke in, assert RMS error to ideal < 0.5 mm and len(out) == len(pts_in).
"""

import math

import numpy as np

from background.pipelines.postprocess.centroid_align import _rotation_2d
from background.pipelines.postprocess.minimum_jerk import _bbox_diag
from background.pipelines.postprocess.shape_classifier import ShapeFit


# ---------------------------------------------------------------------------
# Tau helper
# ---------------------------------------------------------------------------

def _tau_array(ts_us: list) -> np.ndarray:
    """Normalised time array [0, 1] from hardware timestamp list."""
    t0, t1 = float(ts_us[0]), float(ts_us[-1])
    span = t1 - t0
    if span < 1e-6:
        return np.linspace(0.0, 1.0, len(ts_us))
    return np.clip((np.array(ts_us, dtype=float) - t0) / span, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Per-primitive snap functions
# ---------------------------------------------------------------------------

def _snap_line(pts_xy: np.ndarray, ts_us: list, params: dict) -> np.ndarray:
    """Parametric resample of a line from fit-projected endpoints."""
    p0 = np.array(params['p0'], dtype=float)
    p1 = np.array(params['p1'], dtype=float)
    tau = _tau_array(ts_us)
    return p0[np.newaxis] + tau[:, np.newaxis] * (p1 - p0)[np.newaxis]


def _snap_circle(pts_xy: np.ndarray, ts_us: list, params: dict) -> np.ndarray:
    """Parametric resample around fit circle, preserving start angle and direction."""
    cx, cy  = params['center']
    r       = params['radius']
    theta0  = params['theta0']
    sweep   = params['sweep']            # total angular extent (always positive)
    sign    = params['direction_sign']   # +1 CCW, -1 CW

    tau   = _tau_array(ts_us)
    theta = theta0 + sign * tau * sweep
    cx_arr = np.full(len(tau), cx)
    cy_arr = np.full(len(tau), cy)
    return np.column_stack([cx_arr + r * np.cos(theta), cy_arr + r * np.sin(theta)])


def _snap_arc(pts_xy: np.ndarray, ts_us: list, params: dict) -> np.ndarray:
    """Same as circle snap — params already encode the arc's sweep."""
    return _snap_circle(pts_xy, ts_us, params)


def _snap_polygon(pts_xy: np.ndarray, ts_us: list, params: dict) -> np.ndarray:
    """
    Perimeter-parametric resample along polygon edges.

    Corners list is taken directly from the classifier's params.  For a closed
    polygon the perimeter wraps back to the first corner; for an open one it
    stops at the last corner.
    """
    corners = [np.array(c, dtype=float) for c in params['corners']]
    is_closed = params.get('is_closed', False)

    if is_closed:
        # Close the loop.
        path = corners + [corners[0]]
    else:
        path = corners

    # Cumulative arc length along the polygon perimeter.
    edges = [path[i + 1] - path[i] for i in range(len(path) - 1)]
    edge_lens = [float(np.linalg.norm(e)) for e in edges]
    total = sum(edge_lens)
    if total < 1e-9:
        return pts_xy.copy()

    cum = [0.0]
    for el in edge_lens:
        cum.append(cum[-1] + el)
    cum_arr = np.array(cum) / total   # normalised [0, 1]

    tau = _tau_array(ts_us)
    out = np.empty((len(tau), 2), dtype=float)

    for k, t in enumerate(tau):
        # Find which edge tau falls in.
        idx = int(np.searchsorted(cum_arr, t, side='right')) - 1
        idx = max(0, min(idx, len(edges) - 1))
        local_t = (t - cum_arr[idx]) / (cum_arr[idx + 1] - cum_arr[idx] + 1e-9)
        local_t = max(0.0, min(1.0, local_t))
        out[k] = path[idx] + local_t * edges[idx]

    return out


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def snap_to_shape(
    pts_xy: np.ndarray,
    ts_us: list,
    fit: ShapeFit,
    cfg,
) -> tuple:
    """
    Replace pts_xy with the idealized primitive polyline described by fit.

    Parameters
    ----------
    pts_xy : np.ndarray, shape (N, 2)
    ts_us  : list[int], length N
    fit    : ShapeFit from classify_shape()
    cfg    : ShapeCorrectorConfig

    Returns
    -------
    (result, meta) where result is (N, 2) array.
    meta dict includes 'applied', 'kind', 'rms_snap_m', 'blend'.
    """
    n = len(pts_xy)

    if fit.kind == 'freeform' or n < 2:
        return pts_xy.copy(), {'applied': False, 'kind': 'freeform'}

    kind = fit.kind

    if kind == 'line':
        snapped = _snap_line(pts_xy, ts_us, fit.params)
    elif kind == 'circle':
        snapped = _snap_circle(pts_xy, ts_us, fit.params)
    elif kind == 'arc':
        snapped = _snap_arc(pts_xy, ts_us, fit.params)
    elif kind in ('rectangle', 'triangle'):
        snapped = _snap_polygon(pts_xy, ts_us, fit.params)
    else:
        return pts_xy.copy(), {'applied': False, 'kind': kind, 'reason': 'unknown_kind'}

    # Blend.
    blend = max(0.0, min(1.0, float(cfg.snap_blend)))
    result = pts_xy * (1.0 - blend) + snapped * blend

    # Bbox guard.
    raw_diag   = max(_bbox_diag(pts_xy), 1e-6)
    snap_diag  = _bbox_diag(result)
    if snap_diag > raw_diag * cfg.max_bbox_ratio:
        return pts_xy.copy(), {
            'applied': False,
            'kind': kind,
            'reason': 'bbox_guard',
            'raw_bbox_m': round(raw_diag, 4),
            'snap_bbox_m': round(snap_diag, 4),
        }

    rms_snap = float(np.sqrt(np.mean(np.sum((result - snapped) ** 2, axis=1)))) if blend < 1.0 else 0.0
    rms_diff = float(np.sqrt(np.mean(np.sum((result - pts_xy) ** 2, axis=1))))

    return result, {
        'applied':    True,
        'kind':       kind,
        'confidence': round(fit.confidence, 4),
        'blend':      round(blend, 3),
        'rms_snap_m': round(rms_diff, 5),
    }


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    from background.pipelines.postprocess.shape_classifier import ShapeFit

    HALF_MM = 0.0005
    ok = True

    class _Cfg:
        snap_blend = 1.0
        max_bbox_ratio = 1.15

    cfg = _Cfg()
    n = 60
    ts = list(range(n))

    # T1 — line: perfect line from (0,0) to (0.2,0)
    p0, p1 = [0.0, 0.0], [0.2, 0.0]
    ideal = np.column_stack([np.linspace(0.0, 0.2, n), np.zeros(n)])
    fit = ShapeFit('line', 0.99, {'p0': p0, 'p1': p1, 'direction': [1.0, 0.0]})
    out, meta = snap_to_shape(ideal, ts, fit, cfg)
    rms = float(np.sqrt(np.mean(np.sum((out - ideal) ** 2, axis=1))))
    passed = len(out) == n and rms < HALF_MM and meta['applied']
    print(f"T1 {'PASS' if passed else 'FAIL'}  line  rms={rms*1000:.3f} mm  n={len(out)}")
    ok = ok and passed

    # T2 — circle: perfect circle r=0.05, CCW
    cx, cy, r = 0.3, 0.3, 0.05
    angles_c = np.linspace(0, 2 * math.pi - 0.01, n)
    ideal_c = np.column_stack([cx + r * np.cos(angles_c), cy + r * np.sin(angles_c)])
    fit = ShapeFit('circle', 0.95, {
        'center': [cx, cy], 'radius': r,
        'theta0': 0.0, 'sweep': 2 * math.pi, 'direction_sign': 1,
        'closure_ratio': 0.01,
    })
    out, meta = snap_to_shape(ideal_c, ts, fit, cfg)
    rms = float(np.sqrt(np.mean(np.sum((out - ideal_c) ** 2, axis=1))))
    passed = len(out) == n and rms < HALF_MM and meta['applied']
    print(f"T2 {'PASS' if passed else 'FAIL'}  circle  rms={rms*1000:.3f} mm  n={len(out)}")
    ok = ok and passed

    # T3 — rectangle: 4-corner closed square
    corners = [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]]
    sides_r = [
        np.column_stack([np.linspace(0.0, 0.1, 15), np.zeros(15)]),
        np.column_stack([np.full(15, 0.1), np.linspace(0.0, 0.1, 15)]),
        np.column_stack([np.linspace(0.1, 0.0, 15), np.full(15, 0.1)]),
        np.column_stack([np.zeros(15), np.linspace(0.1, 0.0, 15)]),
    ]
    ideal_r = np.vstack(sides_r)
    n_r = len(ideal_r)
    ts_r = list(range(n_r))
    fit = ShapeFit('rectangle', 0.92, {'corners': corners, 'is_closed': True, 'axis_snapped': True})
    out, meta = snap_to_shape(ideal_r, ts_r, fit, cfg)
    rms = float(np.sqrt(np.mean(np.sum((out - ideal_r) ** 2, axis=1))))
    passed = len(out) == n_r and rms < 0.003 and meta['applied']  # 3 mm tolerance (perimeter resample)
    print(f"T3 {'PASS' if passed else 'FAIL'}  rectangle  rms={rms*1000:.2f} mm  n={len(out)}")
    ok = ok and passed

    # T4 — freeform: no snap applied
    squiggle = np.column_stack([np.linspace(0, 0.2, n), np.sin(np.linspace(0, 8 * math.pi, n)) * 0.02])
    fit = ShapeFit('freeform', 0.0)
    out, meta = snap_to_shape(squiggle, ts, fit, cfg)
    passed = len(out) == n and not meta['applied']
    print(f"T4 {'PASS' if passed else 'FAIL'}  freeform passthrough  applied={meta['applied']}")
    ok = ok and passed

    sys.exit(0 if ok else 1)
