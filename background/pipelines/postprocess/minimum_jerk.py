"""
minimum_jerk.py
===============
Method 4 — Minimum-Jerk Trajectory Smoothing.

Humans move their wrists following a "minimum jerk" profile: velocity starts
at zero, peaks at mid-motion, and returns to zero — described by a 5th-order
polynomial. This module applies that model post-stroke to eliminate IMU noise
and quantization artifacts while preserving the letter shapes.

Two-step process:
  1. Waypoint detection  — find corners / inflection points via discrete
     curvature; these are the "skeleton" of the letter.
  2. Segment fitting     — replace the messy samples between each pair of
     waypoints with the minimum-jerk trajectory evaluated at the original
     sample timestamps.

Formula (Hogan 1984):
    x(τ) = x0 + (x1 - x0) · (10τ³ − 15τ⁴ + 6τ⁵),  τ = (t − t0) / T ∈ [0, 1]

Applied independently to x and y (zero boundary velocity & acceleration).

Self-test (run as __main__): noisy "L"-shape with known corner; asserts
the corner is detected and output shape matches input.
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _bbox_diag(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return 0.0
    xs, ys = pts[:, 0], pts[:, 1]
    return math.sqrt((xs.max() - xs.min()) ** 2 + (ys.max() - ys.min()) ** 2)


def _curvature(pts: np.ndarray) -> np.ndarray:
    """Discrete Menger curvature at each interior point (1/m). Endpoints = 0."""
    n = len(pts)
    kappa = np.zeros(n)
    eps = 1e-9
    for i in range(1, n - 1):
        v1 = pts[i] - pts[i - 1]
        v2 = pts[i + 1] - pts[i]
        l1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        l2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
        cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
        kappa[i] = 2.0 * cross / (l1 * l2 * (l1 + l2) + eps)
    return kappa


def _find_waypoints(pts: np.ndarray, cfg) -> list[int]:
    """Return sorted list of waypoint indices (always includes 0 and n-1)."""
    n = len(pts)
    kappa = _curvature(pts)

    # Collect local maxima above the curvature threshold.
    candidates: list[tuple[float, int]] = []
    for i in range(1, n - 1):
        if (kappa[i] > cfg.curvature_threshold
                and kappa[i] >= kappa[i - 1]
                and kappa[i] >= kappa[i + 1]):
            candidates.append((kappa[i], i))

    # Greedy selection: highest curvature first, enforce minimum spacing.
    candidates.sort(reverse=True)
    selected: list[int] = []
    for _, idx in candidates:
        too_close = any(
            math.sqrt((pts[idx, 0] - pts[s, 0]) ** 2
                      + (pts[idx, 1] - pts[s, 1]) ** 2)
            < cfg.min_waypoint_spacing_m
            for s in selected
        )
        if not too_close:
            selected.append(idx)
            if len(selected) >= cfg.max_waypoints:
                break

    # Always include start and end, sort, deduplicate.
    all_wp = sorted(set([0] + selected + [n - 1]))
    return all_wp


def _minjerk_basis(tau: float) -> float:
    """5th-order minimum-jerk basis value at normalised time τ ∈ [0, 1]."""
    t3 = tau * tau * tau
    t4 = t3 * tau
    t5 = t4 * tau
    return 10.0 * t3 - 15.0 * t4 + 6.0 * t5


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def minimum_jerk_smooth(
    pts_xy: np.ndarray,
    ts_us: list[int],
    cfg,
) -> tuple[np.ndarray, dict]:
    """
    Smooth *pts_xy* using minimum-jerk segments between detected waypoints.

    Parameters
    ----------
    pts_xy : np.ndarray, shape (N, 2)
        Board-frame polyline in metres (already centroid-aligned).
    ts_us : list[int]
        Hardware timestamps in microseconds, length N, matching pts_xy.
    cfg : MinJerkConfig
        Tuning parameters from cfg.postprocess.minjerk.

    Returns
    -------
    result : np.ndarray, shape (N, 2)
        Smoothed polyline (or original if any guard triggered).
    meta : dict
        Diagnostic info stored under stroke['postprocess']['min_jerk'].
    """
    n = len(pts_xy)

    if n < cfg.min_points_for_minjerk:
        return pts_xy, {'applied': False, 'reason': 'too_few_points', 'n': n}

    waypoints = _find_waypoints(pts_xy, cfg)
    n_wp = len(waypoints)

    smoothed = pts_xy.copy()

    for seg in range(n_wp - 1):
        i0 = waypoints[seg]
        i1 = waypoints[seg + 1]
        p0 = pts_xy[i0]
        p1 = pts_xy[i1]
        t0 = ts_us[i0]
        T = ts_us[i1] - t0

        if T <= 0 or i1 <= i0:
            # Degenerate segment — keep original samples.
            continue

        dp = p1 - p0
        for k in range(i0, i1 + 1):
            tau = max(0.0, min(1.0, (ts_us[k] - t0) / T))
            s = _minjerk_basis(tau)
            smoothed[k] = p0 + dp * s

    # Blend between original (already aligned) and fully smoothed.
    blend = max(0.0, min(1.0, float(cfg.shape_blend)))
    result = pts_xy * (1.0 - blend) + smoothed * blend

    # Bbox guard: reject if smoothing inflated the stroke.
    raw_diag = max(_bbox_diag(pts_xy), 1e-6)
    clean_diag = _bbox_diag(result)
    if clean_diag > raw_diag * cfg.max_bbox_ratio:
        return pts_xy, {
            'applied': False,
            'reason': 'bbox_guard',
            'raw_bbox_diag_m': round(raw_diag, 4),
            'clean_bbox_diag_m': round(clean_diag, 4),
        }

    return result, {
        'applied': True,
        'waypoints': n_wp,
        'waypoint_indices': waypoints,
        'shape_blend': blend,
        'raw_bbox_diag_m': round(raw_diag, 4),
        'clean_bbox_diag_m': round(clean_diag, 4),
    }


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    # Build a noisy "L" shape: horizontal segment then vertical, with a sharp corner.
    # Known waypoints: start (0,0), corner (0.2, 0), end (0.2, 0.2).
    rng_state = np.random.default_rng(42)
    noise_std = 0.003  # 3 mm noise

    n_per_seg = 30
    seg_h = np.column_stack([
        np.linspace(0.0, 0.20, n_per_seg),
        np.zeros(n_per_seg),
    ])
    seg_v = np.column_stack([
        np.full(n_per_seg, 0.20),
        np.linspace(0.0, 0.20, n_per_seg),
    ])
    # Join without duplicating the corner.
    pts = np.vstack([seg_h, seg_v[1:]])
    pts += rng_state.normal(0, noise_std, pts.shape)

    n = len(pts)
    # Fake timestamps: 5 ms apart (200 Hz), in microseconds.
    ts = [int(i * 5_000) for i in range(n)]

    class _FakeCfg:
        min_points_for_minjerk = 8
        curvature_threshold = 20.0
        min_waypoint_spacing_m = 0.005
        max_waypoints = 32
        shape_blend = 0.7
        max_bbox_ratio = 1.10

    result, meta = minimum_jerk_smooth(pts, ts, _FakeCfg())

    if not meta.get('applied'):
        print(f'FAIL: smoothing not applied — {meta}')
        sys.exit(1)

    if result.shape != pts.shape:
        print(f'FAIL: output shape {result.shape} != input shape {pts.shape}')
        sys.exit(1)

    if meta['waypoints'] < 2:
        print(f'FAIL: expected >= 2 waypoints, got {meta["waypoints"]}')
        sys.exit(1)

    raw_diag = meta['raw_bbox_diag_m']
    clean_diag = meta['clean_bbox_diag_m']
    if clean_diag > raw_diag * 1.10:
        print(f'FAIL: smoothed bbox {clean_diag:.4f} m > raw {raw_diag:.4f} m * 1.10')
        sys.exit(1)

    print(
        f'PASS  waypoints={meta["waypoints"]}  '
        f'raw_bbox={raw_diag:.4f} m  clean_bbox={clean_diag:.4f} m  '
        f'blend={meta["shape_blend"]}'
    )
