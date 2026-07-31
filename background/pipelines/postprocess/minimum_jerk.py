"""
Minimum-jerk trajectory smoothing.

Humans move their wrists following a "minimum jerk" profile: velocity starts
at zero, peaks at mid-motion, and returns to zero, described by a 5th-order
polynomial. This module applies that model post-stroke to eliminate IMU noise
and quantization artifacts while preserving the letter shapes.

Two-step process:
    1.  Waypoint detection  find corners/inflection points via discrete
        curvature; these are the "skeleton" of the letter.
    2.  Segment fitting     replace the messy samples between each pair of
        waypoints with the minimum-jerk trajectory evaluated at the original
        sample timestamps.

Formula (Hogan 1984):
    x(tau) = x0 + (x1 - x0) * (10*tau^3 - 15*tau^4 + 6*tau^5),  tau = (t - t0) / T in [0, 1]

Applied independently to x and y (zero boundary velocity and acceleration).

Usage (Import as helper or run directly to test against a synthetic noisy "L" shape):
    python -m background.pipelines.postprocess.minimum_jerk
"""

import math

import numpy as np


# -----------------------------------------------------------------------------
# Waypoint detection
# -----------------------------------------------------------------------------

def _bounding_box_diagonal(points_xy: np.ndarray) -> float:
    if len(points_xy) < 2:
        return 0.0
    x, y = points_xy[:, 0], points_xy[:, 1]
    return math.sqrt((x.max() - x.min()) ** 2 + (y.max() - y.min()) ** 2)


def _curvature(points_xy: np.ndarray) -> np.ndarray:
    """Discrete Menger curvature at each interior point (1/m). Endpoints are 0."""

    point_count = len(points_xy)
    curvature = np.zeros(point_count)
    epsilon = 1e-9

    for i in range(1, point_count - 1):
        incoming = points_xy[i] - points_xy[i - 1]
        outgoing = points_xy[i + 1] - points_xy[i]
        incoming_length = math.sqrt(incoming[0] ** 2 + incoming[1] ** 2)
        outgoing_length = math.sqrt(outgoing[0] ** 2 + outgoing[1] ** 2)
        cross = abs(incoming[0] * outgoing[1] - incoming[1] * outgoing[0])
        curvature[i] = 2.0 * cross / (
            incoming_length * outgoing_length * (incoming_length + outgoing_length) + epsilon
        )

    return curvature


def _find_waypoints(points_xy: np.ndarray, cfg) -> list[int]:
    """Return sorted waypoint indices (always includes the first and last)."""

    point_count = len(points_xy)
    curvature = _curvature(points_xy)

    # Local maxima above the curvature threshold are waypoint candidates.
    candidates: list[tuple[float, int]] = []
    for i in range(1, point_count - 1):
        if (curvature[i] > cfg.curvature_threshold
                and curvature[i] >= curvature[i - 1]
                and curvature[i] >= curvature[i + 1]):
            candidates.append((curvature[i], i))

    # Greedy selection: highest curvature first, enforce minimum spacing so
    # one sharp corner doesn't get counted as several adjacent waypoints.
    candidates.sort(reverse=True)
    selected: list[int] = []
    for _, index in candidates:
        too_close = any(
            math.sqrt(
                (points_xy[index, 0] - points_xy[kept, 0]) ** 2
                + (points_xy[index, 1] - points_xy[kept, 1]) ** 2
            ) < cfg.min_waypoint_spacing_m
            for kept in selected
        )
        if not too_close:
            selected.append(index)
            if len(selected) >= cfg.max_waypoints:
                break

    return sorted(set([0] + selected + [point_count - 1]))


# -----------------------------------------------------------------------------
# Segment fitting
# -----------------------------------------------------------------------------

def _minimum_jerk_basis(tau: float) -> float:
    """5th-order minimum-jerk basis value at normalized time tau in [0, 1]."""

    tau_cubed = tau * tau * tau
    tau_fourth = tau_cubed * tau
    tau_fifth = tau_fourth * tau
    return 10.0 * tau_cubed - 15.0 * tau_fourth + 6.0 * tau_fifth


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

def minimum_jerk_smooth(
    points_xy: np.ndarray,
    timestamps_us: list[int],
    cfg,
) -> tuple[np.ndarray, dict]:
    """
    Smooth points_xy using minimum-jerk segments between detected waypoints.

    Parameters
    ----------
    points_xy : np.ndarray, shape (N, 2)
        Board-frame polyline in metres (already centroid-aligned).
    timestamps_us : list[int]
        Hardware timestamps in microseconds, length N, matching points_xy.
    cfg : MinJerkConfig
        Tuning parameters from cfg.postprocess.minjerk.

    Returns
    -------
    result : np.ndarray, shape (N, 2)
        Smoothed polyline (or the original if any guard triggered).
    meta : dict
        Diagnostic info stored under stroke['postprocess']['min_jerk'].
    """

    point_count = len(points_xy)

    if point_count < cfg.min_points_for_minjerk:
        return points_xy, {'applied': False, 'reason': 'too_few_points', 'n': point_count}

    waypoints = _find_waypoints(points_xy, cfg)
    waypoint_count = len(waypoints)

    smoothed = points_xy.copy()

    for segment in range(waypoint_count - 1):
        start_index = waypoints[segment]
        end_index = waypoints[segment + 1]
        start_point = points_xy[start_index]
        end_point = points_xy[end_index]
        start_ts = timestamps_us[start_index]
        segment_duration = timestamps_us[end_index] - start_ts

        if segment_duration <= 0 or end_index <= start_index:
            # Degenerate segment - keep the original samples.
            continue

        displacement = end_point - start_point
        for index in range(start_index, end_index + 1):
            tau = max(0.0, min(1.0, (timestamps_us[index] - start_ts) / segment_duration))
            basis = _minimum_jerk_basis(tau)
            smoothed[index] = start_point + displacement * basis

    # Blend between the original (already aligned) and fully smoothed points.
    blend = max(0.0, min(1.0, float(cfg.shape_blend)))
    result = points_xy * (1.0 - blend) + smoothed * blend

    # Bbox guard: reject if smoothing inflated the stroke.
    raw_diagonal = max(_bounding_box_diagonal(points_xy), 1e-6)
    smoothed_diagonal = _bounding_box_diagonal(result)
    if smoothed_diagonal > raw_diagonal * cfg.max_bbox_ratio:
        return points_xy, {
            'applied': False,
            'reason': 'bbox_guard',
            'raw_bbox_diag_m': round(raw_diagonal, 4),
            'clean_bbox_diag_m': round(smoothed_diagonal, 4),
        }

    return result, {
        'applied': True,
        'waypoints': waypoint_count,
        'waypoint_indices': waypoints,
        'shape_blend': blend,
        'raw_bbox_diag_m': round(raw_diagonal, 4),
        'clean_bbox_diag_m': round(smoothed_diagonal, 4),
    }


# -----------------------------------------------------------------------------
# Self-test
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    import sys

    # Noisy "L" shape with a known corner: start (0,0), corner (0.2, 0), end (0.2, 0.2).
    rng = np.random.default_rng(42)
    noise_std_m = 0.003

    points_per_segment = 30
    horizontal_segment = np.column_stack([
        np.linspace(0.0, 0.20, points_per_segment),
        np.zeros(points_per_segment),
    ])
    vertical_segment = np.column_stack([
        np.full(points_per_segment, 0.20),
        np.linspace(0.0, 0.20, points_per_segment),
    ])
    # Join without duplicating the corner.
    points = np.vstack([horizontal_segment, vertical_segment[1:]])
    points += rng.normal(0, noise_std_m, points.shape)

    point_count = len(points)
    # Fake timestamps: 5 ms apart (200 Hz), in microseconds.
    timestamps_us = [int(i * 5_000) for i in range(point_count)]

    class _FakeConfig:
        min_points_for_minjerk = 8
        curvature_threshold = 20.0
        min_waypoint_spacing_m = 0.005
        max_waypoints = 32
        shape_blend = 0.7
        max_bbox_ratio = 1.10

    result, meta = minimum_jerk_smooth(points, timestamps_us, _FakeConfig())

    if not meta.get('applied'):
        print(f'FAIL: smoothing not applied - {meta}')
        sys.exit(1)

    if result.shape != points.shape:
        print(f'FAIL: output shape {result.shape} != input shape {points.shape}')
        sys.exit(1)

    if meta['waypoints'] < 2:
        print(f'FAIL: expected >= 2 waypoints, got {meta["waypoints"]}')
        sys.exit(1)

    raw_diagonal = meta['raw_bbox_diag_m']
    clean_diagonal = meta['clean_bbox_diag_m']
    if clean_diagonal > raw_diagonal * 1.10:
        print(f'FAIL: smoothed bbox {clean_diagonal:.4f} m > raw {raw_diagonal:.4f} m * 1.10')
        sys.exit(1)

    print(
        f'PASS  waypoints={meta["waypoints"]}  '
        f'raw_bbox={raw_diagonal:.4f} m  clean_bbox={clean_diagonal:.4f} m  '
        f'blend={meta["shape_blend"]}'
    )
