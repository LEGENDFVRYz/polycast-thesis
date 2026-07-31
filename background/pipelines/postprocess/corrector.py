"""
StrokePostprocessor - orchestrates the two post-stroke correction passes.

Called once per closed stroke (pen-up event) from
StrokeReconstructor._close_current(), after StrokeFinalizationIMUCleaner has
already run.

Pass A - Rigid-Body Centroid Alignment (centroid_align module)
    Translates the entire IMU polyline so its centroid matches the UWB
    centroid buffered during the stroke. Fixes global placement drift.

Pass B - Minimum-Jerk Smoothing (minimum_jerk module)
    Detects curvature-based waypoints and replaces noisy in-between samples
    with a 5th-order polynomial that mirrors natural wrist kinematics.

The stroke dict is modified in-place (new 'points' key), with the
pre-correction polyline preserved as 'raw_points' for visualizer overlays.
All diagnostics land in stroke['postprocess'].
"""

import numpy as np

from background.pipelines.config import cfg as global_cfg
from background.pipelines.postprocess.centroid_align import align_centroid
from background.pipelines.postprocess.minimum_jerk import minimum_jerk_smooth


class StrokePostprocessor:
    """Applies centroid alignment then minimum-jerk smoothing to closed strokes."""

    def __init__(self):
        self._cfg = global_cfg.postprocess

    def process(
        self,
        stroke: dict,
        uwb_points: list[tuple[float, float, int]],
    ) -> dict:
        """
        Apply centroid alignment then minimum-jerk smoothing to a closed stroke.

        Parameters
        ----------
        stroke : dict
            Closed stroke dict from StrokeReconstructor (must have 'points',
            'start_ts', 'end_ts', 'closed': True).
        uwb_points : list of (x, y, ts_hw)
            UWB samples buffered during this stroke by UWBStrokeBuffer.

        Returns
        -------
        stroke : dict
            Same dict with 'points' replaced by the corrected polyline,
            'raw_points' added, and a 'postprocess' metadata key added.
        """

        cfg = self._cfg
        points = stroke.get('points') or []

        if not cfg.enabled or len(points) < cfg.min_stroke_points:
            stroke['postprocess'] = {
                'applied': False,
                'reason': 'disabled' if not cfg.enabled else 'too_short',
                'n_points': len(points),
            }
            return stroke

        points_xy = np.array([(float(p[0]), float(p[1])) for p in points], dtype=float)
        timestamps_us = [int(p[2]) for p in points]
        uwb_xy = (
            np.array([(float(u[0]), float(u[1])) for u in uwb_points], dtype=float)
            if uwb_points else np.empty((0, 2), dtype=float)
        )

        # Pass A: centroid alignment.
        aligned_points, align_meta = align_centroid(points_xy, uwb_xy, cfg.centroid)

        # Pass B: minimum-jerk smoothing (operates on the aligned points).
        smoothed_points, jerk_meta = minimum_jerk_smooth(aligned_points, timestamps_us, cfg.minjerk)

        # Clamp to board boundaries (mirrors the UWB position filter's hard clamp).
        board_width = global_cfg.anchors.board_size_x
        board_height = global_cfg.anchors.board_size_y
        smoothed_points[:, 0] = np.clip(smoothed_points[:, 0], 0.0, board_width)
        smoothed_points[:, 1] = np.clip(smoothed_points[:, 1], 0.0, board_height)

        # Write back, preserving the original as raw_points for overlays.
        stroke['raw_points'] = stroke.get('raw_points', list(points))
        stroke['points'] = [
            (float(x), float(y), int(ts))
            for (x, y), ts in zip(smoothed_points, timestamps_us)
        ]
        stroke['postprocess'] = {
            'applied': True,
            'n_points': len(points),
            'uwb_points_used': int(len(uwb_xy)),
            'centroid_align': align_meta,
            'min_jerk': jerk_meta,
        }
        return stroke
