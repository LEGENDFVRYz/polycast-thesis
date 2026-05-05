"""
corrector.py
============
StrokePostprocessor — orchestrates the two post-stroke correction passes.

Called once per closed stroke (pen-up event) from StrokeReconstructor._close_current(),
after StrokeFinalizationIMUCleaner has already run.

Pass A — Rigid-Body Centroid Alignment (centroid_align.py)
    Translates the entire IMU polyline so its centroid matches the UWB
    centroid buffered during the stroke. Fixes global placement drift.

Pass B — Minimum-Jerk Smoothing (minimum_jerk.py)
    Detects curvature-based waypoints and replaces noisy in-between samples
    with a 5th-order polynomial that mirrors natural wrist kinematics.

The stroke dict is modified in-place (new 'points' key), with the pre-
correction polyline preserved as 'raw_points' for visualizer overlays.
All diagnostics land in stroke['postprocess'].
"""

import numpy as np

from background.pipelines.config import cfg as global_cfg
from background.pipelines.postprocess.centroid_align import align_centroid
from background.pipelines.postprocess.minimum_jerk import minimum_jerk_smooth
from background.pipelines.postprocess.shape_classifier import classify_shape
from background.pipelines.postprocess.shape_snapper import snap_to_shape


class StrokePostprocessor:

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
            Same dict with 'points' replaced by corrected polyline,
            'raw_points' added, and 'postprocess' metadata key added.
        """
        c = self._cfg
        points = stroke.get('points') or []

        if not c.enabled or len(points) < c.min_stroke_points:
            stroke['postprocess'] = {
                'applied': False,
                'reason': 'disabled' if not c.enabled else 'too_short',
                'n_points': len(points),
            }
            return stroke

        # Build numpy arrays once.
        pts_xy = np.array([(float(p[0]), float(p[1])) for p in points], dtype=float)
        ts_us  = [int(p[2]) for p in points]
        uwb_xy = (
            np.array([(float(u[0]), float(u[1])) for u in uwb_points], dtype=float)
            if uwb_points else np.empty((0, 2), dtype=float)
        )

        # ------------------------------------------------------------------
        # Pass A: centroid alignment
        # ------------------------------------------------------------------
        pts_aligned, align_meta = align_centroid(pts_xy, uwb_xy, c.centroid)

        # ------------------------------------------------------------------
        # Pass B-shape: geometric shape classification + snapping.
        # If a primitive is recognised with sufficient confidence, snap the
        # stroke to the ideal shape and skip min-jerk (Pass B-jerk).
        # ------------------------------------------------------------------
        sc = c.shape
        shape_meta = {'applied': False, 'kind': 'freeform', 'gated_minjerk': False}
        jerk_meta  = {'applied': False, 'reason': 'not_run'}

        if sc.enabled:
            fit = classify_shape(pts_aligned, sc)
            if fit.kind != 'freeform':
                pts_snapped, snap_meta = snap_to_shape(pts_aligned, ts_us, fit, sc)
                if snap_meta.get('applied'):
                    pts_smoothed = pts_snapped
                    shape_meta = {
                        **snap_meta,
                        'kind':        fit.kind,
                        'confidence':  round(fit.confidence, 4),
                        'params':      fit.params,
                        'notes':       fit.notes,
                        'gated_minjerk': True,
                    }
                    jerk_meta = {'applied': False, 'reason': 'gated_by_shape'}
                else:
                    # Snap rejected (bbox guard or unknown kind); fall through to min-jerk.
                    shape_meta = {
                        'applied': False,
                        'kind': fit.kind,
                        'confidence': round(fit.confidence, 4),
                        'gated_minjerk': False,
                        'snap_rejected': snap_meta,
                    }
                    pts_smoothed, jerk_meta = minimum_jerk_smooth(pts_aligned, ts_us, c.minjerk)
            else:
                # Freeform — run min-jerk as today.
                shape_meta = {'applied': False, 'kind': 'freeform', 'gated_minjerk': False}
                pts_smoothed, jerk_meta = minimum_jerk_smooth(pts_aligned, ts_us, c.minjerk)
        else:
            # Shape corrector disabled — run min-jerk as today.
            pts_smoothed, jerk_meta = minimum_jerk_smooth(pts_aligned, ts_us, c.minjerk)

        # ------------------------------------------------------------------
        # Clamp to board boundaries (mirror UWB position filter's hard clamp).
        # ------------------------------------------------------------------
        board_w = global_cfg.anchors.board_size_x
        board_h = global_cfg.anchors.board_size_y
        pts_smoothed[:, 0] = np.clip(pts_smoothed[:, 0], 0.0, board_w)
        pts_smoothed[:, 1] = np.clip(pts_smoothed[:, 1], 0.0, board_h)

        # ------------------------------------------------------------------
        # Write back: preserve original as raw_points for overlays.
        # ------------------------------------------------------------------
        stroke['raw_points'] = stroke.get('raw_points', list(points))
        stroke['points'] = [
            (float(x), float(y), int(t))
            for (x, y), t in zip(pts_smoothed, ts_us)
        ]
        stroke['postprocess'] = {
            'applied': True,
            'n_points': len(points),
            'uwb_points_used': int(len(uwb_xy)),
            'centroid_align': align_meta,
            'shape':    shape_meta,
            'min_jerk': jerk_meta,
        }
        return stroke
