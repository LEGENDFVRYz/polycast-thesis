"""
postprocess/__init__.py — Post-stroke smoothing, triggered on every pen-up.

This module wires the two separate smoothers together:

  1. OnlineTrailSmoother
     Short causal weighted moving average.  Used as a fallback for completed
     strokes and mirrored by visualizer.py during active live drawing.

  2. NoteSmoother
     Offline per-stroke RTS smoother.  Used after stroke completion when the
     fusion engine exposes ESKF history for the closed stroke.
"""

from __future__ import annotations

import numpy as np

from background.pipelines.config import cfg
from background.pipelines.postprocess.trail_smoother import OnlineTrailSmoother
from background.pipelines.postprocess.note_smoother import NoteSmoother


class StrokePostProcessor:
    """Applies post-stroke smoothing to closed fused strokes.

    Mode is read from cfg.postprocess.smoothing_mode at construction time.
    - 'online'  → replay the closed stroke through OnlineTrailSmoother.
    - 'offline' → run NoteSmoother RTS using ESKF stroke history; fall back to
                  online if history is unavailable.
    """

    def __init__(self, fusion_engine=None):
        self._fusion = fusion_engine
        self._mode = cfg.postprocess.smoothing_mode
        self._pcfg = cfg.postprocess
        self._online_smoother = OnlineTrailSmoother()
        self._note_smoother = NoteSmoother()

    def process_closed_stroke(self, stroke: dict) -> dict | None:
        """Smooth a closed stroke and return a new stroke dict.

        Input stroke must contain at least:
            {'points': [(x, y, ts), ...], 'stroke_id': int}
        """
        if not getattr(self._pcfg, 'trail_enabled', True):
            return None

        pts = stroke.get('points', [])
        if len(pts) < 2:
            return None

        if self._mode == 'offline':
            return self._process_offline(stroke, pts)
        return self._process_online(stroke, pts)

    def _process_online(self, stroke: dict, pts: list) -> dict:
        self._online_smoother.reset()
        smoothed_pts = []
        for x, y, ts in pts:
            self._online_smoother.push(x, y)
            sx, sy = self._online_smoother.get()
            smoothed_pts.append((sx, sy, ts))

        return {**stroke, 'points': smoothed_pts, 'smoothed': True, 'smoother': 'online_wma'}

    def _process_offline(self, stroke: dict, pts: list) -> dict | None:
        if self._fusion is None:
            return self._process_online(stroke, pts)

        stroke_id = int(stroke.get('stroke_id', 0))
        history = self._fusion.get_rts_history_slice(stroke_id)
        if not history:
            # Complementary mode or missing ESKF history: no RTS possible.
            return self._process_online(stroke, pts)

        smoothed_xy = self._note_smoother.smooth_stroke(history)
        if len(smoothed_xy) == 0:
            return self._process_online(stroke, pts)

        # Preserve temporal ordering. If spline resampling changed the number of
        # points, spread timestamps across the original stroke time span.
        ts_arr = np.array([t for _, _, t in pts], dtype=np.int64)
        final_ts = np.interp(
            np.linspace(0, 1, len(smoothed_xy)),
            np.linspace(0, 1, len(ts_arr)),
            ts_arr,
        ).astype(np.int64)

        final_pts = [(float(x), float(y), int(t))
                     for (x, y), t in zip(smoothed_xy, final_ts)]

        return {
            **stroke,
            'points': final_pts,
            'smoothed': True,
            'smoother': 'rts_note_smoother',
            'n_raw': len(history),
        }
