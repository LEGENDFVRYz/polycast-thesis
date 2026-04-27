"""
postprocess/__init__.py — Post-stroke smoothing via OnlineTrailSmoother.

Applied once per closed stroke (pen-up event), not during live drawing.
The smoother is reset before each stroke so there is no cross-stroke bleed.
"""

from background.pipelines.postprocess.trail_smoother import OnlineTrailSmoother
from background.pipelines.config import cfg


class StrokePostProcessor:
    """Applies trail smoothing to the fused (x, y) points of a closed stroke."""

    def __init__(self, fusion_engine=None):
        # fusion_engine kept for API compatibility; not used without RTS.
        self._smoother = OnlineTrailSmoother()

    def process_closed_stroke(self, stroke: dict) -> dict | None:
        """Smooth the closed stroke's fused points and return the result.

        Args:
            stroke: dict with at minimum {'points': [(x, y, ts), ...], ...}

        Returns:
            Modified stroke dict with smoothed points and 'smoothed': True,
            or None if the stroke is too short to process.
        """
        if not cfg.postprocess.trail_enabled:
            return None

        pts = stroke.get('points', [])
        if len(pts) < 2:
            return None

        self._smoother.reset()
        smoothed_pts = []
        for x, y, ts in pts:
            self._smoother.push(x, y)
            sx, sy = self._smoother.get()
            smoothed_pts.append((sx, sy, ts))

        return {
            **stroke,
            'points':   smoothed_pts,
            'smoothed': True,
        }
