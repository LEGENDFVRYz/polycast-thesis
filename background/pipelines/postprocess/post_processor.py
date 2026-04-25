"""
[postprocess/post_processor] — StrokePostProcessor orchestrator

Consumes a closed stroke dict (from StrokeReconstructor) and the ESKF
instance whose _rts_buf holds the matching per-step history.  Calls
NoteSmoother to run RTS + spline and returns the "dry ink" stroke dict.

Usage (in visualizer.py or any driver):

    from background.pipelines.postprocess import StrokePostProcessor

    eskf      = ESKF()
    smoother  = StrokePostProcessor(eskf)

    # ... after StrokeReconstructor emits a closed stroke:
    dry = smoother.process_closed_stroke(closed_stroke)
    if dry is not None:
        # render / swap the smoothed polyline
        pass
"""

from __future__ import annotations

import warnings

from background.pipelines.config import cfg
from background.pipelines.postprocess.note_smoother import NoteSmoother


class StrokePostProcessor:
    """Orchestrates RTS + spline smoothing when a stroke closes.

    Parameters
    ----------
    eskf : ESKF
        Live ESKF instance.  Must already have `get_rts_history_slice()`
        (added in eskf.py alongside _rts_buf).
    """

    def __init__(self, eskf) -> None:
        self._eskf    = eskf
        self._smoother = NoteSmoother()

    @property
    def enabled(self) -> bool:
        return cfg.postprocess.rts_enabled

    def process_closed_stroke(self, stroke: dict) -> dict | None:
        """Smooth a closed stroke and return the "dry ink" result.

        Returns None when:
          - rts_enabled is False
          - The stroke has fewer than 2 points
          - The ESKF history slice for this stroke_id is empty or too long

        On any unexpected error a warning is issued and None is returned so
        that the live forward-path rendering is never disrupted.
        """
        if not self.enabled:
            return None

        if len(stroke.get('points', [])) < 2:
            return None

        sid = stroke.get('stroke_id')
        if sid is None:
            return None

        try:
            history_slice = self._eskf.get_rts_history_slice(sid)
        except Exception as exc:
            warnings.warn(f"[postprocess] get_rts_history_slice failed: {exc}")
            return None

        if not history_slice:
            return None

        max_n = cfg.postprocess.rts_max_stroke_samples
        if len(history_slice) > max_n:
            warnings.warn(
                f"[postprocess] stroke {sid} has {len(history_slice)} samples "
                f"(max={max_n}). Falling back to forward output."
            )
            return None

        try:
            dry = self._smoother.smooth_stroke(stroke, history_slice)
        except Exception as exc:
            warnings.warn(f"[postprocess] smooth_stroke failed for stroke {sid}: {exc}")
            return None

        return dry
