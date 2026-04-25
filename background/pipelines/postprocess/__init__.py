"""
[postprocess] — Offline / post-stroke refinement for completed strokes.

Modules:
  rts_smoother   — Rauch-Tung-Striebel backward Kalman pass (pure math).
  note_smoother  — Arc-length spline resample over RTS output.
  post_processor — Orchestrator: consumes closed strokes + ESKF history,
                   emits smoothed "dry ink" stroke dicts.
"""
from __future__ import annotations


def __getattr__(name: str):
    if name == 'rts_smooth_history':
        from background.pipelines.postprocess.rts_smoother import rts_smooth_history
        return rts_smooth_history
    if name == 'NoteSmoother':
        from background.pipelines.postprocess.note_smoother import NoteSmoother
        return NoteSmoother
    if name == 'StrokePostProcessor':
        from background.pipelines.postprocess.post_processor import StrokePostProcessor
        return StrokePostProcessor
    raise AttributeError(
        f"module 'background.pipelines.postprocess' has no attribute {name!r}"
    )


__all__ = ['rts_smooth_history', 'NoteSmoother', 'StrokePostProcessor']
