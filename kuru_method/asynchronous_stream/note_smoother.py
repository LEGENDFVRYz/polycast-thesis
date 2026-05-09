"""
note_smoother.py -- Priority 6 OFFLINE per-stroke smoother for saved notes.

Online and offline smoothing have different objectives:
    * Online (display)  : minimum latency, runs every IMU step. See
                          `trail_smoother.OnlineTrailSmoother`.
    * Offline (saved)   : maximum quality, runs once per finished stroke.
                          That's what this module does.

The offline pass is a Rauch-Tung-Striebel backward smoother over the EKF
state/covariance log of each stroke. It strictly improves over the
forward EKF posterior because future measurements inform past states.
The implementation reuses `ekf_fusion.rts_smooth_history` so live and
offline filters share the same numerics.

Usage
-----
    note = NoteSmoother()
    # Each stroke is recorded between an FSR rise and the matching fall.
    # The caller passes the slice of EKF history that occurred while
    # is_writing == True for that stroke.
    smoothed_xy = note.smooth_stroke(history_slice)
    # Optional cubic-spline resampling of the RTS output for clean
    # rendered notes (off by default; enable with the spline flag).
"""

from __future__ import annotations

from typing import Iterable, List

import numpy as np

from kuru_method.asynchronous_stream.ekf_fusion import rts_smooth_history


class NoteSmoother:
    """Per-stroke RTS smoother for saved notes.

    Stateless across strokes — each call to smooth_stroke() is independent,
    so hover positions or other strokes cannot bleed in. If you want a
    spline pass on top, pass `spline=True` and `n_resample=N`.
    """

    def __init__(self, spline: bool = False, n_resample: int | None = None):
        self.spline      = bool(spline)
        self.n_resample  = n_resample

    # ------------------------------------------------------------------
    def smooth_stroke(self, history_slice: List[dict]) -> np.ndarray:
        """
        Parameters
        ----------
        history_slice : list of EKF history records (each {'x','P','F','Q'})
                        covering exactly one stroke (rising→falling FSR).

        Returns
        -------
        smoothed_xy : np.ndarray shape (M, 2)
        """
        if not history_slice:
            return np.zeros((0, 2))

        xy = rts_smooth_history(history_slice)
        if self.spline and len(xy) >= 4:
            xy = self._spline_resample(xy, self.n_resample)
        return xy

    def smooth_strokes(self,
                       strokes: Iterable[List[dict]]) -> List[np.ndarray]:
        """Convenience: run smooth_stroke over a list of stroke slices."""
        return [self.smooth_stroke(s) for s in strokes]

    # ------------------------------------------------------------------
    @staticmethod
    def _spline_resample(xy: np.ndarray,
                         n_resample: int | None) -> np.ndarray:
        """Cubic-spline reparameterisation of an RTS-smoothed stroke.

        Uses arclength as the parameter so closely-spaced samples don't
        blow up the spline. Off by default; controlled by NoteSmoother
        config flag.
        """
        from scipy.interpolate import CubicSpline

        # Arc-length parameterisation
        diffs = np.diff(xy, axis=0)
        seglen = np.linalg.norm(diffs, axis=1)
        s = np.concatenate([[0.0], np.cumsum(seglen)])
        if s[-1] <= 0:
            return xy
        s_norm = s / s[-1]

        n_out = n_resample if n_resample is not None else len(xy)
        u = np.linspace(0.0, 1.0, n_out)
        cs_x = CubicSpline(s_norm, xy[:, 0])
        cs_y = CubicSpline(s_norm, xy[:, 1])
        return np.stack([cs_x(u), cs_y(u)], axis=1)
