"""
note_smoother.py — OFFLINE per-stroke smoother for completed strokes.

This is the companion to trail_smoother.py:

- trail_smoother.OnlineTrailSmoother
    Runs during active writing.  It receives only (x, y) points and applies a
    short causal weighted moving average, so it has near-zero latency.

- note_smoother.NoteSmoother
    Runs after a stroke closes.  It receives the ESKF state/covariance history
    for exactly that stroke and applies a Rauch–Tung–Striebel backward pass.
    Optional arc-length spline resampling can then clean the rendered note.

The two smoothers are intentionally separate so live latency and final-note
quality can be tuned independently.
"""

from __future__ import annotations

from typing import Iterable, List

import numpy as np

from background.pipelines.config import cfg


class NoteSmoother:
    """Per-stroke RTS smoother for saved/closed strokes.

    Stateless across strokes: every call receives one closed stroke history
    slice and returns a new (N, 2) array.  Hover/relocation samples must not be
    included in the passed history slice.
    """

    def __init__(self,
                 spline: bool | None = None,
                 ds_m: float | None = None,
                 min_points: int | None = None,
                 smoothing_factor: float | None = None):
        pcfg = cfg.postprocess
        self.spline = bool(getattr(pcfg, 'spline_enabled', True) if spline is None else spline)
        self.ds_m = float(pcfg.spline_resample_ds_m if ds_m is None else ds_m)
        self.min_points = int(pcfg.spline_min_points if min_points is None else min_points)
        self.smoothing_factor = float(
            pcfg.spline_smoothing_factor if smoothing_factor is None else smoothing_factor
        )

    def smooth_stroke(self, history_slice: List[dict]) -> np.ndarray:
        """Return RTS-smoothed XY for one closed stroke history slice.

        Expected ESKF history keys per entry:
            x_pred, P_pred  — predicted state/covariance for this IMU step
            x_post, P_post  — posterior after ZUPT / soft-ZUPT updates
            F, Q            — transition and process-noise matrices
            ts, stroke_id   — metadata used by callers
        """
        if not history_slice:
            return np.zeros((0, 2))

        xy = rts_smooth_history(history_slice)
        if self.spline and len(xy) >= self.min_points:
            resampled = spline_resample(
                xy,
                ds_m=self.ds_m,
                min_points=self.min_points,
                smoothing_factor=self.smoothing_factor,
            )
            if resampled is not None:
                xy = resampled
        return xy

    def smooth_strokes(self, strokes: Iterable[List[dict]]) -> list[np.ndarray]:
        return [self.smooth_stroke(s) for s in strokes]


def rts_smooth_history(history: List[dict]) -> np.ndarray:
    """RTS backward pass over one active-writing ESKF stroke.

    Prefer stored ``x_pred`` / ``P_pred`` from history[k+1].  This matters for
    the ESKF because nominal propagation includes the acceleration input term;
    simply using ``F @ x_post`` can be wrong.  For older logs that lack the
    predicted snapshots, this function falls back to the F/Q approximation.
    """
    n = len(history)
    if n == 0:
        return np.zeros((0, 2))
    if n == 1:
        x0 = np.asarray(history[0]['x_post'], dtype=float)
        return x0[0:2].reshape(1, 2)

    xs = [np.asarray(h['x_post'], dtype=float).copy() for h in history]
    Ps = [np.asarray(h['P_post'], dtype=float).copy() for h in history]

    for k in range(n - 2, -1, -1):
        h_next = history[k + 1]
        F = np.asarray(h_next['F'], dtype=float)
        Q = np.asarray(h_next['Q'], dtype=float)

        x_pred = h_next.get('x_pred')
        P_pred = h_next.get('P_pred')
        if x_pred is None:
            x_pred = F @ xs[k]
        else:
            x_pred = np.asarray(x_pred, dtype=float)
        if P_pred is None:
            P_pred = F @ Ps[k] @ F.T + Q
        else:
            P_pred = np.asarray(P_pred, dtype=float)

        try:
            G = Ps[k] @ F.T @ np.linalg.inv(P_pred)
        except np.linalg.LinAlgError:
            continue

        xs[k] = xs[k] + G @ (xs[k + 1] - x_pred)
        Ps[k] = Ps[k] + G @ (Ps[k + 1] - P_pred) @ G.T
        Ps[k] = 0.5 * (Ps[k] + Ps[k].T)

    return np.array([x[0:2] for x in xs])


def spline_resample(xy: np.ndarray,
                    ds_m: float,
                    min_points: int,
                    smoothing_factor: float) -> np.ndarray | None:
    """Resample XY at uniform arc-length spacing using scipy splines."""
    try:
        from scipy.interpolate import splprep, splev
    except ImportError:
        return None

    if len(xy) < min_points:
        return None

    # Remove exact duplicates to avoid degenerate knots.
    _, idx = np.unique(xy, axis=0, return_index=True)
    xy_u = xy[np.sort(idx)]
    if len(xy_u) < min_points:
        return None

    n_u = len(xy_u)
    s_factor = smoothing_factor * n_u

    try:
        tck, _ = splprep([xy_u[:, 0], xy_u[:, 1]], s=s_factor, k=min(3, n_u - 1))
    except Exception:
        return None

    u_dense = np.linspace(0, 1, max(500, n_u * 4))
    pts_dense = np.stack(splev(u_dense, tck), axis=1)
    total_len = float(np.linalg.norm(np.diff(pts_dense, axis=0), axis=1).sum())
    if total_len < ds_m:
        return None

    n_out = max(2, int(round(total_len / ds_m)))
    u_out = np.linspace(0, 1, n_out)
    xs_out, ys_out = splev(u_out, tck)
    return np.stack([xs_out, ys_out], axis=1)
