"""
postprocess/__init__.py — Post-stroke smoothing, triggered on every pen-up.

Two modes, selected via cfg.postprocess.smoothing_mode:

  'online'  — 3-point causal weighted moving average (OnlineTrailSmoother).
              Low-effort, minimal distortion.  Works with any fusion mode.

  'offline' — Rauch-Tung-Striebel backward pass over the ESKF history of
              the closed stroke, followed by optional arc-length spline
              resampling.  Strictly improves on the forward EKF posterior
              because future measurements inform past states.
              Requires fusion_mode='eskf' and cfg.postprocess.rts_enabled=True.

Both modes run once per closed stroke (pen-up event) and are invisible to
the live drawing path — latency is not a concern here.
"""

from __future__ import annotations

from typing import List

import numpy as np

from background.pipelines.postprocess.trail_smoother import OnlineTrailSmoother
from background.pipelines.config import cfg


# ── RTS backward pass ────────────────────────────────────────────────────────

def _rts_smooth_history(history: List[dict]) -> np.ndarray:
    """Co-dev-exact RTS backward pass over ESKF per-step history.

    Each entry in *history* must have the keys written by eskf.py:
        x_post (6,), P_post (6,6), F (6,6), Q (6,6)
    State layout: [p_x, p_y, v_x, v_y, b_ax, b_ay]

    Recomputes P_pred = F @ P_post[k] @ F.T + Q and uses F @ xs[k] as the
    predicted mean — matching the co-dev rts_smooth_history() formula exactly.

    Returns (N, 2) array of smoothed (x, y) positions — one row per step.
    """
    n = len(history)
    if n == 0:
        return np.zeros((0, 2))
    if n == 1:
        return history[0]['x_post'][0:2].reshape(1, 2)

    xs = [h['x_post'].copy() for h in history]   # list of (6,) arrays
    Ps = [h['P_post'].copy() for h in history]   # list of (6,6) arrays

    for k in range(n - 2, -1, -1):
        F = history[k + 1]['F']
        Q = history[k + 1]['Q']
        # Recompute predicted covariance from posterior (co-dev formula).
        P_pred = F @ Ps[k] @ F.T + Q
        try:
            G = Ps[k] @ F.T @ np.linalg.inv(P_pred)
        except np.linalg.LinAlgError:
            continue   # singular — skip step, keep forward estimate
        # Use F @ xs[k] as predicted mean (co-dev formula).
        xs[k] = xs[k] + G @ (xs[k + 1] - F @ xs[k])
        Ps[k] = Ps[k] + G @ (Ps[k + 1] - P_pred) @ G.T

    return np.array([x[0:2] for x in xs])   # (N, 2) — only positions needed


# ── Spline arc-length resample ───────────────────────────────────────────────

def _spline_resample(xy: np.ndarray, ds_m: float, min_points: int,
                     smoothing_factor: float) -> np.ndarray | None:
    """Resample *xy* at uniform arc-length spacing *ds_m* using splprep/splev.

    Returns a new (M, 2) array, or None if the stroke is too short / flat.
    """
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

    # Arc-length of the raw smoothed curve → number of resampling steps.
    u_dense = np.linspace(0, 1, max(500, n_u * 4))
    pts_dense = np.stack(splev(u_dense, tck), axis=1)
    seg_lens  = np.linalg.norm(np.diff(pts_dense, axis=0), axis=1)
    total_len = float(seg_lens.sum())
    if total_len < ds_m:
        return None

    n_out = max(2, int(round(total_len / ds_m)))
    u_out = np.linspace(0, 1, n_out)
    xs_out, ys_out = splev(u_out, tck)
    return np.stack([xs_out, ys_out], axis=1)


# ── Post-processor ───────────────────────────────────────────────────────────

class StrokePostProcessor:
    """Applies post-stroke smoothing to closed fused strokes.

    Mode is read from cfg.postprocess.smoothing_mode at construction time
    so it is fixed for the lifetime of the visualizer session.
    """

    def __init__(self, fusion_engine=None):
        self._fusion      = fusion_engine          # ESKF instance (needed for 'offline')
        self._mode        = cfg.postprocess.smoothing_mode
        self._pcfg        = cfg.postprocess
        self._online_smoother = OnlineTrailSmoother()

    # ── Public API ───────────────────────────────────────────────────────────

    def process_closed_stroke(self, stroke: dict) -> dict | None:
        """Smooth a closed stroke's fused (x, y) points and return the result.

        Args:
            stroke: dict with at minimum {'points': [(x, y, ts), ...], ...}

        Returns:
            Modified stroke dict with smoothed points and 'smoothed': True,
            or None if the stroke is too short or smoothing is disabled.
        """
        if not self._pcfg.trail_enabled:
            return None

        pts = stroke.get('points', [])
        if len(pts) < 2:
            return None

        if self._mode == 'offline':
            return self._process_offline(stroke, pts)
        return self._process_online(stroke, pts)

    # ── Online mode (causal WMA) ─────────────────────────────────────────────

    def _process_online(self, stroke: dict, pts: list) -> dict:
        self._online_smoother.reset()
        smoothed_pts = []
        for x, y, ts in pts:
            self._online_smoother.push(x, y)
            sx, sy = self._online_smoother.get()
            smoothed_pts.append((sx, sy, ts))

        return {**stroke, 'points': smoothed_pts, 'smoothed': True}

    # ── Offline mode (RTS + spline) ──────────────────────────────────────────

    def _process_offline(self, stroke: dict, pts: list) -> dict | None:
        if self._fusion is None:
            return None

        stroke_id = stroke.get('stroke_id', 0)
        history   = self._fusion.get_rts_history_slice(stroke_id)

        if not history:
            # No ESKF history (complementary fusion or RTS disabled) — fall back
            # to online smoother so pen-up always produces a green overlay.
            return self._process_online(stroke, pts)

        # RTS backward pass.
        smoothed_xy = _rts_smooth_history(history)   # (N, 2)

        if len(smoothed_xy) == 0:
            return self._process_online(stroke, pts)

        # Align timestamps: RTS output has one entry per IMU step in the history.
        # Stroke pts may be fewer (deduplication in StrokeReconstructor), so we
        # interpolate timestamps from the original stroke by nearest-neighbour.
        ts_arr = np.array([t for _, _, t in pts])
        rts_ts = np.array([h['ts'] for h in history])

        # Map each RTS step to its nearest original timestamp.
        rts_ts_mapped = np.interp(
            np.linspace(0, 1, len(smoothed_xy)),
            np.linspace(0, 1, len(ts_arr)),
            ts_arr,
        ).astype(np.int64)

        # Optional spline resample.
        pcfg = self._pcfg
        resampled = _spline_resample(
            smoothed_xy,
            ds_m=pcfg.spline_resample_ds_m,
            min_points=pcfg.spline_min_points,
            smoothing_factor=pcfg.spline_smoothing_factor,
        )

        if resampled is not None:
            # Resampled has different length — linearly interpolate timestamps.
            ts_resampled = np.interp(
                np.linspace(0, 1, len(resampled)),
                np.linspace(0, 1, len(rts_ts_mapped)),
                rts_ts_mapped,
            ).astype(np.int64)
            final_pts = [(float(x), float(y), int(t))
                         for (x, y), t in zip(resampled, ts_resampled)]
        else:
            final_pts = [(float(x), float(y), int(t))
                         for (x, y), t in zip(smoothed_xy, rts_ts_mapped)]

        return {
            **stroke,
            'points':   final_pts,
            'smoothed': True,
            'n_raw':    len(history),
        }
