"""
[postprocess/note_smoother] — Geometric refinement of RTS-smoothed strokes

After the RTS backward pass corrects coordinate drift, raw points are still
bound to the 200 Hz IMU sampling grid.  Slow hand motion → clustered points;
fast motion → sparse points.  CubicSpline arc-length reparameterization
produces a visually uniform curve regardless of drawing speed.

Public API:
    NoteSmoother.smooth_stroke(stroke, history_slice) -> dict

    stroke        — closed stroke dict from StrokeReconstructor
                    {'stroke_id', 'points': [(x, y, ts), ...], 'closed': True, ...}
                    Only metadata fields are used (stroke_id, start_ts, end_ts).
                    Position geometry comes entirely from history_slice.
    history_slice — list[dict] from ESKF._rts_buf for this stroke_id
                    each entry: {'ts', 'x_post', 'P_post', 'F', 'Q', 'stroke_id'}
                    Contains ALL Kalman samples with no deduplication.

Returns a copy of stroke with:
    'points'   replaced by the uniformly-spaced smoothed polyline
    'smoothed' = True   (or False when fallback was applied)
    'n_raw'    = Kalman history sample count (diagnostic)
"""

from __future__ import annotations

import warnings
import numpy as np
from scipy.interpolate import CubicSpline

from background.pipelines.config import cfg
from background.pipelines.postprocess.rts_smoother import rts_smooth_history


class NoteSmoother:
    """Applies RTS smoothing + arc-length spline resample to closed strokes."""

    def smooth_stroke(self, stroke: dict, history_slice: list[dict]) -> dict:
        """Return a smoothed copy of stroke.

        Positions come from history_slice (full Kalman history, no dedup).
        stroke is used only for metadata (stroke_id, start_ts, end_ts, closed).
        Falls back to smoothed=False on degenerate inputs.
        """
        result = dict(stroke)
        result['n_raw'] = len(history_slice)

        if not cfg.postprocess.rts_enabled or len(history_slice) < 2:
            result['smoothed'] = False
            return result

        # ── 1. RTS backward pass ─────────────────────────────────────────────
        smoothed_hist = rts_smooth_history(history_slice)

        # Extract 2D board-plane positions from the smoothed state vector.
        # State layout matches eskf.py: x = [p_x, p_y, v_x, v_y, b_ax, b_ay]
        pts_rts = np.array([s['x_smoothed'][0:2] for s in smoothed_hist])  # (N, 2)
        ts_rts  = np.array([s['ts']              for s in smoothed_hist])  # (N,) µs

        n = pts_rts.shape[0]

        # ── 2. Arc-length parameterization ───────────────────────────────────
        deltas    = np.linalg.norm(np.diff(pts_rts, axis=0), axis=1)  # (N-1,)
        arc       = np.concatenate([[0.0], np.cumsum(deltas)])         # (N,)
        total_arc = arc[-1]

        # Degenerate stroke: pen never moved.
        if total_arc < 1e-6:
            result['smoothed'] = False
            return result

        # ── 3. Spline construction ───────────────────────────────────────────
        # Collapse duplicate arc positions (zero-movement steps) so CubicSpline
        # doesn't receive repeated knots.
        unique_mask = np.concatenate([[True], np.diff(arc) > 1e-9])
        arc_u = arc[unique_mask]
        pts_u = pts_rts[unique_mask]

        if len(arc_u) < cfg.postprocess.spline_min_points:
            # Too few unique positions — return RTS-only output without spline.
            raw_pts = [(float(pts_rts[i, 0]), float(pts_rts[i, 1]), int(ts_rts[i]))
                       for i in range(n)]
            result['points']   = raw_pts
            result['smoothed'] = True
            return result

        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            cs_x = CubicSpline(arc_u, pts_u[:, 0])
            cs_y = CubicSpline(arc_u, pts_u[:, 1])

        # ── 4. Uniform resample at ds_m intervals ────────────────────────────
        ds  = cfg.postprocess.spline_resample_ds_m
        s_q = np.arange(0.0, total_arc, ds)
        # Always include the exact endpoint.
        if s_q[-1] < total_arc - 1e-9:
            s_q = np.append(s_q, total_arc)

        x_q = cs_x(s_q)
        y_q = cs_y(s_q)

        # Timestamps: linearly interpolate arc-length → µs timestamp.
        ts_q = np.interp(s_q, arc, ts_rts.astype(float)).astype(int)

        result['points']   = [(float(x_q[i]), float(y_q[i]), int(ts_q[i]))
                              for i in range(len(s_q))]
        result['smoothed'] = True
        return result


# ==============================================================================
# SELF-TEST  (synthetic clustered + sparse hand-drawn polyline)
# ==============================================================================
if __name__ == '__main__':
    import math

    np.random.seed(7)
    smoother = NoteSmoother()

    # Build a synthetic stroke: an arc swept at varying speed.
    # Slow at start/end (clustered), fast in the middle (sparse).
    n_pts   = 80
    theta   = np.linspace(0, math.pi, n_pts)
    # Non-uniform time — simulate slow-fast-slow drawing
    t_param = np.sin(np.linspace(0, math.pi, n_pts)) + 0.05  # speed profile
    t_param = np.cumsum(t_param)
    t_param = t_param / t_param[-1]   # normalise 0..1

    x_raw = np.cos(theta) * 0.30 + np.random.randn(n_pts) * 0.003
    y_raw = np.sin(theta) * 0.20 + np.random.randn(n_pts) * 0.003

    ts_hw = (t_param * 2_000_000).astype(int)   # 2-second stroke in µs

    # stroke dict — note: 'points' here is ignored by smooth_stroke for geometry;
    # only metadata fields (stroke_id, start_ts, end_ts) are carried forward.
    stroke = {
        'stroke_id': 42,
        'points':    [(float(x_raw[i]), float(y_raw[i]), int(ts_hw[i]))
                      for i in range(n_pts)],
        'start_ts':  int(ts_hw[0]),
        'end_ts':    int(ts_hw[-1]),
        'closed':    True,
    }

    # Build synthetic history_slice matching the _rts_buf format.
    # Use identity F/Q and raw x as x_post — geometry accuracy not the focus here.
    I6 = np.eye(6)
    Q6 = np.eye(6) * 1e-4
    history_slice = []
    for i in range(n_pts):
        x_st = np.array([x_raw[i], y_raw[i], 0.0, 0.0, 0.0, 0.0])
        history_slice.append({
            'ts':        int(ts_hw[i]),
            'x_post':    x_st,
            'P_post':    I6 * 0.01,
            'F':         I6.copy(),
            'Q':         Q6.copy(),
            'stroke_id': 42,
        })

    out = smoother.smooth_stroke(stroke, history_slice)

    pts_out = out['points']
    n_out   = len(pts_out)

    # Measure arc-length spacing of the resampled output.
    xs_o = np.array([p[0] for p in pts_out])
    ys_o = np.array([p[1] for p in pts_out])
    ds_actual = np.linalg.norm(
        np.diff(np.stack([xs_o, ys_o], axis=1), axis=0), axis=1
    )

    ds_target = cfg.postprocess.spline_resample_ds_m
    max_dev   = float(np.max(np.abs(ds_actual - ds_target) / ds_target))

    # Endpoint preservation: first and last smoothed points should be close
    # to the first/last raw (history_slice) points.
    p0_raw = np.array([x_raw[0], y_raw[0]])
    pN_raw = np.array([x_raw[-1], y_raw[-1]])
    p0_out = np.array([pts_out[0][0], pts_out[0][1]])
    pN_out = np.array([pts_out[-1][0], pts_out[-1][1]])
    ep_err = max(np.linalg.norm(p0_out - p0_raw), np.linalg.norm(pN_out - pN_raw))

    print("=" * 56)
    print("  NoteSmoother Self-Test (synthetic arc, 80 pts → uniform)")
    print("=" * 56)
    print(f"  Input Kalman samples : {n_pts}")
    print(f"  Output points        : {n_out}")
    print(f"  ds target            : {ds_target * 1000:.1f} mm")
    print(f"  Max spacing dev      : {max_dev * 100:.1f} %  (target < 10 %)")
    print(f"  Endpoint error       : {ep_err * 1000:.2f} mm")
    print(f"  smoothed flag        : {out['smoothed']}")

    ok = out['smoothed'] and max_dev < 0.10 and ep_err < 0.005
    print(f"  Result               : {'PASS' if ok else 'FAIL'}")
    print("=" * 56)
