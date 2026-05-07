"""Stroke-local relative IMU dead reckoner.

Runs alongside (not instead of) the ESKF global state.
Tracks relative displacement since pen-down using the same blended
acceleration the ESKF uses for its prediction step.

State:
    stroke_p_rel  (2D)  — relative displacement since stroke start
    stroke_v_rel  (2D)  — relative velocity

Lifecycle:
    pen-down  → reset(uwb_tip, current_p)
    each CONTACT_DRAWING frame  → update(acc_blend, dt_s, drag_inv_s)
    pen-up    → close_stroke()   (records final displacement)

Does NOT touch ESKF p / v / b_a.  Output is diagnostic + postprocess feed.
"""

import numpy as np
from background.pipelines.config import cfg


class StrokeIMUDeadReckoner:

    def __init__(self) -> None:
        self._dr_cfg = cfg.fusion_eskf.dead_reckoner
        self.stroke_p_rel  = np.zeros(2, dtype=float)
        self.stroke_v_rel  = np.zeros(2, dtype=float)
        self.stroke_origin: np.ndarray | None = None
        self.last_stroke_p_rel = np.zeros(2, dtype=float)
        self.last_stroke_v_rel = np.zeros(2, dtype=float)
        self._last_acc_blend_weight: float = 0.0

    def reset(
        self,
        uwb_tip: np.ndarray | None = None,
        current_p: np.ndarray | None = None,
    ) -> None:
        dr = self._dr_cfg
        self.stroke_p_rel[:] = 0.0
        self.stroke_v_rel[:] = 0.0
        self._last_acc_blend_weight = 0.0

        if uwb_tip is not None and dr.pen_down_snap_to_uwb:
            tip = np.asarray(uwb_tip, dtype=float)
            if current_p is not None:
                dist = float(np.linalg.norm(tip - np.asarray(current_p, dtype=float)))
                self.stroke_origin = tip.copy() if dist <= dr.pen_down_snap_max_dist_m \
                    else np.asarray(current_p, dtype=float).copy()
            else:
                self.stroke_origin = tip.copy()
        elif current_p is not None:
            self.stroke_origin = np.asarray(current_p, dtype=float).copy()
        else:
            self.stroke_origin = None

    def update(
        self,
        acc_blend: np.ndarray,
        dt_s: float,
        drag_inv_s: float = 0.0,
    ) -> None:
        if dt_s <= 0.0:
            return
        a = np.asarray(acc_blend, dtype=float)
        if not np.all(np.isfinite(a)):
            a = np.zeros(2, dtype=float)
        self.stroke_p_rel += self.stroke_v_rel * dt_s + 0.5 * a * dt_s * dt_s
        self.stroke_v_rel += a * dt_s
        self.stroke_v_rel *= max(0.0, 1.0 - drag_inv_s * dt_s)

    def close_stroke(self) -> None:
        self.last_stroke_p_rel = self.stroke_p_rel.copy()
        self.last_stroke_v_rel = self.stroke_v_rel.copy()

    def set_blend_weight(self, w: float) -> None:
        self._last_acc_blend_weight = float(w)

    @property
    def acc_blend_weight(self) -> float:
        return self._last_acc_blend_weight

    def diagnostics(self) -> dict:
        return {
            'stroke_p_rel':     (float(self.stroke_p_rel[0]),  float(self.stroke_p_rel[1])),
            'stroke_v_rel':     (float(self.stroke_v_rel[0]),  float(self.stroke_v_rel[1])),
            'stroke_origin':    (
                (float(self.stroke_origin[0]), float(self.stroke_origin[1]))
                if self.stroke_origin is not None else (0.0, 0.0)
            ),
            'acc_blend_weight': self._last_acc_blend_weight,
        }
