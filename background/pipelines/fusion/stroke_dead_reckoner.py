"""
Stroke-local relative IMU dead reckoner.

Runs alongside the ESKF rather than inside it, integrating the same blended
acceleration to track displacement since pen-down. Because it restarts at every
stroke, its output shows the raw IMU shape of a single stroke without the
accumulated global drift present in the filter's absolute position.

Never touches ESKF position, velocity or bias. The result is used for
diagnostics and as a post-processing input.

Lifecycle:
    pen-down                    reset(uwb_tip, current_p)
    each CONTACT_DRAWING frame  update(acc_blend, dt_s, drag_inv_s)
    pen-up                      close_stroke()
"""

import numpy as np

from background.pipelines.config import cfg


class StrokeIMUDeadReckoner:
    """Integrates per-stroke relative displacement from blended IMU acceleration."""

    def __init__(self) -> None:
        self._dead_reckoner_cfg = cfg.fusion_eskf.dead_reckoner

        self.stroke_p_rel = np.zeros(2, dtype=float)
        self.stroke_v_rel = np.zeros(2, dtype=float)
        self.stroke_origin: np.ndarray | None = None

        self.last_stroke_p_rel = np.zeros(2, dtype=float)
        self.last_stroke_v_rel = np.zeros(2, dtype=float)

        self._last_acc_blend_weight: float = 0.0

    def reset(
        self,
        uwb_tip: np.ndarray | None = None,
        current_p: np.ndarray | None = None,
    ) -> None:
        """
        Start a new stroke, choosing where its relative displacement originates.

        Anchoring to the UWB tip gives the stroke correct absolute placement, but
        only when UWB is close enough to be believable - beyond the configured
        distance the current filter position is the safer origin.
        """

        dead_reckoner_cfg = self._dead_reckoner_cfg

        self.stroke_p_rel[:] = 0.0
        self.stroke_v_rel[:] = 0.0
        self._last_acc_blend_weight = 0.0

        if uwb_tip is not None and dead_reckoner_cfg.pen_down_snap_to_uwb:
            tip = np.asarray(uwb_tip, dtype=float)
            if current_p is not None:
                current = np.asarray(current_p, dtype=float)
                distance = float(np.linalg.norm(tip - current))
                snap_to_uwb = distance <= dead_reckoner_cfg.pen_down_snap_max_dist_m
                self.stroke_origin = tip.copy() if snap_to_uwb else current.copy()
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
        """Integrate one frame of acceleration into relative position and velocity."""

        if dt_s <= 0.0:
            return

        acceleration = np.asarray(acc_blend, dtype=float)
        if not np.all(np.isfinite(acceleration)):
            acceleration = np.zeros(2, dtype=float)

        self.stroke_p_rel += self.stroke_v_rel * dt_s + 0.5 * acceleration * dt_s * dt_s
        self.stroke_v_rel += acceleration * dt_s
        self.stroke_v_rel *= max(0.0, 1.0 - drag_inv_s * dt_s)

    def close_stroke(self) -> None:
        """Latch the final displacement so it survives the next reset()."""

        self.last_stroke_p_rel = self.stroke_p_rel.copy()
        self.last_stroke_v_rel = self.stroke_v_rel.copy()

    def set_blend_weight(self, weight: float) -> None:
        self._last_acc_blend_weight = float(weight)

    @property
    def acc_blend_weight(self) -> float:
        return self._last_acc_blend_weight

    def diagnostics(self) -> dict:
        """Return the current stroke-local state for the fused event payload."""

        origin = (
            (float(self.stroke_origin[0]), float(self.stroke_origin[1]))
            if self.stroke_origin is not None
            else (0.0, 0.0)
        )
        return {
            'stroke_p_rel': (float(self.stroke_p_rel[0]), float(self.stroke_p_rel[1])),
            'stroke_v_rel': (float(self.stroke_v_rel[0]), float(self.stroke_v_rel[1])),
            'stroke_origin': origin,
            'acc_blend_weight': self._last_acc_blend_weight,
        }
