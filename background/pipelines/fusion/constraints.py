"""
Physical constraints applied on top of the Kalman estimate.

These are not part of the filter math. They encode facts about a pen on a
whiteboard that the linear filter cannot express, and they act directly on the
nominal state after the statistical update has run:

  StationaryContactLock
      While the tip is physically planted but not moving, hold the visible tip
      still instead of integrating IMU noise into a visible smear. Covers
      pen-down settling, mid-stroke micro-pauses, and the end-of-stroke hold.

  ActiveStrokeBoundaryGuard
      A safety net, not a normal correction path. If live IMU integration has
      expanded far outside the neighbourhood of the last trusted UWB fix, pull
      it back and remove the outward velocity driving the excursion.
"""

import numpy as np

from background.pipelines.config import cfg

LOCK_REASON_OFF = 'OFF'
LOCK_REASON_PEN_DOWN = 'PENDOWN'
LOCK_REASON_MICRO_PAUSE = 'MICROPAUSE'


class StationaryContactLock:
    """Pins the visible tip while the pen is planted and still."""

    def __init__(self):
        self.active = False
        self.candidate = False
        self.frame_count = 0
        self.anchor_visible: np.ndarray | None = None
        self.correction_magnitude = 0.0
        self.uwb_blend = 0.0
        self.reason = LOCK_REASON_OFF

    def release(self) -> None:
        self.candidate = False
        self.active = False
        self.frame_count = 0
        self.anchor_visible = None
        self.correction_magnitude = 0.0
        self.uwb_blend = 0.0
        self.reason = LOCK_REASON_OFF

    def is_candidate(
        self,
        event: dict,
        board_acceleration: np.ndarray,
        stroke_state: str,
        speed_ms: float,
        omega_in_plane: float,
    ) -> bool:
        """
        Decide whether the tip should be treated as planted this frame.

        CONTACT_STATIC from contact.py is authoritative. The threshold path below
        is a fallback for recordings where that substate is missing or where the
        debounced transition lags the physical reality by a frame or two.
        """

        eskf_cfg = cfg.fusion_eskf
        if not eskf_cfg.contact_static_lock_enabled:
            return False

        if stroke_state == 'CONTACT_STATIC':
            return True

        contact_like = (
            bool(event.get('contact', False))
            or stroke_state in ('CONTACT_DRAWING', 'CONTACT_STATIC')
        )
        if not contact_like or not bool(event.get('is_static', False)):
            return False

        # All three must agree, so a genuine sharp corner - still by one measure
        # but not the others - is not mistaken for a pause and clamped flat.
        return (
            speed_ms <= eskf_cfg.contact_static_lock_speed_thresh_ms
            and float(np.linalg.norm(board_acceleration)) <= eskf_cfg.contact_static_lock_acc_thresh_ms2
            and abs(omega_in_plane) <= eskf_cfg.contact_static_lock_omega_thresh_rads
        )

    def _build_anchor(
        self,
        visible_position: np.ndarray,
        last_uwb_tip: np.ndarray,
        uwb_age_s: float | None,
        pen_down_phase: bool,
    ) -> tuple[np.ndarray, float]:
        """
        Blend the current visible tip toward UWB to form the hold anchor.

        Before the stroke opens no ink is committed, so UWB is allowed to
        dominate and fix placement. Mid-stroke the blend is tiny, because
        snapping during a pause would visibly displace the letter being drawn.
        """

        eskf_cfg = cfg.fusion_eskf
        blend = 0.0

        if uwb_age_s is not None and uwb_age_s <= eskf_cfg.contact_static_lock_uwb_max_age_s:
            blend = (
                eskf_cfg.contact_static_lock_pen_down_uwb_blend
                if pen_down_phase
                else eskf_cfg.contact_static_lock_micro_pause_uwb_blend
            )
            blend = float(np.clip(blend, 0.0, 1.0))

        anchor = (1.0 - blend) * visible_position + blend * last_uwb_tip
        return anchor.astype(float), blend

    def apply(
        self,
        state,
        candidate: bool,
        stroke_active_now: bool,
        stroke_active_prev: bool,
        stroke_state: str,
        uwb_age_s: float | None,
    ) -> None:
        """Hold the visible tip at a stable anchor while the lock is engaged."""

        eskf_cfg = cfg.fusion_eskf
        if not eskf_cfg.contact_static_lock_enabled or not candidate:
            self.release()
            return

        self.candidate = True
        self.frame_count += 1

        pen_down_phase = (not stroke_active_now) or (stroke_active_now and not stroke_active_prev)

        # A pen-down hold is clamped from the very first frame because no ink is
        # at stake; a mid-stroke pause waits, so one borderline frame during a
        # sharp corner cannot freeze a stroke that is actually still moving.
        min_frames = 1 if pen_down_phase else max(1, int(eskf_cfg.contact_static_lock_min_frames))

        if self.anchor_visible is None or pen_down_phase:
            self.anchor_visible, self.uwb_blend = self._build_anchor(
                state.visible_position, state.last_uwb_tip, uwb_age_s, pen_down_phase
            )
        elif self.active:
            # Let a fresh UWB fix trim the held anchor very slowly, so a long
            # pause can drift toward truth without producing a visible snap.
            refreshed_anchor, blend = self._build_anchor(
                state.visible_position, state.last_uwb_tip, uwb_age_s, pen_down_phase=False
            )
            self.anchor_visible = (1.0 - blend) * self.anchor_visible + blend * refreshed_anchor
            self.uwb_blend = blend

        if pen_down_phase:
            self.reason = LOCK_REASON_PEN_DOWN
        elif stroke_active_now:
            self.reason = LOCK_REASON_MICRO_PAUSE
        else:
            self.reason = stroke_state

        # Velocity is killed as soon as the tip looks planted, even before the
        # position freeze engages, since residual velocity is what would smear
        # the next few frames.
        state.velocity *= float(np.clip(eskf_cfg.contact_static_lock_vel_decay, 0.0, 1.0))
        if state.speed < eskf_cfg.contact_static_lock_vel_zero_thresh_ms:
            state.velocity[:] = 0.0

        if self.frame_count < min_frames:
            self.active = False
            self.correction_magnitude = 0.0
            return

        self.active = True
        correction = (self.anchor_visible - state.visible_position) * float(
            np.clip(eskf_cfg.contact_static_lock_pos_alpha, 0.0, 1.0)
        )
        state.position += correction
        self.correction_magnitude = float(np.linalg.norm(correction))

        # The board contact just observed both "not moving" and "not wandering",
        # so shrink the covariance in exactly those dimensions.
        velocity_scale = float(np.clip(eskf_cfg.contact_static_lock_cov_vel_scale, 0.0, 1.0))
        position_scale = float(np.clip(eskf_cfg.contact_static_lock_cov_pos_scale, 0.0, 1.0))
        state.covariance[2, 2] *= velocity_scale
        state.covariance[3, 3] *= velocity_scale
        state.covariance[0, 0] *= position_scale
        state.covariance[1, 1] *= position_scale
        state.apply_covariance_floor()


class ActiveStrokeBoundaryGuard:
    """Bounds active ink to the neighbourhood of the last trusted UWB fix."""

    def __init__(self):
        self.fired = False
        self.correction_magnitude = 0.0

    def _stand_down(self) -> None:
        self.fired = False
        self.correction_magnitude = 0.0

    def apply(self, state, ts: int, stroke_active: bool, tip_locked: bool, last_tip_ts: int | None) -> None:
        """Damp outward drift and, past the hard radius, pull the tip back."""

        eskf_cfg = cfg.fusion_eskf

        # The stationary lock is a stronger physical claim than this guard; if
        # the tip is planted, the lock anchor wins.
        if tip_locked or not stroke_active or not eskf_cfg.active_uwb_guard_enabled:
            self._stand_down()
            return

        if last_tip_ts is None:
            self._stand_down()
            return

        uwb_age_s = max(0.0, (ts - last_tip_ts) / 1_000_000.0)
        if uwb_age_s > eskf_cfg.active_uwb_guard_max_age_s:
            self._stand_down()
            return

        offset_from_uwb = state.visible_position - state.last_uwb_tip
        distance = float(np.linalg.norm(offset_from_uwb))
        radius = float(eskf_cfg.active_uwb_guard_radius_m)

        if not np.isfinite(distance) or distance < 1e-9:
            self._stand_down()
            return

        outward_direction = offset_from_uwb / distance

        # Damping starts inside the hard radius so drift is bled off gradually
        # rather than snapped at the boundary. Only the outward component is
        # removed - tangential motion is what draws curves and corners.
        soft_radius = max(0.020, 0.60 * radius)
        if distance > soft_radius:
            outward_speed = float(np.dot(state.velocity, outward_direction))
            if outward_speed > 0.0:
                damping = float(np.clip(eskf_cfg.active_uwb_outward_velocity_damping, 0.0, 1.0))
                state.velocity -= outward_direction * outward_speed * damping

        if distance <= radius:
            self._stand_down()
            return

        target_visible = state.last_uwb_tip + offset_from_uwb * (radius / distance)
        correction = (target_visible - state.visible_position) * float(eskf_cfg.active_uwb_guard_alpha)
        state.position += correction

        # Whatever velocity produced the excursion is still present; leaving it
        # would just re-trigger the guard on the next frame.
        state.velocity *= 0.55

        self.fired = True
        self.correction_magnitude = float(np.linalg.norm(correction))
