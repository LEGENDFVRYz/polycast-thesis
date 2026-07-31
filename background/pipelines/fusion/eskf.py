"""
Module 7b - Error-State Kalman Filter fusion.

Fuses high-rate IMU motion with low-rate UWB absolute position into a single
pen-tip trajectory on the board plane.

Division of labour between the sensors:
    IMU    short-term stroke shape and fast motion detail
    UWB    long-term absolute anchoring and drift correction
    Force  stroke state, supplied upstream by preprocess/contact.py

The two sensors enter through asymmetric paths. IMU events drive the clock:
they propagate the nominal state and the covariance. UWB events never advance
time; they only correct, and only after clearing the admission gates in
gates.py.

Supporting modules:
    state.py         nominal state, covariance, numerical hygiene
    modes.py         per-regime tuning and fast-mode arming
    gates.py         UWB admission gates and noise scaling
    updates.py       Kalman prediction and measurement-update primitives
    constraints.py   stationary-contact lock and stroke boundary guard
    diagnostics.py   emitted telemetry payload

Input events match preprocess/imu.py -> preprocess/contact.py for IMU, and
preprocess/uwb/position.py for UWB.

Run directly for a live hardware dashboard:
    python -m background.pipelines.fusion.eskf
"""

import math
from collections import deque

import numpy as np

from background.pipelines.config import cfg
from background.pipelines.fusion.constraints import (
    ActiveStrokeBoundaryGuard,
    StationaryContactLock,
)
from background.pipelines.fusion.diagnostics import FilterTelemetry, build_eskf_diagnostics
from background.pipelines.fusion.modes import FusionModeTracker
from background.pipelines.fusion.quaternion import (
    board_axis_indices,
    clip_vector_norm,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_matrix,
    slerp,
)
from background.pipelines.fusion.state import ESKFState
from background.pipelines.fusion.stroke_dead_reckoner import StrokeIMUDeadReckoner
from background.pipelines.fusion.updates import (
    ErrorStateClipLimits,
    apply_measurement_update,
    apply_zero_velocity_update,
    build_process_noise,
    build_transition_matrix,
    position_observation_matrix,
    velocity_observation_matrix,
)
from background.pipelines.fusion import gates

STATE_UWB_CORRECTION = 'UWB_CORRECTION'
STATE_UWB_BOOTSTRAP = 'UWB_BOOTSTRAP'
STATE_UWB_DROPPED = 'UWB_DROPPED'

# Velocity zeroing at pen-down uses a deliberately loose sigma: the intent is to
# bleed off air-move momentum, not to assert the pen is perfectly still.
_PEN_DOWN_ZUPT_SIGMA = 0.05

# A UWB velocity estimate needs three fixes spanning a sane interval before a
# central difference is meaningful; outside this band the samples are stale,
# duplicated, or too closely spaced to differentiate.
_UWB_VELOCITY_MIN_SPAN_S = 0.02
_UWB_VELOCITY_MAX_SPAN_S = 0.5
_UWB_VELOCITY_MAX_PLAUSIBLE_MS = 2.0

# During active ink a correction must never produce a visible fold, so position
# and velocity corrections are far tighter than in air.
_ACTIVE_POSITION_CLIP_M = 0.018
_ACTIVE_VELOCITY_CLIP_MS = 0.025
_AIR_POSITION_CLIP_M = 0.10
_AIR_VELOCITY_CLIP_MS = 0.50


class ESKF:
    """Error-state Kalman filter fusing IMU dead-reckoning with UWB position."""

    def __init__(self):
        board_width = cfg.anchors.board_size_x
        board_height = cfg.anchors.board_size_y

        self.modes = FusionModeTracker()
        self.state = ESKFState(
            board_width,
            board_height,
            position_floor_source=lambda: self.modes.current_parameters().pos_floor,
        )
        self.tip_lock = StationaryContactLock()
        self.boundary_guard = ActiveStrokeBoundaryGuard()
        self.dead_reckoner = StrokeIMUDeadReckoner()
        self.telemetry = FilterTelemetry()

        # Recent (ts, position, velocity, attitude) so a UWB fix can be compared
        # against the state as it was at the UWB timestamp, not the newest frame.
        self._state_history: deque[tuple[int, np.ndarray, np.ndarray, np.ndarray]] = deque(
            maxlen=cfg.fusion_eskf.state_buffer_size
        )

        self._previous_attitude: np.ndarray | None = None
        self._turn_cooldown = 0
        self._stroke_active_prev = False

        self._sustained_static_count = 0

        # Last few accepted UWB fixes, for the central-difference velocity estimate.
        self._uwb_velocity_history: deque[tuple[int, np.ndarray, float]] = deque(maxlen=5)

        self._last_imu_ts: int | None = None
        self._last_uwb_ts: int | None = None
        self.last_uwb_measurement = self.state.position.copy()

        self._last_accepted_tip: np.ndarray | None = None
        self._last_accepted_tip_ts: int | None = None
        self._last_uwb_anchor_ts: int | None = None

        self._have_imu_attitude = False

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def process_event(self, event: dict) -> dict | None:
        """
        Consume one preprocessed sensor event and return the fused result.

        Returns None for events without a timestamp or from an unknown sensor.
        """

        ts = event.get('ts_hw')
        if ts is None:
            return None

        sensor = event.get('sensor')
        if sensor == 'IMU':
            return self._on_imu(event, ts)
        if sensor == 'POSITION':
            return self._on_uwb(event, ts)
        return None

    def reset(self):
        """Return the filter to its initial state."""

        self.__init__()

    # -------------------------------------------------------------------------
    # IMU path - prediction, stillness, contact edges
    # -------------------------------------------------------------------------

    def _on_imu(self, event: dict, ts: int) -> dict:
        dt_s = self._advance_imu_clock(ts)

        self._update_attitude_and_turn(event, dt_s)

        stale_factor = self._uwb_staleness_factor(ts)

        stroke_state = event.get('stroke_state', 'UNKNOWN')
        stroke_active_now = bool(event.get('stroke_active', False))
        stroke_active_prev = self._stroke_active_prev
        self.modes.stroke_state = stroke_state
        self.modes.stroke_active = stroke_active_prev

        board_acceleration_hpf, board_acceleration_raw = self._read_board_acceleration(event)

        tip_is_planted = self.tip_lock.is_candidate(
            event,
            board_acceleration_hpf,
            stroke_state,
            self.state.speed,
            self.telemetry.omega_in_plane,
        )

        mode_parameters = self.modes.update_fast_mode(self.state.speed)
        if tip_is_planted:
            mode_parameters = cfg.fusion_eskf.modes.static
            self.modes.force_stationary()

        acceleration, detail_weight = self._blend_acceleration(
            stroke_state, tip_is_planted, board_acceleration_hpf, board_acceleration_raw
        )
        acceleration = acceleration * mode_parameters.acc_scale

        self._update_dead_reckoner(
            acceleration, dt_s, mode_parameters, stroke_state, tip_is_planted, detail_weight
        )

        self._propagate(acceleration, dt_s, mode_parameters, stale_factor)

        self._apply_stillness(event)

        self._handle_contact_edges(ts, stroke_active_now)
        self._stroke_active_prev = stroke_active_now
        self.modes.stroke_active = stroke_active_now

        self.tip_lock.apply(
            state=self.state,
            candidate=tip_is_planted,
            stroke_active_now=stroke_active_now,
            stroke_active_prev=stroke_active_prev,
            stroke_state=stroke_state,
            uwb_age_s=self._accepted_tip_age_s(ts),
        )

        self.telemetry.record_mode_frame(stroke_active_now, self.modes.in_fast_mode, stroke_state)

        self._state_history.append(
            (ts, self.state.position.copy(), self.state.velocity.copy(), self.state.attitude.copy())
        )

        self.state.clamp_to_board()

        fused = self._emit(
            ts=ts,
            source='IMU',
            state_label=stroke_state,
            stroke_id=event.get('stroke_id', 0),
            stroke_active=event.get('stroke_active', False),
        )

        # contact_raw lets reconstruct.py gate ink on physical contact rather
        # than on the debounced logical stroke. Defaults True for events that
        # predate the field.
        fused['contact_raw'] = bool(event.get('contact', True))

        # Attached only to IMU events: reconstruct.py ignores UWB events for ink
        # but needs per-point IMU samples to clean the stroke after pen-up.
        fused['imu_cleaner'] = {
            'acc_board_hp_tip': (
                float(board_acceleration_hpf[0]),
                float(board_acceleration_hpf[1]),
            ),
            'dt_s': float(dt_s),
            'vel': (float(self.state.velocity[0]), float(self.state.velocity[1])),
            'rel_pos': (float(self.state.position[0]), float(self.state.position[1])),
            'uwb': (float(self.state.last_uwb_tip[0]), float(self.state.last_uwb_tip[1])),
            'contact': bool(event.get('contact', True)),
            'is_static': bool(event.get('is_static', False)),
        }
        return fused

    @staticmethod
    def _read_board_acceleration(event: dict) -> tuple[np.ndarray, np.ndarray]:
        """
        Pull both tip-corrected acceleration paths produced by imu.py.

        The high-pass path has slow bias stripped but keeps fast stroke detail;
        the EMA path retains slow real motion along with the bias. Both already
        carry the rigid-body lever-arm correction.
        """

        high_pass = np.asarray(
            event.get('acc_board_hp_tip') or event.get('acc_board', (0.0, 0.0)), dtype=float
        )
        smoothed = np.asarray(
            event.get('acc_board_tip') or event.get('acc_board', (0.0, 0.0)), dtype=float
        )
        return high_pass, smoothed

    def _blend_acceleration(
        self,
        stroke_state: str,
        tip_is_planted: bool,
        high_pass: np.ndarray,
        smoothed: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        """
        Choose the acceleration driving prediction, based on what the pen is doing.

        Returns (acceleration, detail_weight) where detail_weight is the HPF
        share, reused by the dead reckoner so both integrators stay consistent.
        """

        eskf_cfg = cfg.fusion_eskf
        dead_reckoner_cfg = eskf_cfg.dead_reckoner

        if tip_is_planted:
            return np.zeros(2, dtype=float), 0.0

        if stroke_state == 'CONTACT_DRAWING':
            detail_weight = float(dead_reckoner_cfg.detail_weight)
            acceleration = (
                (smoothed - self.state.accel_bias) * (1.0 - detail_weight)
                + high_pass * detail_weight
            )
            # Clamped only while ink is already committed, so an impulse spike
            # cannot bloom into an oversized loop mid-stroke.
            if eskf_cfg.acc_spike_clamp_enabled and self._stroke_active_prev:
                acceleration = clip_vector_norm(acceleration, eskf_cfg.acc_spike_clamp_ms2)
            return acceleration, detail_weight

        if stroke_state == 'AIR_MOVE':
            detail_weight = float(dead_reckoner_cfg.air_scale)
            acceleration = high_pass * detail_weight
            if eskf_cfg.acc_spike_clamp_enabled:
                acceleration = clip_vector_norm(acceleration, eskf_cfg.acc_spike_clamp_ms2)
            return acceleration, detail_weight

        # CONTACT_STATIC / IDLE / UNKNOWN: integrating here would only add noise.
        return np.zeros(2, dtype=float), 0.0

    def _update_dead_reckoner(
        self,
        acceleration: np.ndarray,
        dt_s: float,
        mode_parameters,
        stroke_state: str,
        tip_is_planted: bool,
        detail_weight: float,
    ) -> None:
        self.dead_reckoner.set_blend_weight(detail_weight)
        if stroke_state == 'CONTACT_DRAWING' and not tip_is_planted:
            self.dead_reckoner.update(
                acc_blend=acceleration,
                dt_s=dt_s,
                drag_inv_s=mode_parameters.drag_inv_s,
            )

    def _propagate(
        self,
        acceleration: np.ndarray,
        dt_s: float,
        mode_parameters,
        stale_factor: float,
    ) -> None:
        """Advance the nominal state and inflate the error covariance."""

        self.state.position += self.state.velocity * dt_s + 0.5 * acceleration * dt_s * dt_s
        self.state.velocity += acceleration * dt_s

        # Drag tightens as UWB goes stale, bounding how far unaided IMU
        # dead-reckoning can wander before the next correction arrives.
        drag = mode_parameters.drag_inv_s * stale_factor
        self.state.velocity *= max(0.0, 1.0 - drag * dt_s)

        if self._stroke_active_prev:
            self.state.velocity = clip_vector_norm(
                self.state.velocity, cfg.fusion_eskf.active_vel_cap_ms
            )

        transition = build_transition_matrix(dt_s)
        process_noise = build_process_noise(dt_s, self.telemetry.turn_detected)

        # Squared because the stale factor inflates a standard deviation, while
        # Q is expressed in variance.
        process_noise = process_noise * (stale_factor ** 2)

        self.state.covariance = (
            transition @ self.state.covariance @ transition.T + process_noise
        )
        self.state.sanitize_covariance()
        self.state.apply_covariance_floor()
        self.state.sanitize_nominal_state()

    def _apply_stillness(self, event: dict) -> None:
        """Run ZUPT when the preprocessor reports the pen as still."""

        if not event.get('is_static', False):
            self._sustained_static_count = 0
            return

        self.telemetry.zupt_fires += 1
        apply_zero_velocity_update(self.state, cfg.fusion_eskf.sigma_zupt)

        # A long uninterrupted still period means residual velocity is drift,
        # not motion. Bias is deliberately kept so its convergence is preserved.
        self._sustained_static_count += 1
        if self._sustained_static_count >= cfg.fusion_eskf.zupt_hard_reset_n:
            self.state.velocity[:] = 0

    def _handle_contact_edges(self, ts: int, stroke_active_now: bool) -> None:
        """Apply the special handling that pen-down and pen-up edges require."""

        eskf_cfg = cfg.fusion_eskf

        if stroke_active_now and not self._stroke_active_prev:
            self.modes.stroke_start_ts = ts
            # Hard zero, not decay: leftover air-move momentum would otherwise
            # hook the first few samples of the stroke.
            self.state.velocity[:] = 0.0
            apply_zero_velocity_update(self.state, _PEN_DOWN_ZUPT_SIGMA)
            self._snap_stroke_start_to_uwb(ts)
            self.dead_reckoner.reset(
                uwb_tip=self.state.last_uwb_tip,
                current_p=self.state.position,
            )
            return

        if (not stroke_active_now) and self._stroke_active_prev:
            self.modes.stroke_start_ts = None

            # If the writer paused before lifting, the locked endpoint is the
            # true end of the ink; the final pen-up frame should not move it.
            locked_endpoint = (
                np.asarray(self.tip_lock.anchor_visible, dtype=float).copy()
                if self.tip_lock.active and self.tip_lock.anchor_visible is not None
                else None
            )

            self.state.velocity[:] = 0.0
            self.dead_reckoner.close_stroke()

            self.state.covariance[2, 2] *= eskf_cfg.stroke_end_p_vel_scale
            self.state.covariance[3, 3] *= eskf_cfg.stroke_end_p_vel_scale
            self.state.apply_covariance_floor()

            # Decaying the placement bias gives the next stroke's own snap and
            # EMA a clean slate rather than inheriting this stroke's offset.
            self.state.position_bias *= eskf_cfg.bias_decay

            if locked_endpoint is not None:
                self.state.position = locked_endpoint - self.state.position_bias


    def _snap_stroke_start_to_uwb(self, ts: int) -> None:
        """
        Soft position pull toward UWB at pen-down so each letter starts in place.

        Targets the last accepted lever-arm-corrected tip rather than the raw tag
        position: snapping to the tag would bake the tag-to-tip offset into the
        start of the stroke as a hidden placement error.
        """

        eskf_cfg = cfg.fusion_eskf
        if self._last_accepted_tip is None or self._last_accepted_tip_ts is None:
            return

        age_s = (ts - self._last_accepted_tip_ts) / 1_000_000.0
        if age_s > eskf_cfg.stroke_start_uwb_max_age_s:
            return

        sigma = eskf_cfg.sigma_uwb * eskf_cfg.stroke_start_sigma_scale
        innovation = self._last_accepted_tip.copy() - self.state.position

        applied, _, _ = apply_measurement_update(
            state=self.state,
            observation_matrix=position_observation_matrix(),
            innovation=innovation,
            measurement_noise=(sigma ** 2) * np.eye(2),
            limits=ErrorStateClipLimits(position_m=0.15, velocity_ms=0.50),
        )
        if applied:
            self.telemetry.stroke_start_snaps += 1

    # -------------------------------------------------------------------------
    # UWB path - gated Kalman correction
    # -------------------------------------------------------------------------

    def _on_uwb(self, event: dict, ts: int) -> dict:
        # UWB never advances the filter clock; it only corrects.
        self._last_uwb_ts = ts

        measurement = self._read_uwb_measurement(event)
        if measurement is None:
            return self._emit(ts, 'POSITION', STATE_UWB_DROPPED, 0, False)

        self.last_uwb_measurement = measurement.copy()

        ts_uwb = event.get('ts_hw', ts)
        interpolated = self._interpolate_state_at(ts_uwb)
        if interpolated is not None:
            reference_position, _, reference_attitude = interpolated
        else:
            reference_position = None
            # At ~200 Hz the newest attitude is under 5 ms stale, which is well
            # inside the accuracy this correction needs.
            reference_attitude = self.state.attitude

        tip_measurement, lever_offset, lever_world = self._correct_for_lever_arm(
            measurement, reference_attitude
        )

        self.telemetry.lever_arm_m = float(np.linalg.norm(lever_offset))
        self.telemetry.lever_arm_world = lever_world
        self.telemetry.uwb_measurement_raw = measurement.copy()
        self.telemetry.uwb_measurement_tip = tip_measurement.copy()
        self.telemetry.velocity_pseudo_applied = False
        self.telemetry.velocity_pseudo_position_delta[:] = 0.0
        self.telemetry.velocity_pseudo_velocity_delta[:] = 0.0

        solve_error = float(event.get('solve_error', 0.0))
        uwb_quality = event.get('uwb_quality', {}) or {}

        rejection = self._screen_measurement(tip_measurement, ts_uwb, solve_error, uwb_quality)
        if rejection is not None:
            return self._emit(ts, 'POSITION', rejection, 0, False)

        bootstrap = self._bootstrap_if_first_fix(tip_measurement, ts, ts_uwb, solve_error)
        if bootstrap is not None:
            return bootstrap

        accepted = self._correct_with_uwb(
            tip_measurement, reference_position, solve_error, uwb_quality
        )
        if not accepted:
            self.telemetry.uwb_rejected += 1
            return self._emit(ts, 'POSITION', gates.REJECT_NLOS, 0, False)

        self.telemetry.uwb_accepted += 1
        # Any accepted fix keeps the staleness clock alive, even when its
        # geometry was not clean enough to also drive a velocity pseudo-update.
        self._record_accepted_fix(tip_measurement, ts_uwb)
        self.telemetry.record_accepted_gain(self._stroke_active_prev, self.modes.in_fast_mode)

        self._track_position_bias(tip_measurement)
        self._maybe_anchor_velocity(tip_measurement, ts_uwb, solve_error)

        self.state.clamp_to_board()

        return self._emit(ts, 'POSITION', STATE_UWB_CORRECTION, 0, False)

    @staticmethod
    def _read_uwb_measurement(event: dict) -> np.ndarray | None:
        """
        Extract the board-plane UWB position from a position-filter event.

        mapped_position carries the clamped-only signal the filter wants, since
        the ESKF already models UWB noise and upstream smoothing would only add
        lag to the correction.
        """

        mapped = event.get('mapped_position')
        if mapped:
            return np.asarray(
                (mapped['board_width_x'], mapped['board_height_y']), dtype=float
            )

        fallback = event.get('pos_clean') or event.get('pos_raw')
        if fallback is None:
            return None
        return np.asarray(fallback, dtype=float)

    def _correct_for_lever_arm(
        self, measurement: np.ndarray, attitude: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Shift a UWB tag fix to the pen tip.

        The tag sits at the back of the marker, so with the pen held
        perpendicular the board-plane offset is near zero, but at a 30 degree
        tilt it reaches roughly 10 cm - far more than the accuracy budget.
        """

        tag_offset_body = np.asarray(cfg.marker.r_uwb_body_m, dtype=float)
        rotation = quaternion_to_rotation_matrix(attitude)
        offset_world = rotation @ tag_offset_body

        first_axis, second_axis = board_axis_indices()
        board_offset = np.array(
            [offset_world[first_axis], offset_world[second_axis]], dtype=float
        )
        return measurement - board_offset, board_offset, offset_world

    def _screen_measurement(
        self,
        tip_measurement: np.ndarray,
        ts_uwb: int,
        solve_error: float,
        uwb_quality: dict,
    ) -> str | None:
        """Run the admission gates, returning a rejection label or None to accept."""

        mode_parameters = self.modes.current_parameters()

        if gates.exceeds_jump_limit(
            tip_measurement,
            ts_uwb,
            self._last_accepted_tip,
            self._last_accepted_tip_ts,
            self.state.speed,
            mode_parameters.jump_speed_max,
        ):
            self.telemetry.uwb_jump_rejected += 1
            self.telemetry.uwb_rejected += 1
            return gates.REJECT_JUMP

        if gates.is_unusable_low_confidence(uwb_quality, self._stroke_active_prev):
            self.telemetry.uwb_rejected += 1
            return gates.REJECT_LOW_CONFIDENCE

        if gates.is_outside_board(
            tip_measurement, self.state.board_width, self.state.board_height
        ):
            self.telemetry.uwb_rejected += 1
            return gates.REJECT_BOARD_MARGIN

        # Lever-arm correction needs a real attitude; before the first IMU frame
        # the identity quaternion would place the tip at an arbitrary offset.
        if not self._have_imu_attitude:
            self.telemetry.uwb_rejected += 1
            return gates.REJECT_AWAITING_IMU

        return None

    def _bootstrap_if_first_fix(
        self, tip_measurement: np.ndarray, ts: int, ts_uwb: int, solve_error: float
    ) -> dict | None:
        """
        Hard-snap onto the first good fix instead of gating against a default state.

        The filter starts at board centre, which is almost certainly wrong, so
        the first clean measurement is treated as truth rather than an outlier.
        """

        if self.telemetry.uwb_accepted != 0:
            return None
        if solve_error > cfg.uwb.trilat_max_residual:
            return None

        self.state.position[:] = tip_measurement
        self.state.velocity[:] = 0.0
        self.telemetry.innovation_norm = 0.0
        self.telemetry.position_gain = 1.0
        self._last_uwb_anchor_ts = ts_uwb
        self._record_accepted_fix(tip_measurement, ts_uwb)
        self.telemetry.uwb_accepted += 1
        self.state.clamp_to_board()
        return self._emit(ts, 'POSITION', STATE_UWB_BOOTSTRAP, 0, False)

    def _correct_with_uwb(
        self,
        tip_measurement: np.ndarray,
        reference_position: np.ndarray | None,
        solve_error: float,
        uwb_quality: dict,
    ) -> bool:
        """
        Apply the Kalman position correction for an admitted UWB fix.

        Returns False when the fix was rejected inside the update, either by the
        NLOS residual gate or the innovation gate.
        """

        eskf_cfg = cfg.fusion_eskf

        if gates.exceeds_nlos_residual(solve_error):
            self.telemetry.innovation_norm = 0.0
            self.telemetry.nlos_scale = eskf_cfg.r_scale_max
            return False

        nlos_scale = gates.nlos_noise_scale(solve_error)
        self.telemetry.nlos_scale = nlos_scale

        mode_parameters = self.modes.current_parameters()
        base_sigma = (
            eskf_cfg.sigma_uwb
            * mode_parameters.sigma_scale
            * gates.quality_noise_multiplier(uwb_quality)
        )

        nominal_position = reference_position if reference_position is not None else self.state.position
        innovation = tip_measurement - nominal_position
        innovation_norm = float(np.linalg.norm(innovation))
        self.telemetry.innovation_norm = innovation_norm
        self.telemetry.uwb_residual_rms = solve_error

        innovation_verdict = self._screen_innovation(
            innovation_norm, tip_measurement, solve_error, uwb_quality
        )
        if innovation_verdict is not None:
            return innovation_verdict
        self.telemetry.innovation_reject_streak = 0

        trust_multiplier = gates.adaptive_trust_multiplier(nlos_scale, self.state.speed)

        direction_factor = gates.direction_disagreement_penalty(
            self.state.velocity,
            innovation,
            self.state.speed,
            innovation_norm,
            mode_parameters.dir_penalty,
        )
        trust_multiplier *= direction_factor
        self.telemetry.direction_factor = direction_factor
        if direction_factor > 1.0:
            self.telemetry.direction_penalty_count += 1

        measurement_noise = ((base_sigma * trust_multiplier) ** 2) * nlos_scale * np.eye(2)

        gain_cap = min(
            mode_parameters.pos_gain_cap
            * self.modes.stroke_age_gain_multiplier(self._last_uwb_ts or 0),
            1.0,
        )

        limits = (
            ErrorStateClipLimits(
                position_m=_ACTIVE_POSITION_CLIP_M,
                velocity_ms=_ACTIVE_VELOCITY_CLIP_MS,
                clip_position_by_norm=True,
                clip_velocity_by_norm=True,
            )
            if self._stroke_active_prev
            else ErrorStateClipLimits(
                position_m=_AIR_POSITION_CLIP_M,
                velocity_ms=_AIR_VELOCITY_CLIP_MS,
                clip_position_by_norm=True,
                clip_velocity_by_norm=True,
            )
        )

        applied, _, position_gain = apply_measurement_update(
            state=self.state,
            observation_matrix=position_observation_matrix(),
            innovation=innovation,
            measurement_noise=measurement_noise,
            limits=limits,
            gain_cap=gain_cap,
        )
        self.telemetry.position_gain = position_gain
        return applied

    def _screen_innovation(
        self,
        innovation_norm: float,
        tip_measurement: np.ndarray,
        solve_error: float,
        uwb_quality: dict,
    ) -> bool | None:
        """
        Decide the fate of a fix that disagrees wildly with the integrated state.

        A UWB-versus-IMU gap this large is non-physical for board writing, and
        letting it through would destabilise the gain and covariance. But a long
        rejection streak against consistently clean geometry means the filter is
        the one that is lost, so it re-localizes instead of rejecting forever.

        Returns None when the fix is within tolerance and the caller should
        continue the normal update, True when a re-localization already handled
        it, and False when the fix was rejected outright.
        """

        eskf_cfg = cfg.fusion_eskf

        if innovation_norm <= eskf_cfg.innov_hard_reject_m:
            return None

        self.telemetry.innovation_reject_streak += 1

        deadlocked = (
            self.telemetry.innovation_reject_streak >= eskf_cfg.innov_recovery_n
            and solve_error <= cfg.uwb.trilat_max_residual
            and not uwb_quality.get('low_confidence', False)
        )
        if deadlocked:
            self.state.snap_to_uwb(tip_measurement)
            self.telemetry.innovation_reject_streak = 0
            self.telemetry.uwb_snap_count += 1
            return True

        self.telemetry.uwb_rejected += 1
        return False

    def _record_accepted_fix(self, tip_measurement: np.ndarray, ts_uwb: int) -> None:
        """Remember an accepted fix as the reference for later gates and guards."""

        self._last_accepted_tip = tip_measurement.copy()
        self._last_accepted_tip_ts = ts_uwb
        self.state.last_uwb_tip = tip_measurement.copy()
        self._last_uwb_anchor_ts = ts_uwb

    def _track_position_bias(self, tip_measurement: np.ndarray) -> None:
        """
        Nudge the global placement bias toward UWB during active ink.

        Corrections land in position_bias rather than position, so the visible
        output drifts toward UWB while the relative IMU stroke shape held in
        position is left untouched. Frozen outside strokes, where the main
        filter corrects position directly.
        """

        if not self._stroke_active_prev:
            return

        eskf_cfg = cfg.fusion_eskf
        placement_error = tip_measurement - self.state.visible_position
        self.state.position_bias += eskf_cfg.bias_uwb_alpha * placement_error

        cap = eskf_cfg.bias_max_m
        self.state.position_bias[0] = float(np.clip(self.state.position_bias[0], -cap, cap))
        self.state.position_bias[1] = float(np.clip(self.state.position_bias[1], -cap, cap))

    def _maybe_anchor_velocity(
        self, tip_measurement: np.ndarray, ts_uwb: int, solve_error: float
    ) -> None:
        """
        Derive a velocity measurement from consecutive UWB fixes and apply it.

        Skipped during fast mode and, by default, during any active stroke:
        anchoring velocity to UWB geometry cancels exactly the IMU shape
        authority those modes exist to grant.
        """

        eskf_cfg = cfg.fusion_eskf
        self._uwb_velocity_history.append((ts_uwb, tip_measurement.copy(), solve_error))

        if self.modes.in_fast_mode:
            return
        if eskf_cfg.uwb_vel_stroke_gate and self._stroke_active_prev:
            return
        if len(self._uwb_velocity_history) < 3:
            return

        (oldest_ts, oldest_pos, oldest_err), (_, _, middle_err), (newest_ts, newest_pos, newest_err) = (
            self._uwb_velocity_history[-3],
            self._uwb_velocity_history[-2],
            self._uwb_velocity_history[-1],
        )

        # A central difference is only meaningful if every sample it spans has
        # clean geometry; one bad fix would dominate the derivative.
        if max(oldest_err, middle_err, newest_err) >= eskf_cfg.sigma_trilat:
            return

        span_s = (newest_ts - oldest_ts) / 1_000_000.0
        if not (_UWB_VELOCITY_MIN_SPAN_S < span_s < _UWB_VELOCITY_MAX_SPAN_S):
            return

        uwb_velocity = (newest_pos - oldest_pos) / span_s
        if float(np.linalg.norm(uwb_velocity)) >= _UWB_VELOCITY_MAX_PLAUSIBLE_MS:
            return

        # Roughly 5 cm of UWB noise over 50 ms reads as 1 m/s. Without this gate
        # that spurious velocity integrated into 40+ cm position excursions.
        deviation = float(np.linalg.norm(uwb_velocity - self.state.velocity))
        if deviation > eskf_cfg.uwb_vel_dev_max:
            return

        average_error = (oldest_err + middle_err + newest_err) / 3.0
        geometry_ratio = average_error / eskf_cfg.sigma_trilat
        sigma = eskf_cfg.sigma_uwb_vel * max(eskf_cfg.sigma_uwb_vel_min_scale, geometry_ratio)
        self.telemetry.velocity_sigma_effective = sigma

        self._apply_velocity_pseudo_measurement(uwb_velocity, sigma)
        self._last_uwb_anchor_ts = ts_uwb

    def _apply_velocity_pseudo_measurement(self, measured_velocity: np.ndarray, sigma: float) -> None:
        """Correct velocity from a UWB-derived estimate without moving active ink."""

        # Position is frozen during a stroke as a hard safety property: with the
        # stroke gate enabled this path should not run at all, but freezing it
        # here means a future config change cannot reintroduce the 8-11 cm
        # in-stroke fold this once produced.
        limits = (
            ErrorStateClipLimits(position_m=0.0, velocity_ms=0.15, freeze_position=True)
            if self._stroke_active_prev
            else ErrorStateClipLimits(position_m=0.08, velocity_ms=1.50)
        )

        applied, error_state, _ = apply_measurement_update(
            state=self.state,
            observation_matrix=velocity_observation_matrix(),
            innovation=measured_velocity - self.state.velocity,
            measurement_noise=(sigma ** 2) * np.eye(2),
            limits=limits,
        )
        if not applied:
            return

        self.telemetry.velocity_pseudo_applied = True
        self.telemetry.velocity_pseudo_position_delta = error_state[0:2].copy()
        self.telemetry.velocity_pseudo_velocity_delta = error_state[2:4].copy()

    # -------------------------------------------------------------------------
    # Timing and attitude
    # -------------------------------------------------------------------------

    def _advance_imu_clock(self, ts: int) -> float:
        """Return seconds since the previous IMU event, guarding against jitter."""

        nominal_dt = 1.0 / cfg.imu.sample_rate_hz

        if self._last_imu_ts is None:
            self._last_imu_ts = ts
            return nominal_dt

        dt_s = (ts - self._last_imu_ts) / 1_000_000.0
        self._last_imu_ts = ts

        if dt_s <= 0.0 or dt_s > 0.5:
            return nominal_dt

        # Hardware timestamps occasionally spike to 8-10x nominal; unclamped,
        # the 0.5 * a * dt^2 term on those frames blows up the position.
        max_dt = cfg.fusion_eskf.imu_dt_max_mult * nominal_dt
        if dt_s > max_dt:
            self.telemetry.dt_clamps += 1
            return max_dt

        return dt_s

    def _uwb_staleness_factor(self, ts: int) -> float:
        """
        Growth factor applied to process noise and drag when UWB has gone quiet.

        Without a recent absolute anchor, unaided IMU integration error grows
        unbounded, so uncertainty is inflated to reflect that.
        """

        eskf_cfg = cfg.fusion_eskf

        stale_s = 0.0
        if self._last_uwb_anchor_ts is not None:
            stale_s = max(0.0, (ts - self._last_uwb_anchor_ts) / 1_000_000.0)

        stale_factor = 1.0
        if stale_s > eskf_cfg.uwb_window_s:
            overrun = (stale_s - eskf_cfg.uwb_window_s) / eskf_cfg.uwb_window_s
            stale_factor = 1.0 + min(eskf_cfg.uwb_stale_max_k, overrun * eskf_cfg.uwb_stale_k)

        self.telemetry.uwb_stale_s = stale_s
        self.telemetry.stale_factor = stale_factor
        return stale_factor

    def _update_attitude_and_turn(self, event: dict, dt_s: float) -> None:
        """Adopt the new attitude and update in-plane angular velocity plus turn state."""

        raw_quaternion = event.get('quat')
        new_attitude = (
            np.asarray(raw_quaternion, dtype=float)
            if raw_quaternion is not None
            else self.state.attitude.copy()
        )

        self._update_turn_detection(
            new_attitude,
            dt_s,
            event.get('jerk', 0.0),
            event.get('omega_world'),
        )

        self.state.attitude = new_attitude
        self._have_imu_attitude = True

    def _update_turn_detection(
        self,
        new_attitude: np.ndarray,
        dt_s: float,
        jerk: float,
        omega_world_event,
    ) -> None:
        """
        Track in-plane angular velocity and flag sustained sharp corners.

        A corner requires both fast rotation and high jerk, sustained across
        several samples: normal handwriting vibration alone routinely trips
        either signal on its own, so a single-sample test fires constantly.
        """

        eskf_cfg = cfg.fusion_eskf

        if omega_world_event is not None:
            # Preferred: imu.py already derived this from the hardware gyro
            # through its filtering chain, so there is one source of truth.
            omega_world = np.asarray(omega_world_event, dtype=float)
            first_axis, second_axis = board_axis_indices()
            self.telemetry.omega_in_plane = math.sqrt(
                float(omega_world[first_axis]) ** 2 + float(omega_world[second_axis]) ** 2
            )
        elif self._previous_attitude is not None and dt_s > 1e-6:
            # Fallback for recordings that predate the omega_world field.
            attitude_delta = quaternion_multiply(
                new_attitude, quaternion_conjugate(self._previous_attitude)
            )
            if attitude_delta[3] < 0:
                attitude_delta = -attitude_delta

            omega_body = 2.0 * attitude_delta[0:3] / dt_s
            omega_world = quaternion_to_rotation_matrix(new_attitude) @ omega_body

            first_axis, second_axis = board_axis_indices()
            self.telemetry.omega_in_plane = math.sqrt(
                omega_world[first_axis] ** 2 + omega_world[second_axis] ** 2
            )
        else:
            self.telemetry.omega_in_plane = 0.0

        is_turning = self.telemetry.omega_in_plane > eskf_cfg.turn_omega_threshold
        is_jerky = jerk > eskf_cfg.turn_jerk_threshold

        if is_turning and is_jerky:
            self.telemetry.turn_arm_count += 1
            if self.telemetry.turn_arm_count >= eskf_cfg.turn_arm_n:
                self._turn_cooldown = eskf_cfg.turn_n_post
        else:
            self.telemetry.turn_arm_count = max(0, self.telemetry.turn_arm_count - 1)

        if self._turn_cooldown > 0:
            self.telemetry.turn_detected = True
            self._turn_cooldown -= 1
        else:
            self.telemetry.turn_detected = False

        self._previous_attitude = new_attitude.copy()

    def _interpolate_state_at(self, ts_uwb: int):
        """
        Look up the nominal state at a UWB timestamp from the IMU history buffer.

        Returns (position, velocity, attitude), or None when the buffer is too
        short or the timestamp is newer than every stored sample.
        """

        if len(self._state_history) < 2:
            return None

        history = list(self._state_history)

        if ts_uwb <= history[0][0]:
            _, position, velocity, attitude = history[0]
            return position.copy(), velocity.copy(), attitude.copy()

        for index in range(len(history) - 1):
            earlier_ts, earlier_p, earlier_v, earlier_q = history[index]
            later_ts, later_p, later_v, later_q = history[index + 1]
            if earlier_ts <= ts_uwb <= later_ts:
                fraction = (
                    (ts_uwb - earlier_ts) / (later_ts - earlier_ts)
                    if later_ts != earlier_ts
                    else 0.0
                )
                return (
                    (1.0 - fraction) * earlier_p + fraction * later_p,
                    (1.0 - fraction) * earlier_v + fraction * later_v,
                    slerp(earlier_q, later_q, fraction),
                )

        return None

    def _accepted_tip_age_s(self, ts: int) -> float | None:
        """Age of the last accepted tip-corrected UWB fix, or None if there is none."""

        if self._last_accepted_tip_ts is None:
            return None
        return max(0.0, (ts - self._last_accepted_tip_ts) / 1_000_000.0)

    # -------------------------------------------------------------------------
    # Output
    # -------------------------------------------------------------------------

    def _emit(
        self, ts: int, source: str, state_label: str, stroke_id: int, stroke_active: bool
    ) -> dict:
        """Build the fused output event for the current filter state."""

        self.boundary_guard.apply(
            state=self.state,
            ts=ts,
            stroke_active=stroke_active,
            tip_locked=self.tip_lock.active,
            last_tip_ts=self._last_accepted_tip_ts,
        )

        # After a covariance reset the position was just re-anchored, so the
        # stroke is broken for one frame rather than drawing a line from the
        # last good ink point to the snapped position.
        if self.state.suppress_next_emit:
            self.state.suppress_next_emit = False
            stroke_active = False

        visible = self.state.visible_position
        output_x = float(np.clip(visible[0], 0.0, self.state.board_width))
        output_y = float(np.clip(visible[1], 0.0, self.state.board_height))

        return {
            'ts_hw': ts,
            'source': source,
            'fused_x': output_x,
            'fused_y': output_y,
            'uwb_x': float(self.last_uwb_measurement[0]),
            'uwb_y': float(self.last_uwb_measurement[1]),
            'state': state_label,
            'fusion_mode': self.modes.current_name(self.tip_lock.active),
            'stroke_id': stroke_id,
            'stroke_active': stroke_active,
            'eskf': build_eskf_diagnostics(
                state=self.state,
                telemetry=self.telemetry,
                modes=self.modes,
                tip_lock=self.tip_lock,
                boundary_guard=self.boundary_guard,
                ts=ts,
            ),
            'dead_reckoning': self.dead_reckoner.diagnostics(),
        }


# -----------------------------------------------------------------------------
# Self-test: live hardware dashboard
# -----------------------------------------------------------------------------
#
# Reading the output:
#   - lever_arm_m is near 0 with the pen perpendicular, ~0.10 m at 30 degrees tilt
#   - acc_board_tip_mag below acc_board_mag during circular strokes (60-90% drop)
#   - omega_world_mag 0-20 rad/s is normal; above 100 indicates a timestamp glitch
#   - alpha_world_mag up to a few hundred rad/s^2 is normal after EMA
#   - jerk above 7000 m/s^3 is a wrist-whip event and should arm turn detection

if __name__ == '__main__':
    import csv
    import os
    import time

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput
    from background.pipelines.preprocess.contact import ContactStateDetector
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver

    DISPLAY_RATE_S = 0.1
    REPORT_NAME = 'eskf_session'

    # Ordered identity -> position -> lever-arm -> rigid-body -> angular ->
    # filter health -> contact. Legacy column names are preserved so old
    # exports diff cleanly.
    CSV_COLUMNS = [
        'ts_hw', 'source', 'state',
        'fused_x', 'fused_y',
        'uwb_x', 'uwb_y',
        'lever_arm_m',
        'acc_board_x', 'acc_board_z',
        'acc_board_tip_x', 'acc_board_tip_z',
        'acc_board_mag', 'acc_board_tip_mag',
        'acc_sensor_mag',
        'jerk',
        'omega_world_x', 'omega_world_y', 'omega_world_z', 'omega_world_mag',
        'omega_body_x', 'omega_body_y', 'omega_body_z',
        'alpha_world_x', 'alpha_world_y', 'alpha_world_z', 'alpha_world_mag',
        'P_pos_trace', 'innovation_norm', 'r_scale',
        'K_pos_diag', 'b_a_x', 'b_a_y', 'b_a_norm',
        'omega_in_plane', 'turn_flag',
        'uwb_residual_rms', 'uwb_accepted', 'uwb_rejected',
        'stroke_id', 'stroke_active', 'is_static', 'contact',
    ]

    def _magnitude(vector) -> float:
        return math.sqrt(sum(component * component for component in vector))

    def render_dashboard(fused: dict, imu_event: dict | None, imu_count: int, uwb_count: int) -> None:
        """Print one refreshed frame of the live console view."""

        diagnostics = fused['eskf']

        if imu_event is not None:
            acc_board = imu_event.get('acc_board', (0.0, 0.0))
            acc_board_tip = imu_event.get('acc_board_tip', (0.0, 0.0))
            omega_world = imu_event.get('omega_world', (0.0, 0.0, 0.0))
            alpha_world = imu_event.get('alpha_world', (0.0, 0.0, 0.0))
            jerk = imu_event.get('jerk', 0.0)
            acc_sensor_mag = _magnitude(imu_event.get('acc_sensor', (0.0, 0.0, 0.0)))
        else:
            acc_board = acc_board_tip = (0.0, 0.0)
            omega_world = alpha_world = (0.0, 0.0, 0.0)
            jerk = acc_sensor_mag = 0.0

        acc_board_mag = _magnitude(acc_board)
        acc_board_tip_mag = _magnitude(acc_board_tip)
        turn_label = 'TURN' if diagnostics['turn_flag'] else '----'

        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"============= LIVE ESKF  ({DISPLAY_RATE_S}s refresh) =============")
        print(f"  State      : {fused['state']:<20}  Source: {fused['source']}")
        print(f"  Stroke     : ID={fused['stroke_id']}  Active={fused['stroke_active']}")
        print("-" * 64)
        print("  [POSITION]")
        print(f"    Fused tip  : X={fused['fused_x']:7.4f} m   Y={fused['fused_y']:7.4f} m")
        print(f"    UWB tag    : X={fused['uwb_x']:7.4f} m   Y={fused['uwb_y']:7.4f} m")
        print(f"    Lever-arm  : {diagnostics.get('lever_arm_m', 0.0) * 100:5.1f} cm  (tag->tip, board plane)")
        print("-" * 64)
        print(f"  [RIGID-BODY TIP CORRECTION]  (rigid_body_enabled={cfg.imu.rigid_body_enabled})")
        print(f"    |acc_board|         : {acc_board_mag:7.4f} m/s^2  (sensor-point, legacy)")
        print(f"    |acc_board_tip|     : {acc_board_tip_mag:7.4f} m/s^2  (tip-corrected, ESKF input)")
        print(f"    reduction           : {max(0.0, acc_board_mag - acc_board_tip_mag):+6.4f} m/s^2")
        print(f"    |acc_sensor| 3D     : {acc_sensor_mag:7.4f} m/s^2")
        print(f"    jerk (body)         : {jerk:9.1f} m/s^3")
        print("-" * 64)
        print("  [ANGULAR KINEMATICS]")
        print(f"    |omega_world|       : {_magnitude(omega_world):7.3f} rad/s")
        print(f"    omega in-plane      : {diagnostics['omega_in_plane']:7.3f} rad/s  [{turn_label}]")
        print(f"    |alpha_world| (EMA) : {_magnitude(alpha_world):7.1f} rad/s^2")
        print("-" * 64)
        print("  [FILTER HEALTH]")
        print(f"    P_pos_trace : {diagnostics['P_pos_trace']:.4f} m     Innovation |y|: {diagnostics['innovation_norm']:.4f} m")
        print(f"    R scale     : {diagnostics['r_scale']:.2f}  (1.0=clean, >3=NLOS)")
        print(f"    K_pos_diag  : {diagnostics['K_pos_diag']:.4f}         UWB resid: {diagnostics['uwb_residual_rms']:.4f} m")
        print(f"    Bias b_a    : ({diagnostics['b_a'][0]:+.4f}, {diagnostics['b_a'][1]:+.4f}) m/s^2")
        print(f"    IMU / UWB   : {imu_count} / {uwb_count}  "
              f"(accepted={diagnostics['uwb_accepted']}  rejected={diagnostics['uwb_rejected']})")
        print("=" * 64)

    SERIAL_PORT = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE = getattr(cfg.serial, 'baud', 115200)

    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    normalizer = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=500)

    imu_prep = IMUPreprocessor()
    contact = ContactStateDetector()

    uwb_offsets = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offsets)
    trilateration = UWBSolver()
    position_filter = UWBPositionFilter()

    eskf = ESKF()

    print("=" * 64)
    print(f"  [TEST] ESKF LIVE - lever-arm correction active - {SERIAL_PORT}")
    print("  Draw strokes. Ctrl+C to stop and export CSV.")
    print("=" * 64)

    last_render = 0.0
    latest_fused = None
    latest_imu_event = None
    imu_count = 0
    uwb_count = 0
    event_log: list[dict] = []

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                aligner.add_events(normalizer.normalize(raw_packets))
                sorted_events = aligner.get_all_sorted()
                aligner.clear()

                for event in sorted_events:
                    if event['sensor'] == 'IMU':
                        preprocessed = imu_prep.process_one(event)
                        if not preprocessed:
                            continue
                        with_contact = contact.process_one(preprocessed)
                        fused = eskf.process_event(with_contact)
                        if not fused:
                            continue
                        imu_count += 1
                        latest_fused = fused
                        latest_imu_event = with_contact
                        # Carried alongside the fused event so the CSV export can
                        # join IMU-side diagnostics that the filter does not emit.
                        fused['_imu_ev'] = with_contact
                        event_log.append(fused)

                    elif event['sensor'] == 'UWB':
                        for ranged in range_prep.feed([event]):
                            solved = trilateration.process_one(ranged)
                            if not solved:
                                continue
                            positioned = position_filter.process_one(solved)
                            if not positioned:
                                continue
                            fused = eskf.process_event(positioned)
                            if not fused:
                                continue
                            uwb_count += 1
                            latest_fused = fused
                            fused['_imu_ev'] = None
                            event_log.append(fused)

            now = time.time()
            if latest_fused and (now - last_render) >= DISPLAY_RATE_S:
                render_dashboard(latest_fused, latest_imu_event, imu_count, uwb_count)
                last_render = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting ESKF.")
        streamer.close()

        print("-" * 64)
        print(f"  IMU events processed : {imu_count}")
        print(f"  UWB events processed : {uwb_count}")
        if latest_fused:
            diagnostics = latest_fused['eskf']
            print(f"  Final fused position : ({latest_fused['fused_x']:.3f}, {latest_fused['fused_y']:.3f}) m")
            print(f"  Final P_pos_trace    : {diagnostics['P_pos_trace']:.4f} m")
            print(f"  UWB accepted/rejected: {diagnostics['uwb_accepted']} / {diagnostics['uwb_rejected']}")
            print(f"  Last lever-arm offset: {diagnostics.get('lever_arm_m', 0.0) * 100:.1f} cm")
        print("=" * 64)

        if not event_log:
            print("No events logged. Exiting.")
            raise SystemExit(0)

        output = ModuleRunOutput('fusion/eskf')
        rows = []
        for fused in event_log:
            diagnostics = fused.get('eskf', {})
            imu_event = fused.get('_imu_ev')
            accel_bias = diagnostics.get('b_a', (0.0, 0.0))

            acc_board = imu_event.get('acc_board', (0.0, 0.0)) if imu_event else (0.0, 0.0)
            acc_board_tip = imu_event.get('acc_board_tip', (0.0, 0.0)) if imu_event else (0.0, 0.0)
            omega_world = imu_event.get('omega_world', (0.0, 0.0, 0.0)) if imu_event else (0.0, 0.0, 0.0)
            omega_body = imu_event.get('omega_body', (0.0, 0.0, 0.0)) if imu_event else (0.0, 0.0, 0.0)
            alpha_world = imu_event.get('alpha_world', (0.0, 0.0, 0.0)) if imu_event else (0.0, 0.0, 0.0)
            acc_sensor = imu_event.get('acc_sensor', (0.0, 0.0, 0.0)) if imu_event else (0.0, 0.0, 0.0)
            jerk = imu_event.get('jerk', 0.0) if imu_event else 0.0
            is_static = bool(imu_event.get('is_static', False)) if imu_event else False
            contact_flag = bool(imu_event.get('contact', False)) if imu_event else False

            rows.append([
                fused.get('ts_hw'),
                fused.get('source'),
                fused.get('state', ''),
                round(fused.get('fused_x', 0.0), 6),
                round(fused.get('fused_y', 0.0), 6),
                round(fused.get('uwb_x', 0.0), 6),
                round(fused.get('uwb_y', 0.0), 6),
                round(diagnostics.get('lever_arm_m', 0.0), 5),
                round(acc_board[0], 6), round(acc_board[1], 6),
                round(acc_board_tip[0], 6), round(acc_board_tip[1], 6),
                round(_magnitude(acc_board), 6), round(_magnitude(acc_board_tip), 6),
                round(_magnitude(acc_sensor), 6),
                round(jerk, 3),
                round(omega_world[0], 5), round(omega_world[1], 5),
                round(omega_world[2], 5), round(_magnitude(omega_world), 5),
                round(omega_body[0], 5), round(omega_body[1], 5), round(omega_body[2], 5),
                round(alpha_world[0], 3), round(alpha_world[1], 3),
                round(alpha_world[2], 3), round(_magnitude(alpha_world), 3),
                round(diagnostics.get('P_pos_trace', 0.0), 6),
                round(diagnostics.get('innovation_norm', 0.0), 6),
                round(diagnostics.get('r_scale', 1.0), 4),
                round(diagnostics.get('K_pos_diag', 0.0), 6),
                round(accel_bias[0], 6), round(accel_bias[1], 6),
                round(diagnostics.get('b_a_norm', 0.0), 6),
                round(diagnostics.get('omega_in_plane', 0.0), 4),
                int(diagnostics.get('turn_flag', False)),
                round(diagnostics.get('uwb_residual_rms', 0.0), 6),
                diagnostics.get('uwb_accepted', 0),
                diagnostics.get('uwb_rejected', 0),
                fused.get('stroke_id', 0),
                int(fused.get('stroke_active', False)),
                int(is_static),
                int(contact_flag),
            ])

        output.save_csv(f"{REPORT_NAME}.csv", rows, header=CSV_COLUMNS)
        output.finish()
