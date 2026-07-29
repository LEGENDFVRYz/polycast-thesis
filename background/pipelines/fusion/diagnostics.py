"""
Fused-event payload assembly.

Builds the dict returned by ESKF.process_event(). The schema is consumed by
reconstruct.py, visualizer.py and test/tuning/scorer.py, so key names and
nesting are a stable contract - rename nothing here without updating them.

Counters are cumulative over the filter's lifetime; instantaneous values reflect
the most recent frame that produced them, and stay latched on frames where the
corresponding path did not run (for example lever-arm values on IMU frames).
"""

import numpy as np


class FilterTelemetry:
    """Mutable record of the most recent value of every diagnostic signal."""

    def __init__(self):
        self.innovation_norm = 0.0
        self.nlos_scale = 1.0
        self.position_gain = 0.0
        self.uwb_residual_rms = 0.0

        self.uwb_accepted = 0
        self.uwb_rejected = 0
        self.uwb_jump_rejected = 0
        self.uwb_snap_count = 0
        self.innovation_reject_streak = 0
        self.stroke_start_snaps = 0

        self.omega_in_plane = 0.0
        self.turn_detected = False
        self.turn_arm_count = 0

        self.lever_arm_m = 0.0
        self.lever_arm_world = np.zeros(3, dtype=float)
        self.uwb_measurement_raw = np.zeros(2, dtype=float)
        self.uwb_measurement_tip = np.zeros(2, dtype=float)

        self.uwb_stale_s = 0.0
        self.stale_factor = 1.0
        self.velocity_sigma_effective = 0.0

        self.direction_factor = 1.0
        self.direction_penalty_count = 0

        self.dt_clamps = 0
        self.zupt_fires = 0

        self.velocity_pseudo_applied = False
        self.velocity_pseudo_position_delta = np.zeros(2, dtype=float)
        self.velocity_pseudo_velocity_delta = np.zeros(2, dtype=float)

        self.frames_contact = 0
        self.frames_fast = 0
        self.frames_air = 0
        self.frames_static = 0

        self.gain_sum_contact = 0.0
        self.gain_count_contact = 0
        self.gain_sum_fast = 0.0
        self.gain_count_fast = 0
        self.gain_sum_air = 0.0
        self.gain_count_air = 0

    def record_mode_frame(self, stroke_active: bool, in_fast_mode: bool, stroke_state: str) -> None:
        """Count this IMU frame against the mode the contact detector reported."""

        if stroke_state == 'CONTACT_DRAWING':
            if in_fast_mode:
                self.frames_fast += 1
            else:
                self.frames_contact += 1
        elif stroke_state == 'AIR_MOVE':
            self.frames_air += 1
        else:
            self.frames_static += 1

    def record_accepted_gain(self, stroke_active: bool, in_fast_mode: bool) -> None:
        """
        Accumulate position gain at UWB-update time.

        Driven by accepted UWB events rather than IMU frames so the reported
        averages reflect the gain actually used, not a stale carry-over value.
        """

        if stroke_active:
            if in_fast_mode:
                self.gain_sum_fast += self.position_gain
                self.gain_count_fast += 1
            else:
                self.gain_sum_contact += self.position_gain
                self.gain_count_contact += 1
        else:
            self.gain_sum_air += self.position_gain
            self.gain_count_air += 1

    @staticmethod
    def _mean(total: float, count: int) -> float:
        return round(total / count if count > 0 else 0.0, 5)

    @property
    def average_gain_contact(self) -> float:
        return self._mean(self.gain_sum_contact, self.gain_count_contact)

    @property
    def average_gain_fast(self) -> float:
        return self._mean(self.gain_sum_fast, self.gain_count_fast)

    @property
    def average_gain_air(self) -> float:
        return self._mean(self.gain_sum_air, self.gain_count_air)


def build_eskf_diagnostics(
    state,
    telemetry: FilterTelemetry,
    modes,
    tip_lock,
    boundary_guard,
    ts: int,
) -> dict:
    """Assemble the nested 'eskf' sub-dict of a fused event."""

    tip_lock_anchor = (
        tip_lock.anchor_visible if tip_lock.anchor_visible is not None else state.visible_position
    )

    return {
        'P_pos_trace': float(np.sqrt(state.covariance[0, 0] + state.covariance[1, 1])),
        'innovation_norm': telemetry.innovation_norm,
        'r_scale': telemetry.nlos_scale,
        'K_pos_diag': telemetry.position_gain,
        'b_a_norm': float(np.linalg.norm(state.accel_bias)),
        'uwb_residual_rms': telemetry.uwb_residual_rms,
        'omega_in_plane': telemetry.omega_in_plane,
        'turn_flag': telemetry.turn_detected,
        'drawing_fast': modes.in_fast_mode,
        'fast_arm_count': modes.fast_arm_count,
        'fast_burst_count': modes.fast_burst_count,
        'b_a': (float(state.accel_bias[0]), float(state.accel_bias[1])),
        'uwb_accepted': telemetry.uwb_accepted,
        'uwb_rejected': telemetry.uwb_rejected,

        'lever_arm_m': telemetry.lever_arm_m,
        'lever_r_world': tuple(float(v) for v in telemetry.lever_arm_world),
        'z_uwb_raw': tuple(float(v) for v in telemetry.uwb_measurement_raw),
        'z_uwb_tip': tuple(float(v) for v in telemetry.uwb_measurement_tip),
        'last_uwb_tip': tuple(float(v) for v in state.last_uwb_tip),

        'uwb_stale_s': round(telemetry.uwb_stale_s, 4),
        'stale_factor': round(telemetry.stale_factor, 4),
        'sigma_v_eff': round(telemetry.velocity_sigma_effective, 5),

        'uwb_jump_rejected': telemetry.uwb_jump_rejected,
        'dir_factor': round(telemetry.direction_factor, 3),
        'dir_penalty_count': telemetry.direction_penalty_count,
        'pos_floor_used': round(state.last_position_floor, 6),

        'dt_clamps': telemetry.dt_clamps,
        'turn_arm_count': telemetry.turn_arm_count,
        'pos_cap_used': round(state.last_position_cap, 6),
        'zupt_fires': telemetry.zupt_fires,
        'cov_resets': state.covariance_resets,

        'innov_reject_streak': telemetry.innovation_reject_streak,
        'uwb_snap_count': telemetry.uwb_snap_count,
        'stroke_start_snaps': telemetry.stroke_start_snaps,

        'stroke_age_s': round(modes.stroke_age_s(ts), 3) if modes.stroke_start_ts else 0.0,
        'age_cap_mult': round(modes.stroke_age_gain_multiplier(ts), 3),

        'p_nominal': (float(state.position[0]), float(state.position[1])),
        'vel_pseudo_applied': bool(telemetry.velocity_pseudo_applied),
        'vel_pseudo_dx_pos': tuple(float(v) for v in telemetry.velocity_pseudo_position_delta),
        'vel_pseudo_dx_vel': tuple(float(v) for v in telemetry.velocity_pseudo_velocity_delta),
        'b_p': (float(state.position_bias[0]), float(state.position_bias[1])),
        'b_p_mag': float(np.linalg.norm(state.position_bias)),

        'active_uwb_guard_fired': bool(boundary_guard.fired),
        'active_uwb_guard_correction': float(boundary_guard.correction_magnitude),

        'tip_lock_active': bool(tip_lock.active),
        'tip_lock_candidate': bool(tip_lock.candidate),
        'tip_lock_count': int(tip_lock.frame_count),
        'tip_lock_correction': float(tip_lock.correction_magnitude),
        'tip_lock_uwb_blend': float(tip_lock.uwb_blend),
        'tip_lock_reason': tip_lock.reason,
        'tip_lock_anchor': tuple(float(v) for v in tip_lock_anchor),

        'frames_contact': telemetry.frames_contact,
        'frames_fast': telemetry.frames_fast,
        'frames_air': telemetry.frames_air,
        'frames_static': telemetry.frames_static,
        'avg_K_contact': telemetry.average_gain_contact,
        'avg_K_fast': telemetry.average_gain_fast,
        'avg_K_air': telemetry.average_gain_air,
    }
