"""
Module 7b — Error-State Kalman Filter (ESKF) Fusion

Upgrade path from baseline.py's complementary filter. Differences:
  - Error-state formulation (linearizes about a nominal state), not full-state.
  - UWB is a weighted correction via Kalman gain, not a fixed-alpha mix.
  - NLOS-adaptive R (consumes trilateration solve_error).
  - UWB↔IMU timestamp interpolation via a short ring buffer.
  - Lever-arm compensation (tip ≠ UWB tag ≠ IMU).
  - Sharp-stroke / turn-aware Q inflation from quaternion-derived ω.

Build is staged — this file grows Step-by-Step per the plan. The current
implementation is Step 1 (skeleton): IMU dead-reckoning + UWB passthrough
with the final output schema wired. Filter math is added in Steps 2–7.

Input events (identical to baseline.py):
    IMU:      from preprocess/imu.py → preprocess/contact.py
    POSITION: from preprocess/uwb/{range, trilateration, position}.py

Output event:
    {
      'ts_hw': int, 'source': 'IMU' | 'POSITION',
      'fused_x': float, 'fused_y': float,
      'uwb_x': float, 'uwb_y': float,
      'state': str, 'stroke_id': int, 'stroke_active': bool,
      'eskf': {
        'P_pos_trace': float,        # √(P[0,0]+P[1,1]), 2D position std proxy
        'innovation_norm': float,    # |y| on last UWB update
        'r_scale': float,            # NLOS gain scaler on last UWB update
        'omega_in_plane': float,     # rad/s  (Step 7)
        'turn_flag': bool,           # (Step 7)
        'b_a': (float, float),       # current accel-bias estimate
      }
    }
"""

import math
from collections import deque

import numpy as np

from background.pipelines.config import cfg


def _q_to_rotation(q: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix (body → world)."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)    ],
        [    2*(x*y + w*z),   1 - 2*(x*x + z*z),    2*(y*z - w*x)    ],
        [    2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """q1 ⊗ q2, both stored as [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate == inverse for unit quaternion: negate xyz, keep w."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions."""
    dot = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)
    theta0 = math.acos(dot)
    theta   = theta0 * t
    q_perp  = q2 - dot * q1
    n = float(np.linalg.norm(q_perp))
    if n < 1e-9:
        return q1.copy()
    q_perp /= n
    return math.cos(theta) * q1 + math.sin(theta) * q_perp


class ESKF:
    def __init__(self):
        ecfg = cfg.fusion_eskf

        # ── Nominal state (propagated directly, not in the filter) ──────────
        bx = cfg.anchors.board_size_x
        by = cfg.anchors.board_size_y
        self.p   = np.array([bx * 0.5, by * 0.5], dtype=float)   # position
        self.v   = np.zeros(2, dtype=float)                      # velocity
        self.b_a = np.zeros(2, dtype=float)                      # accel bias
        self.q   = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)   # identity

        # ── Error-state covariance (6×6 block-diag init) ────────────────────
        # Blocks: δp (2), δv (2), δb_a (2)
        diag = np.array([
            ecfg.p0_pos,  ecfg.p0_pos,
            ecfg.p0_vel,  ecfg.p0_vel,
            ecfg.p0_bias, ecfg.p0_bias,
        ]) ** 2
        self.P = np.diag(diag)

        # ── Ring buffer of (ts, p, v, q) for UWB↔IMU time interpolation ─────
        self._state_buf: deque[tuple[int, np.ndarray, np.ndarray, np.ndarray]] = (
            deque(maxlen=ecfg.state_buffer_size)
        )

        # ── Step-7 turn tracking ────────────────────────────────────────────
        self._prev_quat: np.ndarray | None = None
        self._turn_cooldown = 0
        self._turn_arm_count = 0       # consecutive samples meeting turn+jerk gate
        self._omega_in_plane_last = 0.0
        self._turn_flag_last = False

        # ── Contact-edge tracking (for stroke-start soft ZUPT + 3c sigma) ────
        self._prev_stroke_active = False

        # ── Phase 3a — sustained-static hard reset ───────────────────────────
        self._zupt_hard_count = 0

        # ── Phase 3b — UWB-velocity pseudo-measurement buffer ────────────────
        # Stores (ts_hw, pos, solve_error) of the last N accepted UWB corrections.
        self._uwb_vel_buf: deque[tuple[int, np.ndarray, float]] = deque(maxlen=5)

        # ── Sliding-window safeguard (Rule 3) ────────────────────────────────
        # Tracks when the last UWB velocity anchor happened.
        self._last_uwb_reset_ts: int | None = None
        self._last_stale_s: float = 0.0
        self._last_stale_factor: float = 1.0
        self._last_sigma_v_eff: float = 0.0

        # ── Book-keeping ────────────────────────────────────────────────────
        self.last_ts: int | None = None
        self.last_uwb = self.p.copy()
        self._last_innovation_norm = 0.0
        self._last_r_scale = 1.0
        self._last_K_pos = 0.0
        self._last_uwb_residual_rms = 0.0
        self._uwb_accepted = 0
        self._uwb_rejected = 0
        self._last_lever_arm_m = 0.0
        self._last_lever_r_world = np.zeros(3, dtype=float)
        self._last_z_uwb_raw     = np.zeros(2, dtype=float)

        # ── Adaptive trust tracking ─────────────────────────────────────────
        self._last_z_tip: np.ndarray | None = None   # last accepted tip position (jump gate)
        self._last_z_tip_ts: int | None = None        # timestamp of _last_z_tip
        self._uwb_jump_rejected = 0                   # cumulative jump-gate rejects
        self._last_dir_factor: float = 1.0            # last direction-disagreement penalty
        self._last_pos_floor_used: float = 0.025 ** 2 # last position floor value (m²)
        self._last_pos_cap_used: float = (0.025 * 6.0) ** 2  # last position cap value (m²)
        self._dir_penalty_count = 0                   # frames where dir_factor > 1
        self._zupt_fires = 0                           # cumulative _zupt_update invocations

        # ── dt jitter tracking ──────────────────────────────────────────────
        self._dt_clamps = 0          # cumulative count of dt-jitter clamps
        self._cov_resets = 0         # cumulative covariance reset events (NaN recovery)

        # ── DRAWING_FAST mode tracking ───────────────────────────────────────
        self._fast_arm_count: int   = 0    # consecutive frames at/above speed threshold
        self._fast_burst_count: int = 0    # hold-down counter after trigger fires
        self._in_fast_mode: bool    = False

        # ── Mode statistics (accelerate per-regime tuning) ───────────────────
        self._frames_contact = 0     # CONTACT_DRAWING IMU frames
        self._frames_air = 0         # AIR_MOVE IMU frames
        self._frames_static = 0      # IDLE / CONTACT_STATIC IMU frames
        self._k_contact_sum = 0.0    # sum of K_pos_diag during contact frames
        self._k_air_sum = 0.0        # sum of K_pos_diag during air frames

        self._board_w = bx
        self._board_h = by

    # ────────────────────────────────────────────────────────────────────────
    # Public API
    # ────────────────────────────────────────────────────────────────────────
    def process_event(self, ev: dict) -> dict | None:
        ts = ev.get('ts_hw')
        if ts is None:
            return None

        sensor = ev.get('sensor')
        if sensor == 'IMU':
            return self._on_imu(ev, ts)
        if sensor == 'POSITION':
            return self._on_uwb(ev, ts)
        return None

    def reset(self):
        self.__init__()

    def _resolve_mode(self):
        """Return the FusionModeParams for the current filter state.

        DRAWING_FAST arms after drawing_fast_min_frames consecutive frames where
        tip speed >= drawing_fast_speed_thresh, then holds for drawing_fast_burst_frames
        additional IMU frames before returning to DRAWING.  This gives IMU a bounded
        authority window on short fast strokes without letting it drift indefinitely.
        Resets immediately on pen-up.
        """
        ecfg = cfg.fusion_eskf
        if self._prev_stroke_active:
            speed = float(np.linalg.norm(self.v))
            if speed >= ecfg.drawing_fast_speed_thresh:
                self._fast_arm_count += 1
            else:
                self._fast_arm_count = max(0, self._fast_arm_count - 1)

            triggered = self._fast_arm_count >= ecfg.drawing_fast_min_frames
            if triggered:
                # Re-arm the burst hold-down every time the trigger condition holds.
                self._fast_burst_count = ecfg.drawing_fast_burst_frames
            elif self._fast_burst_count > 0:
                self._fast_burst_count -= 1

            self._in_fast_mode = self._fast_burst_count > 0
            if self._in_fast_mode:
                return ecfg.modes.drawing_fast
            return ecfg.modes.drawing
        self._fast_arm_count = 0
        self._fast_burst_count = 0
        self._in_fast_mode = False
        return ecfg.modes.air

    # ────────────────────────────────────────────────────────────────────────
    # IMU path — prediction + ZUPT update + turn-aware Q (Steps 2 + 7)
    # ────────────────────────────────────────────────────────────────────────
    def _on_imu(self, ev: dict, ts: int) -> dict:
        dt_s = self._advance_clock(ts)

        # Step 7: Update quaternion first so omega and Q use the current sample.
        q_raw = ev.get('quat')
        q_new = np.asarray(q_raw, dtype=float) if q_raw is not None else self.q.copy()
        
        # Use precomputed ω from imu.py when available; fall back to quat-diff.
        current_jerk    = ev.get('jerk', 0.0)
        omega_world_ev  = ev.get('omega_world')   # None on older recorded events
        self._update_omega_and_turn(q_new, dt_s, current_jerk, omega_world_ev)
        
        self.q = q_new

        # Sliding-window safeguard (Rule 3): track how long since the last UWB
        # velocity anchor.  When stale, inflate Q and tighten velocity drag so
        # IMU dead-reckoning can't accumulate unbounded error.
        ecfg = cfg.fusion_eskf
        stale_s = 0.0
        if self._last_uwb_reset_ts is not None:
            stale_s = max(0.0, (ts - self._last_uwb_reset_ts) / 1_000_000.0)
        stale_factor = 1.0
        if stale_s > ecfg.uwb_window_s:
            over = (stale_s - ecfg.uwb_window_s) / ecfg.uwb_window_s
            stale_factor = 1.0 + min(ecfg.uwb_stale_max_k, over * ecfg.uwb_stale_k)
        self._last_stale_s = stale_s
        self._last_stale_factor = stale_factor

        # 1. Nominal state propagation (mid-point integration).
        # Prefer tip-corrected acc (lever-arm kinematics applied in imu.py);
        # fall back to raw sensor-point acc for compatibility with older recorded events.
        acc_src = (
            ev.get('acc_board_hp_tip')
            or ev.get('acc_board_tip')
            or ev.get('acc_board', (0.0, 0.0))
        )
        acc = np.asarray(acc_src, dtype=float)
        mode_p = self._resolve_mode()
        a   = (acc - self.b_a) * mode_p.acc_scale
        self.p += self.v * dt_s + 0.5 * a * dt_s * dt_s
        self.v += a * dt_s
        # Velocity drag — further scaled by stale_factor when UWB is silent.
        drag_inv_s = mode_p.drag_inv_s * stale_factor
        self.v *= max(0.0, 1.0 - drag_inv_s * dt_s)

        # 2. Error-state covariance propagation:   P ← F·P·Fᵀ + Q
        #    Q is turn-aware; also inflated by stale_factor² when UWB is silent.
        F = self._build_F(dt_s)
        Q = self._build_Q(dt_s) * (stale_factor ** 2)
        self.P = F @ self.P @ F.T + Q
        self._sanitize_covariance()
        self._apply_covariance_floor()
        self._sanitize_state()

        # 3. ZUPT pseudo-measurement (v = 0) when the IMU preprocessor
        #    flags the pen as still.
        if ev.get('is_static', False):
            self._zupt_update()
            # Phase 3a: sustained static → hard-zero velocity after N samples.
            # Prevents drift from compounding when the pen sits still between strokes.
            # b_a is deliberately kept so bias convergence is not disrupted.
            self._zupt_hard_count += 1
            if self._zupt_hard_count >= cfg.fusion_eskf.zupt_hard_reset_n:
                self.v[:] = 0
        else:
            self._zupt_hard_count = 0

        # 4. Contact edge handling.
        #    Rising edge (inactive → active): soft ZUPT — tip velocity near zero.
        #    Falling edge (active → inactive): damp lingering momentum so it
        #    doesn't leak into the next stroke.
        stroke_active_now = bool(ev.get('stroke_active', False))
        if stroke_active_now and not self._prev_stroke_active:
            self._zupt_soft_update(sigma=0.05)
        elif (not stroke_active_now) and self._prev_stroke_active:
            ecfg_se = cfg.fusion_eskf
            self.v *= ecfg_se.stroke_end_v_decay
            self.P[2, 2] *= ecfg_se.stroke_end_p_vel_scale
            self.P[3, 3] *= ecfg_se.stroke_end_p_vel_scale
            self._apply_covariance_floor()
        self._prev_stroke_active = stroke_active_now

        # Mode statistics — accumulate per-frame counters for tuning diagnostics.
        stroke_state = ev.get('stroke_state', 'UNKNOWN')
        if stroke_state == 'CONTACT_DRAWING':
            self._frames_contact += 1
            self._k_contact_sum  += self._last_K_pos
        elif stroke_state == 'AIR_MOVE':
            self._frames_air += 1
            self._k_air_sum  += self._last_K_pos
        else:
            self._frames_static += 1

        # Snapshot for UWB time interpolation (Step 4 consumes this).
        self._state_buf.append((ts, self.p.copy(), self.v.copy(), self.q.copy()))

        self._clamp_to_board()

        return self._emit(
            ts       = ts,
            source   = 'IMU',
            state    = stroke_state,
            sid      = ev.get('stroke_id', 0),
            active   = ev.get('stroke_active', False),
        )

    # ────────────────────────────────────────────────────────────────────────
    # UWB path — Kalman correction (Steps 3+4)
    #   Step 5 adds NLOS-adaptive R.
    #   Step 6 adds lever-arm compensation on the measurement.
    # ────────────────────────────────────────────────────────────────────────
    def _on_uwb(self, ev: dict, ts: int) -> dict:
        self._advance_clock(ts)

        # Prefer the board-clamped, alpha-beta smoothed position so the position
        # filter's protection actually reaches the Kalman update. Only fall back to
        # pos_raw when neither mapped_position nor pos_clean is present.
        mapped = ev.get('mapped_position')
        if mapped:
            uwb = (mapped['board_width_x'], mapped['board_height_y'])
        else:
            uwb = ev.get('pos_clean') or ev.get('pos_raw')
        if uwb is None:
            return self._emit(ts, 'POSITION', 'UWB_DROPPED', 0, False)

        z = np.asarray(uwb, dtype=float)
        self.last_uwb = z.copy()

        # Time interpolation: innovation against state at UWB timestamp (Step 4).
        ts_uwb = ev.get('ts_hw', ts)
        interp = self._interpolate_at(ts_uwb)
        if interp is not None:
            p_ref, _, q_ref = interp
        else:
            p_ref = None
            q_ref = self.q  # latest nominal quaternion — ≤5 ms stale at 200 Hz

        # Step 6: Lever-arm correction — UWB measures the tag (back of pen),
        # not the tip. Subtract the rotated body-frame offset to get tip position.
        # At perpendicular hold the correction is ~0; at 30° tilt it is ~10 cm.
        r_board_offset, lever_r_world = self._uwb_lever_arm_board(q_ref)
        z_tip = z - r_board_offset
        self._last_lever_arm_m   = float(np.linalg.norm(r_board_offset))
        self._last_lever_r_world = lever_r_world
        self._last_z_uwb_raw     = z.copy()

        # Step 5: NLOS-adaptive R — consume trilateration residual.
        solve_error = float(ev.get('solve_error', 0.0))

        # UWB jump gate: reject when UWB-implied tip speed far exceeds physical pen
        # limits AND IMU velocity doesn't confirm the fast move.
        # Ceiling is mode-dependent: drawing allows faster legitimate strokes.
        ecfg_j   = cfg.fusion_eskf
        mode_j   = self._resolve_mode()
        if (self._last_z_tip is not None and self._last_z_tip_ts is not None):
            dt_uwb_j  = max((ts_uwb - self._last_z_tip_ts) / 1_000_000.0, 1e-6)
            uwb_speed = float(np.linalg.norm(z_tip - self._last_z_tip)) / dt_uwb_j
            imu_speed = float(np.linalg.norm(self.v))
            if uwb_speed > mode_j.jump_speed_max and imu_speed < ecfg_j.uwb_jump_imu_speed_min:
                self._uwb_jump_rejected += 1
                self._uwb_rejected += 1
                return self._emit(ts, 'POSITION', 'UWB_JUMP_REJECT', 0, False)

        # In air, low-confidence UWB (warning band) provides almost no useful
        # correction while K_air is already near zero. Rejecting it prevents
        # the alpha-beta tail from nudging the state toward a bad measurement.
        quality = ev.get('uwb_quality', {})
        stroke_active = ev.get('stroke_active', False)
        if quality.get('low_confidence') and not stroke_active:
            self._uwb_rejected += 1
            return self._emit(ts, 'POSITION', 'UWB_LOW_CONF_REJECT', 0, False)

        # Board-margin gate: reject positions that are physically outside the board
        # with a small tolerance. Catches bad trilateration solves that slip past
        # the residual threshold after lever-arm correction.
        _margin = 0.03
        if (z_tip[0] < -_margin or z_tip[0] > self._board_w + _margin or
                z_tip[1] < -_margin or z_tip[1] > self._board_h + _margin):
            self._uwb_rejected += 1
            return self._emit(ts, 'POSITION', 'UWB_BOARD_MARGIN_REJECT', 0, False)

        accepted = self._uwb_update(z_tip, p_ref, solve_error, ev.get('uwb_quality'))
        if not accepted:
            self._uwb_rejected += 1
            return self._emit(ts, 'POSITION', 'UWB_NLOS_REJECT', 0, False)
        self._uwb_accepted += 1
        # Any accepted UWB position fix keeps the sliding-window clock alive,
        # even when geometry isn't clean enough for a velocity pseudo-update.
        self._last_uwb_reset_ts = ts_uwb
        self._last_z_tip = z_tip.copy()
        self._last_z_tip_ts = ts_uwb

        # Phase 3b: UWB-velocity pseudo-measurement (Rule 2 — UWB Sync).
        # Always push to buffer (solve_error stored so bad samples are detectable).
        self._uwb_vel_buf.append((ts_uwb, z_tip.copy(), solve_error))
        # Skip velocity anchor during DRAWING_FAST — anchoring v to UWB geometry
        # would cancel the IMU shape authority the mode is designed to grant.
        if not self._in_fast_mode:
            ecfg_v = cfg.fusion_eskf
            if len(self._uwb_vel_buf) >= 3:
                (t0, p0, e0), (t1, p1, e1), (t2, p2, e2) = (
                    self._uwb_vel_buf[-3], self._uwb_vel_buf[-2], self._uwb_vel_buf[-1]
                )
                # All three samples must have clean geometry.
                if max(e0, e1, e2) < ecfg_v.sigma_trilat:
                    dt_vel = (t2 - t0) / 1_000_000.0          # span of central difference
                    if 0.02 < dt_vel < 0.5:                   # guard stale / duplicate ts
                        v_uwb = (p2 - p0) / dt_vel            # central difference at t1
                        if float(np.linalg.norm(v_uwb)) < 2.0:
                            # Adaptive sigma: tighter when geometry is cleaner.
                            e_avg   = (e0 + e1 + e2) / 3.0
                            ratio   = e_avg / ecfg_v.sigma_trilat  # 0 → pristine, 1 → threshold
                            sigma_v = ecfg_v.sigma_uwb_vel * max(ecfg_v.sigma_uwb_vel_min_scale, ratio)
                            self._last_sigma_v_eff = sigma_v
                            self._velocity_pseudo_update(v_uwb, sigma_v)
                            self._last_uwb_reset_ts = ts_uwb

        self._clamp_to_board()

        return self._emit(
            ts       = ts,
            source   = 'POSITION',
            state    = 'UWB_CORRECTION',
            sid      = 0,
            active   = False,
        )

    def _uwb_update(self,
                    z:           np.ndarray,
                    p_ref:       np.ndarray | None = None,
                    solve_error: float = 0.0,
                    uwb_quality: dict | None = None) -> bool:
        """Kalman update with H = [I 0 0].

        p_ref        — time-interpolated tip position at UWB ts (Step 4).
        solve_error  — trilateration RMS residual; drives adaptive R (Step 5).

        Returns True if update was applied, False if hard-rejected (NLOS).
        """
        ecfg = cfg.fusion_eskf

        # Hard reject: trilateration residual far exceeds nominal (NLOS).
        hard_thresh = ecfg.hard_reject_mult * cfg.uwb.trilat_max_residual
        if solve_error > hard_thresh:
            self._last_innovation_norm = 0.0
            self._last_r_scale = ecfg.r_scale_max
            return False

        # Adaptive R — scale measurement noise by NLOS severity.
        if ecfg.k_nlos > 0.0 and ecfg.sigma_trilat > 0.0:
            ratio   = solve_error / ecfg.sigma_trilat
            r_scale = 1.0 + ecfg.k_nlos * ratio * ratio
            r_scale = max(1.0, min(ecfg.r_scale_max, r_scale))
        else:
            r_scale = 1.0
        self._last_r_scale = r_scale

        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        mode_p = self._resolve_mode()

        quality = uwb_quality or {}
        uwb_quality_mult = 1.0
        if quality.get('was_clamped'):
            uwb_quality_mult *= 4.0
        if quality.get('low_confidence'):
            uwb_quality_mult *= 3.0

        sigma = ecfg.sigma_uwb * mode_p.sigma_scale * uwb_quality_mult

        # Innovation  y = z − p_ref  (time-aligned) — computed early so direction
        # check can use it before building R.
        p_nom = p_ref if p_ref is not None else self.p
        y = z - p_nom
        self._last_innovation_norm = float(np.linalg.norm(y))
        self._last_uwb_residual_rms = solve_error

        # Hard reject on large UWB↔IMU position disagreement before it can
        # destabilise K or P (a clean trilateration can still disagree with
        # the integrated IMU state after long dead-reckoning).
        if self._last_innovation_norm > ecfg.innov_hard_reject_m:
            self._uwb_rejected += 1
            return False

        # --- ADAPTIVE TRUST LOGIC ---
        current_speed = float(np.linalg.norm(self.v))
        is_bad_uwb    = r_scale > 1.05
        is_fast_move  = current_speed > 0.08

        if not is_bad_uwb:
            adaptive_multiplier = 1.0
        elif is_fast_move:
            adaptive_multiplier = 4.0
        else:
            adaptive_multiplier = 1.8

        # Direction-disagreement gate — penalty comes from mode table.
        # Relaxed during drawing (handwriting has legitimate backward curves).
        v_norm = current_speed
        y_norm = self._last_innovation_norm
        dir_factor = 1.0
        if v_norm > ecfg.dir_check_v_min and y_norm > ecfg.dir_check_y_min:
            cosang = float(np.dot(self.v, y) / (v_norm * y_norm))
            if cosang < ecfg.dir_check_cos_thresh:
                dir_factor = mode_p.dir_penalty
        adaptive_multiplier *= dir_factor
        self._last_dir_factor = dir_factor
        if dir_factor > 1.0:
            self._dir_penalty_count += 1

        R = ((sigma * adaptive_multiplier) ** 2) * r_scale * np.eye(2)
        # ----------------------------

        S = H @ self.P @ H.T + R                  # 2×2
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×2
        self._last_K_pos = float(K[0, 0])

        dx = K @ y
        if not np.all(np.isfinite(dx)):
            return False
        dx[0:2] = np.clip(dx[0:2], -0.08, 0.08)
        dx[2:4] = np.clip(dx[2:4], -0.50, 0.50)
        dx[4:6] = np.clip(dx[4:6], -0.03, 0.03)
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        I = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._sanitize_covariance()
        self._apply_covariance_floor()
        self._sanitize_state()

        return True

    def _interpolate_at(self, ts_uwb: int):
        """Bracket-and-lerp lookup in the IMU ring buffer.

        Returns (p, v, q) interpolated at ts_uwb, or None when the buffer
        has fewer than 2 entries or ts_uwb is newer than all buffered states.
        """
        if len(self._state_buf) < 2:
            return None

        buf = list(self._state_buf)   # oldest → newest

        # UWB timestamp predates the oldest buffered IMU state — use oldest.
        if ts_uwb <= buf[0][0]:
            _, p0, v0, q0 = buf[0]
            return p0.copy(), v0.copy(), q0.copy()

        # Find the bracketing pair.
        for i in range(len(buf) - 1):
            t1, p1, v1, q1 = buf[i]
            t2, p2, v2, q2 = buf[i + 1]
            if t1 <= ts_uwb <= t2:
                alpha = (ts_uwb - t1) / (t2 - t1) if t2 != t1 else 0.0
                return (
                    (1.0 - alpha) * p1 + alpha * p2,
                    (1.0 - alpha) * v1 + alpha * v2,
                    _slerp(q1, q2, alpha),
                )

        # ts_uwb is newer than all buffered states — caller uses current p.
        return None

    def _apply_covariance_floor(self):
        ecfg  = cfg.fusion_eskf
        mode_p = self._resolve_mode()
        pos_floor = mode_p.pos_floor ** 2
        pos_cap   = (mode_p.pos_floor * ecfg.pos_cap_mult) ** 2
        vel_floor = ecfg.vel_floor ** 2
        self._last_pos_floor_used = pos_floor
        self._last_pos_cap_used   = pos_cap

        for i in [0, 1]:
            self.P[i, i] = min(max(self.P[i, i], pos_floor), pos_cap)

        for i in [2, 3]:
            self.P[i, i] = max(self.P[i, i], vel_floor)

    def _reset_covariance(self):
        ecfg = cfg.fusion_eskf
        diag = np.array([
            ecfg.p0_pos,  ecfg.p0_pos,
            ecfg.p0_vel,  ecfg.p0_vel,
            ecfg.p0_bias, ecfg.p0_bias,
        ]) ** 2
        self.P = np.diag(diag)
        self._cov_resets += 1
        print(f"[ESKF] P became non-finite — covariance reset (total: {self._cov_resets})")

    def _sanitize_covariance(self):
        if not np.all(np.isfinite(self.P)):
            self._reset_covariance()
            return
        # Force symmetry to counteract float accumulation drift
        self.P = 0.5 * (self.P + self.P.T)
        # Clamp bias-block diagonal — nothing else constrains it when sigma_b_a is tiny
        for i in [4, 5]:
            self.P[i, i] = max(1e-10, min(self.P[i, i], 0.25))

    def _sanitize_state(self):
        if not np.all(np.isfinite(self.b_a)):
            self.b_a[:] = 0.0
        if not np.all(np.isfinite(self.v)):
            self.v[:] = 0.0
        if not np.all(np.isfinite(self.p)):
            self.p[:] = np.array([self._board_w * 0.5, self._board_h * 0.5])

    # ────────────────────────────────────────────────────────────────────────
    # Filter math
    # ────────────────────────────────────────────────────────────────────────
    def _build_F(self, dt: float) -> np.ndarray:
        """Error-state transition. Blocks (δp, δv, δb_a) of 2 each.

        Since  a_true = acc_board − b_a,  p̈ = a_true,  v̇ = a_true:
            ∂δp/∂δv   =  dt · I
            ∂δp/∂δb_a = −0.5·dt² · I
            ∂δv/∂δb_a = −dt · I
        """
        F = np.eye(6)
        I2 = np.eye(2)
        F[0:2, 2:4] = dt * I2
        F[0:2, 4:6] = -0.5 * dt * dt * I2
        F[2:4, 4:6] = -dt * I2
        return F

    def _build_Q(self, dt: float) -> np.ndarray:
        """Discrete-time process noise.

        Accel white-noise (σ_a) enters via the kinematic chain — it couples
        δp and δv with cross-covariance. Bias random walk (σ_b_a) drives
        only the δb_a block. σ_a is inflated during sharp-stroke windows
        (turn_flag=True) so UWB can correct shape aggressively at corners.
        """
        ecfg = cfg.fusion_eskf
        sa_eff = ecfg.sigma_a * (ecfg.turn_k_q if self._turn_flag_last else 1.0)
        sa2 = sa_eff * sa_eff
        sb2 = ecfg.sigma_b_a * ecfg.sigma_b_a

        I2 = np.eye(2)
        Q = np.zeros((6, 6))
        # δp–δp
        Q[0:2, 0:2] = 0.25 * sa2 * dt ** 4 * I2
        # δp–δv (cross, both signs)
        Q[0:2, 2:4] = 0.50 * sa2 * dt ** 3 * I2
        Q[2:4, 0:2] = 0.50 * sa2 * dt ** 3 * I2
        # δv–δv
        Q[2:4, 2:4] = sa2 * dt * dt * I2
        # δb_a–δb_a
        Q[4:6, 4:6] = sb2 * dt * I2
        return Q

    def _zupt_update(self):
        """Kalman update for the zero-velocity pseudo-measurement (z = 0)."""
        self._zupt_fires += 1
        self._zupt_soft_update(sigma=cfg.fusion_eskf.sigma_zupt)

    def _zupt_soft_update(self, sigma: float):
        """Zero-velocity pseudo-measurement with caller-supplied noise sigma."""
        H = np.zeros((2, 6))
        H[0, 2] = 1.0
        H[1, 3] = 1.0

        R = (sigma ** 2) * np.eye(2)

        # Innovation y = z − H·x_nom  with z = 0  →  y = −v_nom
        y = -self.v.copy()

        S = H @ self.P @ H.T + R                  # 2×2
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×2

        dx = K @ y                                # 6-vec
        if not np.all(np.isfinite(dx)):
            return
        dx[0:2] = np.clip(dx[0:2], -0.08, 0.08)
        dx[2:4] = np.clip(dx[2:4], -0.50, 0.50)
        dx[4:6] = np.clip(dx[4:6], -0.03, 0.03)
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        # Joseph-form covariance update (numerically stable).
        I  = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._sanitize_covariance()
        self._apply_covariance_floor()
        self._sanitize_state()
        
        
    def _velocity_pseudo_update(self, v_meas: np.ndarray, sigma: float):
        """Kalman update for a UWB-derived velocity pseudo-measurement.

        H = [0 I 0]  (velocity block at columns 2–3).
        Innovation y = v_meas − v_nom.
        Uses the same Joseph-form update as ZUPT for numerical stability.
        """
        H = np.zeros((2, 6))
        H[0, 2] = 1.0
        H[1, 3] = 1.0

        R = (sigma ** 2) * np.eye(2)
        y = v_meas - self.v

        S   = H @ self.P @ H.T + R
        K   = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ y
        if not np.all(np.isfinite(dx)):
            return
        dx[0:2] = np.clip(dx[0:2], -0.08, 0.08)
        dx[2:4] = np.clip(dx[2:4], -1.50, 1.50)
        dx[4:6] = np.clip(dx[4:6], -0.03, 0.03)
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        I   = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._sanitize_covariance()
        self._apply_covariance_floor()
        self._sanitize_state()

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────
    def _update_omega_and_turn(self, q_new: np.ndarray, dt_s: float,
                               current_jerk: float = 0.0,
                               omega_world_ev=None):
        """Derive in-plane angular velocity; update turn flag.

        omega_world_ev — precomputed world-frame ω (3-tuple) from the IMU
        preprocessor's rigid-body block.  When supplied, the quaternion-diff
        derivation is skipped (single source of truth, no duplicate math).
        Falls back to Δquat when the field is absent (e.g. older recordings).

        ω ≈ 2·(q_k ⊗ q_{k-1}⁻¹).xyz / dt  (small-angle, fallback only).
        If ω_in_plane exceeds threshold AND jerk is high, inflate Q for
        turn_n_post frames (corner detection).
        """
        ecfg = cfg.fusion_eskf

        if omega_world_ev is not None:
            # Fast path — use precomputed ω from imu.py (avoids duplicate quat diff).
            omega_world = np.asarray(omega_world_ev, dtype=float)
            axis_map = {'x': 0, 'y': 1, 'z': 2}
            ax0 = axis_map[cfg.imu.board_axes[0]]
            ax1 = axis_map[cfg.imu.board_axes[1]]
            ω_ip = math.sqrt(float(omega_world[ax0])**2 + float(omega_world[ax1])**2)
            self._omega_in_plane_last = ω_ip
        elif self._prev_quat is not None and dt_s > 1e-6:
            # Fallback path — derive ω from consecutive quaternions.
            q_delta = _quat_multiply(q_new, _quat_conjugate(self._prev_quat))
            if q_delta[3] < 0:           # choose shorter arc
                q_delta = -q_delta
            omega_body  = 2.0 * q_delta[0:3] / dt_s
            omega_world = _q_to_rotation(q_new) @ omega_body

            axis_map = {'x': 0, 'y': 1, 'z': 2}
            ax0 = axis_map[cfg.imu.board_axes[0]]
            ax1 = axis_map[cfg.imu.board_axes[1]]
            ω_ip = math.sqrt(omega_world[ax0]**2 + omega_world[ax1]**2)
            self._omega_in_plane_last = ω_ip
        else:
            self._omega_in_plane_last = 0.0

        # Turn detection: corner only when turning fast AND jerk is high,
        # sustained for turn_arm_n consecutive samples.  The arm counter
        # de-noises single-sample vibration spikes that plagued fast writing
        # (median jerk 600–760 m/s³ was tripping the old hardcoded 800 threshold).
        is_turning = self._omega_in_plane_last > ecfg.turn_omega_threshold
        is_jerky   = current_jerk > ecfg.turn_jerk_threshold
        if is_turning and is_jerky:
            self._turn_arm_count += 1
            if self._turn_arm_count >= ecfg.turn_arm_n:
                self._turn_cooldown = ecfg.turn_n_post
        else:
            self._turn_arm_count = max(0, self._turn_arm_count - 1)

        if self._turn_cooldown > 0:
            self._turn_flag_last = True
            self._turn_cooldown -= 1
        else:
            self._turn_flag_last = False

        self._prev_quat = q_new.copy()

    def _uwb_lever_arm_board(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (2D board-plane offset, full 3-vector r_world) from tip to UWB tag.

        r_UWB_body is rotated to world frame via q, then projected onto the board
        plane defined by cfg.imu.board_axes.  For a perpendicular pen the 2D offset
        is ~0; for a tilted pen it is real.  The full r_world is returned for
        diagnostics (log lever_r_world to verify perpendicular-hold assumption).
        """
        r_body  = np.asarray(cfg.marker.r_uwb_body_m, dtype=float)
        R       = _q_to_rotation(q)
        r_world = R @ r_body

        axis_map = {'x': 0, 'y': 1, 'z': 2}
        ax0 = axis_map[cfg.imu.board_axes[0]]
        ax1 = axis_map[cfg.imu.board_axes[1]]
        return np.array([r_world[ax0], r_world[ax1]], dtype=float), r_world

    def _advance_clock(self, ts: int) -> float:
        """Returns dt (s) since the previous event; handles gaps and init."""
        dt_nom = 1.0 / cfg.imu.sample_rate_hz
        if self.last_ts is None:
            self.last_ts = ts
            return dt_nom

        dt_s = (ts - self.last_ts) / 1_000_000.0
        self.last_ts = ts
        if dt_s <= 0.0 or dt_s > 0.5:
            # Monotonicity / long-gap guard: treat as nominal step.
            return dt_nom
        # Soft cap: IMU timestamps can spike 8–10× nominal (hardware jitter).
        # Clamping prevents 0.5·a·dt² from blowing up on those frames.
        dt_max = cfg.fusion_eskf.imu_dt_max_mult * dt_nom
        if dt_s > dt_max:
            self._dt_clamps += 1
            return dt_max
        return dt_s

    def _clamp_to_board(self):
        self.p[0] = max(0.0, min(self._board_w, self.p[0]))
        self.p[1] = max(0.0, min(self._board_h, self.p[1]))

    def _emit(self, ts: int, source: str, state: str, sid: int, active: bool) -> dict:
        return {
            'ts_hw': ts,
            'source': source,
            'fused_x': float(self.p[0]),
            'fused_y': float(self.p[1]),
            'uwb_x': float(self.last_uwb[0]),
            'uwb_y': float(self.last_uwb[1]),
            'state': state,
            'stroke_id': sid,
            'stroke_active': active,
            'eskf': {
                'P_pos_trace':       float(np.sqrt(self.P[0, 0] + self.P[1, 1])),
                'innovation_norm':   self._last_innovation_norm,
                'r_scale':           self._last_r_scale,
                'K_pos_diag':        self._last_K_pos,
                'b_a_norm':          float(np.linalg.norm(self.b_a)),
                'uwb_residual_rms':  self._last_uwb_residual_rms,
                'omega_in_plane':    self._omega_in_plane_last,
                'turn_flag':         self._turn_flag_last,
                'drawing_fast':      self._in_fast_mode,
                'fast_arm_count':    self._fast_arm_count,
                'fast_burst_count':  self._fast_burst_count,
                'b_a':               (float(self.b_a[0]), float(self.b_a[1])),
                'uwb_accepted':      self._uwb_accepted,
                'uwb_rejected':      self._uwb_rejected,
                # Lever-arm diagnostics (updated on every UWB frame; 0/zeros on IMU frames)
                'lever_arm_m':       self._last_lever_arm_m,
                'lever_r_world':     tuple(float(v) for v in self._last_lever_r_world),
                'z_uwb_raw':         tuple(float(v) for v in self._last_z_uwb_raw),
                # Sliding-window diagnostics
                'uwb_stale_s':       round(self._last_stale_s, 4),
                'stale_factor':      round(self._last_stale_factor, 4),
                'sigma_v_eff':       round(self._last_sigma_v_eff, 5),
                # Adaptive trust diagnostics
                'uwb_jump_rejected': self._uwb_jump_rejected,
                'dir_factor':        round(self._last_dir_factor, 3),
                'dir_penalty_count': self._dir_penalty_count,
                'pos_floor_used':    round(self._last_pos_floor_used, 6),
                # New diagnostics (abc4/5/6 regression fixes)
                'dt_clamps':         self._dt_clamps,
                'turn_arm_count':    self._turn_arm_count,
                'pos_cap_used':      round(self._last_pos_cap_used, 6),
                'zupt_fires':        self._zupt_fires,
                'cov_resets':        self._cov_resets,
                # Mode statistics — counters for per-regime tuning
                'frames_contact':    self._frames_contact,
                'frames_air':        self._frames_air,
                'frames_static':     self._frames_static,
                'avg_K_contact':     round(
                    self._k_contact_sum / self._frames_contact
                    if self._frames_contact > 0 else 0.0, 5),
                'avg_K_air':         round(
                    self._k_air_sum / self._frames_air
                    if self._frames_air > 0 else 0.0, 5),
            },
        }


# ==============================================================================
# LIVE HARDWARE SELF-TEST
#   SerialStreamer → Normalizer → TimeAlign → (IMU | UWB) → ESKF
#   Prints a dashboard; Ctrl+C stops and exports a CSV.
# ==============================================================================
if __name__ == '__main__':
    import time
    import os
    import csv

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    from background.pipelines.cleaner.unpacker       import SerialStreamer
    from background.pipelines.cleaner.normalizer     import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    from background.pipelines.preprocess.imu         import IMUPreprocessor
    from background.pipelines.preprocess.contact     import ContactStateDetector
    from background.pipelines.preprocess.uwb.range         import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.preprocess.uwb.position      import UWBPositionFilter

    SERIAL_PORT  = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE    = getattr(cfg.serial, 'baud', 115200)
    DISPLAY_RATE = 0.1

    streamer   = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm       = StreamNormalizer()
    aligner    = TimeAlignLayer(buffer_size=500)

    imu_prep   = IMUPreprocessor()
    contact    = ContactStateDetector()

    uwb_offs   = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offs)
    trilat     = UWBSolver()
    pos_filter = UWBPositionFilter()

    eskf = ESKF()

    print("=" * 64)
    print(f"  [TEST] ESKF LIVE — lever-arm correction active — {SERIAL_PORT}")
    print("  Draw strokes. Ctrl+C to stop and export CSV.")
    print("=" * 64)

    last_print_time = 0.0
    latest      = None
    latest_imu  = None   # most-recent pre-fusion IMU event
    imu_count   = 0
    uwb_count   = 0
    event_log   = []     # every fused event → CSV on exit

    try:
        while True:
            raw = streamer.read_new_packets()
            if raw:
                evs = norm.normalize(raw)
                aligner.add_events(evs)
                sorted_evs = aligner.get_all_sorted()
                aligner.clear()

                for ev in sorted_evs:
                    if ev['sensor'] == 'IMU':
                        p = imu_prep.process_one(ev)
                        if p:
                            s = contact.process_one(p)
                            fused = eskf.process_event(s)
                            if fused:
                                imu_count += 1
                                latest     = fused
                                latest_imu = s
                                fused['_imu_ev'] = s   # keep IMU event for CSV columns
                                event_log.append(fused)
                    elif ev['sensor'] == 'UWB':
                        for r in range_prep.feed([ev]):
                            raw_pos = trilat.process_one(r)
                            if raw_pos:
                                clean = pos_filter.process_one(raw_pos)
                                if clean:
                                    fused = eskf.process_event(clean)
                                    if fused:
                                        uwb_count += 1
                                        latest    = fused
                                        fused['_imu_ev'] = None
                                        event_log.append(fused)

            now = time.time()
            if latest and (now - last_print_time) >= DISPLAY_RATE:
                os.system('cls' if os.name == 'nt' else 'clear')
                e   = latest['eskf']
                imu = latest_imu  # may be None briefly on startup

                # ── Rigid-body / lever-arm diagnostics from the IMU event ──
                if imu is not None:
                    ab      = imu.get('acc_board',     (0.0, 0.0))
                    ab_tip  = imu.get('acc_board_tip', (0.0, 0.0))
                    ow      = imu.get('omega_world',   (0.0, 0.0, 0.0))
                    aw      = imu.get('alpha_world',   (0.0, 0.0, 0.0))
                    jerk    = imu.get('jerk', 0.0)
                    acc_sensor_mag = math.sqrt(sum(x*x for x in imu.get('acc_sensor', (0,0,0))))
                else:
                    ab = ab_tip = (0.0, 0.0)
                    ow = aw = (0.0, 0.0, 0.0)
                    jerk = acc_sensor_mag = 0.0

                acc_board_mag     = math.sqrt(ab[0]**2     + ab[1]**2)
                acc_board_tip_mag = math.sqrt(ab_tip[0]**2 + ab_tip[1]**2)
                omega_world_mag   = math.sqrt(ow[0]**2 + ow[1]**2 + ow[2]**2)
                alpha_world_mag   = math.sqrt(aw[0]**2 + aw[1]**2 + aw[2]**2)

                lever_arm_m = e.get('lever_arm_m', 0.0)
                turn_str    = "TURN" if e['turn_flag'] else "----"

                print(f"============= LIVE ESKF  ({DISPLAY_RATE}s refresh) =============")
                print(f"  State      : {latest['state']:<20}  Source: {latest['source']}")
                print(f"  Stroke     : ID={latest['stroke_id']}  Active={latest['stroke_active']}")
                print("─" * 64)
                print(f"  [POSITION]")
                print(f"    Fused tip  : X={latest['fused_x']:7.4f} m   Y={latest['fused_y']:7.4f} m")
                print(f"    UWB tag    : X={latest['uwb_x']:7.4f} m   Y={latest['uwb_y']:7.4f} m")
                print(f"    Lever-arm  : {lever_arm_m*100:5.1f} cm  (UWB tag→tip offset in board plane)")
                print("─" * 64)
                print(f"  [RIGID-BODY TIP CORRECTION]  (rigid_body_enabled={cfg.imu.rigid_body_enabled})")
                print(f"    |acc_board|         : {acc_board_mag:7.4f} m/s²  (sensor-point, legacy)")
                print(f"    |acc_board_tip|     : {acc_board_tip_mag:7.4f} m/s²  (tip-corrected ← ESKF uses this)")
                print(f"    reduction           : {max(0.0, acc_board_mag - acc_board_tip_mag):+6.4f} m/s²")
                print(f"    |acc_sensor| 3D     : {acc_sensor_mag:7.4f} m/s²")
                print(f"    jerk (body)         : {jerk:9.1f} m/s³  (raw wrist whip)")
                print("─" * 64)
                print(f"  [ANGULAR KINEMATICS]")
                print(f"    |ω_world|           : {omega_world_mag:7.3f} rad/s  (expect 0–20 during writing)")
                print(f"    ω in-plane (board)  : {e['omega_in_plane']:7.3f} rad/s  [{turn_str}]")
                print(f"    |α_world| (EMA)     : {alpha_world_mag:7.1f} rad/s²")
                print("─" * 64)
                print(f"  [FILTER HEALTH]")
                print(f"    P_pos_trace : {e['P_pos_trace']:.4f} m     Innovation |y|: {e['innovation_norm']:.4f} m")
                print(f"    R scale     : {e['r_scale']:.2f}  (1.0=clean, >3=NLOS)")
                print(f"    K_pos_diag  : {e['K_pos_diag']:.4f}         UWB resid: {e['uwb_residual_rms']:.4f} m")
                print(f"    Bias b_a    : ({e['b_a'][0]:+.4f}, {e['b_a'][1]:+.4f}) m/s²")
                print(f"    IMU / UWB   : {imu_count} / {uwb_count}  (accepted={e['uwb_accepted']}  rejected={e['uwb_rejected']})")
                print("=" * 64)
                last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting ESKF.")
        streamer.close()

        # ── Session summary ──────────────────────────────────────────────────
        print("-" * 64)
        print(f"  IMU events processed : {imu_count}")
        print(f"  UWB events processed : {uwb_count}")
        if latest:
            e = latest['eskf']
            print(f"  Final fused position : ({latest['fused_x']:.3f}, {latest['fused_y']:.3f}) m")
            print(f"  Final P_pos_trace    : {e['P_pos_trace']:.4f} m")
            print(f"  UWB accepted/rejected: {e['uwb_accepted']} / {e['uwb_rejected']}")
            print(f"  Last lever-arm offset: {e.get('lever_arm_m', 0.0)*100:.1f} cm")
        print("=" * 64)

        if not event_log:
            print("No events logged. Exiting.")
            exit()

        # ── CSV export ───────────────────────────────────────────────────────
        # Columns are ordered: identity → position → lever-arm → rigid-body
        # → angular kinematics → filter health → contact/stroke.
        # Legacy columns keep the same names so old CSVs diff cleanly.
        csv_filename = "eskf_session.csv"
        _CSV_COLS = [
            # Identity
            'ts_hw', 'source', 'state',
            # Fused tip position (lever-arm corrected)
            'fused_x', 'fused_y',
            # Raw UWB tag position (not tip-corrected — for comparison)
            'uwb_x', 'uwb_y',
            # Lever-arm
            'lever_arm_m',
            # Rigid-body tip correction diagnostics (from IMU event)
            'acc_board_x', 'acc_board_z',          # sensor-point (legacy)
            'acc_board_tip_x', 'acc_board_tip_z',  # tip-corrected (canonical)
            'acc_board_mag', 'acc_board_tip_mag',  # magnitudes for quick diff
            'acc_sensor_mag',                       # raw 3D sensor magnitude
            'jerk',                                 # body-frame wrist-whip indicator
            # Angular kinematics (from IMU event)
            'omega_world_x', 'omega_world_y', 'omega_world_z', 'omega_world_mag',
            'omega_body_x',  'omega_body_y',  'omega_body_z',
            'alpha_world_x', 'alpha_world_y', 'alpha_world_z', 'alpha_world_mag',
            # Filter health (from eskf sub-dict)
            'P_pos_trace', 'innovation_norm', 'r_scale',
            'K_pos_diag', 'b_a_x', 'b_a_y', 'b_a_norm',
            'omega_in_plane', 'turn_flag',
            'uwb_residual_rms', 'uwb_accepted', 'uwb_rejected',
            # Contact / stroke
            'stroke_id', 'stroke_active', 'is_static', 'contact',
        ]
        print(f"[EXPORT] Writing {len(event_log)} rows → {csv_filename} ...")
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(_CSV_COLS)
            for ev in event_log:
                e   = ev.get('eskf', {})
                imu = ev.get('_imu_ev')   # attached in the event loop above
                ba  = e.get('b_a', (0.0, 0.0))

                # IMU-side fields (zero-fill on UWB rows)
                ab     = imu.get('acc_board',     (0.0, 0.0)) if imu else (0.0, 0.0)
                ab_tip = imu.get('acc_board_tip', (0.0, 0.0)) if imu else (0.0, 0.0)
                ow     = imu.get('omega_world',   (0.0, 0.0, 0.0)) if imu else (0.0, 0.0, 0.0)
                ob     = imu.get('omega_body',    (0.0, 0.0, 0.0)) if imu else (0.0, 0.0, 0.0)
                aw     = imu.get('alpha_world',   (0.0, 0.0, 0.0)) if imu else (0.0, 0.0, 0.0)
                asens  = imu.get('acc_sensor',    (0.0, 0.0, 0.0)) if imu else (0.0, 0.0, 0.0)
                jerk   = imu.get('jerk', 0.0)  if imu else 0.0
                is_static = bool(imu.get('is_static', False)) if imu else False
                contact   = bool(imu.get('contact',   False)) if imu else False

                ab_mag     = math.sqrt(ab[0]**2     + ab[1]**2)
                ab_tip_mag = math.sqrt(ab_tip[0]**2 + ab_tip[1]**2)
                asens_mag  = math.sqrt(sum(x*x for x in asens))
                ow_mag     = math.sqrt(ow[0]**2 + ow[1]**2 + ow[2]**2)
                aw_mag     = math.sqrt(aw[0]**2 + aw[1]**2 + aw[2]**2)

                writer.writerow([
                    # Identity
                    ev.get('ts_hw'),
                    ev.get('source'),
                    ev.get('state', ''),
                    # Fused position
                    round(ev.get('fused_x', 0.0), 6),
                    round(ev.get('fused_y', 0.0), 6),
                    # UWB tag position
                    round(ev.get('uwb_x', 0.0), 6),
                    round(ev.get('uwb_y', 0.0), 6),
                    # Lever-arm
                    round(e.get('lever_arm_m', 0.0), 5),
                    # Rigid-body diagnostics
                    round(ab[0], 6),
                    round(ab[1], 6),
                    round(ab_tip[0], 6),
                    round(ab_tip[1], 6),
                    round(ab_mag, 6),
                    round(ab_tip_mag, 6),
                    round(asens_mag, 6),
                    round(jerk, 3),
                    # Angular kinematics
                    round(ow[0], 5), round(ow[1], 5), round(ow[2], 5), round(ow_mag, 5),
                    round(ob[0], 5), round(ob[1], 5), round(ob[2], 5),
                    round(aw[0], 3), round(aw[1], 3), round(aw[2], 3), round(aw_mag, 3),
                    # Filter health
                    round(e.get('P_pos_trace', 0.0), 6),
                    round(e.get('innovation_norm', 0.0), 6),
                    round(e.get('r_scale', 1.0), 4),
                    round(e.get('K_pos_diag', 0.0), 6),
                    round(ba[0], 6),
                    round(ba[1], 6),
                    round(e.get('b_a_norm', 0.0), 6),
                    round(e.get('omega_in_plane', 0.0), 4),
                    int(e.get('turn_flag', False)),
                    round(e.get('uwb_residual_rms', 0.0), 6),
                    e.get('uwb_accepted', 0),
                    e.get('uwb_rejected', 0),
                    # Contact / stroke
                    ev.get('stroke_id', 0),
                    int(ev.get('stroke_active', False)),
                    int(is_static),
                    int(contact),
                ])
        print(f"[EXPORT] Saved → {csv_filename}")
        print()
        print("  Verification tips:")
        print("  - lever_arm_m ≈ 0 when pen perpendicular; ~0.10 m at 30° tilt")
        print("  - acc_board_tip_mag < acc_board_mag during circular strokes (60–90% drop)")
        print("  - omega_world_mag: 0–20 rad/s normal; >100 = timestamp glitch")
        print("  - alpha_world_mag: up to a few hundred rad/s² normal after EMA")
        print("  - jerk > 7000 m/s³ = wrist whip event (expected, turn detection arms)")
        print("  Rename before next session: e.g. eskf_with_leverarm.csv")
        print("=" * 64)
