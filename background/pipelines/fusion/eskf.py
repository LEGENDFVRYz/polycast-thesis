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
from background.pipelines.fusion.stroke_dead_reckoner import StrokeIMUDeadReckoner


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


def _clip_vec_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    """Limit a 2D correction by vector magnitude instead of per-axis only."""
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n <= max_norm or n < 1e-12:
        return v
    return v * (max_norm / n)


class ESKF:
    def __init__(self):
        ecfg = cfg.fusion_eskf

        # ── Nominal state (propagated directly, not in the filter) ──────────
        bx = cfg.anchors.board_size_x
        by = cfg.anchors.board_size_y
        self.p   = np.array([bx * 0.5, by * 0.5], dtype=float)   # position
        self.v   = np.zeros(2, dtype=float)                      # velocity
        self.b_a = np.zeros(2, dtype=float)                      # accel bias
        self.b_p = np.zeros(2, dtype=float)                      # position bias (Phase 4 EMA tracker)
        self._active_uwb_guard_fired = False
        self._active_uwb_guard_correction = 0.0

        # Mode-aware stationary-contact clamp.  When contact.py reports
        # CONTACT_STATIC, or when force/contact + low motion indicates that the
        # tip is planted, the visible tip is pinned to a local anchor and
        # velocity is aggressively killed.  This prevents start/end hold drift.
        self._tip_lock_active = False
        self._tip_lock_candidate = False
        self._tip_lock_count = 0
        self._tip_lock_anchor_visible: np.ndarray | None = None
        self._tip_lock_correction = 0.0
        self._tip_lock_uwb_blend = 0.0
        self._tip_lock_reason = 'OFF'

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
        self._last_stroke_state: str = 'UNKNOWN'   # last stroke_state string from IMU event

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
        self.last_ts: int | None = None        # IMU-only propagation clock
        self.last_uwb_ts: int | None = None    # UWB arrival timestamp (diagnostics only)
        self.last_uwb = self.p.copy()
        self.last_uwb_tip = self.p.copy()  # last accepted lever-arm-corrected UWB tip position
        self._dead_reckoner = StrokeIMUDeadReckoner()
        self._last_uwb_fix_ts: int | None = None   # hw ts of the last stored last_uwb
        self._last_innovation_norm = 0.0
        self._last_r_scale = 1.0
        self._last_K_pos = 0.0
        self._last_uwb_residual_rms = 0.0
        self._uwb_accepted = 0
        self._uwb_rejected = 0
        self._stroke_start_snaps = 0   # number of pen-down soft snaps applied
        self._stroke_start_ts: int | None = None   # hw ts of current stroke's pen-down edge
        self._have_imu_attitude = False
        self._last_lever_arm_m = 0.0
        self._last_lever_r_world = np.zeros(3, dtype=float)
        self._last_z_uwb_raw     = np.zeros(2, dtype=float)
        self._last_z_uwb_tip     = np.zeros(2, dtype=float)
        self._vel_pseudo_applied = False
        self._vel_pseudo_dx_pos  = np.zeros(2, dtype=float)
        self._vel_pseudo_dx_vel  = np.zeros(2, dtype=float)

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
        self._dt_clamps = 0                 # cumulative count of dt-jitter clamps
        self._cov_resets = 0                # cumulative covariance reset events (NaN recovery)
        self._cov_reset_suppress = False    # True for one emit cycle after a covariance reset

        # ── Innovation-deadlock recovery ─────────────────────────────────────
        # Counts consecutive UWB updates rejected by the innovation hard-gate.
        # When the streak reaches innov_recovery_n with clean UWB geometry, the
        # filter is assumed lost and _snap_to_uwb() re-localizes the position.
        self._innov_reject_streak = 0
        self._uwb_snap_count = 0     # cumulative re-localizations for diagnostics

        # ── DRAWING_FAST mode tracking ───────────────────────────────────────
        self._fast_arm_count: int   = 0    # consecutive frames at/above speed threshold
        self._fast_burst_count: int = 0    # hold-down counter after trigger fires
        self._in_fast_mode: bool    = False

        # ── Mode statistics (accelerate per-regime tuning) ───────────────────
        # Frame counters are driven by IMU events (accurate mode classification).
        self._frames_contact = 0     # CONTACT_DRAWING (normal speed) IMU frames
        self._frames_fast    = 0     # DRAWING_FAST IMU frames
        self._frames_air     = 0     # AIR_MOVE IMU frames
        self._frames_static  = 0     # IDLE / CONTACT_STATIC IMU frames
        # K-sum counters are driven by accepted UWB events so the averages reflect
        # actual Kalman gain at UWB-update time, not a stale carry-over value.
        self._k_contact_sum  = 0.0   # K_pos sum during accepted UWB fixes in normal-contact mode
        self._k_contact_uwb  = 0     # accepted UWB count during normal-contact mode
        self._k_fast_sum     = 0.0   # K_pos sum during accepted UWB fixes in fast-contact mode
        self._k_fast_uwb     = 0     # accepted UWB count during fast-contact mode
        self._k_air_sum      = 0.0   # K_pos sum during accepted UWB fixes in air mode
        self._k_air_uwb      = 0     # accepted UWB count during air mode

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

    def _update_and_resolve_mode(self):
        """Update DRAWING_FAST state and return mode params — call ONCE per IMU frame.

        DRAWING_FAST arms after drawing_fast_min_frames consecutive frames where
        tip speed >= drawing_fast_speed_thresh, then holds for drawing_fast_burst_frames
        additional IMU frames before returning to DRAWING.  This gives IMU a bounded
        authority window on short fast strokes without letting it drift indefinitely.
        Resets immediately on pen-up.

        This method mutates _fast_arm_count, _fast_burst_count, _in_fast_mode.
        Use _mode_params() everywhere else (pure read, no side effects).
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

    def _mode_params(self):
        """Return current mode params — pure read, no side effects.

        Uses the state already computed by _update_and_resolve_mode() on the last
        IMU frame.  Safe to call from UWB path, covariance helpers, etc.
        """
        ecfg = cfg.fusion_eskf
        if self._prev_stroke_active:
            return ecfg.modes.drawing_fast if self._in_fast_mode else ecfg.modes.drawing
        if self._last_stroke_state in ('IDLE', 'CONTACT_STATIC'):
            return ecfg.modes.static
        return ecfg.modes.air

    def _mode_name(self) -> str:
        """Human-readable fusion mode name matching _mode_params() — pure read."""
        if self._tip_lock_active:
            return 'CONTACT_STATIC_LOCK'
        if self._prev_stroke_active:
            return 'DRAWING_FAST' if self._in_fast_mode else 'CONTACT_DRAWING'
        if self._last_stroke_state in ('IDLE', 'CONTACT_STATIC'):
            return 'IDLE'
        return 'AIR_MOVE'

    def _stroke_age_cap_mult(self, ts: int) -> float:
        """Return pos_gain_cap multiplier based on active-stroke age.

        Linearly ramps from 1.0 (pure IMU shape authority for short strokes) up
        to age_ramp_mult_max (drift guard for long geometric strokes).  Returns
        1.0 when no stroke is active so AIR/STATIC caps are unaffected.
        """
        if self._stroke_start_ts is None:
            return 1.0
        ecfg = cfg.fusion_eskf
        age_s = (ts - self._stroke_start_ts) * 1e-6   # µs → s
        if age_s <= ecfg.age_ramp_start_s:
            return 1.0
        if age_s >= ecfg.age_ramp_end_s:
            return ecfg.age_ramp_mult_max
        t = (age_s - ecfg.age_ramp_start_s) / (ecfg.age_ramp_end_s - ecfg.age_ramp_start_s)
        return 1.0 + t * (ecfg.age_ramp_mult_max - 1.0)

    # ────────────────────────────────────────────────────────────────────────
    # IMU path — prediction + ZUPT update + turn-aware Q (Steps 2 + 7)
    # ────────────────────────────────────────────────────────────────────────
    def _on_imu(self, ev: dict, ts: int) -> dict:
        dt_s = self._advance_imu_clock(ts)

        # Step 7: Update quaternion first so omega and Q use the current sample.
        q_raw = ev.get('quat')
        q_new = np.asarray(q_raw, dtype=float) if q_raw is not None else self.q.copy()
        
        # Use precomputed ω from imu.py when available; fall back to quat-diff.
        current_jerk    = ev.get('jerk', 0.0)
        omega_world_ev  = ev.get('omega_world')   # None on older recorded events
        self._update_omega_and_turn(q_new, dt_s, current_jerk, omega_world_ev)
        
        self.q = q_new
        self._have_imu_attitude = True

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
        # Separate the two tip-corrected sources produced by imu.py:
        #   acc_hpf — Path C (HPF): bias stripped, fast-motion detail preserved.
        #   acc_raw — Path A (EMA): bias still present, slow motion preserved.
        # Both already have rigid-body lever-arm correction applied.
        acc_hpf = np.asarray(
            ev.get('acc_board_hp_tip') or ev.get('acc_board', (0.0, 0.0)),
            dtype=float)
        acc_raw = np.asarray(
            ev.get('acc_board_tip') or ev.get('acc_board', (0.0, 0.0)),
            dtype=float)

        # Read contact state before propagation.  contact.py already separates
        # CONTACT_STATIC from CONTACT_DRAWING; the clamp uses that mode to avoid
        # integrating IMU noise while the physical tip is planted.
        stroke_state = ev.get('stroke_state', 'UNKNOWN')
        stroke_active_now = bool(ev.get('stroke_active', False))
        stroke_active_prev = self._prev_stroke_active
        self._last_stroke_state = stroke_state   # used by _mode_params / _mode_name
        contact_static_candidate = self._contact_static_candidate(ev, acc_hpf, stroke_state)

        mode_p = self._update_and_resolve_mode()
        ecfg = cfg.fusion_eskf
        dr_cfg = ecfg.dead_reckoner

        # Contact-state-aware blended acceleration.
        # CONTACT_DRAWING: blend raw-minus-bias (physical slow motion) with HPF
        #   (bias-free fast detail) — preserves both slow strokes and fast curvature.
        # AIR_MOVE: minimal HPF only — limits air drift without blackout.
        # CONTACT_STATIC / IDLE / UNKNOWN: zero — no integration while planted.
        detail_w = 0.0
        if contact_static_candidate:
            mode_p = ecfg.modes.static
            a = np.zeros(2, dtype=float)
            self._in_fast_mode = False
            self._fast_arm_count = 0
            self._fast_burst_count = 0
        elif stroke_state == 'CONTACT_DRAWING':
            detail_w = float(dr_cfg.detail_weight)
            a = (acc_raw - self.b_a) * (1.0 - detail_w) + acc_hpf * detail_w
            if ecfg.acc_spike_clamp_enabled and self._prev_stroke_active:
                a = _clip_vec_norm(a, ecfg.acc_spike_clamp_ms2)
        elif stroke_state == 'AIR_MOVE':
            detail_w = float(dr_cfg.air_scale)
            a = acc_hpf * detail_w
            if ecfg.acc_spike_clamp_enabled:
                a = _clip_vec_norm(a, ecfg.acc_spike_clamp_ms2)
        else:
            # CONTACT_STATIC / IDLE / UNKNOWN
            a = np.zeros(2, dtype=float)

        # Keep acc pointing at the HPF source so downstream consumers
        # (imu_cleaner payload, spike-clamp paths that reference acc) are unaffected.
        acc = acc_hpf

        # Dead reckoner: accumulate stroke-local relative displacement alongside
        # the global ESKF state. Only active during confirmed ink frames.
        self._dead_reckoner.set_blend_weight(detail_w)
        if stroke_state == 'CONTACT_DRAWING' and not contact_static_candidate:
            self._dead_reckoner.update(
                acc_blend=a,
                dt_s=dt_s,
                drag_inv_s=mode_p.drag_inv_s,
            )

        self.p += self.v * dt_s + 0.5 * a * dt_s * dt_s
        self.v += a * dt_s
        # Velocity drag — further scaled by stale_factor when UWB is silent.
        drag_inv_s = mode_p.drag_inv_s * stale_factor
        self.v *= max(0.0, 1.0 - drag_inv_s * dt_s)
        # Active-stroke safety cap: prevents acceleration bursts from becoming
        # 30–40 cm loops while still allowing fast handwriting.
        if self._prev_stroke_active:
            self.v = _clip_vec_norm(self.v, cfg.fusion_eskf.active_vel_cap_ms)

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
        if stroke_active_now and not self._prev_stroke_active:
            self._stroke_start_ts = ts
            # Hard velocity zero: prevents air-move momentum from hooking stroke start.
            self.v[:] = 0.0
            self._zupt_soft_update(sigma=0.05)
            self._stroke_start_uwb_snap(ts)
            self._dead_reckoner.reset(
                uwb_tip=self.last_uwb_tip,
                current_p=self.p,
            )
        elif (not stroke_active_now) and self._prev_stroke_active:
            self._stroke_start_ts = None
            ecfg_se = cfg.fusion_eskf
            # If the user paused at the end before lifting, preserve the locked
            # endpoint instead of letting the final pen-up frame move it.
            locked_endpoint = (
                np.asarray(self._tip_lock_anchor_visible, dtype=float).copy()
                if self._tip_lock_active and self._tip_lock_anchor_visible is not None
                else None
            )
            # Explicit hard zero: stroke_end_v_decay=0.0 achieves this, but stated
            # explicitly so a future config change cannot leak momentum across strokes.
            self.v[:] = 0.0
            self._dead_reckoner.close_stroke()
            self.P[2, 2] *= ecfg_se.stroke_end_p_vel_scale
            self.P[3, 3] *= ecfg_se.stroke_end_p_vel_scale
            self._apply_covariance_floor()
            # Phase 4: decay position bias on pen-up so the next stroke starts
            # with a fresh (near-zero) b_p, giving the stroke-start snap and
            # the new stroke's own EMA a clean slate.
            self.b_p *= ecfg_se.bias_decay
            if locked_endpoint is not None:
                self.p = locked_endpoint - self.b_p
        self._prev_stroke_active = stroke_active_now

        # Contact-static tip lock: pen-down stabilization, mid-stroke micro-pause
        # hold, and pen-up final stabilization all share this mode-aware clamp.
        self._apply_contact_static_lock(
            ts=ts,
            candidate=contact_static_candidate,
            stroke_active_now=stroke_active_now,
            stroke_active_prev=stroke_active_prev,
            stroke_state=stroke_state,
        )

        # IMU frame counters — mode classification from contact detector.
        # K averages are accumulated in _on_uwb (UWB-event-driven) so they
        # reflect the actual gain at update time, not a stale carry-over.
        if stroke_state == 'CONTACT_DRAWING':
            if self._in_fast_mode:
                self._frames_fast += 1
            else:
                self._frames_contact += 1
        elif stroke_state == 'AIR_MOVE':
            self._frames_air += 1
        else:
            self._frames_static += 1

        # Snapshot for UWB time interpolation (Step 4 consumes this).
        self._state_buf.append((ts, self.p.copy(), self.v.copy(), self.q.copy()))

        self._clamp_to_board()

        out = self._emit(
            ts       = ts,
            source   = 'IMU',
            state    = stroke_state,
            sid      = ev.get('stroke_id', 0),
            active   = ev.get('stroke_active', False),
        )
        # Forward physical contact flag so reconstruct.py can gate ink strictly.
        # contact_raw=True is the safe default for events that predate this field.
        out['contact_raw'] = bool(ev.get('contact', True))

        # Stroke-finalization IMU cleaner payload. This is intentionally attached
        # only to IMU-originated fused events because reconstruct.py ignores UWB
        # events for ink, but needs per-point IMU samples after pen-up.
        out['imu_cleaner'] = {
            'acc_board_hp_tip': (float(acc[0]), float(acc[1])),
            'dt_s': float(dt_s),
            'vel': (float(self.v[0]), float(self.v[1])),
            'rel_pos': (float(self.p[0]), float(self.p[1])),
            'uwb': (float(self.last_uwb_tip[0]), float(self.last_uwb_tip[1])),
            'contact': bool(ev.get('contact', True)),
            'is_static': bool(ev.get('is_static', False)),
        }
        return out

    # ────────────────────────────────────────────────────────────────────────
    # UWB path — Kalman correction (Steps 3+4)
    #   Step 5 adds NLOS-adaptive R.
    #   Step 6 adds lever-arm compensation on the measurement.
    # ────────────────────────────────────────────────────────────────────────
    def _on_uwb(self, ev: dict, ts: int) -> dict:
        self.last_uwb_ts = ts   # UWB does not propagate state; do not advance IMU clock

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
        self._last_uwb_fix_ts = ts_uwb   # record for stroke-start snap age check
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
        self._last_z_uwb_tip     = z_tip.copy()
        self._vel_pseudo_applied = False
        self._vel_pseudo_dx_pos[:] = 0.0
        self._vel_pseudo_dx_vel[:] = 0.0

        # Step 5: NLOS-adaptive R — consume trilateration residual.
        solve_error = float(ev.get('solve_error', 0.0))

        # UWB jump gate: reject when UWB-implied tip speed far exceeds physical pen
        # limits AND IMU velocity doesn't confirm the fast move.
        # Ceiling is mode-dependent: drawing allows faster legitimate strokes.
        ecfg_j   = cfg.fusion_eskf
        mode_j   = self._mode_params()
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
        stroke_active = self._prev_stroke_active  # UWB events do not carry contact state reliably
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

        if not self._have_imu_attitude:
            self._uwb_rejected += 1
            return self._emit(ts, 'POSITION', 'UWB_WAIT_IMU_INIT', 0, False)

        # First valid UWB lock: snap ESKF position to UWB before normal innovation gating.
        if self._uwb_accepted == 0:
            if solve_error <= cfg.uwb.trilat_max_residual:
                self.p[:] = z_tip
                self.v[:] = 0.0
                self._last_innovation_norm = 0.0
                self._last_K_pos = 1.0
                self._last_uwb_reset_ts = ts_uwb
                self._last_z_tip = z_tip.copy()
                self._last_z_tip_ts = ts_uwb
                self.last_uwb_tip = z_tip.copy()
                self._uwb_accepted += 1
                self._clamp_to_board()
                return self._emit(ts, 'POSITION', 'UWB_BOOTSTRAP', 0, False)

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
        self.last_uwb_tip = z_tip.copy()

        # K statistics — accumulated here (UWB-event-driven) so panel averages
        # reflect actual gain at update time, not stale IMU carry-overs.
        if self._prev_stroke_active:
            if self._in_fast_mode:
                self._k_fast_sum += self._last_K_pos
                self._k_fast_uwb += 1
            else:
                self._k_contact_sum += self._last_K_pos
                self._k_contact_uwb += 1
        else:
            self._k_air_sum += self._last_K_pos
            self._k_air_uwb += 1

        # Phase 4 — in-stroke position bias (EMA tracker).
        # During CONTACT_DRAWING or DRAWING_FAST, nudge b_p toward the UWB-vs-
        # visible-position residual.  This shifts the output (p + b_p) toward UWB
        # without touching p, so the relative IMU stroke shape is preserved.
        # During AIR / IDLE, b_p is frozen; p is corrected directly by the main filter.
        if self._prev_stroke_active:
            ecfg_bp = cfg.fusion_eskf
            visible = self.p + self.b_p
            y_bp    = z_tip - visible
            self.b_p += ecfg_bp.bias_uwb_alpha * y_bp
            cap = ecfg_bp.bias_max_m
            self.b_p[0] = float(np.clip(self.b_p[0], -cap, cap))
            self.b_p[1] = float(np.clip(self.b_p[1], -cap, cap))

        # Phase 3b: UWB-velocity pseudo-measurement (Rule 2 — UWB Sync).
        # Always push to buffer (solve_error stored so bad samples are detectable).
        self._uwb_vel_buf.append((ts_uwb, z_tip.copy(), solve_error))
        # Skip velocity anchor during DRAWING_FAST — anchoring v to UWB geometry
        # would cancel the IMU shape authority the mode is designed to grant.
        # Optional A/B gate: also skip during any active stroke to prevent UWB
        # velocity from fighting IMU curved motion (uwb_vel_stroke_gate=True).
        ecfg_gate = cfg.fusion_eskf
        if not self._in_fast_mode and not (ecfg_gate.uwb_vel_stroke_gate and self._prev_stroke_active):
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
                            # Deviation gate: skip if UWB-derived velocity disagrees too
                            # much with the current filter velocity.  Noisy UWB positions
                            # (5 cm over 50 ms → 1 m/s) with P_vel floor kept the Kalman
                            # gain ~0.9, injecting large spurious velocities that caused
                            # the position to integrate 40+ cm in the wrong direction.
                            vel_dev = float(np.linalg.norm(v_uwb - self.v))
                            if vel_dev <= ecfg_v.uwb_vel_dev_max:
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

        mode_p = self._mode_params()

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
            self._innov_reject_streak += 1
            # Recovery: if the streak is long enough AND UWB geometry is clean AND
            # the reading is high-confidence, the filter is the one that is lost —
            # not the UWB.  Snap back to the UWB position to break the deadlock.
            if (self._innov_reject_streak >= ecfg.innov_recovery_n
                    and solve_error <= cfg.uwb.trilat_max_residual
                    and not quality.get('low_confidence', False)):
                self._snap_to_uwb(z)
                self._innov_reject_streak = 0
                self._uwb_snap_count += 1
                return True
            self._uwb_rejected += 1
            return False
        self._innov_reject_streak = 0

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

        # Phase 4c: per-mode position-gain cap — with stroke-age drift guard.
        # During strokes (CONTACT_DRAWING / DRAWING_FAST) the base cap is kept
        # very small so UWB provides only a tiny stabilising nudge — enough to
        # prevent IMU runaway but not enough to sculpt the letter shape.
        # The stroke-age multiplier linearly relaxes the cap for long strokes
        # (> age_ramp_start_s) so geometric shapes get stronger UWB pull while
        # short handwriting letters keep full IMU shape authority.
        # During air/idle the mode cap is 1.0 (disabled) so UWB re-anchors freely.
        cap = min(mode_p.pos_gain_cap * self._stroke_age_cap_mult(self.last_uwb_ts or 0), 1.0)
        if cap < 1.0:
            K[0:2, :] = np.clip(K[0:2, :], -cap, cap)

        self._last_K_pos = float(K[0, 0])

        dx = K @ y
        if not np.all(np.isfinite(dx)):
            return False
        
        if self._prev_stroke_active:
            # UWB can gently pull ink back to the board reference, but every
            # accepted correction is limited so it cannot create a visible fold.
            dx[0:2] = _clip_vec_norm(dx[0:2], 0.018)
            dx[2:4] = _clip_vec_norm(dx[2:4], 0.025)
        else:
            dx[0:2] = _clip_vec_norm(dx[0:2], 0.10)
            dx[2:4] = _clip_vec_norm(dx[2:4], 0.50)
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

    def _snap_to_uwb(self, z: np.ndarray):
        """Soft re-localization after innovation-gate deadlock.

        Sets position to z, zeros velocity, and resets the position/velocity
        covariance blocks to their initial widths.  Bias estimate is preserved
        so convergence from prior good epochs is not discarded.
        Called only when innov_reject_streak >= innov_recovery_n with clean UWB.
        """
        ecfg = cfg.fusion_eskf
        self.p[:] = z
        self.v[:] = 0.0
        self.P[0, 0] = self.P[1, 1] = ecfg.p0_pos ** 2
        self.P[2, 2] = self.P[3, 3] = ecfg.p0_vel ** 2
        self.P[0:4, 4:6] = 0.0
        self.P[4:6, 0:4] = 0.0
        self._apply_covariance_floor()

    def _apply_covariance_floor(self):
        ecfg  = cfg.fusion_eskf
        mode_p = self._mode_params()
        pos_floor = mode_p.pos_floor ** 2
        pos_cap   = (mode_p.pos_floor * ecfg.pos_cap_mult) ** 2
        vel_floor = ecfg.vel_floor ** 2
        self._last_pos_floor_used = pos_floor
        self._last_pos_cap_used   = pos_cap

        for i in [0, 1]:
            self.P[i, i] = min(max(self.P[i, i], pos_floor), pos_cap)

        for i in [2, 3]:
            self.P[i, i] = max(self.P[i, i], vel_floor)

        # Hard numerical cap on the entire covariance matrix to prevent the
        # off-diagonal cross-terms (position-velocity coupling from F·P·Fᵀ)
        # from growing large enough to overflow the Joseph-form matmul.
        # 1e6 is far above any physically meaningful covariance for a 1.25×1.20 m board.
        np.clip(self.P, -1e6, 1e6, out=self.P)

    def _reset_covariance(self):
        ecfg = cfg.fusion_eskf
        diag = np.array([
            ecfg.p0_pos,  ecfg.p0_pos,
            ecfg.p0_vel,  ecfg.p0_vel,
            ecfg.p0_bias, ecfg.p0_bias,
        ]) ** 2
        self.P = np.diag(diag)
        # Snap position to last known UWB tip to prevent a finite-but-wrong p
        # from being emitted as a teleport line on the next frame.
        if self.last_uwb_tip is not None and np.all(np.isfinite(self.last_uwb_tip)):
            self.p[:] = self.last_uwb_tip
        else:
            self.p[:] = np.array([self._board_w * 0.5, self._board_h * 0.5])
        self.v[:] = 0.0
        self.b_a[:] = 0.0
        self.b_p[:] = 0.0
        self._cov_reset_suppress = True  # suppress the very next emit to avoid a teleport segment
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
        if not np.all(np.isfinite(self.b_p)):
            self.b_p[:] = 0.0

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

    def _stroke_start_uwb_snap(self, ts: int):
        """Soft UWB position pseudo-measurement on pen-down rising edge.

        Pulls the filter's position toward the last known UWB fix so each
        stroke starts at the correct board location.  Only fires when a fresh
        UWB fix is available (age < stroke_start_uwb_max_age_s).
        """
        ecfg = cfg.fusion_eskf
        # Use the last ACCEPTED lever-arm-corrected UWB tip target, not the raw
        # UWB tag position. Snapping to the raw tag can inject a tip/tag offset
        # at pen-down and starts the stroke with a hidden placement error.
        if self._last_z_tip is None or self._last_z_tip_ts is None:
            return
        age_s = (ts - self._last_z_tip_ts) / 1_000_000.0
        if age_s > ecfg.stroke_start_uwb_max_age_s:
            return
        target = self._last_z_tip.copy()

        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        sigma = ecfg.sigma_uwb * ecfg.stroke_start_sigma_scale
        R = (sigma ** 2) * np.eye(2)

        y = target - self.p
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ y
        if not np.all(np.isfinite(dx)):
            return
        dx[0:2] = np.clip(dx[0:2], -0.15, 0.15)
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
        self._stroke_start_snaps += 1

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
        if self._prev_stroke_active:
            # Safety: a UWB-derived velocity update must never teleport active ink.
            # With uwb_vel_stroke_gate=True this branch should normally not run,
            # but keeping it here prevents future config changes from reintroducing
            # the 8-11 cm in-stroke fold observed in the visualizer.
            dx[0:2] = 0.0
            dx[2:4] = np.clip(dx[2:4], -0.15, 0.15)
        else:
            dx[0:2] = np.clip(dx[0:2], -0.08, 0.08)
            dx[2:4] = np.clip(dx[2:4], -1.50, 1.50)
        dx[4:6] = np.clip(dx[4:6], -0.03, 0.03)
        self._vel_pseudo_applied = True
        self._vel_pseudo_dx_pos = dx[0:2].copy()
        self._vel_pseudo_dx_vel = dx[2:4].copy()
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

    def _advance_imu_clock(self, ts: int) -> float:
        """Returns dt (s) since the previous IMU event; handles gaps and init."""
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

    def _fresh_uwb_age_s(self, ts: int) -> float | None:
        """Age of the last accepted tip-corrected UWB fix, or None if absent."""
        if self._last_z_tip_ts is None:
            return None
        return max(0.0, (ts - self._last_z_tip_ts) / 1_000_000.0)

    def _contact_static_candidate(self, ev: dict, acc: np.ndarray, stroke_state: str) -> bool:
        """Return True when the physical tip should be treated as planted.

        CONTACT_STATIC from contact.py is the primary signal.  The threshold path
        is a compatibility fallback for logs where the substate is not populated
        or where a debounced transition is lagging by a frame or two.
        """
        ecfg = cfg.fusion_eskf
        if not ecfg.contact_static_lock_enabled:
            return False

        if stroke_state == 'CONTACT_STATIC':
            return True

        # Fallback: raw/contact + stillness + low predicted speed + low angular
        # motion.  This avoids false locking during actual drawing corners.
        contact_like = bool(ev.get('contact', False)) or stroke_state in ('CONTACT_DRAWING', 'CONTACT_STATIC')
        if not contact_like or not bool(ev.get('is_static', False)):
            return False

        speed = float(np.linalg.norm(self.v))
        amag = float(np.linalg.norm(acc))
        omega = abs(float(self._omega_in_plane_last))
        return (
            speed <= ecfg.contact_static_lock_speed_thresh_ms and
            amag <= ecfg.contact_static_lock_acc_thresh_ms2 and
            omega <= ecfg.contact_static_lock_omega_thresh_rads
        )

    def _make_contact_lock_anchor(self, ts: int, pen_down_phase: bool) -> tuple[np.ndarray, float]:
        """Visible-position anchor for stationary contact.

        Before the logical stroke opens, UWB is allowed to dominate because no ink
        should be committed yet.  During an active stroke, the anchor is mostly the
        current visible fused point so micro-pauses do not snap the drawn letter.
        """
        ecfg = cfg.fusion_eskf
        visible = self.p + self.b_p
        blend = 0.0
        age_s = self._fresh_uwb_age_s(ts)
        if age_s is not None and age_s <= ecfg.contact_static_lock_uwb_max_age_s:
            blend = (
                ecfg.contact_static_lock_pen_down_uwb_blend
                if pen_down_phase else
                ecfg.contact_static_lock_micro_pause_uwb_blend
            )
            blend = float(np.clip(blend, 0.0, 1.0))
        anchor = (1.0 - blend) * visible + blend * self.last_uwb_tip
        return anchor.astype(float), blend

    def _release_contact_static_lock(self):
        self._tip_lock_candidate = False
        self._tip_lock_active = False
        self._tip_lock_count = 0
        self._tip_lock_anchor_visible = None
        self._tip_lock_correction = 0.0
        self._tip_lock_uwb_blend = 0.0
        self._tip_lock_reason = 'OFF'

    def _apply_contact_static_lock(
        self,
        ts: int,
        candidate: bool,
        stroke_active_now: bool,
        stroke_active_prev: bool,
        stroke_state: str,
    ):
        """Pin the visible tip during contact-stationary periods.

        Covers three cases:
          1. pen-down stabilization before stroke_active opens,
          2. CONTACT_STATIC micro-pauses while stroke_active remains true,
          3. end-hold stabilization before the pen-up edge closes the stroke.
        """
        ecfg = cfg.fusion_eskf
        if not ecfg.contact_static_lock_enabled or not candidate:
            self._release_contact_static_lock()
            return

        self._tip_lock_candidate = True
        self._tip_lock_count += 1

        # If the logical stroke is not open yet, this is a pen-down/start hold.
        # It should be clamped immediately and strongly UWB-anchored.
        pen_down_phase = (not stroke_active_now) or (stroke_active_now and not stroke_active_prev)
        min_frames = 1 if pen_down_phase else max(1, int(ecfg.contact_static_lock_min_frames))

        if self._tip_lock_anchor_visible is None or pen_down_phase:
            self._tip_lock_anchor_visible, self._tip_lock_uwb_blend = self._make_contact_lock_anchor(
                ts, pen_down_phase=pen_down_phase
            )
        elif self._tip_lock_active:
            # During a held micro-pause, optionally let a fresh UWB fix very slowly
            # trim the anchor without creating a visible snap.
            anchor_new, blend = self._make_contact_lock_anchor(ts, pen_down_phase=False)
            self._tip_lock_anchor_visible = (1.0 - blend) * self._tip_lock_anchor_visible + blend * anchor_new
            self._tip_lock_uwb_blend = blend

        self._tip_lock_reason = 'PENDOWN' if pen_down_phase else ('MICROPAUSE' if stroke_active_now else stroke_state)

        # Always kill velocity while the candidate persists; only freeze position
        # after min_frames for active micro-pauses so actual sharp corners are not
        # over-clamped by a single borderline static frame.
        self.v *= float(np.clip(ecfg.contact_static_lock_vel_decay, 0.0, 1.0))
        if float(np.linalg.norm(self.v)) < ecfg.contact_static_lock_vel_zero_thresh_ms:
            self.v[:] = 0.0

        if self._tip_lock_count < min_frames:
            self._tip_lock_active = False
            self._tip_lock_correction = 0.0
            return

        self._tip_lock_active = True
        visible = self.p + self.b_p
        correction = (self._tip_lock_anchor_visible - visible) * float(
            np.clip(ecfg.contact_static_lock_pos_alpha, 0.0, 1.0)
        )
        self.p += correction
        self._tip_lock_correction = float(np.linalg.norm(correction))

        # Shrink covariance in the dimensions the physical board constraint just
        # observed: velocity should be near zero, position should not be wandering.
        self.P[2, 2] *= float(np.clip(ecfg.contact_static_lock_cov_vel_scale, 0.0, 1.0))
        self.P[3, 3] *= float(np.clip(ecfg.contact_static_lock_cov_vel_scale, 0.0, 1.0))
        self.P[0, 0] *= float(np.clip(ecfg.contact_static_lock_cov_pos_scale, 0.0, 1.0))
        self.P[1, 1] *= float(np.clip(ecfg.contact_static_lock_cov_pos_scale, 0.0, 1.0))
        self._apply_covariance_floor()

    def _apply_active_uwb_boundary_guard(self, ts: int, active: bool):
        """Bound active ink around the last accepted tip-corrected UWB point.

        The IMU is still allowed to make the local shape. This guard only fires
        when the visible state has expanded unrealistically far from the UWB
        neighbourhood, which is the failure pattern seen in abc/cat tests.
        """
        ecfg = cfg.fusion_eskf
        if self._tip_lock_active:
            # The stationary-contact clamp is a stronger physical constraint than
            # the UWB boundary guard: while the tip is planted, hold the lock anchor.
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return
        if not active or not ecfg.active_uwb_guard_enabled:
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return
        if self._last_z_tip_ts is None:
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return
        age_s = max(0.0, (ts - self._last_z_tip_ts) / 1_000_000.0)
        if age_s > ecfg.active_uwb_guard_max_age_s:
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return

        visible = self.p + self.b_p
        delta = visible - self.last_uwb_tip
        dist = float(np.linalg.norm(delta))
        radius = float(ecfg.active_uwb_guard_radius_m)
        if not np.isfinite(dist) or dist < 1e-9:
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return

        # Tethered visual-test logic: if the fused state is already drifting
        # away from the UWB neighbourhood, remove the outward velocity component
        # before it integrates into an oversized loop. Tangential velocity is
        # preserved, so corners/curves can still be IMU-shaped.
        unit = delta / dist
        soft_radius = max(0.020, 0.60 * radius)
        if dist > soft_radius:
            outward_v = float(np.dot(self.v, unit))
            if outward_v > 0.0:
                damp = float(np.clip(ecfg.active_uwb_outward_velocity_damping, 0.0, 1.0))
                self.v -= unit * outward_v * damp

        if dist <= radius:
            self._active_uwb_guard_fired = False
            self._active_uwb_guard_correction = 0.0
            return

        target_visible = self.last_uwb_tip + delta * (radius / dist)
        correction = (target_visible - visible) * float(ecfg.active_uwb_guard_alpha)
        self.p += correction
        # If the guard fired, residual velocity is probably the cause; damp it.
        self.v *= 0.55
        self._active_uwb_guard_fired = True
        self._active_uwb_guard_correction = float(np.linalg.norm(correction))

    def _emit(self, ts: int, source: str, state: str, sid: int, active: bool) -> dict:
        self._apply_active_uwb_boundary_guard(ts, active)
        # After a covariance reset the position was just re-anchored to UWB; suppress
        # stroke_active for this one frame so the visualizer breaks the polyline rather
        # than connecting the last good ink point to the snapped reset position.
        if self._cov_reset_suppress:
            self._cov_reset_suppress = False
            active = False
        # Visible output = p + b_p.  During drawing b_p provides a slow global
        # offset correction; during air b_p is frozen near zero (decayed on pen-up).
        p_out_x = float(np.clip(self.p[0] + self.b_p[0], 0.0, self._board_w))
        p_out_y = float(np.clip(self.p[1] + self.b_p[1], 0.0, self._board_h))
        return {
            'ts_hw': ts,
            'source': source,
            'fused_x': p_out_x,
            'fused_y': p_out_y,
            'uwb_x': float(self.last_uwb[0]),
            'uwb_y': float(self.last_uwb[1]),
            'state': state,
            'fusion_mode': self._mode_name(),
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
                'z_uwb_tip':         tuple(float(v) for v in self._last_z_uwb_tip),
                'last_uwb_tip':      tuple(float(v) for v in self.last_uwb_tip),
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
                # Innovation-deadlock recovery diagnostics
                'innov_reject_streak': self._innov_reject_streak,
                'uwb_snap_count':      self._uwb_snap_count,
                # Stroke-start soft snap diagnostics
                'stroke_start_snaps':  self._stroke_start_snaps,
                # Stroke-age drift guard diagnostics
                'stroke_age_s':  round((ts - self._stroke_start_ts) * 1e-6, 3) if self._stroke_start_ts else 0.0,
                'age_cap_mult':  round(self._stroke_age_cap_mult(ts), 3),
                # Phase 4 — position bias diagnostics
                'p_nominal': (float(self.p[0]), float(self.p[1])),
                'vel_pseudo_applied': bool(self._vel_pseudo_applied),
                'vel_pseudo_dx_pos': tuple(float(v) for v in self._vel_pseudo_dx_pos),
                'vel_pseudo_dx_vel': tuple(float(v) for v in self._vel_pseudo_dx_vel),
                'b_p':     (float(self.b_p[0]), float(self.b_p[1])),
                'b_p_mag': float(np.linalg.norm(self.b_p)),
                'active_uwb_guard_fired': bool(self._active_uwb_guard_fired),
                'active_uwb_guard_correction': float(self._active_uwb_guard_correction),
                # Stationary-contact tip lock diagnostics
                'tip_lock_active': bool(self._tip_lock_active),
                'tip_lock_candidate': bool(self._tip_lock_candidate),
                'tip_lock_count': int(self._tip_lock_count),
                'tip_lock_correction': float(self._tip_lock_correction),
                'tip_lock_uwb_blend': float(self._tip_lock_uwb_blend),
                'tip_lock_reason': self._tip_lock_reason,
                'tip_lock_anchor': tuple(float(v) for v in (self._tip_lock_anchor_visible if self._tip_lock_anchor_visible is not None else (self.p + self.b_p))),
                # Mode statistics — counters for per-regime tuning
                'frames_contact':    self._frames_contact,
                'frames_fast':       self._frames_fast,
                'frames_air':        self._frames_air,
                'frames_static':     self._frames_static,
                'avg_K_contact':     round(
                    self._k_contact_sum / self._k_contact_uwb
                    if self._k_contact_uwb > 0 else 0.0, 5),
                'avg_K_fast':        round(
                    self._k_fast_sum / self._k_fast_uwb
                    if self._k_fast_uwb > 0 else 0.0, 5),
                'avg_K_air':         round(
                    self._k_air_sum / self._k_air_uwb
                    if self._k_air_uwb > 0 else 0.0, 5),
            },
            'dead_reckoning': self._dead_reckoner.diagnostics(),
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
