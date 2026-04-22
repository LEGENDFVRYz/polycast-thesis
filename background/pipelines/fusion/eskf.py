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
        self._omega_in_plane_last = 0.0
        self._turn_flag_last = False

        # ── Contact-edge tracking (for stroke-start soft ZUPT + 3c sigma) ────
        self._prev_stroke_active = False

        # ── Phase 3a — sustained-static hard reset ───────────────────────────
        self._zupt_hard_count = 0

        # ── Phase 3b — UWB-velocity pseudo-measurement buffer ────────────────
        # Stores (ts_hw, pos) of the last N accepted pristine UWB corrections.
        self._uwb_vel_buf: deque[tuple[int, np.ndarray]] = deque(maxlen=5)

        # ── Book-keeping ────────────────────────────────────────────────────
        self.last_ts: int | None = None
        self.last_uwb = self.p.copy()
        self._last_innovation_norm = 0.0
        self._last_r_scale = 1.0
        self._last_K_pos = 0.0
        self._last_uwb_residual_rms = 0.0
        self._uwb_accepted = 0
        self._uwb_rejected = 0

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

    # ────────────────────────────────────────────────────────────────────────
    # IMU path — prediction + ZUPT update + turn-aware Q (Steps 2 + 7)
    # ────────────────────────────────────────────────────────────────────────
    def _on_imu(self, ev: dict, ts: int) -> dict:
        dt_s = self._advance_clock(ts)

        # Step 7: Update quaternion first so omega and Q use the current sample.
        q_raw = ev.get('quat')
        q_new = np.asarray(q_raw, dtype=float) if q_raw is not None else self.q.copy()
        self._update_omega_and_turn(q_new, dt_s)
        self.q = q_new

        # 1. Nominal state propagation (mid-point integration).
        acc = np.asarray(ev.get('acc_board', (0.0, 0.0)), dtype=float)
        a   = acc - self.b_a
        self.p += self.v * dt_s + 0.5 * a * dt_s * dt_s
        self.v += a * dt_s
        # Velocity drag — damps rotational-acc integration runaway between UWB corrections.
        # IMU sits 200mm from tip: circular motion generates ~1000 m/s3 apparent acc.
        # Drag prevents that from accumulating into multi-cm position error per UWB cycle.
        self.v *= max(0.0, 1.0 - cfg.fusion_eskf.velocity_drag_inv_s * dt_s)

        # 2. Error-state covariance propagation:   P ← F·P·Fᵀ + Q
        #    Q is now turn-aware — σ_a is inflated during sharp-stroke windows.
        F = self._build_F(dt_s)
        Q = self._build_Q(dt_s)
        self.P = F @ self.P @ F.T + Q

        # 3. ZUPT pseudo-measurement (v = 0) when the IMU preprocessor
        #    flags the pen as still.
        if ev.get('is_static', False):
            self._zupt_update()
            # Phase 3a: sustained static → hard-zero velocity after N samples.
            # Prevents drift from compounding when the pen sits still between strokes.
            # b_a is deliberately kept so bias convergence is not disrupted.
            self._zupt_hard_count += 1
            if self._zupt_hard_count >= cfg.fusion_eskf.zupt_hard_reset_n:
                self.v[:] = 0.0
        else:
            self._zupt_hard_count = 0

        # 4. Contact rising-edge soft ZUPT: tip just pressed on board →
        #    tip velocity should be near zero (pen end may still wiggle, but
        #    the tip is constrained). Looser sigma than normal ZUPT.
        stroke_active_now = bool(ev.get('stroke_active', False))
        if stroke_active_now and not self._prev_stroke_active:
            self._zupt_soft_update(sigma=0.05)
        self._prev_stroke_active = stroke_active_now

        # Snapshot for UWB time interpolation (Step 4 consumes this).
        self._state_buf.append((ts, self.p.copy(), self.v.copy(), self.q.copy()))

        self._clamp_to_board()

        return self._emit(
            ts       = ts,
            source   = 'IMU',
            state    = ev.get('stroke_state', 'UNKNOWN'),
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

        # Use pos_raw — position.py's EMA would double-smooth what ESKF already manages.
        # Fall back to pos_clean if raw is absent (shouldn't happen in normal flow).
        uwb = ev.get('pos_raw') or ev.get('pos_clean')
        if uwb is None:
            return self._emit(ts, 'POSITION', 'UWB_DROPPED', 0, False)

        z = np.asarray(uwb, dtype=float)
        self.last_uwb = z.copy()

        # Time interpolation: innovation against state at UWB timestamp (Step 4).
        ts_uwb = ev.get('ts_hw', ts)
        interp = self._interpolate_at(ts_uwb)
        p_ref  = interp[0] if interp is not None else None

        # Lever-arm: per bg_rules.md the pen is held perpendicular to the board
        # (constant offset ≈ 0) — skip rotation-based correction for this iteration.
        z_tip = z

        # Step 5: NLOS-adaptive R — consume trilateration residual.
        solve_error = float(ev.get('solve_error', 0.0))

        accepted = self._uwb_update(z_tip, p_ref, solve_error)
        if not accepted:
            self._uwb_rejected += 1
            return self._emit(ts, 'POSITION', 'UWB_NLOS_REJECT', 0, False)
        self._uwb_accepted += 1

        # Phase 3b: UWB-velocity pseudo-measurement on pristine trilateration.
        # When solve_error < sigma_trilat the geometry is well-conditioned; two
        # consecutive clean positions give a reliable velocity estimate that
        # prevents IMU velocity from drifting between UWB fixes.
        self._uwb_vel_buf.append((ts_uwb, z_tip.copy()))
        if solve_error < cfg.fusion_eskf.sigma_trilat and len(self._uwb_vel_buf) >= 2:
            t1, p1 = self._uwb_vel_buf[-2]
            t2, p2 = self._uwb_vel_buf[-1]
            dt_vel = (t2 - t1) / 1_000_000.0
            if 0.01 < dt_vel < 0.5:                       # guard against stale/duplicate ts
                v_uwb = (p2 - p1) / dt_vel
                if float(np.linalg.norm(v_uwb)) < 2.0:   # clamp to physical pen-speed limit
                    self._velocity_pseudo_update(v_uwb, cfg.fusion_eskf.sigma_uwb_vel)

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
                    solve_error: float = 0.0) -> bool:
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

        sigma = ecfg.sigma_uwb
        # Phase 3c: pen physically on the board → trust UWB more during active strokes.
        if self._prev_stroke_active:
            sigma *= ecfg.contact_sigma_scale
        R = (sigma * sigma) * r_scale * np.eye(2)

        # Innovation  y = z − p_ref  (time-aligned)
        p_nom = p_ref if p_ref is not None else self.p
        y = z - p_nom
        self._last_innovation_norm = float(np.linalg.norm(y))
        self._last_uwb_residual_rms = solve_error

        S = H @ self.P @ H.T + R                  # 2×2
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×2
        self._last_K_pos = float(K[0, 0])

        dx = K @ y
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        I = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
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
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        # Joseph-form covariance update (numerically stable).
        I  = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

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

        dx       = K @ y
        self.p   += dx[0:2]
        self.v   += dx[2:4]
        self.b_a += dx[4:6]

        I   = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────
    def _update_omega_and_turn(self, q_new: np.ndarray, dt_s: float):
        """Derive in-plane angular velocity from Δquat; update turn flag.

        ω ≈ 2·(q_k ⊗ q_{k-1}⁻¹).xyz / dt  (small-angle approximation).
        If ω_in_plane exceeds threshold, inflate Q for turn_n_post frames.
        """
        ecfg = cfg.fusion_eskf
        if self._prev_quat is not None and dt_s > 1e-6:
            q_delta = _quat_multiply(q_new, _quat_conjugate(self._prev_quat))
            if q_delta[3] < 0:           # choose shorter arc
                q_delta = -q_delta
            omega_body  = 2.0 * q_delta[0:3] / dt_s
            omega_world = _q_to_rotation(q_new) @ omega_body

            axis_map = {'x': 0, 'y': 1, 'z': 2}
            ax0 = axis_map[cfg.imu.board_axes[0]]
            ax1 = axis_map[cfg.imu.board_axes[1]]
            ω_ip = math.sqrt(omega_world[ax0]**2 + omega_world[ax1]**2)

            if ω_ip > ecfg.turn_omega_threshold:
                self._turn_cooldown = ecfg.turn_n_post
            self._omega_in_plane_last = ω_ip
        else:
            self._omega_in_plane_last = 0.0

        if self._turn_cooldown > 0:
            self._turn_flag_last = True
            self._turn_cooldown -= 1
        else:
            self._turn_flag_last = False

        self._prev_quat = q_new.copy()

    def _uwb_lever_arm_board(self, q: np.ndarray) -> np.ndarray:
        """Return the 2D board-plane offset from tip to UWB tag.

        r_UWB_body (along pen z-axis) is rotated to world frame via q, then
        projected onto the board plane defined by cfg.imu.board_axes.
        For a perpendicular pen the offset is ~0; for a tilted pen it is real.
        """
        r_body = np.asarray(cfg.marker.r_uwb_body_m, dtype=float)
        R      = _q_to_rotation(q)
        r_world = R @ r_body

        axis_map = {'x': 0, 'y': 1, 'z': 2}
        ax0 = axis_map[cfg.imu.board_axes[0]]
        ax1 = axis_map[cfg.imu.board_axes[1]]
        return np.array([r_world[ax0], r_world[ax1]], dtype=float)

    def _advance_clock(self, ts: int) -> float:
        """Returns dt (s) since the previous event; handles gaps and init."""
        if self.last_ts is None:
            self.last_ts = ts
            return 1.0 / cfg.imu.sample_rate_hz

        dt_s = (ts - self.last_ts) / 1_000_000.0
        self.last_ts = ts
        if dt_s <= 0.0 or dt_s > 0.5:
            # Monotonicity / long-gap guard: treat as nominal step.
            return 1.0 / cfg.imu.sample_rate_hz
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
                'b_a':               (float(self.b_a[0]), float(self.b_a[1])),
                'uwb_accepted':      self._uwb_accepted,
                'uwb_rejected':      self._uwb_rejected,
            },
        }


# ==============================================================================
# LIVE HARDWARE SELF-TEST
#   SerialStreamer → Normalizer → TimeAlign → (IMU | UWB) → ESKF
#   Prints a dashboard; Ctrl+C stops and prints a terse summary.
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

    print("=" * 60)
    print(f"  [TEST] MODULE 7b LIVE: ESKF (skeleton) on {SERIAL_PORT}")
    print("  Draw strokes. Ctrl+C to stop.")
    print("=" * 60)

    last_print_time = 0.0
    latest      = None
    latest_imu  = None   # most-recent pre-fusion IMU event (for acc_board diagnostic)
    imu_count   = 0
    uwb_count   = 0
    event_log   = []     # accumulates every fused event for CSV export

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
                                # Decorate with is_static before logging so CSV
                                # has the IMU-side flag alongside fusion state.
                                fused['_is_static'] = bool(s.get('is_static', False))
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
                                        latest = fused
                                        fused['_is_static'] = False
                                        event_log.append(fused)

            now = time.time()
            if latest and (now - last_print_time) >= DISPLAY_RATE:
                os.system('cls' if os.name == 'nt' else 'clear')
                e = latest['eskf']

                # acc_board magnitude — sanity-check Path-A amplitude (expect ~1–3 m/s² when writing)
                if latest_imu is not None:
                    ab = latest_imu.get('acc_board', (0.0, 0.0))
                    acc_board_mag = math.sqrt(ab[0]**2 + ab[1]**2)
                else:
                    acc_board_mag = 0.0

                print(f"========= LIVE ESKF ({DISPLAY_RATE}s) =========")
                print(f"  State      : {latest['state']}")
                print(f"  Stroke ID  : {latest['stroke_id']} (Active: {latest['stroke_active']})")
                print(f"  Source     : {latest['source']}")
                print("-" * 50)
                print(f"  Fused Pos  : X: {latest['fused_x']:6.3f} m | Y: {latest['fused_y']:6.3f} m")
                print(f"  UWB Anchor : X: {latest['uwb_x']:6.3f} m | Y: {latest['uwb_y']:6.3f} m")
                print(f"  |acc_board|: {acc_board_mag:.4f} m/s²  (Path-A; expect 1–3 when writing)")
                print(f"  K_pos_diag : {e['K_pos_diag']:.4f}")
                print(f"  uwb_resid  : {e['uwb_residual_rms']:.4f} m")
                print(f"  P_pos_trace: {e['P_pos_trace']:.4f} m")
                print(f"  Bias b_a   : ({e['b_a'][0]:+.4f}, {e['b_a'][1]:+.4f}) m/s²")
                print(f"  Last |y|   : {e['innovation_norm']:.4f} m  (UWB innovation)")
                print(f"  R scale    : {e['r_scale']:.2f}  (1.0=clean, high=NLOS)")
                turn_str = "TURN" if e['turn_flag'] else "    "
                print(f"  ω in-plane : {e['omega_in_plane']:6.3f} rad/s  [{turn_str}]")
                print(f"  IMU / UWB  : {imu_count} / {uwb_count}  (accepted={e['uwb_accepted']} rejected={e['uwb_rejected']})")
                print("=" * 52)
                last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Halting ESKF.")
        streamer.close()

        # ── Terse session summary ────────────────────────────────────────────
        print("-" * 60)
        print(f"  IMU events processed : {imu_count}")
        print(f"  UWB events processed : {uwb_count}")
        if latest:
            e = latest['eskf']
            print(f"  Final fused position : ({latest['fused_x']:.3f}, {latest['fused_y']:.3f}) m")
            print(f"  Final P_pos_trace    : {e['P_pos_trace']:.4f} m")
            print(f"  UWB accepted/rejected: {e['uwb_accepted']} / {e['uwb_rejected']}")
        print("=" * 60)

        if not event_log:
            print("No events logged. Exiting.")
            exit()

        # ── CSV export ───────────────────────────────────────────────────────
        # One row per fused event (both IMU and UWB source).
        # Columns are stable across phases so CSVs can be overlaid for comparison.
        csv_filename = "eskf_session.csv"
        _CSV_COLS = [
            'ts_hw', 'source',
            'fused_x', 'fused_y',
            'uwb_x', 'uwb_y',
            'stroke_id', 'stroke_active', 'is_static', 'state',
            'P_pos_trace', 'innovation_norm', 'r_scale',
            'K_pos_diag', 'b_a_x', 'b_a_y', 'b_a_norm',
            'omega_in_plane', 'turn_flag',
            'uwb_residual_rms', 'uwb_accepted', 'uwb_rejected',
        ]
        print(f"[EXPORT] Writing {len(event_log)} rows to {csv_filename} ...")
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(_CSV_COLS)
            for ev in event_log:
                e = ev.get('eskf', {})
                ba = e.get('b_a', (0.0, 0.0))
                writer.writerow([
                    ev.get('ts_hw'),
                    ev.get('source'),
                    round(ev.get('fused_x', 0.0), 6),
                    round(ev.get('fused_y', 0.0), 6),
                    round(ev.get('uwb_x', 0.0), 6),
                    round(ev.get('uwb_y', 0.0), 6),
                    ev.get('stroke_id', 0),
                    int(ev.get('stroke_active', False)),
                    int(ev.get('_is_static', False)),
                    ev.get('state', ''),
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
                ])
        print(f"[EXPORT] Saved to {csv_filename}")
        print()
        print("  Phase comparison tip: rename each run's CSV before the next")
        print("  session (e.g. eskf_phase2.csv, eskf_phase3.csv) and diff the")
        print("  P_pos_trace, innovation_norm, and uwb_accepted columns.")
        print("=" * 60)
