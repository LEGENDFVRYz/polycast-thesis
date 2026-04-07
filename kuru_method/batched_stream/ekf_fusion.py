"""
ekf_fusion.py  —  PolyCast UWB + IMU Extended Kalman Filter
============================================================
Architecture: tightly coupled, 6-state EKF.

State vector
------------
    x = [px, py, vx, vy, bax, bay]

    px, py   — 2D position on whiteboard (m)
    vx, vy   — velocity (m/s)
    bax, bay — accelerometer bias in whiteboard frame (m/s²)
               Bias drifts slowly; the EKF estimates and removes it.

Predict step  (IMU-driven, 100 Hz, dt ≈ 10 ms)
-----------------------------------------------
    Input: a_wb = [ax_wb, ay_wb]  from IMUIntegrator

    ax_corr = ax_wb - bax
    ay_corr = ay_wb - bay

    px' = px + vx·dt + ½·ax_corr·dt²
    py' = py + vy·dt + ½·ay_corr·dt²
    vx' = vx + ax_corr·dt
    vy' = vy + ay_corr·dt
    bax' = bax   (random walk)
    bay' = bay   (random walk)

    Jacobian F is the 6×6 linearisation of the above.

    Process noise Q:
        — σ_acc² × standard constant-velocity Q (couples pos, vel)
        — σ_bias² × dt × I₂ (bias random walk)

Update step  (UWB-driven, ~20 Hz, per anchor, scalar measurement)
------------------------------------------------------------------
    Tightly coupled: each raw UWB range is one scalar measurement.
    Using the 2D-projected range (same fix as the old IRLS solver):

        d_2d_meas = sqrt( max(0, d_raw² − dz_i²) )
        h_i(x)   = sqrt( (px − ax_i)² + (py − ay_i)² )

        H_i = [ dx/h_i, dy/h_i, 0, 0, 0, 0 ]   (1×6 Jacobian)

    Anchors are processed one at a time (sequential EKF update).
    Each anchor is independently gated by a chi²(1) test.

Innovation gate  (per anchor)
------------------------------
    mahal² = innov² / S  where S = H·P·Hᵀ + R_eff

    Gate: chi²(1, 0.99) = 6.635
    R_eff = (σ_uwb / quality_weight)²   — noisier anchors get softer gate

Velocity damping on prolonged UWB outage
----------------------------------------
    If all anchors are rejected for more than MAX_CONSEC_OUTAGE consecutive
    packets, the velocity is multiplied by VELOCITY_DAMP each predict step.
    This stops the filter from coasting to an implausible position when
    the pen is completely occluded (e.g. hand blocks all LOS paths).

Tuning constants — all in each class's  # ── Tuning  block
--------------------------------------------------------------
See the module docstring above for what each constant controls.
The most impactful are:
    SIGMA_ACC      — set to max expected stroke acceleration
    SIGMA_UWB      — set to post-calibration per-anchor noise (run calibrate.py)
    GATE_CHI2      — increase to 10.83 if fast replants are lagging
    MAX_CONSEC_OUTAGE — increase to 10 if you write slowly with long pauses
"""

import numpy as np
from imu_integrator import IMUIntegrator, quat_to_rotmat
from button_detector import ButtonContactDetector

# ── Physical constants ─────────────────────────────────────────────────
MARKER_LENGTH = 0.21   # m — UWB tag z-position above board surface


# ─────────────────────────────────────────────────────────────────────
#  TIGHTLY COUPLED EKF
# ─────────────────────────────────────────────────────────────────────
class TightlyCoupledEKF:
    """
    Core 6-state EKF.  Called from EKFFusionEngine — do not call directly.
    """

    # ── Tuning ─────────────────────────────────────────────────────────

    # Process noise
    SIGMA_ACC  = 1.5     # m/s²      Lowered from 2.5: with stale UWB skipped, the gate
                         #           sees less noise.  Typical stroke accel is 1-2 m/s²;
                         #           the chi² gate handles occasional peaks.
    SIGMA_BIAS = 0.002   # m/s²/√s   Reduced from 0.005: prevents the bias estimate from
                         #           oscillating during fast circular motion, where equal
                         #           positive/negative accelerations confuse the estimator.

    # Measurement noise
    SIGMA_UWB  = 0.080   # m         Raised from 0.060: per-anchor raw std is 0.19-0.39m.
                         #           With fresh-only UWB updates each one carries more
                         #           weight, so R should be conservative to avoid yanking.

    # Chi-squared gate threshold (1 degree of freedom, scalar range measurement)
    GATE_CHI2  = 6.635

    # Prolonged UWB outage handling (all anchors rejected N consecutive packets)
    MAX_CONSEC_OUTAGE = 8
    VELOCITY_DAMP     = 0.88  # per-predict multiplier when UWB is blacked out

    # Lifted-pen velocity damping (applied every predict step when contact = 0)
    # Decays residual writing velocity so the position does not drift when the
    # pen is lifted after a fast stroke.
    # 0.90 per 10ms step:  after 200ms → 12% remaining,  after 500ms → ~0.5%
    LIFTED_VEL_DAMP   = 0.90

    # Slow-motion velocity decay (contact = 1, speed below threshold).
    # During deliberate slow strokes (e.g. careful rectangle corners), small
    # accumulated IMU noise can build up.  A gentle per-step decay keeps
    # velocity honest without affecting fast strokes at all.
    SLOW_SPEED_THRESH = 0.06   # m/s — below this, slow decay is applied
    SLOW_VEL_DECAY    = 0.92   # per 10ms step when slow and pen is down

    # Zero-velocity update (ZUPT) — pseudo-measurement [vx, vy] = 0
    # Triggered when bias-corrected acceleration stays below threshold for
    # ZUPT_WINDOW consecutive IMU samples.  Prevents drift at corners/pauses.
    ZUPT_ACCEL_THRESH = 0.15   # m/s² — below this the marker is "stationary"
    ZUPT_WINDOW       = 5      # consecutive low-accel samples to trigger
    ZUPT_R_VEL        = 0.001  # (m/s)² — measurement noise for zero-velocity

    # Speed above which bias estimation is frozen during a UWB update.
    # During fast circular motion the centripetal acceleration has equal + and -
    # phases.  Sampled at irregular UWB intervals these appear as spurious bias
    # signals and cause the bias estimate to oscillate.  Freezing above this
    # speed limits bias updates to slow/stationary periods where genuine DC
    # bias is reliably observable.  0.12 m/s ≈ slow deliberate stroke speed.
    BIAS_FREEZE_SPEED = 0.12   # m/s

    # ── init ───────────────────────────────────────────────────────────

    def __init__(self, anchors: np.ndarray, tag_z: float = MARKER_LENGTH):
        self.anchors  = np.asarray(anchors, dtype=float)  # (N, 3)
        self._tag_z   = tag_z
        self._dz_sq   = (tag_z - self.anchors[:, 2]) ** 2  # pre-computed

        self._x    = None   # state vector [px, py, vx, vy, bax, bay]
        self._P    = None   # 6×6 covariance

        # Diagnostics
        self._consec_outage   = 0
        self._zupt_count      = 0
        self._n_imu_steps     = 0
        self._n_uwb_accepted  = 0
        self._n_uwb_rejected  = 0

        # RTS smoother history (offline CSV mode only)
        self.record_history   = False
        self._history         = []    # list of dicts with x, P, F, Q per predict step

    # ── predict step (IMU) ─────────────────────────────────────────────

    def predict(self, a_wb: np.ndarray, dt: float, is_contact: bool = True):
        """
        IMU-driven predict step.

        Parameters
        ----------
        a_wb       : np.ndarray (2,)  whiteboard-frame linear acceleration (m/s²)
        dt         : float            time step (s), typically 0.010
        is_contact : bool             True = pen touching board, False = pen lifted.
                                      When False, residual writing velocity is damped
                                      every step so the position does not drift after
                                      a fast stroke ends.
        """
        if self._x is None:
            return

        ax_wb, ay_wb = float(a_wb[0]), float(a_wb[1])

        # Subtract current bias estimate
        ax = ax_wb - self._x[4]
        ay = ay_wb - self._x[5]

        # Velocity damping — two independent triggers:
        #   1. Prolonged UWB outage: no anchor accepted for N consecutive packets
        #   2. Pen lifted: damp residual writing velocity after each stroke ends
        # Note: SLOW_VEL_DECAY was removed. With UWB-first ordering, position is
        # already anchored at the start of each 50ms window. Decaying velocity
        # during slow writing creates artificial UWB innovations that corrupt the
        # bias estimate (see: feedback loop that drove bias_y to -0.09 incorrectly).
        if self._consec_outage > self.MAX_CONSEC_OUTAGE:
            self._x[2] *= self.VELOCITY_DAMP
            self._x[3] *= self.VELOCITY_DAMP
        elif not is_contact:
            self._x[2] *= self.LIFTED_VEL_DAMP
            self._x[3] *= self.LIFTED_VEL_DAMP

        # State transition (constant-acceleration model)
        dt2 = dt * dt
        self._x[0] += self._x[2] * dt + 0.5 * ax * dt2
        self._x[1] += self._x[3] * dt + 0.5 * ay * dt2
        self._x[2] += ax * dt
        self._x[3] += ay * dt
        # bax, bay unchanged

        # Jacobian F
        F = np.array([
            [1, 0, dt,  0, -0.5*dt2, 0        ],
            [0, 1,  0, dt,  0,       -0.5*dt2 ],
            [0, 0,  1,  0, -dt,       0       ],
            [0, 0,  0,  1,  0,       -dt      ],
            [0, 0,  0,  0,  1,        0       ],
            [0, 0,  0,  0,  0,        1       ],
        ], dtype=float)

        # Process noise Q
        sa2 = self.SIGMA_ACC  ** 2
        sb2 = (self.SIGMA_BIAS * np.sqrt(dt)) ** 2
        Q   = np.zeros((6, 6))
        Q[0, 0] = sa2 * dt2*dt2 / 4
        Q[1, 1] = sa2 * dt2*dt2 / 4
        Q[0, 2] = Q[2, 0] = sa2 * dt2 * dt / 2
        Q[1, 3] = Q[3, 1] = sa2 * dt2 * dt / 2
        Q[2, 2] = sa2 * dt2
        Q[3, 3] = sa2 * dt2
        Q[4, 4] = sb2
        Q[5, 5] = sb2

        # Covariance predict
        self._P = F @ self._P @ F.T + Q
        self._P = 0.5 * (self._P + self._P.T)   # enforce symmetry
        self._n_imu_steps += 1

        # ZUPT: clamp velocity when stationary (pen down, low acceleration)
        if is_contact:
            a_wb_mag = np.sqrt(ax * ax + ay * ay)
            self.check_zupt(a_wb_mag)
        else:
            self._zupt_count = 0

        # RTS history: save state after each predict step
        if self.record_history:
            self._history.append({
                'x': self._x.copy(),
                'P': self._P.copy(),
                'F': F.copy(),
                'Q': Q.copy(),
            })

    # ── zero-velocity update (ZUPT) ──────────────────────────────────

    def _zupt_update(self):
        """Apply zero-velocity pseudo-measurement [vx, vy] = [0, 0]."""
        if self._x is None:
            return
        H = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
        ], dtype=float)
        y = -self._x[2:4]                        # innovation = 0 - [vx, vy]
        R = np.eye(2) * self.ZUPT_R_VEL
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        I_KH = np.eye(6) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
        self._P = 0.5 * (self._P + self._P.T)

    def check_zupt(self, a_wb_mag: float):
        """Trigger ZUPT if acceleration stays below threshold for ZUPT_WINDOW samples."""
        if a_wb_mag < self.ZUPT_ACCEL_THRESH:
            self._zupt_count += 1
        else:
            self._zupt_count = 0
        if self._zupt_count >= self.ZUPT_WINDOW:
            self._zupt_update()

    # ── update step (UWB) ──────────────────────────────────────────────

    def update_uwb(self, raw_dists, quality_weights=None):
        """
        UWB range measurement update (tightly coupled, one anchor at a time).

        Parameters
        ----------
        raw_dists       : array-like (4,)  UWB distances after offset correction (m)
        quality_weights : array-like (4,)  per-anchor quality [0.1, 1.0] or None

        Returns
        -------
        accepted : list[int]  anchor indices that passed the chi² gate
        rejected : list[int]  anchor indices that were rejected
        """
        if self._x is None:
            return [], []

        n_anchors = len(self.anchors)
        if quality_weights is None:
            quality_weights = [1.0] * n_anchors

        accepted, rejected = [], []
        px, py = self._x[0], self._x[1]

        for i in range(min(n_anchors, len(raw_dists))):
            d_raw = float(raw_dists[i])
            qw    = float(quality_weights[i])

            if not np.isfinite(d_raw) or d_raw < 0.05:
                continue

            ax_i, ay_i = self.anchors[i, 0], self.anchors[i, 1]
            dz_sq_i    = float(self._dz_sq[i])

            # 2D-projected measurement (eliminates z ambiguity)
            d2d_sq = d_raw**2 - dz_sq_i
            if d2d_sq <= 0:
                continue
            d2d_meas = np.sqrt(d2d_sq)

            # Predicted 2D distance from current state
            dx = px - ax_i
            dy = py - ay_i
            r2d_pred = np.sqrt(dx*dx + dy*dy)
            if r2d_pred < 1e-6:
                continue

            # Innovation (scalar)
            innov = d2d_meas - r2d_pred

            # Measurement Jacobian H (1×6)
            H = np.array([[dx / r2d_pred, dy / r2d_pred,
                           0.0, 0.0, 0.0, 0.0]])

            # Effective measurement variance — inflate for low-quality anchors
            R_eff = (self.SIGMA_UWB / max(qw, 0.1)) ** 2

            # Innovation covariance (scalar)
            S = float((H @ self._P @ H.T).item()) + R_eff

            # Chi-squared gate (1 DOF)
            mahal_sq = (innov * innov) / S
            if mahal_sq > self.GATE_CHI2:
                rejected.append(i)
                self._n_uwb_rejected += 1
                continue

            # Kalman gain (6×1)
            K = (self._P @ H.T) / S

            # Speed-gated bias scaling (Lorentzian soft gate):
            # Full bias update at rest → attenuated during fast motion.
            # This prevents fast circular motion from corrupting the bias
            # estimate (circular accel has equal +/- phases that look like
            # oscillating bias when sampled at irregular UWB intervals).
            #
            # bias_scale at speed=0:              1.00 (full update)
            # bias_scale at speed=SCALE_SPEED:    0.50
            # bias_scale at speed=3×SCALE_SPEED:  0.10
            speed = float(np.linalg.norm(self._x[2:4]))
            bias_scale = 1.0 / (1.0 + (speed / self.BIAS_FREEZE_SPEED) ** 2)
            K[4] *= bias_scale
            K[5] *= bias_scale

            # State update
            self._x = self._x + K.ravel() * innov
            # Keep current px, py for subsequent anchors in this packet
            px, py = self._x[0], self._x[1]

            # Covariance update — Joseph form for numerical stability
            I_KH    = np.eye(6) - K @ H
            self._P = I_KH @ self._P @ I_KH.T + (R_eff * K) @ K.T
            self._P = 0.5 * (self._P + self._P.T)

            accepted.append(i)
            self._n_uwb_accepted += 1

        # Update outage counter
        if accepted:
            self._consec_outage = 0
        else:
            self._consec_outage += 1

        return accepted, rejected

    # ── initialisation ─────────────────────────────────────────────────

    def initialize(self, px: float, py: float):
        """
        Cold-start: set initial position from IRLS fix.
        Velocity and bias start at zero.
        Covariance reflects uncertainty in the IRLS fix (~50 cm).
        """
        self._x = np.array([px, py, 0.0, 0.0, 0.0, 0.0])
        self._P = np.diag([0.25, 0.25, 2.0, 2.0, 0.25, 0.25])
        print(f"[EKF] Initialised at ({px:.3f}, {py:.3f})")

    # ── properties ─────────────────────────────────────────────────────

    @property
    def position(self) -> np.ndarray:
        return self._x[:2].copy() if self._x is not None else None

    @property
    def velocity(self) -> np.ndarray:
        return self._x[2:4].copy() if self._x is not None else None

    @property
    def bias(self) -> np.ndarray:
        return self._x[4:6].copy() if self._x is not None else None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def consecutive_outage(self) -> int:
        return self._consec_outage

    @property
    def diagnostics(self) -> dict:
        return {
            'imu_steps'    : self._n_imu_steps,
            'uwb_accepted' : self._n_uwb_accepted,
            'uwb_rejected' : self._n_uwb_rejected,
            'consec_outage': self._consec_outage,
            'bias'         : self.bias,
            'velocity'     : self.velocity,
        }

    # ── RTS backward smoother (offline only) ──────────────────────────

    def rts_smooth(self):
        """
        Rauch-Tung-Striebel backward smoother over recorded history.

        Returns
        -------
        smoothed_positions : np.ndarray (N, 2)  — optimal position estimates
        """
        n = len(self._history)
        if n < 2:
            return np.array([h['x'][:2] for h in self._history])

        xs = [h['x'].copy() for h in self._history]
        Ps = [h['P'].copy() for h in self._history]

        for k in range(n - 2, -1, -1):
            F = self._history[k + 1]['F']
            Q = self._history[k + 1]['Q']
            P_pred = F @ Ps[k] @ F.T + Q
            try:
                G = Ps[k] @ F.T @ np.linalg.inv(P_pred)
            except np.linalg.LinAlgError:
                continue
            xs[k] = xs[k] + G @ (xs[k + 1] - F @ xs[k])
            Ps[k] = Ps[k] + G @ (Ps[k + 1] - P_pred) @ G.T

        return np.array([x[:2] for x in xs])


# ─────────────────────────────────────────────────────────────────────
#  FUSION ENGINE — top-level interface for main_ekf.py
# ─────────────────────────────────────────────────────────────────────
class EKFFusionEngine:
    """
    Wires TightlyCoupledEKF + IMUIntegrator + ButtonContactDetector.

    Per-packet call sequence (matches DataStream packet structure):
        1.  5 × IMU predict steps  (dt = packet_dt / 5 ≈ 10 ms)
        2.  1 × UWB measurement update  (all 4 anchors, gated)
        3.  Contact state updated from last IMU sample in the batch

    Cold-start
    ----------
    The first UWB packet is fed to the old IRLS solver (imported from
    fusion_engine.py) to get an initial position estimate.  The EKF is
    then initialised at that position.  All subsequent updates use the EKF.

    update() return values
    ----------------------
        pos        np.ndarray (2,) — EKF position [px, py] on whiteboard
        velocity   np.ndarray (2,) — EKF velocity [vx, vy]
        is_writing bool            — contact state from button detector
        accepted   list[int]       — anchor indices that passed gate
        rejected   list[int]       — anchor indices that were rejected
    """

    def __init__(self):
        self.anchors = np.array([
            [0.00, 0.00, 0.07],   # A0 — bottom-left
            [1.23, 0.00, 0.07],   # A1 — bottom-right
            [1.23, 1.23, 0.07],   # A2 — top-right
            [0.00, 1.23, 0.07],   # A3 — top-left
        ], dtype=float)

        self._ekf     = TightlyCoupledEKF(self.anchors, tag_z=MARKER_LENGTH)
        self._imu     = IMUIntegrator()
        self._contact = ButtonContactDetector()

        self._last_ts     = None
        self._last_uwb_ts = None
        self._is_writing  = False

        # Cold-start IRLS (reuses existing solver)
        from kuru_method.batched_stream.fusion_engine import IRLSTrilateration
        _bmin = [-0.30, -0.30, -0.50]
        _bmax = [ 1.55,  1.55,  1.00]
        self._irls = IRLSTrilateration(
            self.anchors, _bmin, _bmax, tag_z=MARKER_LENGTH)

        # Robust cold-start: collect several IRLS solutions and use their
        # median.  A single IRLS packet can land way outside the board (as
        # seen with init at (1.55, -0.30)), corrupting the first second of
        # tracking.  Valid solutions must lie within COLD_MARGIN of the board.
        self._cold_buf       = []    # accumulates (px, py) before init
        self._COLD_N         = 5     # packets to median-average
        self._COLD_MARGIN    = 0.25  # m — reject IRLS fixes outside board+margin

    # ── main entry point ───────────────────────────────────────────────

    def update(self, imu_batch, filtered_dists,
               quality_weights=None, packet_ts=None, uwb_ts=None):
        """
        Full fusion update for one DataStream packet.

        Parameters
        ----------
        imu_batch       : list of 5 dicts (keys: 'quat', 'acc', 'force', 'ts')
        filtered_dists  : tuple (d0, d1, d2, d3)  preprocessed UWB distances
        quality_weights : tuple (w0, w1, w2, w3)  per-anchor quality or None
        packet_ts       : batch_ts in microseconds for adaptive dt
        uwb_ts          : UWB measurement timestamp — used to detect stale data

        Returns
        -------
        pos        : np.ndarray (2,)
        velocity   : np.ndarray (2,)
        is_writing : bool
        accepted   : list[int]
        rejected   : list[int]
        """
        # Adaptive inter-packet dt
        packet_dt = 0.050
        if packet_ts is not None and self._last_ts is not None:
            dt_us = packet_ts - self._last_ts
            if 0 < dt_us < 2_000_000:
                packet_dt = dt_us / 1_000_000.0
        if packet_ts is not None:
            self._last_ts = packet_ts

        dt_imu = packet_dt / max(len(imu_batch), 1)  # ≈ 10 ms

        # ── Cold-start: buffer IRLS solutions until we have a stable fix ─
        if not self._ekf.initialized:
            pos_xyz, _ = self._irls.solve(filtered_dists)
            px, py = float(pos_xyz[0]), float(pos_xyz[1])

            # Accept only positions within board bounds + margin
            board_max = float(np.max(self.anchors[:, :2])) + self._COLD_MARGIN
            if (-self._COLD_MARGIN <= px <= board_max and
                    -self._COLD_MARGIN <= py <= board_max):
                self._cold_buf.append((px, py))

            if len(self._cold_buf) >= self._COLD_N:
                # Median across buffered solutions — robust to single bad fix
                init_x = float(np.median([p[0] for p in self._cold_buf]))
                init_y = float(np.median([p[1] for p in self._cold_buf]))
                self._ekf.initialize(init_x, init_y)
                self._cold_buf.clear()
            else:
                # Not ready yet — return latest raw IRLS as preliminary position
                # so the visualiser shows something while we accumulate.
                n = len(self._cold_buf)
                print(f"[EKF] Cold-start: buffering IRLS solution "
                      f"{n}/{self._COLD_N}  raw=({px:.3f},{py:.3f})")
                return (np.array([px, py]), np.zeros(2),
                        self._is_writing, [], [])

        # ── 1. UWB update FIRST (skip if stale) ─────────────────────────
        # Correct the position at the START of this time window before the
        # IMU predict steps advance it forward.  This means each IMU step
        # begins from a UWB-anchored position, so the trajectory within the
        # 50ms window (the stroke shape) comes purely from the IMU.
        #
        # Stale detection: UWB updates at ~10 Hz but packets arrive at ~20 Hz.
        # Every other packet carries the same uwb_ts and identical distances.
        # Feeding stale data into the EKF double-counts the correction,
        # creating a sawtooth that destroys shape.  Skip entirely when stale.
        uwb_is_fresh = True
        if uwb_ts is not None:
            if self._last_uwb_ts is not None and uwb_ts == self._last_uwb_ts:
                uwb_is_fresh = False
            self._last_uwb_ts = uwb_ts

        if uwb_is_fresh:
            accepted, rejected = self._ekf.update_uwb(filtered_dists, quality_weights)
        else:
            accepted, rejected = [], []

        # Warn on prolonged outage
        n_out = self._ekf.consecutive_outage
        if n_out == self._ekf.MAX_CONSEC_OUTAGE:
            print(f"[EKF] WARNING: {n_out} consecutive UWB outage packets "
                  f"({n_out*50} ms) — velocity damping active")

        # ── 2. IMU predict (5 steps, advance through this time window) ─
        prev_writing = self._is_writing
        for sample in imu_batch:
            a_wb = self._imu.get_wb_acceleration(sample['quat'], sample['acc'])

            # Evaluate contact state first — predict() uses it for damping
            state, _ = self._contact.process(sample['force'])
            self._is_writing = bool(state)

            # Zero velocity on pen touchdown (transition lifting → writing).
            # Prevents residual lifted velocity from corrupting stroke start.
            if self._is_writing and not prev_writing:
                self._ekf._x[2] = 0.0
                self._ekf._x[3] = 0.0

            prev_writing = self._is_writing
            self._ekf.predict(a_wb, dt_imu, is_contact=self._is_writing)

        pos = self._ekf.position
        vel = self._ekf.velocity
        return pos, vel, self._is_writing, accepted, rejected

    # ── properties ─────────────────────────────────────────────────────

    @property
    def ekf(self) -> TightlyCoupledEKF:
        return self._ekf

    @property
    def imu_integrator(self) -> IMUIntegrator:
        return self._imu

    @property
    def is_writing(self) -> bool:
        return self._is_writing

    @property
    def diagnostics(self) -> dict:
        d = self._ekf.diagnostics
        d['heading_locked'] = self._imu.heading_locked
        d['heading_vec']    = self._imu.heading_vec
        return d