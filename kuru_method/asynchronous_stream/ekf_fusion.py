"""
ekf_fusion.py  —  PolyCast UWB + IMU Extended Kalman Filter (Async Stream)
==========================================================================
Architecture: tightly coupled, 6-state EKF with event-driven processing.

State vector
------------
    x = [px, py, vx, vy, bax, bay]

    px, py   — 2D position on whiteboard (m)
    vx, vy   — velocity (m/s)
    bax, bay — accelerometer bias in whiteboard frame (m/s^2)

Event-driven processing (replaces batched update)
---------------------------------------------------
    IMU packet arrives (~100 Hz)  ->  EKF predict step
    UWB packet arrives (~10 Hz)   ->  EKF measurement update

    Each packet carries its own micros() timestamp for exact dt computation.
    No stale UWB detection needed — each UWB packet is inherently fresh.

Predict step  (IMU-driven, ~100 Hz, dt from timestamps)
-------------------------------------------------------
    Input: a_wb = [ax_wb, ay_wb]  from IMUIntegrator
    Same constant-acceleration model as batched version.

Update step  (UWB-driven, ~10 Hz, per anchor, scalar measurement)
------------------------------------------------------------------
    Tightly coupled: each raw UWB range is one scalar measurement.
    2D-projected range: d_2d_meas = sqrt( max(0, d_raw^2 - dz_i^2) )
    Sequential per-anchor update with chi^2 gating.
"""

import numpy as np
from config import ANCHORS, MARKER_LENGTH
from imu_integrator import IMUIntegrator, quat_to_rotmat
from force_detector import ForceContactDetector


# -------------------------------------------------------------------------
#  TIGHTLY COUPLED EKF
# -------------------------------------------------------------------------
class TightlyCoupledEKF:
    """
    Core 6-state EKF.  Called from AsyncEKFFusionEngine — do not call directly.
    """

    # -- Tuning -------------------------------------------------------------

    # Process noise
    SIGMA_ACC  = 0.3     # m/s^2  (was 1.5 — lowered to trust IMU more)
    SIGMA_BIAS = 0.0005  # m/s^2/sqrt(s)  (was 0.002 — slower bias drift)

    # Measurement noise
    SIGMA_UWB  = 0.15    # m  (was 0.08 — raised to match observed UWB noise)

    # Chi-squared gate threshold (1 degree of freedom, scalar range measurement)
    GATE_CHI2  = 3.841   # 95% CL  (was 6.635 / 99% — tighter outlier rejection)

    # Prolonged UWB outage handling (all anchors rejected N consecutive packets)
    MAX_CONSEC_OUTAGE = 8
    VELOCITY_DAMP     = 0.85  # per-predict multiplier when UWB is blacked out

    # Lifted-pen velocity damping (applied every predict step when contact = 0)
    LIFTED_VEL_DAMP   = 0.80

    # Slow-motion velocity decay
    SLOW_SPEED_THRESH = 0.04   # m/s
    SLOW_VEL_DECAY    = 0.85

    # Hard velocity cap (pen writing never exceeds this)
    MAX_WRITING_SPEED = 0.50   # m/s

    # Zero-velocity update (ZUPT)
    ZUPT_ACCEL_THRESH = 0.10   # m/s^2
    ZUPT_WINDOW       = 3      # consecutive low-accel samples to trigger
    ZUPT_R_VEL        = 0.0005 # (m/s)^2

    # Speed above which bias estimation is frozen during a UWB update
    BIAS_FREEZE_SPEED = 0.05   # m/s  (was 0.12 — freeze bias during any motion)

    # -- init ---------------------------------------------------------------

    def __init__(self, anchors: np.ndarray, tag_z: float = MARKER_LENGTH):
        self.anchors  = np.asarray(anchors, dtype=float)  # (N, 3)
        self._tag_z   = tag_z
        self._dz_sq   = (tag_z - self.anchors[:, 2]) ** 2  # pre-computed

        self._x    = None   # state vector [px, py, vx, vy, bax, bay]
        self._P    = None   # 6x6 covariance

        # Diagnostics
        self._consec_outage   = 0
        self._zupt_count      = 0
        self._n_imu_steps     = 0
        self._n_uwb_accepted  = 0
        self._n_uwb_rejected  = 0

        # Innovation-based adaptive measurement noise (per anchor)
        self._innov_var   = np.ones(len(anchors)) * 0.015
        self._innov_alpha = 0.05   # EMA smoothing for innovation variance

        # RTS smoother history (offline CSV mode only)
        self.record_history   = False
        self._history         = []

    # -- predict step (IMU) -------------------------------------------------

    def predict(self, a_wb: np.ndarray, dt: float, is_contact: bool = True):
        """
        IMU-driven predict step.

        Parameters
        ----------
        a_wb       : np.ndarray (2,)  whiteboard-frame linear acceleration (m/s^2)
        dt         : float            time step (s), typically 0.010
        is_contact : bool             True = pen touching board
        """
        if self._x is None:
            return

        ax_wb, ay_wb = float(a_wb[0]), float(a_wb[1])

        # Subtract current bias estimate
        ax = ax_wb - self._x[4]
        ay = ay_wb - self._x[5]

        # Velocity damping
        if self._consec_outage > self.MAX_CONSEC_OUTAGE:
            self._x[2] *= self.VELOCITY_DAMP
            self._x[3] *= self.VELOCITY_DAMP
        elif not is_contact:
            self._x[2] *= self.LIFTED_VEL_DAMP
            self._x[3] *= self.LIFTED_VEL_DAMP
        else:
            # Slow-motion decay (pen down, gentle creep suppression)
            spd = np.sqrt(self._x[2]**2 + self._x[3]**2)
            if spd < self.SLOW_SPEED_THRESH:
                self._x[2] *= self.SLOW_VEL_DECAY
                self._x[3] *= self.SLOW_VEL_DECAY

        # State transition (constant-acceleration model)
        dt2 = dt * dt
        self._x[0] += self._x[2] * dt + 0.5 * ax * dt2
        self._x[1] += self._x[3] * dt + 0.5 * ay * dt2
        self._x[2] += ax * dt
        self._x[3] += ay * dt

        # Hard velocity cap
        spd = np.sqrt(self._x[2]**2 + self._x[3]**2)
        if spd > self.MAX_WRITING_SPEED:
            scale = self.MAX_WRITING_SPEED / spd
            self._x[2] *= scale
            self._x[3] *= scale

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

    # -- zero-velocity update (ZUPT) ----------------------------------------

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

    # -- update step (UWB) --------------------------------------------------

    def update_uwb(self, raw_dists, quality_weights=None):
        """
        UWB range measurement update (tightly coupled, one anchor at a time).

        Parameters
        ----------
        raw_dists       : array-like (4,)  UWB distances after offset correction (m)
        quality_weights : array-like (4,)  per-anchor quality [0.1, 1.0] or None

        Returns
        -------
        accepted : list[int]  anchor indices that passed the chi^2 gate
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

            # Measurement Jacobian H (1x6)
            H = np.array([[dx / r2d_pred, dy / r2d_pred,
                           0.0, 0.0, 0.0, 0.0]])

            # Adaptive measurement variance: use max of configured and observed
            self._innov_var[i] = ((1 - self._innov_alpha) * self._innov_var[i]
                                  + self._innov_alpha * innov * innov)
            R_base = (self.SIGMA_UWB / max(qw, 0.1)) ** 2
            R_eff  = max(R_base, self._innov_var[i])

            # Innovation covariance (scalar)
            S = float((H @ self._P @ H.T).item()) + R_eff

            # Chi-squared gate (1 DOF)
            mahal_sq = (innov * innov) / S
            if mahal_sq > self.GATE_CHI2:
                rejected.append(i)
                self._n_uwb_rejected += 1
                continue

            # Kalman gain (6x1)
            K = (self._P @ H.T) / S

            # Hard bias freeze during motion
            speed = float(np.linalg.norm(self._x[2:4]))
            if speed > self.BIAS_FREEZE_SPEED:
                K[4] = 0.0
                K[5] = 0.0

            # State update
            self._x = self._x + K.ravel() * innov

            # Clamp bias to physically realistic bounds (BNO085)
            self._x[4] = np.clip(self._x[4], -0.10, 0.10)
            self._x[5] = np.clip(self._x[5], -0.10, 0.10)

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

    # -- initialisation -----------------------------------------------------

    def initialize(self, px: float, py: float):
        """
        Cold-start: set initial position from IRLS fix.
        Velocity and bias start at zero.
        """
        self._x = np.array([px, py, 0.0, 0.0, 0.0, 0.0])
        self._P = np.diag([0.25, 0.25, 2.0, 2.0, 0.01, 0.01])
        print(f"[EKF] Initialised at ({px:.3f}, {py:.3f})")

    # -- properties ---------------------------------------------------------

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

    # -- RTS backward smoother (offline only) --------------------------------

    def rts_smooth(self):
        """
        Rauch-Tung-Striebel backward smoother over recorded history.

        Returns
        -------
        smoothed_positions : np.ndarray (N, 2)
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


# -------------------------------------------------------------------------
#  ASYNC FUSION ENGINE — event-driven interface for main_ekf.py
# -------------------------------------------------------------------------
class AsyncEKFFusionEngine:
    """
    Event-driven fusion engine for the asynchronous sensor stream.

    Two separate entry points replace the batched update():
        process_imu(pkt)  — single IMU sample -> EKF predict
        process_uwb(dists, weights) — UWB ranges -> EKF update

    Cold-start
    ----------
    The first UWB packets are fed to the IRLS solver to get an initial
    position estimate.  The EKF is initialised at the median of the first
    _COLD_N valid IRLS solutions.

    process_imu() return values
    ----------------------------
        pos        np.ndarray (2,) or None — EKF position
        vel        np.ndarray (2,) or None — EKF velocity
        is_writing bool                    — contact state

    process_uwb() return values
    ----------------------------
        pos        np.ndarray (2,) or None — EKF position
        vel        np.ndarray (2,) or None — EKF velocity
        accepted   list[int]               — anchor indices that passed gate
        rejected   list[int]               — anchor indices rejected
    """

    def __init__(self):
        self.anchors = ANCHORS.copy()

        self._ekf     = TightlyCoupledEKF(self.anchors, tag_z=MARKER_LENGTH)
        self._imu     = IMUIntegrator()
        self._contact = ForceContactDetector()

        self._last_imu_ts = None
        self._is_writing  = False
        self._prev_writing = False

        # Cold-start IRLS (reuses existing solver)
        from fusion_engine import IRLSTrilateration
        _bmin = [-0.30, -0.30, -0.50]
        _bmax = [ 1.55,  1.55,  1.00]
        self._irls = IRLSTrilateration(
            self.anchors, _bmin, _bmax, tag_z=MARKER_LENGTH)

        # Robust cold-start: collect several IRLS solutions and use their
        # median.  A single IRLS packet can land way outside the board.
        self._cold_buf       = []
        self._COLD_N         = 5     # packets to median-average
        self._COLD_MARGIN    = 0.25  # m — reject IRLS fixes outside board+margin

    # -- IMU predict --------------------------------------------------------

    def process_imu(self, imu_pkt: dict):
        """
        Single IMU predict step.

        Parameters
        ----------
        imu_pkt : dict with keys 'quat', 'acc', 'force', 'ts'

        Returns
        -------
        pos        : np.ndarray (2,) or None
        vel        : np.ndarray (2,) or None
        is_writing : bool
        """
        if not self._ekf.initialized:
            # Still in cold-start — process contact but skip predict
            state, _ = self._contact.process(imu_pkt['force'])
            self._is_writing = bool(state)
            return None, None, self._is_writing

        # Compute dt from per-packet microsecond timestamps
        dt = 0.010   # default 10ms (100 Hz)
        ts = imu_pkt.get('ts')
        if ts is not None and self._last_imu_ts is not None:
            dt_us = ts - self._last_imu_ts
            if 0 < dt_us < 2_000_000:
                dt = dt_us / 1_000_000.0
        if ts is not None:
            self._last_imu_ts = ts

        # Body -> whiteboard acceleration
        a_wb = self._imu.get_wb_acceleration(imu_pkt['quat'], imu_pkt['acc'])

        # Contact detection
        state, _ = self._contact.process(imu_pkt['force'])
        self._is_writing = bool(state)

        # Zero velocity on pen touchdown (transition lifting -> writing)
        if self._is_writing and not self._prev_writing:
            self._ekf._x[2] = 0.0
            self._ekf._x[3] = 0.0

        self._prev_writing = self._is_writing

        # EKF predict
        self._ekf.predict(a_wb, dt, is_contact=self._is_writing)

        return self._ekf.position, self._ekf.velocity, self._is_writing

    # -- UWB update ---------------------------------------------------------

    def process_uwb(self, filtered_dists, quality_weights=None):
        """
        UWB measurement update.

        Parameters
        ----------
        filtered_dists  : tuple (d0, d1, d2, d3) — preprocessed (despiked) distances
        quality_weights : tuple (w0, w1, w2, w3) — per-anchor quality or None

        Returns
        -------
        pos      : np.ndarray (2,) or None
        vel      : np.ndarray (2,) or None
        accepted : list[int]
        rejected : list[int]
        """
        # -- Cold-start: buffer IRLS solutions until stable fix ------------
        if not self._ekf.initialized:
            pos_xyz, _ = self._irls.solve(filtered_dists)
            px, py = float(pos_xyz[0]), float(pos_xyz[1])

            # Accept only positions within board bounds + margin
            board_max = float(np.max(self.anchors[:, :2])) + self._COLD_MARGIN
            if (-self._COLD_MARGIN <= px <= board_max and
                    -self._COLD_MARGIN <= py <= board_max):
                self._cold_buf.append((px, py))

            if len(self._cold_buf) >= self._COLD_N:
                init_x = float(np.median([p[0] for p in self._cold_buf]))
                init_y = float(np.median([p[1] for p in self._cold_buf]))
                self._ekf.initialize(init_x, init_y)
                self._cold_buf.clear()
            else:
                n = len(self._cold_buf)
                print(f"[EKF] Cold-start: buffering IRLS solution "
                      f"{n}/{self._COLD_N}  raw=({px:.3f},{py:.3f})")
                return (np.array([px, py]), np.zeros(2), [], [])

        # -- Warm: EKF update -----------------------------------------------
        accepted, rejected = self._ekf.update_uwb(filtered_dists, quality_weights)

        # Warn on prolonged outage
        n_out = self._ekf.consecutive_outage
        if n_out == self._ekf.MAX_CONSEC_OUTAGE:
            print(f"[EKF] WARNING: {n_out} consecutive UWB outage packets "
                  f"— velocity damping active")

        return self._ekf.position, self._ekf.velocity, accepted, rejected

    # -- properties ---------------------------------------------------------

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
