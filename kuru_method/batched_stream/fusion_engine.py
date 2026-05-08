"""
fusion_engine.py  —  PolyCast UWB Position Solver
==================================================
Two-layer pipeline:

    Layer 1: IRLSTrilateration
        2D IRLS with Huber M-estimator.
        Solves tag XY using projected horizontal distances.

    Layer 2: PositionKalmanFilter
        Constant-velocity KF on [px, py, vx, vy] with a
        chi-squared innovation gate to block NLOS spikes.

WHERE TO FIND THE TUNING KNOBS
================================
All tuning constants are class-level attributes, clearly marked
with a  # Tuning  comment block.  Nothing else needs to change.

IRLSTrilateration
-----------------
    _HUBER_DELTA   Residual (m) above which an anchor is down-weighted.
                   Default 0.08.  Raise if good anchors near board edges
                   are being suppressed; lower if NLOS still pulls the fix.

    _N_ITERATIONS  IRLS passes.  Default 5.  3 is usually enough;
                   7 for extra precision at ~2 ms extra CPU per packet.

PositionKalmanFilter
--------------------
    _SIGMA_ACC     Max marker acceleration in m/s².  Default 2.0.
                   Must be >= centripetal accel of tightest curve:
                       a = v^2 / r.
                   At v=0.4 m/s, r=0.10 m -> a=1.6 m/s^2.  2.0 is fine.
                   Higher = trusts measurements more (faster, noisier).

    _SIGMA_MEAS    Base measurement noise in m.  Default 0.030.
                   Set to your expected post-calibration position error.
                   Before calibrate.py: use 0.050.
                   After calibrate.py:  use 0.025-0.035.

    _GATE_CHI2     Chi-squared gate threshold.
                   chi2(2, 0.99)  = 9.21   ->  99%  of valid meas pass
                   chi2(2, 0.999) = 13.82  ->  99.9% pass (more permissive)
                   Default 9.21.
                   INCREASE to 13.82 if fast pen replants are lagging.
                   DECREASE to 6.0 if spikes still get through post-calib.

    _MAX_CONSEC_REJECTS
                   Gate rejections in a row before velocity damping starts.
                   Default 6 (= 300 ms at 50 ms/packet).
                   This is what stops the long coast line you saw in the
                   screenshot: after 300 ms of total NLOS the velocity
                   is progressively killed so the filter stops drifting.
                   Raise to 10 if you often pause >300 ms on the board.

    _VELOCITY_DAMP Per-packet velocity multiplier during prolonged outage.
                   Default 0.75 -> velocity halves every ~2 rejected packets.
                   Raise toward 0.95 for slower decay (more coasting);
                   lower toward 0.50 for a faster hard stop.
"""

import numpy as np
from scipy.optimize import least_squares


# ── Physical constant ──────────────────────────────────────────────────
MARKER_LENGTH = 0.21   # metres — distance from tip to UWB tag antenna


# ─────────────────────────────────────────────────────────────────────
#  LAYER 1 — IRLS TRILATERATION
# ─────────────────────────────────────────────────────────────────────
class IRLSTrilateration:
    """
    Solves for tag XY from anchor distances using IRLS + Huber weights.

    2D-projection mode (tag_z supplied — strongly recommended):
        d_2d[i] = sqrt( max(0, d_3d[i]^2 - (tag_z - anchor_z[i])^2) )

    This removes the unobservable Z degree of freedom when all anchors
    are coplanar, which was the original 20-30 cm error source.
    """

    # ── Tuning ────────────────────────────────────────────────────────
    _HUBER_DELTA  = 0.08   # m  residual above which anchor is penalised
    _N_ITERATIONS = 5      #    IRLS refinement passes
    _MIN_ANCHORS  = 2      #    minimum valid anchors to attempt a solve
    _VALID_THR    = 0.05   # m  distances below this are treated as missing

    def __init__(self, anchors, bounds_min, bounds_max, tag_z=None):
        self.anchors    = np.asarray(anchors, dtype=float)
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max
        self._tag_z     = tag_z

        if tag_z is not None:
            self._anch_2d = self.anchors[:, :2]
            self._dz_sq   = (tag_z - self.anchors[:, 2]) ** 2
            init_xy       = np.mean(self._anch_2d, axis=0)
            self.last_pos = np.array([init_xy[0], init_xy[1], tag_z])
        else:
            self.last_pos = np.mean(self.anchors, axis=0).copy()

    def solve(self, dists, quality_weights=None):
        if self._tag_z is not None:
            return self._solve_2d(dists, quality_weights)
        return self._solve_3d(dists, quality_weights)

    def _solve_2d(self, dists, quality_weights=None):
        n         = min(len(self._anch_2d), len(dists))
        dists_arr = np.asarray(dists[:n], dtype=float)
        d2d       = np.sqrt(np.maximum(0.0, dists_arr**2 - self._dz_sq[:n]))

        valid_idx = [i for i in range(n)
                     if dists[i] > self._VALID_THR and d2d[i] > 0.01]
        if len(valid_idx) < self._MIN_ANCHORS:
            return self.last_pos.copy(), np.zeros(len(dists))

        v_anch = self._anch_2d[valid_idx]
        v_dist = d2d[valid_idx]
        v_qw   = (np.array([quality_weights[i] for i in valid_idx], dtype=float)
                  if quality_weights is not None else np.ones(len(valid_idx)))

        irls_w = np.ones(len(valid_idx))
        guess  = self.last_pos[:2].copy()

        for _ in range(self._N_ITERATIONS):
            res = least_squares(
                self._weighted_residuals, guess,
                bounds=(self.bounds_min[:2], self.bounds_max[:2]),
                args=(v_anch, v_dist, irls_w * v_qw),
                method='trf', loss='linear',
                max_nfev=40, ftol=1e-4, xtol=1e-4,
            )
            guess  = res.x
            d_calc = np.linalg.norm(v_anch - guess, axis=1)
            irls_w = self._huber_weights(d_calc - v_dist)

        self.last_pos[0] = guess[0]
        self.last_pos[1] = guess[1]

        full_res = np.zeros(len(dists))
        d_calc   = np.linalg.norm(v_anch - guess, axis=1)
        for j, i in enumerate(valid_idx):
            full_res[i] = d_calc[j] - d2d[i]

        return np.array([guess[0], guess[1], self._tag_z]), full_res

    def _solve_3d(self, dists, quality_weights=None):
        n         = min(len(self.anchors), len(dists))
        valid_idx = [i for i in range(n) if dists[i] > self._VALID_THR]
        if len(valid_idx) < self._MIN_ANCHORS:
            return self.last_pos.copy(), np.zeros(len(dists))

        v_anch = self.anchors[valid_idx]
        v_dist = np.array([dists[i] for i in valid_idx], dtype=float)
        v_qw   = (np.array([quality_weights[i] for i in valid_idx], dtype=float)
                  if quality_weights is not None else np.ones(len(valid_idx)))

        irls_w = np.ones(len(valid_idx))
        guess  = self.last_pos.copy()

        for _ in range(self._N_ITERATIONS):
            result = least_squares(
                self._weighted_residuals, guess,
                bounds=(self.bounds_min, self.bounds_max),
                args=(v_anch, v_dist, irls_w * v_qw),
                method='trf', loss='linear',
                max_nfev=40, ftol=1e-4, xtol=1e-4,
            )
            guess  = result.x
            d_calc = np.linalg.norm(v_anch - guess, axis=1)
            irls_w = self._huber_weights(d_calc - v_dist)

        self.last_pos = guess
        full_res = np.zeros(len(dists))
        d_calc   = np.linalg.norm(v_anch - guess, axis=1)
        for j, i in enumerate(valid_idx):
            full_res[i] = d_calc[j] - dists[i]
        return guess, full_res

    @staticmethod
    def _weighted_residuals(pos, anchors, dists, weights):
        return weights * (np.linalg.norm(anchors - pos, axis=1) - dists)

    def _huber_weights(self, residuals):
        abs_r = np.abs(residuals)
        return np.where(abs_r <= self._HUBER_DELTA,
                        1.0, self._HUBER_DELTA / (abs_r + 1e-9))


# ─────────────────────────────────────────────────────────────────────
#  LAYER 2 — POSITION KALMAN FILTER
# ─────────────────────────────────────────────────────────────────────
class PositionKalmanFilter:
    """
    Linear KF on [px, py, vx, vy] with chi-squared innovation gate.

    On each packet:
        1. Predict forward using constant-velocity model.
        2. Compute Mahalanobis distance of the incoming measurement.
        3. If d^2 > _GATE_CHI2 -> reject, predict-only, keep velocity clean.
        4. Else -> standard Kalman correction.
        5. After _MAX_CONSEC_REJECTS rejections -> damp velocity by
           _VELOCITY_DAMP each packet to stop runaway coasting.

    See the module docstring for the full tuning guide.
    """

    # ── Tuning ────────────────────────────────────────────────────────
    _SIGMA_ACC          = 2.0    # m/s^2  process noise
    _SIGMA_MEAS         = 0.030  # m      base measurement noise
    _GATE_CHI2          = 9.21   # chi2(2, 0.99) gate threshold
    _MAX_CONSEC_REJECTS = 6      # packets before velocity damping starts
    _VELOCITY_DAMP      = 0.75   # per-packet velocity multiplier during outage

    def __init__(self, nominal_dt: float = 0.05):
        self._dt             = nominal_dt
        self._x              = None
        self._P              = None
        self._H              = np.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0]], dtype=float)
        self._R              = (self._SIGMA_MEAS ** 2) * np.eye(2)
        self._consec_rejects = 0

    def update(self, meas_xy, dt: float = None):
        """
        Parameters
        ----------
        meas_xy     : [x, y] from trilateration
        dt          : elapsed time since last call (s)

        Returns
        -------
        position    : np.ndarray [x, y]
        velocity    : np.ndarray [vx, vy]
        gate_passed : bool  (False = measurement was rejected as NLOS outlier)
        """
        if dt is None or not (0.005 < dt < 2.0):
            dt = self._dt

        F, Q = self._make_FQ(dt)
        z    = np.asarray(meas_xy[:2], dtype=float)

        # Cold-start
        if self._x is None:
            self._x              = np.array([z[0], z[1], 0.0, 0.0])
            self._P              = np.diag([0.3, 0.3, 1.0, 1.0])
            self._consec_rejects = 0
            return self._x[:2].copy(), self._x[2:].copy(), True

        # Predict
        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + Q

        # Chi-squared gate
        innov    = z - self._H @ x_pred
        S        = self._H @ P_pred @ self._H.T + self._R
        S_inv    = np.linalg.inv(S)
        mahal_sq = float(innov @ S_inv @ innov)

        if mahal_sq > self._GATE_CHI2:
            # Outlier: predict-only, velocity completely preserved
            self._consec_rejects += 1
            if self._consec_rejects > self._MAX_CONSEC_REJECTS:
                # Prolonged NLOS outage: kill velocity gradually
                x_pred[2] *= self._VELOCITY_DAMP
                x_pred[3] *= self._VELOCITY_DAMP
            self._x = x_pred
            self._P = P_pred
            return self._x[:2].copy(), self._x[2:].copy(), False

        # Valid measurement: full Kalman correction
        self._consec_rejects = 0
        K       = P_pred @ self._H.T @ S_inv
        self._x = x_pred + K @ innov
        self._P = (np.eye(4) - K @ self._H) @ P_pred

        return self._x[:2].copy(), self._x[2:].copy(), True

    def predict_only(self, dt: float = None):
        """Advance without a measurement (explicit NLOS gap)."""
        if self._x is None:
            return None, None
        if dt is None:
            dt = self._dt
        F, Q    = self._make_FQ(dt)
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        return self._x[:2].copy(), self._x[2:].copy()

    @property
    def position(self):
        return self._x[:2].copy() if self._x is not None else None

    @property
    def velocity(self):
        return self._x[2:].copy() if self._x is not None else None

    @property
    def consecutive_rejects(self):
        """Packets rejected in a row.  >6 means active NLOS outage."""
        return self._consec_rejects

    def _make_FQ(self, dt):
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)
        q = self._SIGMA_ACC ** 2
        Q = q * np.array([
            [dt**4/4, 0,       dt**3/2, 0      ],
            [0,       dt**4/4, 0,       dt**3/2],
            [dt**3/2, 0,       dt**2,   0      ],
            [0,       dt**3/2, 0,       dt**2  ],
        ], dtype=float)
        return F, Q


# ─────────────────────────────────────────────────────────────────────
#  FUSION ENGINE
# ─────────────────────────────────────────────────────────────────────
class FusionEngine:
    """
    Wires IRLSTrilateration + PositionKalmanFilter together.

    update() always returns exactly three values:
        pos_raw, pos_clean, gate_passed
    """

    def __init__(self):
        # Anchor positions [x, y, z] in metres.
        # z = 0.07: anchors are 7 cm in front of the whiteboard surface.
        # Update x/y to match your physical setup exactly.
        self.anchors = np.array([
            [0.00, 0.00, 0.07],   # A0 — bottom-left
            [1.23, 0.00, 0.07],   # A1 — bottom-right
            [1.23, 1.23, 0.07],   # A2 — top-right
            [0.00, 1.23, 0.07],   # A3 — top-left
        ], dtype=float)

        _bmin  = [-0.30, -0.30, -0.50]
        _bmax  = [ 1.55,  1.55,  1.00]
        _tag_z = MARKER_LENGTH

        self._trilat_raw   = IRLSTrilateration(self.anchors, _bmin, _bmax, tag_z=_tag_z)
        self._trilat_clean = IRLSTrilateration(self.anchors, _bmin, _bmax, tag_z=_tag_z)
        self._kf           = PositionKalmanFilter(nominal_dt=0.05)
        self._last_ts      = None

    def update(self, raw_dists, filtered_dists, quality_weights=None, packet_ts=None):
        """
        Parameters
        ----------
        raw_dists       : 4-tuple raw UWB distances (m)
        filtered_dists  : 4-tuple preprocessed distances (m)
        quality_weights : 4-tuple per-anchor weights [0.1, 1.0] or None
        packet_ts       : batch_ts (us) for adaptive dt

        Returns
        -------
        pos_raw    : np.ndarray [x, y, z]  IRLS on raw (diagnostic)
        pos_clean  : np.ndarray [x, y, z]  IRLS + KF, gated  <- use this
        gate_passed: bool  False = KF rejected measurement as NLOS outlier
        """
        # Adaptive dt
        dt = None
        if packet_ts is not None and self._last_ts is not None:
            dt_us = packet_ts - self._last_ts
            if 0 < dt_us < 2_000_000:
                dt = dt_us / 1_000_000.0
        if packet_ts is not None:
            self._last_ts = packet_ts

        # Layer 1: IRLS
        pos_raw_xyz, _   = self._trilat_raw.solve(raw_dists)
        pos_clean_xyz, _ = self._trilat_clean.solve(filtered_dists, quality_weights)

        # Layer 2: KF with chi-squared gate
        kf_xy, _, gate_passed = self._kf.update(pos_clean_xyz, dt=dt)

        pos_raw   = pos_raw_xyz
        pos_clean = np.array([kf_xy[0], kf_xy[1], pos_clean_xyz[2]])

        return pos_raw, pos_clean, gate_passed

    @property
    def kf_consecutive_rejects(self) -> int:
        """Consecutive gate rejections. >6 = active NLOS outage."""
        return self._kf.consecutive_rejects