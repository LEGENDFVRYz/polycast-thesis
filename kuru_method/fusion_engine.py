"""
fusion_engine.py  —  PolyCast UWB Position Solver
==================================================
Two-layer position pipeline:

Layer 1 — IRLS Trilateration (IRLSTrilateration)
    Iteratively Reweighted Least Squares.  Starts from a standard
    least-squares solve then alternately reweights anchors using
    Huber M-estimator weights, down-weighting outlier anchors each
    iteration.  Also accepts per-anchor quality weights from the
    UWBPreprocessor to further suppress NLOS anchors.

Layer 2 — Kalman Filter (PositionKalmanFilter)
    Constant-velocity 2D KF on [x, y, vx, vy].
    Smooths noisy trilateration output and handles missing
    measurements gracefully (predict-only when no anchors valid).
    dt is kept adaptive using actual packet timestamps.

FusionEngine wires the two layers together and exposes a single
update() call to main.py.
"""

import numpy as np
from scipy.optimize import least_squares


# ─────────────────────────────────────────────────────────────────────
#  LAYER 1 — IRLS TRILATERATION
# ─────────────────────────────────────────────────────────────────────
class IRLSTrilateration:
    """
    Solves for [x, y, z] from a set of anchor distances using
    Iteratively Reweighted Least Squares with Huber M-estimator.

    Why IRLS over plain least-squares?
        Plain LS minimises the sum of squared residuals equally across
        all anchors.  A single NLOS anchor that reads 20 cm too far
        will pull the estimated position heavily.  IRLS detects such
        outliers via large residuals and reduces their influence
        automatically — without needing to know which anchor is bad.

    The loop (typically 4 iterations) converges quickly because the
    Huber weights change little after the second iteration.
    """

    # Tuning
    _HUBER_DELTA  = 0.08    # 8 cm — residuals above this are outlier-penalised
    _N_ITERATIONS = 4       # IRLS iterations (3-5 typical for UWB)
    _MIN_ANCHORS  = 2       # Minimum valid anchors to attempt a solve
    _VALID_THR    = 0.05    # Distances below this (m) are treated as missing

    def __init__(self, anchors, bounds_min, bounds_max):
        self.anchors    = np.asarray(anchors,    dtype=float)
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max
        self.last_pos   = np.mean(self.anchors, axis=0).copy()

    # ── public API ────────────────────────────────────────────────────

    def solve(self, dists, quality_weights=None):
        """
        Parameters
        ----------
        dists           : sequence of distances (m), one per anchor
        quality_weights : optional per-anchor float in [0.1, 1.0]
                          from UWBPreprocessor; None → uniform weights

        Returns
        -------
        position   : np.ndarray [x, y, z]
        residuals  : np.ndarray per-anchor signed residual (m)
                     (0.0 for invalid / missing anchors)
        """
        n_anchors = min(len(self.anchors), len(dists))

        # Select valid anchors
        valid_idx = [i for i in range(n_anchors) if dists[i] > self._VALID_THR]

        if len(valid_idx) < self._MIN_ANCHORS:
            return self.last_pos.copy(), np.zeros(len(dists))

        v_anch = self.anchors[valid_idx]
        v_dist = np.array([dists[i] for i in valid_idx], dtype=float)

        if quality_weights is not None:
            v_qw = np.array([quality_weights[i] for i in valid_idx], dtype=float)
        else:
            v_qw = np.ones(len(valid_idx))

        # ── IRLS loop ─────────────────────────────────────────────────
        irls_w    = np.ones(len(valid_idx))
        guess     = self.last_pos.copy()

        for _ in range(self._N_ITERATIONS):
            combined_w = irls_w * v_qw          # merge Huber + quality weights

            result = least_squares(
                self._weighted_residuals,
                guess,
                bounds=(self.bounds_min, self.bounds_max),
                args=(v_anch, v_dist, combined_w),
                method='trf',
                loss='linear',
                max_nfev=40,
                ftol=1e-4,
                xtol=1e-4,
            )

            guess = result.x

            # Recompute raw residuals for next Huber weighting step
            dist_calc = np.linalg.norm(v_anch - guess, axis=1)
            raw_res   = dist_calc - v_dist
            irls_w    = self._huber_weights(raw_res)

        self.last_pos = guess

        # ── Build full residual vector (zeros for invalid anchors) ─────
        full_res = np.zeros(len(dists))
        dist_calc = np.linalg.norm(v_anch - guess, axis=1)
        for j, i in enumerate(valid_idx):
            full_res[i] = dist_calc[j] - dists[i]

        return guess, full_res

    # ── internals ─────────────────────────────────────────────────────

    @staticmethod
    def _weighted_residuals(pos, anchors, dists, weights):
        dist_calc = np.linalg.norm(anchors - pos, axis=1)
        return weights * (dist_calc - dists)

    def _huber_weights(self, residuals):
        """Huber M-estimator: full weight inside delta, 1/|r| outside."""
        abs_r = np.abs(residuals)
        return np.where(
            abs_r <= self._HUBER_DELTA,
            1.0,
            self._HUBER_DELTA / (abs_r + 1e-9),
        )


# ─────────────────────────────────────────────────────────────────────
#  LAYER 2 — POSITION KALMAN FILTER
# ─────────────────────────────────────────────────────────────────────
class PositionKalmanFilter:
    """
    Linear KF on state  x = [px, py, vx, vy].
    Measurement: [px, py] from IRLS trilateration.

    iTrackU insight: IMU handles sub-second bridging; UWB provides
    the long-term anchor.  Here the KF plays an analogous role —
    it propagates a velocity estimate between UWB updates and
    smooths the trilateration jitter.

    dt is updated dynamically each call for precise timing.
    """

    # Process noise: max expected marker acceleration (m/s²)
    _SIGMA_ACC  = 2.0

    # Measurement noise: expected residual error after IRLS + preprocessing (m)
    _SIGMA_MEAS = 0.045

    def __init__(self, nominal_dt: float = 0.05):
        self._dt  = nominal_dt
        self._x   = None                        # state vector [px, py, vx, vy]
        self._P   = None                        # covariance matrix
        self._H   = np.array([[1,0,0,0],        # measurement matrix
                               [0,1,0,0]], dtype=float)
        self._R   = (self._SIGMA_MEAS**2) * np.eye(2)

    # ── public API ────────────────────────────────────────────────────

    def update(self, meas_xy, dt: float = None):
        """
        Predict + correct step.

        Parameters
        ----------
        meas_xy : [x, y] from trilateration
        dt      : elapsed time since last call (s); uses nominal_dt if None

        Returns
        -------
        position : np.ndarray [x, y] (smoothed)
        velocity : np.ndarray [vx, vy] (estimated marker velocity)
        """
        if dt is None or not (0.005 < dt < 2.0):
            dt = self._dt

        F, Q = self._make_FQ(dt)

        z = np.asarray(meas_xy[:2], dtype=float)

        if self._x is None:
            # First measurement — cold-start
            self._x = np.array([z[0], z[1], 0.0, 0.0])
            self._P = np.diag([0.5, 0.5, 1.0, 1.0])
            return self._x[:2].copy(), self._x[2:].copy()

        # Predict
        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + Q

        # Kalman gain
        S = self._H @ P_pred @ self._H.T + self._R
        K = P_pred @ self._H.T @ np.linalg.inv(S)

        # Correct
        innov  = z - self._H @ x_pred
        self._x = x_pred + K @ innov
        self._P = (np.eye(4) - K @ self._H) @ P_pred

        return self._x[:2].copy(), self._x[2:].copy()

    def predict_only(self, dt: float = None):
        """Advance the state without a measurement (UWB gap / NLOS outage)."""
        if self._x is None:
            return None, None
        if dt is None:
            dt = self._dt
        F, Q   = self._make_FQ(dt)
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        return self._x[:2].copy(), self._x[2:].copy()

    @property
    def position(self):
        return self._x[:2].copy() if self._x is not None else None

    @property
    def velocity(self):
        return self._x[2:].copy() if self._x is not None else None

    # ── internals ─────────────────────────────────────────────────────

    def _make_FQ(self, dt):
        """Build state-transition matrix F and process noise Q for given dt."""
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
#  FUSION ENGINE  —  public interface used by main.py
# ─────────────────────────────────────────────────────────────────────
class FusionEngine:
    """
    Wires IRLSTrilateration + PositionKalmanFilter together.

    update(raw_dists, filtered_dists, quality_weights, dt)
        → (pos_raw, pos_clean)

    pos_raw   = IRLS only on raw distances (diagnostic / comparison)
    pos_clean = IRLS on filtered distances with quality weights, then KF
                (this is the high-quality estimate to use downstream)
    """

    def __init__(self):
        # ── Anchor positions [x, y, z] in metres ──────────────────────
        # Update these to match your physical whiteboard corners.
        # Current config: 3-anchor prototype (A0 at origin is commented
        # out matching the original code; swap to 4 when ready).
        self.anchors = np.array([
            [0.00, 0.00, 0.07],   # A0 — bottom-left
            [1.23, 0.00, 0.07],   # A1 — bottom-right
            [1.23, 1.23, 0.07],   # A2 — top-right
            [0.00, 1.23, 0.07],   # A3 — top-left
        ], dtype=float)

        _bmin = [-0.5, -0.5, -0.5]
        _bmax = [ 2.0,  2.0,  1.0]

        # Two independent IRLS solvers: one for raw (diagnostic), one for clean
        self._trilat_raw   = IRLSTrilateration(self.anchors, _bmin, _bmax)
        self._trilat_clean = IRLSTrilateration(self.anchors, _bmin, _bmax)

        # KF only on the cleaned path
        self._kf = PositionKalmanFilter(nominal_dt=0.05)

        self._last_ts = None    # for adaptive dt calculation

    # ── public API ────────────────────────────────────────────────────

    def update(self, raw_dists, filtered_dists, quality_weights=None, packet_ts=None):
        """
        Parameters
        ----------
        raw_dists       : 4-tuple of raw UWB distances (m)
        filtered_dists  : 4-tuple of preprocessed distances (m)
        quality_weights : 4-tuple of per-anchor weights [0.1, 1.0] or None
        packet_ts       : batch_ts from packet (µs) for adaptive dt; None → nominal

        Returns
        -------
        pos_raw   : np.ndarray [x, y, z]  — IRLS on raw (for comparison)
        pos_clean : np.ndarray [x, y, z]  — IRLS + KF on filtered (use this)
        """
        # ── Adaptive dt ───────────────────────────────────────────────
        dt = None
        if packet_ts is not None and self._last_ts is not None:
            dt_us = packet_ts - self._last_ts
            if 0 < dt_us < 2_000_000:          # sanity: 0 – 2 s
                dt = dt_us / 1_000_000.0
        if packet_ts is not None:
            self._last_ts = packet_ts

        # ── Layer 1: IRLS solves ──────────────────────────────────────
        pos_raw_xyz, _   = self._trilat_raw.solve(raw_dists)
        pos_clean_xyz, _ = self._trilat_clean.solve(filtered_dists, quality_weights)

        # ── Layer 2: Kalman filter on cleaned path ────────────────────
        kf_xy, _ = self._kf.update(pos_clean_xyz, dt=dt)

        # Assemble final 3-element vectors
        pos_raw   = pos_raw_xyz
        pos_clean = np.array([kf_xy[0], kf_xy[1],
                               pos_clean_xyz[2] if len(pos_clean_xyz) > 2 else 0.0])

        return pos_raw, pos_clean