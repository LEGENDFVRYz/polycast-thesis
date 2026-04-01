"""
fusion_engine.py  —  PolyCast UWB Position Solver
==================================================
Two-layer position pipeline:

Layer 1 — IRLS Trilateration (IRLSTrilateration)
    Iteratively Reweighted Least Squares with Huber M-estimator.
    When `tag_z` is supplied (the normal operating mode), the solver
    reduces the problem to 2D:

        d_2d[i] = sqrt( max(0,  d_3d[i]²  −  (tag_z − anchor_z[i])² ) )

    This projection removes the Z degree of freedom that is otherwise
    unobservable when all anchors are coplanar.  Without it the
    optimizer wanders freely in Z (within the ±0.5 m bounds), which
    corrupts the X/Y solution as well.

Layer 2 — Kalman Filter (PositionKalmanFilter)
    Constant-velocity 2D KF on [x, y, vx, vy].
    Smooths noisy trilateration output and handles missing
    measurements gracefully (predict-only when no anchors are valid).
    dt is kept adaptive using actual packet timestamps.

FusionEngine wires the two layers together and exposes a single
update() call to main.py.

Physical model
--------------
The UWB tag sits at the REAR END of a 0.21 m marker.  When the marker
is held straight against the whiteboard (perpendicular, no tilt) the
tag is at:

    tag_position = (tip_x,  tip_y,  MARKER_LENGTH)

Because the offset is purely along Z in this mode, the tip position
reported by the solver equals the tag's X/Y directly.

When IMU tilt correction is added later, replace the fixed `_tag_z`
with a per-packet value derived from the quaternion:

    offset_body = np.array([0, 0, -MARKER_LENGTH])
    offset_world = rotation_matrix(quat) @ offset_body
    tag_z_dynamic = tip_z - offset_world[2]          # typically ≈ tip_z + MARKER_LENGTH

Pass `tag_z_dynamic` into each solve() call via the optional `tag_z`
override on IRLSTrilateration.solve().
"""

import numpy as np
from scipy.optimize import least_squares


# ── Physical constants ─────────────────────────────────────────────────
MARKER_LENGTH = 0.21   # metres — distance from marker tip to UWB tag antenna


# ─────────────────────────────────────────────────────────────────────
#  LAYER 1 — IRLS TRILATERATION
# ─────────────────────────────────────────────────────────────────────
class IRLSTrilateration:
    """
    Solves for the tag position from a set of anchor distances using
    Iteratively Reweighted Least Squares with Huber M-estimator.

    Two operating modes
    -------------------
    2D mode  (tag_z is not None — recommended for whiteboard use)
        The known vertical separation between the tag and each anchor is
        used to project 3D measured distances into the horizontal plane:

            d_2d = sqrt( max(0, d_3d² − dz²) )

        where  dz = tag_z − anchor_z.

        IRLS then runs in XY only, returning [x, y, tag_z].

        Why this matters: with all anchors at the same Z, the 3D problem
        is rank-deficient in Z.  The solver wanders Z freely within its
        bounds, and that wandering biases X/Y as well.  Fixing Z removes
        the problem entirely.

    3D mode  (tag_z is None)
        Kept as a fallback / diagnostic path.  Use only when anchors are
        NOT coplanar.
    """

    # Tuning
    _HUBER_DELTA  = 0.08    # 8 cm — residuals above this are outlier-penalised
    _N_ITERATIONS = 5       # IRLS iterations (3-5 typical for UWB; 5 for stability)
    _MIN_ANCHORS  = 2       # minimum valid anchors to attempt a solve
    _VALID_THR    = 0.05    # distances below this (m) are treated as missing

    def __init__(self, anchors, bounds_min, bounds_max, tag_z=None):
        """
        Parameters
        ----------
        anchors    : array-like, shape (N, 3) — anchor positions [x, y, z]
        bounds_min : lower bounds for the solver  (3-element for 3D, or 2-element used for 2D)
        bounds_max : upper bounds
        tag_z      : fixed tag height (m); enables 2D projection mode when set
        """
        self.anchors    = np.asarray(anchors, dtype=float)
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max
        self._tag_z     = tag_z

        if tag_z is not None:
            # Pre-compute 2D quantities
            self._anch_2d = self.anchors[:, :2]
            # Squared Z-separation between the tag and each anchor
            self._dz_sq   = (tag_z - self.anchors[:, 2]) ** 2
            init_xy = np.mean(self._anch_2d, axis=0)
            # last_pos stores [x, y, tag_z] so update_predictions can use 3D
            self.last_pos = np.array([init_xy[0], init_xy[1], tag_z])
        else:
            self.last_pos = np.mean(self.anchors, axis=0).copy()

    # ── public API ────────────────────────────────────────────────────

    def solve(self, dists, quality_weights=None):
        """
        Parameters
        ----------
        dists           : sequence of 3D distances (m), one per anchor
        quality_weights : optional per-anchor float in [0.1, 1.0]
                          from UWBPreprocessor; None → uniform weights

        Returns
        -------
        position  : np.ndarray [x, y, z]
        residuals : np.ndarray per-anchor signed residual (m)
                    (0.0 for invalid / skipped anchors)
        """
        if self._tag_z is not None:
            return self._solve_2d(dists, quality_weights)
        return self._solve_3d(dists, quality_weights)

    # ── 2D solve (recommended when anchors are coplanar) ──────────────

    def _solve_2d(self, dists, quality_weights=None):
        """
        Project 3D UWB distances to 2D by removing the known Z component,
        then run IRLS in the XY plane only.

        Projection:  d_2d = sqrt( max(0, d_3d² − dz²) )

        Anchors where the measured distance is smaller than the geometric
        Z separation (impossible geometry — indicates a bad reading) are
        automatically excluded by the validity check.
        """
        n = min(len(self._anch_2d), len(dists))
        dists_arr = np.asarray(dists[:n], dtype=float)

        # Project 3D → 2D horizontal distances
        dz_sq = self._dz_sq[:n]
        d2d   = np.sqrt(np.maximum(0.0, dists_arr ** 2 - dz_sq))

        # An anchor is valid only if:
        #   (a) original 3D distance is above the noise floor
        #   (b) the projection produced a non-trivial 2D distance
        valid_idx = [
            i for i in range(n)
            if dists[i] > self._VALID_THR and d2d[i] > 0.01
        ]

        if len(valid_idx) < self._MIN_ANCHORS:
            return self.last_pos.copy(), np.zeros(len(dists))

        v_anch = self._anch_2d[valid_idx]
        v_dist = d2d[valid_idx]
        v_qw   = (
            np.array([quality_weights[i] for i in valid_idx], dtype=float)
            if quality_weights is not None
            else np.ones(len(valid_idx))
        )

        # 2D bounds (first two elements of the supplied bounds)
        bmin_2d = self.bounds_min[:2]
        bmax_2d = self.bounds_max[:2]

        # IRLS loop in XY
        irls_w = np.ones(len(valid_idx))
        guess  = self.last_pos[:2].copy()

        for _ in range(self._N_ITERATIONS):
            combined_w = irls_w * v_qw
            res = least_squares(
                self._weighted_residuals,
                guess,
                bounds=(bmin_2d, bmax_2d),
                args=(v_anch, v_dist, combined_w),
                method='trf',
                loss='linear',
                max_nfev=40,
                ftol=1e-4,
                xtol=1e-4,
            )
            guess  = res.x
            d_calc = np.linalg.norm(v_anch - guess, axis=1)
            irls_w = self._huber_weights(d_calc - v_dist)

        # Persist [x, y, tag_z] for warm-starting next call and for
        # update_predictions (which needs a full 3D position)
        self.last_pos[0] = guess[0]
        self.last_pos[1] = guess[1]
        # last_pos[2] stays as tag_z (set at construction)

        # Build full residual vector (0 for invalid / skipped anchors)
        full_res = np.zeros(len(dists))
        d_calc   = np.linalg.norm(v_anch - guess, axis=1)
        for j, i in enumerate(valid_idx):
            full_res[i] = d_calc[j] - d2d[i]

        return np.array([guess[0], guess[1], self._tag_z]), full_res

    # ── 3D solve (fallback / diagnostic) ─────────────────────────────

    def _solve_3d(self, dists, quality_weights=None):
        n_anchors = min(len(self.anchors), len(dists))
        valid_idx = [i for i in range(n_anchors) if dists[i] > self._VALID_THR]

        if len(valid_idx) < self._MIN_ANCHORS:
            return self.last_pos.copy(), np.zeros(len(dists))

        v_anch = self.anchors[valid_idx]
        v_dist = np.array([dists[i] for i in valid_idx], dtype=float)
        v_qw   = (
            np.array([quality_weights[i] for i in valid_idx], dtype=float)
            if quality_weights is not None
            else np.ones(len(valid_idx))
        )

        irls_w = np.ones(len(valid_idx))
        guess  = self.last_pos.copy()

        for _ in range(self._N_ITERATIONS):
            combined_w = irls_w * v_qw
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
            guess  = result.x
            d_calc = np.linalg.norm(v_anch - guess, axis=1)
            irls_w = self._huber_weights(d_calc - v_dist)

        self.last_pos = guess

        full_res = np.zeros(len(dists))
        d_calc   = np.linalg.norm(v_anch - guess, axis=1)
        for j, i in enumerate(valid_idx):
            full_res[i] = d_calc[j] - dists[i]

        return guess, full_res

    # ── internals ─────────────────────────────────────────────────────

    @staticmethod
    def _weighted_residuals(pos, anchors, dists, weights):
        d_calc = np.linalg.norm(anchors - pos, axis=1)
        return weights * (d_calc - dists)

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
    # Writing speed is ~0.5 m/s; quick strokes can be 2–3 m/s²
    _SIGMA_ACC  = 2.5

    # Measurement noise: expected residual after IRLS + 2D projection (m)
    # With coplanar-anchor Z fixed, this is tighter than the 3D case
    _SIGMA_MEAS = 0.04

    def __init__(self, nominal_dt: float = 0.05):
        self._dt = nominal_dt
        self._x  = None                         # state vector [px, py, vx, vy]
        self._P  = None                         # covariance matrix
        self._H  = np.array([[1, 0, 0, 0],      # measurement matrix
                              [0, 1, 0, 0]], dtype=float)
        self._R  = (self._SIGMA_MEAS ** 2) * np.eye(2)

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
        z    = np.asarray(meas_xy[:2], dtype=float)

        if self._x is None:
            # Cold-start: trust the first trilateration fix directly
            self._x = np.array([z[0], z[1], 0.0, 0.0])
            self._P = np.diag([0.3, 0.3, 1.0, 1.0])
            return self._x[:2].copy(), self._x[2:].copy()

        # Predict
        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + Q

        # Kalman gain
        S = self._H @ P_pred @ self._H.T + self._R
        K = P_pred @ self._H.T @ np.linalg.inv(S)

        # Correct
        innov   = z - self._H @ x_pred
        self._x = x_pred + K @ innov
        self._P = (np.eye(4) - K @ self._H) @ P_pred

        return self._x[:2].copy(), self._x[2:].copy()

    def predict_only(self, dt: float = None):
        """Advance the state without a measurement (UWB gap / NLOS outage)."""
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

    # ── internals ─────────────────────────────────────────────────────

    def _make_FQ(self, dt):
        """Build state-transition F and process noise Q for given dt."""
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

    update(raw_dists, filtered_dists, quality_weights, packet_ts)
        → (pos_raw, pos_clean)

    pos_raw   = IRLS on raw distances, 2D-projected (diagnostic / comparison)
    pos_clean = IRLS on filtered distances with quality weights, then KF
                (use this for all downstream processing)

    Both solvers run in 2D-projection mode (tag_z = MARKER_LENGTH) so that
    Z-axis ambiguity is eliminated.  The returned positions are the MARKER
    TIP position: because the marker is held straight, tip_xy == tag_xy.

    IMU tilt correction (future work)
    ----------------------------------
    When the marker is tilted by angle θ from perpendicular:
        tip_x = tag_x − MARKER_LENGTH * sin(θ) * cos(φ)
        tip_y = tag_y − MARKER_LENGTH * sin(θ) * sin(φ)
    where (θ, φ) come from the BNO085 quaternion.  Apply this correction
    *after* position solving, in main.py, using the IMU batch from the
    same packet.
    """

    def __init__(self):
        # ── Anchor positions [x, y, z] in metres ──────────────────────
        # z=0.07 = anchors mounted 7 cm in front of the whiteboard surface.
        # Update x/y to match your physical whiteboard corners exactly.
        self.anchors = np.array([
            [0.00, 0.00, 0.07],   # A0 — bottom-left
            [1.23, 0.00, 0.07],   # A1 — bottom-right
            [1.23, 1.23, 0.07],   # A2 — top-right
            [0.00, 1.23, 0.07],   # A3 — top-left
        ], dtype=float)

        # Solver bounds — XY only (first two elements used in 2D mode;
        # third element kept for API compatibility with 3D fallback)
        _bmin = [-0.30, -0.30, -0.50]
        _bmax = [ 1.55,  1.55,  1.00]

        # Tag Z when marker held straight: tip at board surface → tag at
        # z = MARKER_LENGTH above the board surface
        _tag_z = MARKER_LENGTH

        # Two independent IRLS solvers: raw (diagnostic) and clean (output)
        self._trilat_raw   = IRLSTrilateration(self.anchors, _bmin, _bmax, tag_z=_tag_z)
        self._trilat_clean = IRLSTrilateration(self.anchors, _bmin, _bmax, tag_z=_tag_z)

        # KF runs on the cleaned 2D path only
        self._kf = PositionKalmanFilter(nominal_dt=0.05)

        self._last_ts = None    # for adaptive dt

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

        Both z components equal MARKER_LENGTH (tag position, not tip position).
        When the marker is held straight, tip_x == pos_clean[0],
        tip_y == pos_clean[1].
        """
        # ── Adaptive dt ───────────────────────────────────────────────
        dt = None
        if packet_ts is not None and self._last_ts is not None:
            dt_us = packet_ts - self._last_ts
            if 0 < dt_us < 2_000_000:           # sanity: 0 – 2 s
                dt = dt_us / 1_000_000.0
        if packet_ts is not None:
            self._last_ts = packet_ts

        # ── Layer 1: IRLS solves (2D-projected) ───────────────────────
        pos_raw_xyz, _   = self._trilat_raw.solve(raw_dists)
        pos_clean_xyz, _ = self._trilat_clean.solve(filtered_dists, quality_weights)

        # ── Layer 2: Kalman filter on cleaned path ────────────────────
        kf_xy, _ = self._kf.update(pos_clean_xyz, dt=dt)

        # Assemble output — Z = MARKER_LENGTH (= tag z, not tip z)
        pos_raw   = pos_raw_xyz
        pos_clean = np.array([kf_xy[0], kf_xy[1], pos_clean_xyz[2]])

        return pos_raw, pos_clean