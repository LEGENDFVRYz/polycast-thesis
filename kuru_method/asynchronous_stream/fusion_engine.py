"""
fusion_engine.py  —  PolyCast IRLS Trilateration (Async Stream)
===============================================================
Used exclusively for EKF cold-start: provides an initial 2D position
estimate from the first few UWB range measurements before the EKF is
initialised.

Layer: IRLSTrilateration
    2D IRLS with Huber M-estimator.
    Solves tag XY using projected horizontal distances.

The PositionKalmanFilter and FusionEngine classes from the batched
pipeline are NOT needed here — the tightly-coupled EKF replaces them.

TUNING KNOBS
============
IRLSTrilateration
-----------------
    _HUBER_DELTA   Residual (m) above which an anchor is down-weighted.
                   Default 0.08.

    _N_ITERATIONS  IRLS passes.  Default 5.  3 is usually enough;
                   7 for extra precision at ~2 ms extra CPU per packet.
"""

import numpy as np
from scipy.optimize import least_squares


# -- Physical constant ------------------------------------------------------
MARKER_LENGTH = 0.21   # metres — distance from tip to UWB tag antenna


# -------------------------------------------------------------------------
#  IRLS TRILATERATION
# -------------------------------------------------------------------------
class IRLSTrilateration:
    """
    Solves for tag XY from anchor distances using IRLS + Huber weights.

    2D-projection mode (tag_z supplied — strongly recommended):
        d_2d[i] = sqrt( max(0, d_3d[i]^2 - (tag_z - anchor_z[i])^2) )

    This removes the unobservable Z degree of freedom when all anchors
    are coplanar, which was the original 20-30 cm error source.
    """

    # -- Tuning ------------------------------------------------------------
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
