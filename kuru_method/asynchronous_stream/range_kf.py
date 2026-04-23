"""
range_kf.py  —  Per-anchor 1D range Kalman pre-filter (Item B + C)
===================================================================
Scalar 2-state Kalman filter on each anchor's UWB range.

Reference
---------
Zou, L. et al. (2023).  *An Improved UWB/IMU Tightly Coupled Indoor
Positioning Algorithm.*  §3.1 (per-anchor range pre-filter) and §3.3
(posterior-range feedback from the EKF).

State per anchor i
------------------
    s_i = [r_i, r_dot_i]   (range and range-rate, both in metres / m·s⁻¹)

    Constant-velocity model:
        r'_i      = r_i + r_dot_i · dt
        r_dot'_i  = r_dot_i

    Process noise enters via r_dot:
        sigma_rdot = 0.3 m/s   (a writing stroke barely exceeds this)

Two measurement channels
------------------------
    1. Raw UWB range            sigma = SIGMA_UWB (0.08 m)
    2. EKF posterior range      sigma = FEEDBACK_SIGMA (default 2 × SIGMA_UWB)

The feedback channel (Item C) acts as a soft prior — large enough that
the raw ranges still dominate, but small enough to suppress slowly-
varying per-anchor bias that the chi-squared gate cannot catch.

Re-initialisation
-----------------
    On the first sample, or after a gap > MAX_DT_S, the filter snaps to
    the latest raw measurement (state = [z, 0]) and inflates covariance
    so the next few samples drive convergence quickly.
"""

from __future__ import annotations
import numpy as np


SIGMA_RDOT_DEFAULT     = 0.8   # m/s  process noise on range-rate
                               # (0.3 under-sized: natural writing causes ranges
                               # to change at up to 1 m/s during fast strokes)
MAX_DT_S               = 0.5   # s    gap above which we re-init
INIT_VAR_R             = 1.0   # m^2  initial range variance (huge)
INIT_VAR_RDOT          = 1.0   # (m/s)^2 initial range-rate variance


class PerAnchorRangeKF:
    """One scalar range Kalman per anchor (state = [r, r_dot])."""

    def __init__(self,
                 sigma_meas: float,
                 sigma_rdot: float = SIGMA_RDOT_DEFAULT,
                 sigma_feedback: float | None = None):
        self.sigma_meas     = float(sigma_meas)
        self.sigma_rdot     = float(sigma_rdot)
        # Item C: posterior-range feedback measurement noise.  Defaults
        # to 2 × sigma_meas so feedback is a soft prior, not a slave loop.
        self.sigma_feedback = (float(sigma_feedback)
                               if sigma_feedback is not None
                               else 2.0 * self.sigma_meas)
        self._x = None    # state (2,)
        self._P = None    # covariance (2,2)

    # -- public API ---------------------------------------------------------

    def step(self, dt: float, z_raw: float) -> float:
        """
        Predict + update with the latest raw range.

        Parameters
        ----------
        dt    : time since last call (s).  Use 0.1 if you don't track it.
        z_raw : raw UWB range (m).

        Returns
        -------
        r_filt : filtered range estimate (m).  Equals z_raw on first call
                 (or after a gap), so callers can swap raw -> filtered
                 without changing the cold-start behaviour.
        """
        # Re-init on first call or long gap.
        if self._x is None or not np.isfinite(dt) or dt <= 0.0 or dt > MAX_DT_S:
            self._x = np.array([float(z_raw), 0.0])
            self._P = np.diag([INIT_VAR_R, INIT_VAR_RDOT])
            return float(z_raw)

        self._predict(dt)
        self._update_scalar(z_raw, self.sigma_meas)
        return float(self._x[0])

    def update_feedback(self, r_post: float) -> None:
        """
        Item C: fold the EKF posterior range back into the per-anchor
        filter as a second measurement with its own noise scale.  No
        predict step here — caller has already advanced time via step().
        """
        if self._x is None or not np.isfinite(r_post):
            return
        self._update_scalar(float(r_post), self.sigma_feedback)

    @property
    def range(self) -> float | None:
        return float(self._x[0]) if self._x is not None else None

    @property
    def rate(self) -> float | None:
        return float(self._x[1]) if self._x is not None else None

    def reset(self) -> None:
        self._x = None
        self._P = None

    # -- internal -----------------------------------------------------------

    def _predict(self, dt: float) -> None:
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        # Discrete white-noise constant-velocity Q (Q-formulation from
        # Bar-Shalom, Li & Kirubarajan, Estimation with Applications 6.3.1)
        sa2 = self.sigma_rdot ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = np.array([[dt4 / 4.0, dt3 / 2.0],
                      [dt3 / 2.0, dt2      ]]) * sa2
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        self._P = 0.5 * (self._P + self._P.T)

    def _update_scalar(self, z: float, sigma: float) -> None:
        H = np.array([1.0, 0.0])             # observe r only
        S = float(H @ self._P @ H + sigma * sigma)
        K = (self._P @ H) / S                # (2,)
        innov = float(z) - float(H @ self._x)
        self._x = self._x + K * innov
        I_KH = np.eye(2) - np.outer(K, H)
        # Joseph form for numerical stability under small/large sigma.
        self._P = I_KH @ self._P @ I_KH.T + (sigma * sigma) * np.outer(K, K)
        self._P = 0.5 * (self._P + self._P.T)


class PerAnchorRangeKFBank:
    """Convenience wrapper — one PerAnchorRangeKF per anchor index."""

    def __init__(self,
                 n_anchors: int,
                 sigma_meas: float,
                 sigma_rdot: float = SIGMA_RDOT_DEFAULT,
                 sigma_feedback: float | None = None):
        self._kfs = [PerAnchorRangeKF(sigma_meas, sigma_rdot, sigma_feedback)
                     for _ in range(n_anchors)]

    def step_all(self, dt: float, raw_ranges) -> np.ndarray:
        """Run a predict+update for every anchor; return filtered ranges."""
        out = np.full(len(self._kfs), np.nan, dtype=float)
        for i, z in enumerate(raw_ranges[:len(self._kfs)]):
            if np.isfinite(z):
                out[i] = self._kfs[i].step(float(dt), float(z))
            else:
                # Skip predict on missing measurement — preserves last state.
                out[i] = self._kfs[i].range if self._kfs[i].range is not None else np.nan
        return out

    def update_feedback(self, posterior_ranges) -> None:
        for i, r in enumerate(posterior_ranges[:len(self._kfs)]):
            if r is not None and np.isfinite(r):
                self._kfs[i].update_feedback(float(r))

    def reset(self) -> None:
        for kf in self._kfs:
            kf.reset()

    def __getitem__(self, i: int) -> PerAnchorRangeKF:
        return self._kfs[i]

    def __len__(self) -> int:
        return len(self._kfs)
