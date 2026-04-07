"""
preprocessor.py  —  PolyCast Signal Preprocessor (Async Stream)
===============================================================
UWBPreprocessor:
    Per-anchor 3-stage pipeline: validity gate -> median despike -> variable-rate EMA.
    Returns filtered distances AND quality weights [0.1 -> 1.0]
    so the EKF can perform weighted measurement updates.

    Two output paths:
        filtered (EMA-smoothed)  — for visualisation
        despiked (median only)   — for EKF (avoids EMA lag / temporal correlation)

AnchorQualityTracker:
    Per-anchor NLOS detection via rolling variance + innovation scoring.

IMUPreprocessor:
    Quaternion normalization guard.
"""

import numpy as np
from collections import deque


# -------------------------------------------------------------------------
#  PER-ANCHOR QUALITY TRACKER
#  Tracks two independent NLOS indicators per anchor:
#    1. Recent ranging variance  ->  high sigma^2 = multipath / NLOS
#    2. Innovation (measured vs. predicted)  ->  large jump = outlier
# -------------------------------------------------------------------------
class AnchorQualityTracker:
    """
    Maintains a rolling variance window and an expected-distance
    prediction from the last solved position.  Combines both into a
    scalar quality weight in [0.1, 1.0]:
        1.0  ->  clean LOS, low variance, consistent with prediction
        0.1  ->  NLOS suspect, high variance or large jump
    """

    # Tuning constants
    _VAR_SCALE    = 80.0   # sigma^2 penalty slope (80 -> weight ~ 0.56 at sigma^2=0.01 m^2)
    _INNOV_SIGMA  = 0.12   # Innovation sigma (m); 12 cm -> half-weight at innovation = 12 cm
    _MIN_WEIGHT   = 0.10   # Floor — never zero-out an anchor completely

    def __init__(self, window: int = 15):
        self._window    = deque(maxlen=window)
        self._predicted = None          # Expected distance fed back from FusionEngine

    # -- public API --------------------------------------------------------

    def push(self, distance: float) -> None:
        self._window.append(distance)

    def set_prediction(self, dist: float) -> None:
        self._predicted = dist

    def weight(self, raw_dist: float) -> float:
        """Return combined quality weight for the current raw measurement."""
        var_w   = self._variance_weight()
        innov_w = self._innovation_weight(raw_dist)

        # Geometric mean: both must be good for the anchor to score high
        combined = float(np.sqrt(var_w * innov_w))
        return max(self._MIN_WEIGHT, min(1.0, combined))

    # -- internals ---------------------------------------------------------

    def _variance_weight(self) -> float:
        if len(self._window) < 4:
            return 1.0                          # not enough data -> optimistic
        var = float(np.var(self._window))
        return 1.0 / (1.0 + self._VAR_SCALE * var)

    def _innovation_weight(self, raw_dist: float) -> float:
        if self._predicted is None:
            return 1.0                          # no prediction yet -> neutral
        innov = abs(raw_dist - self._predicted)
        # Gaussian-shaped penalty centred on zero innovation
        return float(np.exp(-0.5 * (innov / self._INNOV_SIGMA) ** 2))


# -------------------------------------------------------------------------
#  UWB PREPROCESSOR
# -------------------------------------------------------------------------
class UWBPreprocessor:
    """
    3-stage pipeline per anchor:
        Stage 1 — Validity gate      (clamp out-of-range readings)
        Stage 2 — Median despike     (reject impulse noise)
        Stage 3 — Variable-rate EMA  (smooth; faster response on large step)

    Returns:
        (filtered_dists, quality_weights, despiked_dists)   all as 4-tuples

    Call update_predictions(pos_xyz, anchor_positions) after each
    EKF update to feed innovation signals back into the NLOS detectors.
    """

    # EMA parameters — alpha adapts based on how large the step is
    _EMA_BASE  = 0.30   # Normal-movement EMA coefficient
    _EMA_FAST  = 0.70   # Fast-movement EMA coefficient (large step detected)
    _STEP_THR  = 0.12   # Distance change (m) that triggers fast mode

    def __init__(self, spike_window: int = 7, max_range: float = 6.0, offsets: tuple = (0.0, 0.0, 0.0, 0.0)):
        self.max_range = max_range
        self.n         = 4
        self.offsets   = offsets

        self._med_bufs = [deque(maxlen=spike_window) for _ in range(self.n)]
        self._ema      = [None] * self.n
        self._last     = [0.0]  * self.n
        self._trackers = [AnchorQualityTracker() for _ in range(self.n)]

        # Smaller window for EKF path (offset + despike, NO EMA — avoids lag
        # and temporal correlation that violate the EKF's white-noise assumption)
        self._med_bufs_ekf = [deque(maxlen=3) for _ in range(self.n)]
        self._last_ekf     = [0.0] * self.n

    # -- main entry point --------------------------------------------------

    def process(self, d0: float, d1: float, d2: float, d3: float):
        """
        Returns:
            filtered  : tuple(float x 4)  — offset + median + EMA  (low-noise, for visualisation)
            weights   : tuple(float x 4)  — per-anchor quality [0.1, 1.0]
            despiked  : tuple(float x 4)  — offset + small median, NO EMA  (for EKF measurement update)

        The 'despiked' output avoids the EMA's lag (~40 ms) and temporal
        correlation, satisfying the EKF's white-measurement-noise assumption.
        """
        filtered = []
        weights  = []
        despiked = []

        for i, raw in enumerate([d0, d1, d2, d3]):
            # -- Stage 1: validity gate (shared) ---------------------------
            raw_with_offset = raw + self.offsets[i]
            if (raw_with_offset <= 0.05 or raw_with_offset > self.max_range
                    or not np.isfinite(raw_with_offset)):
                cooked = self._last[i]
            else:
                cooked = raw_with_offset

            # -- Stage 2 (EKF path): small-window median only -------------
            self._med_bufs_ekf[i].append(cooked)
            ekf_val = float(np.median(self._med_bufs_ekf[i]))
            if ekf_val > 0.05:
                self._last_ekf[i] = ekf_val
            despiked.append(self._last_ekf[i])

            # -- Stage 2 (vis path): larger-window median despike ----------
            self._med_bufs[i].append(cooked)
            median_val = float(np.median(self._med_bufs[i]))

            # -- Stage 3 (vis path): variable-rate EMA ---------------------
            if self._ema[i] is None:
                self._ema[i] = median_val
            else:
                step  = abs(median_val - self._ema[i])
                alpha = self._EMA_FAST if step > self._STEP_THR else self._EMA_BASE
                self._ema[i] = alpha * median_val + (1.0 - alpha) * self._ema[i]

            smooth = self._ema[i]
            self._last[i] = smooth

            # -- NLOS quality scoring --------------------------------------
            self._trackers[i].push(median_val)
            w = self._trackers[i].weight(cooked)

            filtered.append(smooth)
            weights.append(w)

        return tuple(filtered), tuple(weights), tuple(despiked)

    # -- feedback from EKF -------------------------------------------------

    def update_predictions(self, position_xyz, anchor_positions) -> None:
        """
        Feed the latest position estimate back into per-anchor NLOS trackers
        so innovation can be measured on the next cycle.

        position_xyz    : array-like [x, y, z]
        anchor_positions: array-like shape (N, 3)
        """
        if position_xyz is None:
            return
        pos = np.asarray(position_xyz, dtype=float)
        for i, a in enumerate(anchor_positions):
            if i < self.n:
                pred = float(np.linalg.norm(pos - np.asarray(a, dtype=float)))
                self._trackers[i].set_prediction(pred)


# -------------------------------------------------------------------------
#  IMU PREPROCESSOR
# -------------------------------------------------------------------------
class IMUPreprocessor:
    """
    Normalises the quaternion from the BNO085 sensor hub.
    The BNO085 already runs Hillcrest SH-2 sensor fusion, so
    we only need the normalization guard — no manual Madgwick/Mahony.
    """

    def process_sample(self, qx, qy, qz, qw, ax, ay, az):
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0, ax, ay, az)   # identity quaternion fallback
        return (qx/norm, qy/norm, qz/norm, qw/norm, ax, ay, az)
