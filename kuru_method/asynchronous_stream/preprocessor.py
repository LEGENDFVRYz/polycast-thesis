"""
preprocessor.py  —  PolyCast Signal Preprocessor (Async Stream)
===============================================================
UWBPreprocessor:
    Per-anchor pipeline with TWO output paths:

    EKF path (Fix A — kill pre-filter lag):
        offset + validity gate  ->  "ekf_dists" (fed to EKF innovation)
        NO median, NO EMA. The EKF already runs a chi^2 gate, adaptive R,
        NLOS muting, and a per-anchor 1-D range Kalman (Item B) — stacking
        a 3-sample median on top only added ~100 ms of lag that smeared the
        tight-coupling innovation.

    Visualisation path:
        offset + gate -> 7-sample median -> variable-rate EMA -> "filtered"

    Both paths share the per-anchor AnchorQualityTracker (NLOS weights).

AnchorQualityTracker:
    Per-anchor NLOS detection via rolling variance + innovation scoring.

IMUPreprocessor:
    Quaternion normalization guard.
"""

import numpy as np
from collections import deque

from kuru_method.asynchronous_stream.config import UWB_HZ

# Visualisation-path median window expressed in seconds and translated into
# samples via RATE_PROFILE.UWB_HZ (Priority 1). 0.7 s @ ~50 Hz = 35 samples,
# matching the previous hardcoded value while staying invariant to rate
# changes.
_VIS_SPIKE_WINDOW_S = 0.70
# AnchorQualityTracker rolling-variance window. 1.5 s of UWB samples is long
# enough to characterise an anchor's noise floor without smoothing across a
# whole writing stroke.
_QUALITY_WINDOW_S   = 1.50


# -------------------------------------------------------------------------
#  PER-ANCHOR QUALITY TRACKER
#  Tracks two independent NLOS indicators per anchor:
#    1. Recent ranging variance  ->  high sigma^2 = multipath / NLOS
#    2. Innovation (measured vs. predicted)  ->  large jump = outlier
# -------------------------------------------------------------------------
class AnchorQualityTracker:
    """
    Per-anchor NLOS quality (Priority 3 — residual-based variance).

    The previous design scored an anchor by the variance of its raw range
    over a rolling window. That penalised fast strokes the same way it
    penalised genuine multipath: a clean anchor watching a 1 m/s pen
    looked just as "noisy" as a flapping anchor on a stationary marker.

    The fix is to compute variance over the *residual* `r = raw − pred`,
    where `pred = ‖p_post − a_i‖` from the most recent EKF posterior.
    Real motion shows up symmetrically in raw and pred and cancels in the
    residual, while genuine multipath / NLOS does not.

    During cold-start (before the EKF has a posterior) we fall back to
    raw-range variance — labelled in `last_used_metric` so verifiers can
    audit the regime.

    Returns a scalar weight in [_MIN_WEIGHT, 1.0]:
        1.0  ->  clean LOS, low residual variance, small innovation
        0.05 ->  NLOS suspect, high residual variance or large jump
    """

    # Tuning constants
    # _VAR_SCALE retuned to residual scale (LOS residual sigma ~ 0.05 m
    # gives variance ~ 0.0025 m^2 -> weight ~ 0.77; multipath bursts at
    # sigma ~ 0.20 m give variance ~ 0.04 -> weight ~ 0.17).
    _VAR_SCALE    = 120.0
    _INNOV_SIGMA  = 0.08   # innovation sigma (m); half-weight at |innov|=8 cm
    _MIN_WEIGHT   = 0.05   # floor — near-zero gain for chronically bad anchors

    def __init__(self, window: int | None = None):
        if window is None:
            window = max(8, int(round(_QUALITY_WINDOW_S * UWB_HZ)))
        # Two parallel windows: one for residuals (used when we have a
        # prediction), one for raw (cold-start fallback). Same length.
        self._resid_buf = deque(maxlen=window)
        self._raw_buf   = deque(maxlen=window)
        self._predicted = None          # latest predicted distance (m)
        # Diagnostic — which metric drove the last weight() call.
        self.last_used_metric = 'cold_start_raw'

    # -- public API --------------------------------------------------------

    def push(self, distance: float) -> None:
        """Push a fresh raw range. Residual is appended too iff we have
        a prediction; otherwise only raw is recorded."""
        self._raw_buf.append(float(distance))
        if self._predicted is not None:
            self._resid_buf.append(float(distance) - float(self._predicted))

    def set_prediction(self, dist: float) -> None:
        self._predicted = float(dist) if np.isfinite(dist) else None

    def weight(self, raw_dist: float) -> float:
        """Return combined quality weight for the current raw measurement."""
        var_w   = self._variance_weight()
        innov_w = self._innovation_weight(raw_dist)
        combined = float(np.sqrt(var_w * innov_w))
        return max(self._MIN_WEIGHT, min(1.0, combined))

    # -- internals ---------------------------------------------------------

    def _variance_weight(self) -> float:
        # Prefer residual variance (motion cancels). Fall back to raw
        # variance only during cold-start when no prediction is available.
        if len(self._resid_buf) >= 4:
            var = float(np.var(self._resid_buf))
            self.last_used_metric = 'residual'
        elif len(self._raw_buf) >= 4:
            var = float(np.var(self._raw_buf))
            self.last_used_metric = 'cold_start_raw'
        else:
            self.last_used_metric = 'insufficient'
            return 1.0
        return 1.0 / (1.0 + self._VAR_SCALE * var)

    def _innovation_weight(self, raw_dist: float) -> float:
        if self._predicted is None:
            return 1.0
        innov = abs(raw_dist - self._predicted)
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
    _STEP_THR  = 0.024   # Distance change (m) that triggers fast mode

    def __init__(self, spike_window: int | None = None,
                 max_range: float = 6.0,
                 offsets: tuple = (0.0, 0.0, 0.0, 0.0)):
        if spike_window is None:
            spike_window = max(5, int(round(_VIS_SPIKE_WINDOW_S * UWB_HZ)))
        self.max_range = max_range
        self.n         = 4
        self.offsets   = offsets

        self._med_bufs = [deque(maxlen=spike_window) for _ in range(self.n)]
        self._ema      = [None] * self.n
        self._last     = [0.0]  * self.n
        self._trackers = [AnchorQualityTracker() for _ in range(self.n)]

        # Last valid gated value (per-anchor) — replayed when a raw sample
        # fails the validity check. No median buffer on the EKF path anymore.
        self._last_ekf = [0.0] * self.n

    # -- main entry point --------------------------------------------------

    def process(self, d0: float, d1: float, d2: float, d3: float):
        """
        Returns:
            filtered  : tuple(float x 4)  — offset + median + EMA  (for VIS only)
            weights   : tuple(float x 4)  — per-anchor quality [0.1, 1.0]
            ekf_dists : tuple(float x 4)  — offset + gate only, NO median, NO EMA
                                            (fed to EKF; innovation stays lag-free)

        The third returned tuple is what the EKF should consume. The EKF has
        its own Mahalanobis gate, adaptive R, NLOS muting, and per-anchor
        1-D range Kalman (Item B) — pre-smoothing here only added lag that
        smeared the tight-coupling innovation across ~100 ms of sensor history.
        """
        filtered  = []
        weights   = []
        ekf_dists = []

        for i, raw in enumerate([d0, d1, d2, d3]):
            # -- Stage 1: validity gate (shared) ---------------------------
            raw_with_offset = raw + self.offsets[i]
            valid = (np.isfinite(raw_with_offset)
                     and 0.05 < raw_with_offset <= self.max_range)
            if valid:
                cooked = float(raw_with_offset)
                self._last_ekf[i] = cooked          # cache for VIS hold-last
            else:
                cooked = float('nan')               # Priority 2: NO replay

            # -- EKF path: emit NaN on invalid (Priority 2) ----------------
            # The EKF must see missing data explicitly so it can skip the
            # update (no innovation, no NIS, no Joseph step). The per-anchor
            # range Kalman handles NaN by running predict only.
            ekf_dists.append(cooked)

            # -- Vis path: hold-last on invalid is fine (display only) -----
            vis_val = cooked if valid else self._last_ekf[i]
            self._med_bufs[i].append(vis_val)
            median_val = float(np.median(self._med_bufs[i]))

            if self._ema[i] is None:
                self._ema[i] = median_val
            else:
                step  = abs(median_val - self._ema[i])
                alpha = self._EMA_FAST if step > self._STEP_THR else self._EMA_BASE
                self._ema[i] = alpha * median_val + (1.0 - alpha) * self._ema[i]

            smooth = self._ema[i]
            self._last[i] = smooth

            # -- NLOS quality scoring --------------------------------------
            # On dropout, neither push nor weight uses NaN — the tracker
            # simply does not advance this cycle, and the previous weight
            # is reused. The EKF will skip this anchor's update anyway.
            if valid:
                self._trackers[i].push(median_val)
                w = self._trackers[i].weight(cooked)
            else:
                w = self._trackers[i].weight(self._last_ekf[i])

            filtered.append(smooth)
            weights.append(w)

        return tuple(filtered), tuple(weights), tuple(ekf_dists)

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
