"""
Internal Helper (uwb/range) - Per-Anchor Range Kalman Pre-Filter

Applies a Kalman filter to each anchor's measured range before
trilateration. Module 5 (`range.py`) owns the ranging stage and delegates 
filtering here. 

A separate constant-velocity Kalman filter runs per anchor over the state

    s = [r, r_dot]

where `r` is range (m) and `r_dot` is range rate (m/s). Process noise is
modeled on `r_dot`, allowing the filter to follow smooth range changes while
treating abrupt deviations as measurement noise.

Why not a median or EMA?

    Unlike a median or EMA, which smooths by averaging and can blur real motion, 
    the Kalman filter predicts how the range should change and only corrects measurements 
    that don't match that prediction. This reduces UWB noise while preserving fast movements 
    needed for accurate trilateration.

NOTE: Rejected measurements are not ignored. The filter still predicts forward,
allowing the estimate to coast at its last estimated velocity until a new
measurement is accepted, avoiding artificial jumps.

Reference:
    Zou, L. et al. (2023). *An Improved UWB/IMU Tightly Coupled Indoor
    Positioning Algorithm*, Section 3.1.

Usage:
    tracker = RangeTracker()
    filtered = tracker.update(anchor_index, calibrated_range, dt_s)
    coasted = tracker.predict_only(anchor_index, dt_s)
"""

from background.pipelines.config import cfg

# Initial range-rate variance. Kept large because the first measurement
# reveals the range, but not how fast it is changing.
_INITIAL_RATE_VARIANCE = 1.0


class _AnchorState:
    """
    Two-state filter state for one anchor: range, range-rate, and covariance.

    The covariance is symmetric, so three scalars describe it fully. They are
    held unpacked rather than as a matrix because this runs once per anchor per
    sample and never needs matrix algebra.
    """

    __slots__ = (
        'range_m',
        'range_rate_ms',
        'range_variance',
        'range_rate_covariance',
        'rate_variance',
    )

    def __init__(self, range_m: float, range_variance: float):
        self.range_m = range_m
        self.range_rate_ms = 0.0

        self.range_variance = range_variance
        self.range_rate_covariance = 0.0
        self.rate_variance = _INITIAL_RATE_VARIANCE


class RangeTracker:
    """Holds one independent constant-velocity range filter per anchor."""

    def __init__(self, anchor_count: int | None = None):
        self._anchor_count = anchor_count or cfg.uwb.num_anchors
        self._states: list[_AnchorState | None] = [None] * self._anchor_count

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def update(self, index: int, measured_range: float, dt_s: float) -> float:
        """
        Fold one accepted range into the anchor's filter and return the estimate.

        The first sample for an anchor seeds the filter and is returned
        unchanged, since there is no motion history to correct it against.
        """

        state = self._states[index]
        if state is None:
            self._states[index] = _AnchorState(
                float(measured_range), cfg.uwb.range_kf_sigma_meas_m ** 2
            )
            return float(measured_range)

        self._predict(state, dt_s)
        self._correct(state, float(measured_range))
        return state.range_m

    def predict_only(self, index: int, dt_s: float) -> float | None:
        """
        Advance an anchor without a measurement, for a rejected sample.

        Returns None when the anchor has never been seeded, so the caller can
        fall back to whatever it uses before the filter has any history.
        """

        state = self._states[index]
        if state is None:
            return None

        self._predict(state, dt_s)
        return state.range_m

    def reseed(self, index: int, measured_range: float):
        """
        Restart one anchor's filter at a known range.

        Called when the jump gate gives up and resyncs: the filter's history
        describes a range the tag has already left, so carrying it forward would
        fight every subsequent sample.
        """

        self._states[index] = _AnchorState(
            float(measured_range), cfg.uwb.range_kf_sigma_meas_m ** 2
        )

    def reset(self):
        """Clear every anchor's filter state."""

        self._states = [None] * self._anchor_count

    # -------------------------------------------------------------------------
    # Filter steps
    # -------------------------------------------------------------------------

    @staticmethod
    def _predict(state: _AnchorState, dt_s: float):
        """Advance the constant-velocity model and grow the covariance."""

        state.range_m += state.range_rate_ms * dt_s

        # Predict covariance (P = FPF' + Q) expanded for the 2×2 model. Written
        # directly to avoid NumPy overhead; all terms use the pre-update covariance.
        propagated_range_variance = (
            state.range_variance
            + dt_s * (2.0 * state.range_rate_covariance + dt_s * state.rate_variance)
        )
        propagated_rate_covariance = (
            state.range_rate_covariance
            + dt_s * state.rate_variance
        )

        # Continuous white-noise-acceleration Q: uncertainty enters through the
        # rate, then integrates down into the range over the interval.
        rate_noise_variance = cfg.uwb.range_kf_sigma_rate_ms ** 2

        state.range_variance = (
            propagated_range_variance
            + rate_noise_variance * dt_s ** 3 / 3.0
        )
        state.range_rate_covariance = (
            propagated_rate_covariance
            + rate_noise_variance * dt_s ** 2 / 2.0
        )
        state.rate_variance += rate_noise_variance * dt_s

    @staticmethod
    def _correct(state: _AnchorState, measured_range: float):
        """Pull the state toward one range measurement."""

        innovation = measured_range - state.range_m
        innovation_variance = (
            state.range_variance
            + cfg.uwb.range_kf_sigma_meas_m ** 2
        )
        if innovation_variance <= 0.0:
            return

        range_gain = state.range_variance / innovation_variance
        rate_gain = state.range_rate_covariance / innovation_variance

        state.range_m += range_gain * innovation
        state.range_rate_ms += rate_gain * innovation

        # Covariance update: P = (I - KH)P for H = [1, 0]. 
        # Joseph form is unnecessary for this scalar measurement. Update order 
        # matters since each step depends on the previous covariance values.
        state.rate_variance -= rate_gain * state.range_rate_covariance
        state.range_rate_covariance -= rate_gain * state.range_variance
        state.range_variance -= range_gain * state.range_variance


# =============================================================================
# MODULE TESTING
#   Replays a synthetic range that moves at a known rate under added noise and
#   reports how much of that noise the filter removes, plus the lag it costs.
#
#   A filter that scores a large noise reduction with several samples of lag is
#   mistuned toward smoothing: raise range_kf_sigma_rate_ms until it tracks.
#
#   Run:  python -m background.pipelines.preprocess.uwb._range_kf
# =============================================================================
if __name__ == '__main__':
    import math
    import random

    SAMPLE_COUNT = 600
    DT_S = 1.0 / cfg.uwb.rate_hz
    NOISE_SIGMA_M = 0.05

    # Samples discarded before scoring, so the seeding transient does not count
    # against the filter: a freshly seeded anchor starts with no rate estimate
    # and needs a few samples before it is tracking anything.
    WARMUP_SAMPLES = 50

    # Widest shift searched when estimating lag. Beyond a quarter of the
    # oscillation period the best-fit alignment stops being meaningful.
    MAX_LAG_SAMPLES = 12

    random.seed(7)
    tracker = RangeTracker(anchor_count=1)

    true_ranges = []
    measured_ranges = []
    filtered_ranges = []

    for step in range(SAMPLE_COUNT):
        elapsed_s = step * DT_S
        # A range that oscillates the way one does while a letter is written.
        true_range = 1.20 + 0.06 * math.sin(2.0 * math.pi * 0.8 * elapsed_s)
        noisy_range = true_range + random.gauss(0.0, NOISE_SIGMA_M)

        true_ranges.append(true_range)
        measured_ranges.append(noisy_range)
        filtered_ranges.append(tracker.update(0, noisy_range, DT_S))

    def rms_error(values, reference):
        squared_errors = [
            (value - expected) ** 2 for value, expected in zip(values, reference)
        ]
        return math.sqrt(sum(squared_errors) / len(squared_errors))

    scored = slice(WARMUP_SAMPLES, None)
    raw_rms = rms_error(measured_ranges[scored], true_ranges[scored])
    filtered_rms = rms_error(filtered_ranges[scored], true_ranges[scored])

    # Re-score the filtered signal against progressively earlier truth. The
    # shift that fits best is the filter's lag.
    best_lag, best_rms = 0, filtered_rms
    for lag in range(1, MAX_LAG_SAMPLES):
        shifted = rms_error(
            filtered_ranges[WARMUP_SAMPLES + lag:],
            true_ranges[WARMUP_SAMPLES:len(true_ranges) - lag],
        )
        if shifted < best_rms:
            best_lag, best_rms = lag, shifted

    print('=' * 60)
    print('  [TEST] MODULE 5b: Per-anchor range Kalman pre-filter')
    print('=' * 60)
    print(f'  samples              : {SAMPLE_COUNT} @ {cfg.uwb.rate_hz:.0f} Hz')
    print(f'  injected noise sigma : {NOISE_SIGMA_M * 100:.1f} cm')
    print(f'  sigma_rate / sigma_z : {cfg.uwb.range_kf_sigma_rate_ms} m/s'
          f' / {cfg.uwb.range_kf_sigma_meas_m} m')
    print('-' * 60)
    print(f'  raw RMS error        : {raw_rms * 100:6.2f} cm')
    print(f'  filtered RMS error   : {filtered_rms * 100:6.2f} cm')
    print(f'  noise removed        : {100 * (1 - filtered_rms / raw_rms):6.1f} %')
    print(f'  best-fit lag         : {best_lag} samples'
          f' ({best_lag * DT_S * 1000:.0f} ms)')
    print('=' * 60)
