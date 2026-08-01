"""
Module 5 - UWB Range Preprocessor

Cleans raw anchor distances before trilateration tries to solve a position from
them. A single bad range corrupts the whole geometric solve, so each anchor is
filtered independently and flagged rather than silently trusted.

Process Flow:
    1. Blind-spot reject   drop physically impossible readings outright
    2. Offset calibration  correct the antenna delay measured for this anchor
    3. Sanity check        confirm the range is within the room's plausible bounds
    4. Jump detection      reject impossible movement between samples
    5. Anti-lockout        force a resync if the jump gate has held too long
    6. Median filter       suppress isolated spikes
    7. Adaptive EMA        smooth, adapting to the real inter-sample interval

Steps 4 and 5 are in tension by design: the jump gate rejects NLOS spikes, but
without the anti-lockout escape a genuine fast move would be rejected forever
because every new sample is measured against a frozen estimate.

Two range series come out of this stage, and they are not interchangeable:

    solver_dists  steps 1-5 only. The trilateration solver consumes this.
    clean_dists   steps 1-7, i.e. additionally smoothed. Display and
                  diagnostics only.

Smoothing spans a few hundred milliseconds, which is also how long a single
handwriting stroke lasts, so a smoothed range has had the pen motion averaged
out of it. Solving positions from that collapses the stroke toward a line.
Rejection gates are shared because they discard bad samples outright rather
than blurring good ones.

Input:  normalized UWB events from "normalizer" module
Output: {'sensor', 'ts_hw', 'packet_id', 'raw_dists', 'clean_dists',
         'solver_dists', 'valid_mask', 'outlier_flags'}

Usage (Import as stage or run directly to trace normalized events):
    python -m background.pipelines.preprocess.uwb.range
"""

import math
import statistics
from collections import deque

from background.pipelines.config import cfg

MICROSECONDS_PER_SECOND = 1_000_000

# Ranges at or below this are the hardware's blind-spot signal, not a distance:
# a blocked anchor reports 0.0 or a near-zero value rather than failing.
_BLIND_SPOT_RANGE_M = 0.05

# Emitted when an anchor is rejected before any history exists to fall back on.
_NO_ESTIMATE_SENTINEL = -1.0

# Consecutive jump rejections tolerated before the estimate is forced to resync.
_MAX_CONSECUTIVE_JUMPS = 5

# Median-to-estimate gap that counts as real movement rather than noise. 
# Above it the smoothing opens up so the filter catches up instead of lagging.
_FAST_MOVEMENT_DELTA_M = 0.10
_FAST_MOVEMENT_ALPHA_GAIN = 3.0


class UWBRangePreprocessor:
    """Per-anchor range cleaner holding independent filter state for each anchor."""

    def __init__(self, offsets: tuple = (0.0, 0.0, 0.0, 0.0)):
        anchor_count = cfg.uwb.num_anchors

        # Antenna-delay calibration, measured per anchor.
        self.offsets = offsets

        self._history: list[deque] = [
            deque(maxlen=cfg.uwb.median_window) for _ in range(anchor_count)
        ]
        self._ema: list[float | None] = [None] * anchor_count
        self._prev_clean: list[float | None] = [None] * anchor_count
        self._jump_count = [0] * anchor_count

        self.max_jumps = _MAX_CONSECUTIVE_JUMPS
        self._prev_ts: int | None = None

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def feed(self, events: list[dict]) -> list[dict]:
        """Process a batch, skipping non-UWB events and unusable samples."""

        cleaned = []
        for event in events:
            if event.get('sensor') != 'UWB':
                continue
            result = self.process_one(event)
            if result:
                cleaned.append(result)
        return cleaned

    def process_one(self, event: dict) -> dict | None:
        """
        Clean one UWB reading, filtering every anchor independently.

        Returns None when the event lacks ranges or carries an unexpected anchor
        count, since the per-anchor state arrays are sized from config.
        """

        ts = event.get('ts_hw')
        distances = event.get('dists')

        if ts is None or distances is None:
            return None
        if len(distances) != cfg.uwb.num_anchors:
            return None

        dt_s = self._elapsed_seconds(ts)
        raw_distances = tuple(distances)

        clean_distances = []
        solver_distances = []
        valid_mask = []
        outlier_flags = []

        for index, raw_distance in enumerate(raw_distances):
            valid, outlier, clean_value, solver_value = self._process_anchor(
                index, raw_distance, dt_s
            )
            clean_distances.append(clean_value)
            solver_distances.append(solver_value)
            valid_mask.append(valid)
            outlier_flags.append(outlier)

        return {
            'sensor': 'UWB',
            'ts_hw': ts,
            'packet_id': event.get('packet_id'),
            'raw_dists': raw_distances,
            'clean_dists': tuple(clean_distances),
            'solver_dists': tuple(solver_distances),
            'valid_mask': tuple(valid_mask),
            'outlier_flags': tuple(outlier_flags),
        }

    def reset(self):
        """Clear all per-anchor filter state."""

        anchor_count = cfg.uwb.num_anchors
        self._history = [deque(maxlen=cfg.uwb.median_window) for _ in range(anchor_count)]
        self._ema = [None] * anchor_count
        self._prev_clean = [None] * anchor_count
        self._prev_ts = None

    # -------------------------------------------------------------------------
    # Per-anchor filtering
    # -------------------------------------------------------------------------

    def _elapsed_seconds(self, ts: int) -> float:
        """Seconds since the previous reading, falling back to the nominal rate."""

        nominal_dt_s = 1.0 / cfg.uwb.rate_hz

        if self._prev_ts is None:
            self._prev_ts = ts
            return nominal_dt_s

        dt_s = (ts - self._prev_ts) / MICROSECONDS_PER_SECOND
        self._prev_ts = ts
        return dt_s if dt_s > 0 else nominal_dt_s

    def _process_anchor(self, index: int, raw_distance, dt_s: float = None):
        """
        Filter one anchor's range.

        Returns (valid, outlier, clean_value, solver_value):
            valid        the reading is trustworthy enough for trilateration
            outlier      this sample was suspicious; the values are a fallback
            clean_value  median+EMA smoothed range, for display and diagnostics
            solver_value calibrated range with no median and no EMA, for the
                         trilateration solver

        The two outputs differ only on the accepted path. Smoothing spans
        hundreds of milliseconds, which is the same timescale as a handwriting
        stroke, so feeding it to the solver removes the motion being measured.
        Rejection gates apply to both - they drop bad samples rather than
        blurring good ones.
        """

        if self._is_blind_spot(raw_distance):
            # Hold the last good estimate rather than letting a blocked anchor
            # poison the median buffer.
            held = self._ema[index]
            held = held if held is not None else _NO_ESTIMATE_SENTINEL
            return False, True, held, held

        calibrated = raw_distance + self.offsets[index]

        within_room = (
            cfg.pipeline.uwb_min_range_m <= calibrated <= cfg.pipeline.uwb_max_range_m
        )
        jumped = self._detect_jump(index, calibrated, within_room)

        if jumped:
            jumped = self._register_jump(index, calibrated)
        else:
            self._jump_count[index] = 0

        is_outlier = (not within_room) or jumped
        if is_outlier:
            # Every fallback branch already reports an unsmoothed value, so the
            # display and solver signals agree here.
            valid, outlier, fallback = self._fallback_value(index, calibrated, within_room)
            return valid, outlier, fallback, fallback

        clean_value = self._smooth(index, calibrated, dt_s)
        self._prev_clean[index] = clean_value
        return True, False, clean_value, calibrated

    @staticmethod
    def _is_blind_spot(raw_distance) -> bool:
        """True for readings the hardware emits when it cannot see the anchor."""

        return (
            raw_distance is None
            or math.isnan(raw_distance)
            or raw_distance <= _BLIND_SPOT_RANGE_M
        )

    def _detect_jump(self, index: int, calibrated: float, within_room: bool) -> bool:
        """True when the range moved further than the tag physically could."""

        if not within_room or self._prev_clean[index] is None:
            return False
        return abs(calibrated - self._prev_clean[index]) > cfg.uwb.max_range_jump_m

    def _register_jump(self, index: int, calibrated: float) -> bool:
        """
        Count a jump rejection, forcing a resync once the gate has held too long.

        Without this escape a genuine fast move would be rejected indefinitely:
        each new sample is compared against an estimate that never advances, so
        the gate would keep firing forever. Returns whether the jump still stands.
        """

        self._jump_count[index] += 1
        if self._jump_count[index] < self.max_jumps:
            return True

        self._jump_count[index] = 0
        self._ema[index] = calibrated
        self._history[index].clear()
        return False

    def _fallback_value(self, index: int, calibrated: float, within_room: bool):
        """
        Choose what to report for a rejected sample.

        A first-ever sample that trips the jump gate is accepted anyway: with no
        history there is nothing to have jumped from, and rejecting it would
        leave the anchor unseeded forever.
        """

        if self._ema[index] is not None:
            return False, True, self._ema[index]

        if within_room:
            return True, False, calibrated

        return False, True, calibrated

    def _smooth(self, index: int, calibrated: float, dt_s: float) -> float:
        """
        Median-filter then EMA-smooth an accepted range.

        The EMA constant is derived from the actual inter-sample interval, so
        smoothing stays consistent despite serial and UWB jitter rather than
        varying with how fast samples happen to arrive.
        """

        self._history[index].append(calibrated)
        median_value = statistics.median(self._history[index])

        interval_s = dt_s if dt_s is not None else 1.0 / cfg.uwb.rate_hz
        base_alpha = 1.0 - math.exp(-interval_s / cfg.uwb.range_tau_s)

        if self._ema[index] is None:
            self._ema[index] = median_value
            return self._ema[index]

        if abs(median_value - self._ema[index]) > _FAST_MOVEMENT_DELTA_M:
            alpha = min(base_alpha * _FAST_MOVEMENT_ALPHA_GAIN, 1.0)
        else:
            alpha = base_alpha

        self._ema[index] = alpha * median_value + (1.0 - alpha) * self._ema[index]
        return self._ema[index]


# =============================================================================
# MODULE TESTING
#   Live range dashboard: raw versus filtered distance per anchor, with rejected
#   spikes marked. Exports a CSV and a 2x2 per-anchor plot on Ctrl+C.
#
#   Watch for an anchor whose raw trace is noisy while the others are clean -
#   that is usually a blocked or badly placed anchor, not a filter problem.
#
#   Run:  python -m background.pipelines.preprocess.uwb.range
# =============================================================================
if __name__ == '__main__':
    import csv
    import os
    import time

    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput

    DISPLAY_RATE_S = 0.2
    CSV_FILENAME = 'uwb_range_report.csv'
    PLOT_FILENAME = 'uwb_range_report.png'

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    preprocessor = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)

    print("=" * 60)
    print(f"  UWB Data Logger & Reporter: {cfg.serial.port}")
    print(f"  Active Calibration Offsets: {cfg.uwb.range_offsets_m}")
    print("  Collecting data silently... Press Ctrl+C to stop and generate reports.")
    print("=" * 60)

    event_log = []
    last_print_time = 0.0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                cleaned = preprocessor.feed(normalizer.normalize(raw_packets))
                if cleaned:
                    event_log.extend(cleaned)

                    now = time.time()
                    if now - last_print_time >= DISPLAY_RATE_S:
                        latest = cleaned[-1]
                        os.system('cls' if os.name == 'nt' else 'clear')
                        print(f"========= LIVE UWB PREPROCESSOR ({DISPLAY_RATE_S}s update) =========")
                        print(f"  Pkt ID     : {latest['packet_id']}")
                        print(f"  TS (hw)    : {latest['ts_hw']}")
                        print("-" * 57)

                        for anchor in range(cfg.uwb.num_anchors):
                            rejected = (
                                not latest['valid_mask'][anchor]
                                or latest['outlier_flags'][anchor]
                            )
                            status = 'OUTLIER/SPIKE' if rejected else 'VALID'
                            print(f"  Anchor {anchor}: "
                                  f"Raw: {latest['raw_dists'][anchor]:6.2f} m  |  "
                                  f"Clean: {latest['clean_dists'][anchor]:6.2f} m  | [{status}]")

                        print("=========================================================")
                        last_print_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected to report. Exiting.")
            raise SystemExit(0)

        print(f"Captured {len(event_log)} UWB events. Generating reports...")

        output = ModuleRunOutput('preprocess/uwb/range')
        output.save_csv(
            CSV_FILENAME,
            [[row['ts_hw'], row['packet_id'],
              *row['raw_dists'], *row['clean_dists'],
              *[int(flag) for flag in row['outlier_flags']]]
             for row in event_log],
            header=['ts_hw', 'packet_id',
                    'raw_0', 'raw_1', 'raw_2', 'raw_3',
                    'clean_0', 'clean_1', 'clean_2', 'clean_3',
                    'outlier_0', 'outlier_1', 'outlier_2', 'outlier_3'],
        )

        print("[PLOT] Rendering visual report...")
        start_ts = event_log[0]['ts_hw']
        seconds = [(row['ts_hw'] - start_ts) / MICROSECONDS_PER_SECOND for row in event_log]

        figure, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
        figure.suptitle('UWB Range Preprocessor: Raw vs. Clean (Spike Rejection)',
                        fontsize=16, fontweight='bold')

        for anchor, axis in enumerate(axes.flatten()):
            raw_values = [row['raw_dists'][anchor] for row in event_log]
            clean_values = [row['clean_dists'][anchor] for row in event_log]
            outliers = [row['outlier_flags'][anchor] for row in event_log]

            axis.plot(seconds, raw_values, label='Raw Distance',
                      color='red', alpha=0.3, linewidth=1.5)
            axis.plot(seconds, clean_values, label='Clean Distance (Median+EMA)',
                      color='blue', linewidth=2)

            spike_times = [seconds[i] for i, flag in enumerate(outliers) if flag]
            spike_values = [raw_values[i] for i, flag in enumerate(outliers) if flag]
            if spike_times:
                axis.scatter(spike_times, spike_values, color='black',
                             marker='x', s=50, label='Rejected Spike')

            axis.set_title(f'Anchor A{anchor}')
            axis.set_ylabel('Distance (Metres)')
            axis.grid(True, linestyle='--', alpha=0.6)
            if anchor == 1:
                axis.legend(loc='upper right')

        axes[1, 0].set_xlabel('Time (Seconds)')
        axes[1, 1].set_xlabel('Time (Seconds)')

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.savefig(output.path(PLOT_FILENAME), dpi=300)
        output.record(PLOT_FILENAME)
        output.finish()
        plt.show()
