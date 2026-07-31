"""
Module 3 - Time Alignment Layer

Merges the independently-arriving IMU and UWB event streams into one buffer
ordered by hardware timestamp, and reports how well the sensors are actually
keeping time.

This module only reorders and measures; it never transforms event payloads.

The two sensors arrive interleaved over one serial link but are sampled by
different hardware at different rates, so a batch read can contain events whose
timestamps overlap the previous batch. Sorting here is what lets the fusion
stage assume monotonic time.

Live use is the ingest path only:

    aligner.add_events(normalizer.normalize(packets))
    for event in aligner.get_all_sorted():
        ...
    aligner.clear()

The nearest-neighbour queries and get_timing_profile() are diagnostics, used by
timing analysis and this module's self-test rather than the live pipeline; the
ESKF does its own interpolation against its own state history.

Run directly to check sensor timing against live hardware:
    python -m background.pipelines.cleaner.time_alignment
"""

import statistics
from collections import deque

MICROSECONDS_PER_MILLISECOND = 1000

# Rolling history depths for interval statistics, sized to cover a few seconds
# at each sensor's nominal rate.
_IMU_HISTORY_SAMPLES = 500
_UWB_HISTORY_SAMPLES = 200


class TimeAlignLayer:
    """Timestamp-ordered buffer over the merged sensor streams."""

    def __init__(self, buffer_size: int = 2000):
        """
        Args:
            buffer_size: Maximum events retained; oldest are evicted first.
        """

        self._buffer: list[dict] = []
        self._buffer_size = buffer_size

        self._imu_ts_history: deque[int] = deque(maxlen=_IMU_HISTORY_SAMPLES)
        self._uwb_ts_history: deque[int] = deque(maxlen=_UWB_HISTORY_SAMPLES)

    # -------------------------------------------------------------------------
    # Ingestion
    # -------------------------------------------------------------------------

    def add_events(self, events: list[dict]):
        """
        Merge a batch of normalized events into the timestamp-ordered buffer.

        Events without a timestamp are skipped: they cannot be placed in the
        ordering, and passing them on would break the monotonic-time guarantee
        that downstream stages depend on.
        """

        for event in events:
            ts = event.get('ts_hw')
            if ts is None:
                continue

            if event['sensor'] == 'IMU':
                self._imu_ts_history.append(ts)
            elif event['sensor'] == 'UWB':
                self._uwb_ts_history.append(ts)

            self._buffer.append(event)

        # Sorting the whole buffer looks wasteful next to merging the new batch
        # into the already-ordered remainder, but it is measurably faster here:
        #
        # Benchmarked before changing it - do not "optimize" into a manual merge.
        #
        self._buffer.sort(key=lambda event: event['ts_hw'])

        if len(self._buffer) > self._buffer_size:
            self._buffer = self._buffer[-self._buffer_size:]

    def clear(self):
        self._buffer.clear()
        self._imu_ts_history.clear()
        self._uwb_ts_history.clear()

    # -------------------------------------------------------------------------
    # Queries
    # -------------------------------------------------------------------------

    def get_all_sorted(self, sensor: str | None = None) -> list[dict]:
        """Return buffered events in timestamp order, optionally one sensor only."""

        if sensor:
            return [event for event in self._buffer if event['sensor'] == sensor]
        return list(self._buffer)

    def get_nearest_imu_before(self, ts: int) -> dict | None:
        """Return the latest IMU event at or before ts, or None if there is none."""

        result = None
        for event in self._buffer:
            if event['sensor'] != 'IMU':
                continue
            if event['ts_hw'] <= ts:
                result = event
            else:
                # Buffer is ascending, so the first later event ends the search.
                break
        return result

    def get_nearest_imu_after(self, ts: int) -> dict | None:
        """Return the earliest IMU event strictly after ts, or None."""

        for event in self._buffer:
            if event['sensor'] == 'IMU' and event['ts_hw'] > ts:
                return event
        return None

    def get_imu_between(self, ts_start: int, ts_end: int) -> list[dict]:
        """Return IMU events with ts_hw inside [ts_start, ts_end], inclusive."""

        return [
            event for event in self._buffer
            if event['sensor'] == 'IMU' and ts_start <= event['ts_hw'] <= ts_end
        ]

    # -------------------------------------------------------------------------
    # Timing Analytics
    # -------------------------------------------------------------------------

    def get_timing_profile(self) -> dict:
        """
        Summarize per-sensor sample intervals and UWB-to-IMU arrival offsets.

        Interval spread reveals jitter and dropouts; the max interval is the
        worst observed gap. Values are in the hardware timestamp unit
        (microseconds) despite the historical _ms key suffixes, which are kept
        so existing dashboards and logs continue to resolve.
        """

        profile = {}
        profile.update(self._interval_stats(list(self._imu_ts_history), 'imu'))
        profile.update(self._interval_stats(list(self._uwb_ts_history), 'uwb'))
        profile['uwb_to_imu_delays'] = self._uwb_to_imu_delays()
        return profile

    @staticmethod
    def _interval_stats(timestamps: list[int], prefix: str) -> dict:
        """Mean, spread and extremes of consecutive timestamp deltas."""

        stats = {}

        if len(timestamps) >= 2:
            intervals = [
                timestamps[i + 1] - timestamps[i]
                for i in range(len(timestamps) - 1)
            ]
            stats[f'{prefix}_interval_mean_ms'] = round(statistics.mean(intervals), 2)
            stats[f'{prefix}_interval_std_ms'] = (
                round(statistics.stdev(intervals), 2) if len(intervals) > 1 else 0.0
            )
            stats[f'{prefix}_interval_min_ms'] = min(intervals)
            stats[f'{prefix}_interval_max_ms'] = max(intervals)
        else:
            stats[f'{prefix}_interval_mean_ms'] = None
            stats[f'{prefix}_interval_std_ms'] = None
            stats[f'{prefix}_interval_min_ms'] = None
            stats[f'{prefix}_interval_max_ms'] = None

        stats[f'{prefix}_sample_count'] = len(timestamps)
        return stats

    def _uwb_to_imu_delays(self) -> list[dict]:
        """
        Per-UWB-event offsets to the bracketing IMU samples.

        Shows how far each UWB fix sits from the IMU samples around it, which
        bounds the interpolation error when the two streams are combined.
        """

        records = []
        for uwb_event in self.get_all_sorted(sensor='UWB'):
            ts = uwb_event['ts_hw']
            before = self.get_nearest_imu_before(ts)
            after = self.get_nearest_imu_after(ts)

            records.append({
                'uwb_ts': ts,
                'uwb_packet_id': uwb_event['packet_id'],
                'nearest_imu_before_ts': before['ts_hw'] if before else None,
                'nearest_imu_after_ts': after['ts_hw'] if after else None,
                'delay_before_ms': (ts - before['ts_hw']) if before else None,
                'delay_after_ms': (after['ts_hw'] - ts) if after else None,
            })
        return records


# =============================================================================
# MODULE TESTING
#   Live timing profile: per-sensor sample intervals, jitter, worst gaps, and
#   how far each UWB fix sits from its bracketing IMU samples.
#
#   That last figure bounds the interpolation error the fusion stage inherits,
#   so check here before blaming fusion for placement problems.
#
#   Run:  python -m background.pipelines.cleaner.time_alignment
# =============================================================================
if __name__ == '__main__':
    import os
    import time

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.config import cfg

    PROFILE_UPDATE_RATE_S = 1.0

    def _render_interval_block(profile: dict, sensor: str) -> None:
        prefix = sensor.lower()
        sample_count = profile[f'{prefix}_sample_count']
        print(f"[{sensor} Timing] Samples Analyzed: {sample_count}")
        if sample_count > 1:
            print(f"  Mean interval : {profile[f'{prefix}_interval_mean_ms']} us")
            print(f"  Std (jitter)  : {profile[f'{prefix}_interval_std_ms']} us")
            print(f"  Min interval  : {profile[f'{prefix}_interval_min_ms']} us")
            print(f"  Max gap       : {profile[f'{prefix}_interval_max_ms']} us")

    def _render_timing_profile(profile: dict) -> None:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"========= LIVE TIMING PROFILE ({PROFILE_UPDATE_RATE_S}s update) =========")

        _render_interval_block(profile, 'IMU')
        print()
        _render_interval_block(profile, 'UWB')

        delays = profile.get('uwb_to_imu_delays')
        if delays:
            latest = delays[-1]
            print("\n[Latest UWB-to-IMU Sync]")
            print(f"  UWB Pkt ID: {latest['uwb_packet_id']} @ TS_HW: {latest['uwb_ts']}")
            print(f"  IMU Before Delay: {latest['delay_before_ms']} us")
            print(f"  IMU After Delay : {latest['delay_after_ms']} us")

        print("========================================================")

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=2000)

    print("=" * 60)
    print(f"  PolyCast Time Alignment Test: {cfg.serial.port}")
    print("  Collecting data and building timing profiles...")
    print("=" * 60)

    last_render = time.time()

    try:
        while True:
            packets = streamer.read_new_packets()
            if packets:
                aligner.add_events(normalizer.normalize(packets))

            now = time.time()
            if now - last_render >= PROFILE_UPDATE_RATE_S:
                _render_timing_profile(aligner.get_timing_profile())
                last_render = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping Streamer...")
        streamer.close()
        print("Done.")
