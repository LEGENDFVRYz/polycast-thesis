"""
Module 3 — Time Alignment Layer

Responsibility:
    - Ingest normalized events from tv2_normalizer.py
    - Sort events by hardware timestamp (ts_hw)
    - Measure real sensor timing: intervals, jitter, gaps, dropout
    - Build the temporal bridge needed before sensor fusion

This module does NOT transform the data. It only understands time.
Add additional function for interpolation logic in the future sensor fusion.
"""

import statistics
from collections import deque


class TimeAlignLayer:
    """
    Manages a time-sorted buffer of normalized sensor events,
    with timing analytics and nearest-sample queries.

    Typical usage in a live loop:

        aligner = TimeAlignLayer(buffer_size=2000)

        while True:
            packets = streamer.read_new_packets()
            events  = normalizer.normalize(packets)
            aligner.add_events(events)

            # Query (for sensor fusion)
            nearest = aligner.get_nearest_imu_before(some_uwb_ts)
            profile = aligner.get_timing_profile()
    """

    def __init__(self, buffer_size: int = 2000):
        """
        Args:
            buffer_size: Maximum number of events to hold in the rolling
                         sorted buffer. Older events are evicted first.
        """
        self._buffer: list[dict] = []       # sorted by ts_hw ascending
        self._buffer_size = buffer_size

        # Separate ordered queues for per-sensor interval tracking
        self._imu_ts_history: deque[int] = deque(maxlen=500)
        self._uwb_ts_history: deque[int] = deque(maxlen=200)


    # --- Ingestion ---
    def add_events(self, events: list[dict]):
        """
        Add a batch of normalized events. Events are merged into the
        internal sorted buffer by ts_hw.

        Args:
            events: List of normalized event dicts from normalizer.
        """
        for ev in events:
            ts = ev.get('ts_hw')
            
            if ts is None:
                continue    # Skip error events

            # Track per-sensor timestamps for interval
            if ev['sensor'] == 'IMU':
                self._imu_ts_history.append(ts)
            elif ev['sensor'] == 'UWB':
                self._uwb_ts_history.append(ts)

            self._buffer.append(ev)

        # Keep buffer sorted and capped
        self._buffer.sort(key=lambda e: e['ts_hw'])
        if len(self._buffer) > self._buffer_size:
            self._buffer = self._buffer[-self._buffer_size:]

    def clear(self):
        self._buffer.clear()
        self._imu_ts_history.clear()
        self._uwb_ts_history.clear()


    # --- (Helper) Nearest-Neighbor Queries ---
    def get_nearest_imu_before(self, ts: int) -> dict | None:
        """
        Return the IMU event with the largest ts_hw that is still ≤ ts.
        Returns None if no IMU event exists before or at ts.
        """
        result = None
        for ev in self._buffer:
            if ev['sensor'] != 'IMU':
                continue
            if ev['ts_hw'] <= ts:
                result = ev  # Keep overwriting — buffer is sorted ascending
            else:
                break
        return result

    def get_nearest_imu_after(self, ts: int) -> dict | None:
        """
        Return the IMU event with the smallest ts_hw that is strictly > ts.
        Returns None if no IMU event exists after ts.
        """
        for ev in self._buffer:
            if ev['sensor'] == 'IMU' and ev['ts_hw'] > ts:
                return ev
        return None

    def get_imu_between(self, ts_start: int, ts_end: int) -> list[dict]:
        """
        Return all IMU events whose ts_hw is in [ts_start, ts_end] inclusive.
        Returned list is sorted by ts_hw ascending.

        Args:
            ts_start: Lower bound timestamp (inclusive).
            ts_end:   Upper bound timestamp (inclusive).
        """
        return [
            ev for ev in self._buffer
            if ev['sensor'] == 'IMU'
            and ts_start <= ev['ts_hw'] <= ts_end
        ]

    def get_all_sorted(self, sensor: str | None = None) -> list[dict]:
        """
        Return all buffered events sorted by ts_hw.
        Optionally filter by sensor type ('IMU' or 'UWB').
        """
        if sensor:
            return [ev for ev in self._buffer if ev['sensor'] == sensor]
        return list(self._buffer)


    # --- Analytics ---
    def get_timing_profile(self) -> dict:
        """
        Compute and return timing statistics from buffered data.

        Returns a dict with keys:
            imu_interval_mean_ms    — average time between IMU samples
            imu_interval_std_ms     — std dev of that interval (jitter)
            imu_interval_min_ms     — minimum observed interval
            imu_interval_max_ms     — maximum observed interval (worst gap)
            imu_sample_count        — total IMU timestamps were analyzed
            uwb_interval_mean_ms    — average time between UWB readings
            uwb_interval_std_ms     — std dev of UWB interval
            uwb_interval_min_ms     — minimum UWB interval
            uwb_interval_max_ms     — maximum UWB interval (drop packet detector)
            uwb_sample_count        — total UWB timestamps were analyzed
            uwb_to_imu_delays       — specific UWB dict with nearest before/after delays
        """
        profile = {}

        # IMU interval stats
        imu_list = list(self._imu_ts_history)
        if len(imu_list) >= 2:
            imu_diffs = [imu_list[i+1] - imu_list[i] for i in range(len(imu_list)-1)]
            profile['imu_interval_mean_ms'] = round(statistics.mean(imu_diffs), 2)
            profile['imu_interval_std_ms']  = round(statistics.stdev(imu_diffs), 2) if len(imu_diffs) > 1 else 0.0
            profile['imu_interval_min_ms']  = min(imu_diffs)
            profile['imu_interval_max_ms']  = max(imu_diffs)
        else:
            profile['imu_interval_mean_ms'] = None
            profile['imu_interval_std_ms']  = None
            profile['imu_interval_min_ms']  = None
            profile['imu_interval_max_ms']  = None
        profile['imu_sample_count'] = len(imu_list)

        # UWB interval stats
        uwb_list = list(self._uwb_ts_history)
        if len(uwb_list) >= 2:
            uwb_diffs = [uwb_list[i+1] - uwb_list[i] for i in range(len(uwb_list)-1)]
            profile['uwb_interval_mean_ms'] = round(statistics.mean(uwb_diffs), 2)
            profile['uwb_interval_std_ms']  = round(statistics.stdev(uwb_diffs), 2) if len(uwb_diffs) > 1 else 0.0
            profile['uwb_interval_min_ms']  = min(uwb_diffs)
            profile['uwb_interval_max_ms']  = max(uwb_diffs)
        else:
            profile['uwb_interval_mean_ms'] = None
            profile['uwb_interval_std_ms']  = None
            profile['uwb_interval_min_ms']  = None
            profile['uwb_interval_max_ms']  = None
        profile['uwb_sample_count'] = len(uwb_list)

        # UWB-to-IMU nearest-neighbor delay analysis
        uwb_events = self.get_all_sorted(sensor='UWB')
        delay_records = []
        
        for uwb_ev in uwb_events:
            ts = uwb_ev['ts_hw']
            before = self.get_nearest_imu_before(ts)
            after  = self.get_nearest_imu_after(ts)
            
            delay_records.append({
                'uwb_ts':                   ts,
                'uwb_packet_id':            uwb_ev['packet_id'],
                'nearest_imu_before_ts':    before['ts_hw'] if before else None,
                'nearest_imu_after_ts':     after['ts_hw']  if after  else None,
                'delay_before_ms':          (ts - before['ts_hw']) if before else None,
                'delay_after_ms':           (after['ts_hw'] - ts)  if after  else None,
            })
        
        
        profile['uwb_to_imu_delays'] = delay_records
        return profile


# ==============================================================================
# LIVE HARDWARE VALIDATION
#   - Normalizes via tv2_normalizer.StreamNormalizer
#   - Feeds events into TimeAlignLayer and prints real-time validation
# ==============================================================================
if __name__ == '__main__':
    
    import time
    import os
    from .unpacker import SerialStreamer
    from .normalizer import StreamNormalizer

    # CONFIGURATION
    SERIAL_PORT = 'COM20'       # temporarily, virtual com
    BAUD_RATE = 115200
    PROFILE_UPDATE_RATE = 1.0   # display rate (seconds)

    # Initialize pipeline modules
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=2000)

    print("=" * 60)
    print(f"  PolyCast Time Alignment Test: {SERIAL_PORT}")
    print("  Collecting data and building timing profiles...")
    print("=" * 60)

    last_print_time = time.time()

    try:
        while True:
            
            raw_packets = streamer.read_new_packets()
            
            if raw_packets:
                
                # Normalization
                events = norm.normalize(raw_packets)
                
                # Add to Time Alignment Buffer
                aligner.add_events(events)
            
            # Print Output Review Periodically
            current_time = time.time()
            
            if current_time - last_print_time >= PROFILE_UPDATE_RATE:
                
                profile = aligner.get_timing_profile()
                
                # Clear terminal for dashboard effect
                os.system('cls' if os.name == 'nt' else 'clear')
                
                print(f"========= LIVE TIMING PROFILE ({PROFILE_UPDATE_RATE}s update) =========")
                
                print(f"[IMU Timing] Samples Analyzed: {profile['imu_sample_count']}")
                if profile['imu_sample_count'] > 1:
                    print(f"  Mean interval : {profile['imu_interval_mean_ms']} µs")
                    print(f"  Std (jitter)  : {profile['imu_interval_std_ms']} µs")
                    print(f"  Min interval  : {profile['imu_interval_min_ms']} µs")
                    print(f"  Max gap       : {profile['imu_interval_max_ms']} µs")

                print(f"\n[UWB Timing] Samples Analyzed: {profile['uwb_sample_count']}")
                if profile['uwb_sample_count'] > 1:
                    print(f"  Mean interval : {profile['uwb_interval_mean_ms']} µs")
                    print(f"  Std (jitter)  : {profile['uwb_interval_std_ms']} µs")
                    print(f"  Min interval  : {profile['uwb_interval_min_ms']} µs")
                    print(f"  Max gap       : {profile['uwb_interval_max_ms']} µs")

                # Show the most recent delay gap between UWB and IMU
                if profile.get('uwb_to_imu_delays'):
                    latest_delay = profile['uwb_to_imu_delays'][-1]
                    print(f"\n[Latest UWB-to-IMU Sync]")
                    print(f"  UWB Pkt ID: {latest_delay['uwb_packet_id']} @ TS_HW: {latest_delay['uwb_ts']}")
                    print(f"  IMU Before Delay: {latest_delay['delay_before_ms']} µs")
                    print(f"  IMU After Delay : {latest_delay['delay_after_ms']} µs")

                print("========================================================")
                
                
                last_print_time = current_time

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping Streamer...")
        streamer.close()
        print("Done.")