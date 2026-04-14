"""
Module 2 — Stream Normalizer

Responsibility:
    - Consume raw packet dictionaries from tv1_unpacker.py
    - Standardize all events into a consistent schema
    - NOTE: Because the new async dual-stream hardware sends unbatched data, 
      the unpacker already outputs flat events. This normalizer now acts as a 
      schema validator, pass-through, and statistical tracker for the pipeline.

Input  (from tv1_unpacker.read_new_packets):
    IMU event: { 'sensor': 'IMU', 'packet_id': int, 'sample_idx': 0, 'quat': (...), 'acc': (...), 'force': float, 'ts_hw': int }
    UWB event: { 'sensor': 'UWB', 'packet_id': int, 'sample_idx': 0, 'dists': (...), 'ts_hw': int }

Output (normalized event schema):
    IMU event:
    {
        'sensor':     'IMU',
        'packet_id':  int,
        'sample_idx': int,      # always 0 (new async format)
        'ts_hw':      int,      # hardware timestamp from sensor
        'quat':       (qx, qy, qz, qw),
        'acc':        (ax, ay, az),
        'force':      float
    }

    UWB event:
    {
        'sensor':     'UWB',
        'packet_id':  int,
        'sample_idx': 0,        # always 0
        'ts_hw':      int,
        'dists':      (d0, d1, d2, d3)
    }
"""

class StreamNormalizer:
    """
    Converts raw packet dictionaries from the unpacker into a flat,
    standardized event stream — one dict per sensor reading.
    """

    def __init__(self):
        self._total_imu_events = 0
        self._total_uwb_events = 0
        self._total_packets_in = 0
    
    def normalize(self, packets: list[dict]) -> list[dict]:
        """
        Main entry point.

        Args:
            packets: List of raw packet dicts returned by
                     tv1_unpacker.SerialStreamer.read_new_packets()

        Returns:
            Flat list of normalized event dicts (preserve order).
        """
        events = []
        
        for pkt in packets:
            self._total_packets_in += 1
            # The new unpacker uses 'sensor' instead of 'type'
            sensor_type = pkt.get('sensor')

            if sensor_type == 'IMU':
                events.extend(self._flatten_imu(pkt))
                
            elif sensor_type == 'UWB':
                ev = self._flatten_uwb(pkt)
                if ev:
                    events.append(ev)
                    self._total_uwb_events += 1

        return events

    def stats(self) -> dict:
        """
        Returns cumulative normalization statistics since instantiation.
        Useful for validating event counts in tests.
        """
        return {
            'packets_in':   self._total_packets_in,
            'imu_events':   self._total_imu_events,
            'uwb_events':   self._total_uwb_events,
            'total_events': self._total_imu_events + self._total_uwb_events,
        }

    def reset_stats(self):
        self._total_imu_events = 0
        self._total_uwb_events = 0
        self._total_packets_in = 0

    # --- private helpers ---
    def _flatten_imu(self, pkt: dict) -> list[dict]:
        """
        Passes through the already-flat IMU event from the new unpacker.
        """
        # We wrap it in a list so normalize() can still use .extend()
        self._total_imu_events += 1
        return [{
            'sensor':     'IMU',
            'packet_id':  pkt.get('packet_id'),
            'sample_idx': pkt.get('sample_idx', 0),
            'ts_hw':      pkt.get('ts_hw'),
            'quat':       pkt.get('quat'),
            'acc':        pkt.get('acc'),
            'force':      pkt.get('force'),
        }]

    def _flatten_uwb(self, pkt: dict) -> dict | None:
        """
        Passes through the already-flat UWB event from the new unpacker.
        """
        return {
            'sensor':     'UWB',
            'packet_id':  pkt.get('packet_id'),
            'sample_idx': pkt.get('sample_idx', 0),
            'ts_hw':      pkt.get('ts_hw'),
            'dists':      pkt.get('dists'),
        }


# ==============================================================================
# LIVE HARDWARE VALIDATION
#   - Connects to tv1_unpacker.SerialStreamer
#   - Normalizes the fetched hardware data
# ==============================================================================
if __name__ == '__main__':
    
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.config import cfg
    import time
    
    # CONFIGURATION
    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    
    # Initialize modules
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()

    print("=" * 60)
    print(f"  --- Pipeline Testing: @{SERIAL_PORT} ----")
    print("  > Normalizing hardware packets into flat events...")
    print("=" * 60)
    
    try:
        while True:
            
            raw_packets = streamer.read_new_packets()
            
            if raw_packets:
                
                events = norm.normalize(raw_packets)
                
                # Print the flattened events
                for ev in events:
                    sensor = ev['sensor']
                    ts = ev['ts_hw']
                    
                    if sensor == 'IMU':
                        # Show packet ID and sample index to verify flattening
                        print(f"[{sensor}] Pkt:{ev['packet_id']} Sub:{ev['sample_idx']} | TS:{ts} | Force:{ev['force']:.2f}")
                    
                    elif sensor == 'UWB':
                        print(f"[{sensor}] Pkt:{ev['packet_id']} | TS:{ts} | Dist:{ev['dists']}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        stats = norm.stats()
        print("\n" + "=" * 60)
        print("  STREAM STOPPED")
        print(f"  Total Packets In: {stats['packets_in']}")
        print(f"  IMU Events (Flattened): {stats['imu_events']}")
        print(f"  UWB Events: {stats['uwb_events']}")
        print("=" * 60)
        streamer.close()