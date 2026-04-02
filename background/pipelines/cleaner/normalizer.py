"""
Module 2 — Stream Normalizer

Responsibility:
    - Consume raw packet dictionaries from tv1_unpacker.py
    - Flatten batched IMU packets (3 samples each) into individual events
    - Standardize all events into a consistent schema

Input  (from tv1_unpacker.read_new_packets):
    IMU packet: { 'type': 'IMU', 'id': int, 'samples': [ {ts, quat, acc, force}, ... ] }
    UWB packet: { 'type': 'UWB', 'id': int, 'pos': (x,y), 'dists': (d0,d1,d2), 'ts': int }

Output (normalized event schema):
    IMU event:
    {
        'sensor':     'IMU',
        'packet_id':  int,
        'sample_idx': int,      # 0, 1, or 2 within the batch
        'ts_hw':      int,      # hardware timestamp from sensor
        'quat':       (qx, qy, qz, qw),
        'acc':        (ax, ay, az),
        'force':      float
    }

    UWB event:
    {
        'sensor':     'UWB',
        'packet_id':  int,
        'sample_idx': 0,        # always 0 (UWB is not batched)
        'ts_hw':      int,
        'pos':        (x, y),
        'dists':      (d0, d1, d2)
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
            One IMU packet → 3 IMU events.
            One UWB packet → 1 UWB event.
        """
        events = []
        
        for pkt in packets:
            self._total_packets_in += 1
            pkt_type = pkt.get('type')

            if pkt_type == 'IMU':
                events.extend(self._flatten_imu(pkt))
                
            elif pkt_type == 'UWB':
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
        Unwraps one IMU packet (3 samples) into 3 individual events.
        """
        events = []
        packet_id = pkt.get('id')
        samples   = pkt.get('samples', [])

        for idx, sample in enumerate(samples):
            ev = {
                'sensor':     'IMU',
                'packet_id':  packet_id,
                'sample_idx': idx,
                'ts_hw':      sample.get('ts'),
                'quat':       sample.get('quat'),
                'acc':        sample.get('acc'),
                'force':      sample.get('force'),
            }
            events.append(ev)
            self._total_imu_events += 1

        return events

    def _flatten_uwb(self, pkt: dict) -> dict | None:
        """
        Unwraps one UWB packet into a single event.
        """
        return {
            'sensor':     'UWB',
            'packet_id':  pkt.get('id'),
            'sample_idx': 0,
            'ts_hw':      pkt.get('ts'),
            'pos':        pkt.get('pos'),
            'dists':      pkt.get('dists'),
        }


# ==============================================================================
# LIVE HARDWARE VALIDATION
#   - Connects to tv1_unpacker.SerialStreamer
#   - Normalizes the fetched hardware data
# ==============================================================================
if __name__ == '__main__':
    
    from tv1_unpacker import SerialStreamer
    import time
    
    # CONFIGURATION
    SERIAL_PORT = 'COM20'  # temporarily, virtual com
    BAUD_RATE = 115200
    
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
                        print(f"[{sensor}] Pkt:{ev['packet_id']} | TS:{ts} | Pos:{ev['pos']}")

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
