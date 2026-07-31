"""
Module 2 - Stream Normalizer

Validates and republishes unpacker events under a fixed schema, and counts what
passes through.

The hardware now sends one unbatched reading per line, so the unpacker already
emits flat events and no reshaping is left to do. 

(Legacy Code for sudden changes)

What remains is a scheme boundary: every downstream stage can rely on the exact key 
set declared here regardless of which wire layout produced the event, since optional 
fields are normalized to an explicit None rather than being absent.

Output schema:
    IMU: {'sensor', 'packet_id', 'sample_idx', 'ts_hw', 'quat', 'acc', 'gyro', 'force'}
    UWB: {'sensor', 'packet_id', 'sample_idx', 'ts_hw', 'dists'}

Usage (Import as helper or run directly to trace normalized events):
    python -m background.pipelines.cleaner.normalizer
"""

_DEFAULT_SAMPLE_INDEX = 0


class StreamNormalizer:
    """Republishes raw unpacker packets under the canonical event schema."""

    def __init__(self):
        self._total_imu_events = 0
        self._total_uwb_events = 0
        self._total_packets_in = 0

    def normalize(self, packets: list[dict]) -> list[dict]:
        """
        Convert a batch of raw packets into normalized events, preserving order.

        Packets from an unrecognized sensor are dropped rather than passed on,
        so a malformed event cannot reach a stage that assumes the schema.
        """

        events = []

        for packet in packets:
            self._total_packets_in += 1
            sensor = packet.get('sensor')

            if sensor == 'IMU':
                events.append(self._normalize_imu(packet))
                self._total_imu_events += 1
            elif sensor == 'UWB':
                events.append(self._normalize_uwb(packet))
                self._total_uwb_events += 1

        return events

    def stats(self) -> dict:
        """Return cumulative event counts since construction."""

        return {
            'packets_in': self._total_packets_in,
            'imu_events': self._total_imu_events,
            'uwb_events': self._total_uwb_events,
            'total_events': self._total_imu_events + self._total_uwb_events,
        }

    def reset_stats(self):
        self._total_imu_events = 0
        self._total_uwb_events = 0
        self._total_packets_in = 0

    @staticmethod
    def _normalize_imu(packet: dict) -> dict:
        return {
            'sensor': 'IMU',
            'packet_id': packet.get('packet_id'),
            'sample_idx': packet.get('sample_idx', _DEFAULT_SAMPLE_INDEX),
            'ts_hw': packet.get('ts_hw'),
            'quat': packet.get('quat'),
            'acc': packet.get('acc'),
            # Explicit None on legacy packets that carry no gyro, so imu.py can
            # detect the absence and fall back to quaternion-derived omega.
            'gyro': packet.get('gyro'),
            'force': packet.get('force'),
        }

    @staticmethod
    def _normalize_uwb(packet: dict) -> dict:
        return {
            'sensor': 'UWB',
            'packet_id': packet.get('packet_id'),
            'sample_idx': packet.get('sample_idx', _DEFAULT_SAMPLE_INDEX),
            'ts_hw': packet.get('ts_hw'),
            'dists': packet.get('dists'),
        }


# =============================================================================
# MODULE TESTING
#   Live hardware trace: prints every normalized event as it arrives, then a
#   final count summary. Use this to confirm the schema and the IMU/UWB event
#   split before moving on to the time-alignment stage.
#
#   Run:  python -m background.pipelines.cleaner.normalizer
# =============================================================================
if __name__ == '__main__':
    import time

    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.config import cfg

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()

    print("=" * 60)
    print(f"  --- Pipeline Testing: @{cfg.serial.port} ----")
    print("  > Normalizing hardware packets into flat events...")
    print("=" * 60)

    try:
        while True:
            raw_packets = streamer.read_new_packets()

            for event in normalizer.normalize(raw_packets):
                if event['sensor'] == 'IMU':
                    print(f"[IMU] Pkt:{event['packet_id']} Sub:{event['sample_idx']} "
                          f"| TS:{event['ts_hw']} | Force:{event['force']:.2f}")
                else:
                    print(f"[UWB] Pkt:{event['packet_id']} "
                          f"| TS:{event['ts_hw']} | Dist:{event['dists']}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        stats = normalizer.stats()
        print("\n" + "=" * 60)
        print("  STREAM STOPPED")
        print(f"  Total Packets In: {stats['packets_in']}")
        print(f"  IMU Events (Flattened): {stats['imu_events']}")
        print(f"  UWB Events: {stats['uwb_events']}")
        print("=" * 60)
        streamer.close()
