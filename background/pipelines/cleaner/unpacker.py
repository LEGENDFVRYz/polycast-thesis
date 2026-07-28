"""
Module 1 - Stream Unpacker

Reads the data stream sends by the ESP32 receiver over serial communication and
parses each line into a flat event dictionary. 

This is the entry point of the pipeline: everything downstream consumes the dicts produced here.

Data format (one sensor reading per line):
    IMU, with gyro:         I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<force>,<ts>
    IMU, legacy:            I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
    UWB:                    U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

Both IMU layouts are accepted so recordings made before the gyro field was added
still replay. Layouts are told apart by field count alone.

Output events:
    IMU: {'sensor', 'packet_id', 'sample_idx', 'quat', 'acc', 'gyro'?, 'force', 'ts_hw'}
    UWB: {'sensor', 'packet_id', 'sample_idx', 'dists', 'ts_hw'}

    Note: Timestamps (`ts_hw`) are hardware microseconds throughout the pipeline.
    
Usage (Import as helper or run directly to inspect the raw stream):
    python -m background.pipelines.cleaner.unpacker
"""

import time
import serial

from background.pipelines.config import cfg

MICROSECONDS_PER_SECOND = 1_000_000

# Field counts that identify each supported line layout.
_IMU_FIELDS_WITH_GYRO = 14
_IMU_FIELDS_LEGACY = 11
_UWB_FIELDS = 7

# The receiver emits one reading per line, so there are no sub-samples to index.
_SAMPLE_INDEX = 0

# Seconds to wait after opening the port for the board to finish resetting.
# Opening a serial connection toggles DTR, which reboots the ESP32; reading
# before it is up returns a partial boot banner instead of packets.
_BOARD_RESET_DELAY_S = 2.0


# -----------------------------------------------------------------------------
# Line Parsing
# -----------------------------------------------------------------------------

def _parse_imu_with_gyro(fields: list[str]) -> dict:
    """Parse the extended IMU layout that carries hardware-calibrated gyro."""

    return {
        'sensor': 'IMU',
        'packet_id': int(fields[1]),
        'sample_idx': _SAMPLE_INDEX,
        'quat': (float(fields[2]), float(fields[3]), float(fields[4]), float(fields[5])),
        'acc': (float(fields[6]), float(fields[7]), float(fields[8])),
        'gyro': (float(fields[9]), float(fields[10]), float(fields[11])),
        'force': float(fields[12]),
        'ts_hw': int(fields[13]),
    }


def _parse_imu_legacy(fields: list[str]) -> dict:
    """
    Parse the original IMU layout, which predates the gyro field.

    No 'gyro' key is emitted; imu.py falls back to deriving angular velocity
    from consecutive quaternions when the field is absent.
    """

    return {
        'sensor': 'IMU',
        'packet_id': int(fields[1]),
        'sample_idx': _SAMPLE_INDEX,
        'quat': (float(fields[2]), float(fields[3]), float(fields[4]), float(fields[5])),
        'acc': (float(fields[6]), float(fields[7]), float(fields[8])),
        'force': float(fields[9]),
        'ts_hw': int(fields[10]),
    }


def _parse_uwb(fields: list[str]) -> dict:
    """Parse a UWB line carrying one range per anchor."""

    return {
        'sensor': 'UWB',
        'packet_id': int(fields[1]),
        'sample_idx': _SAMPLE_INDEX,
        'dists': (float(fields[2]), float(fields[3]), float(fields[4]), float(fields[5])),
        'ts_hw': int(fields[6]),
    }


def parse_packet_line(line: str) -> dict | None:
    """
    Parse one CSV line into an event dict, or None if it is not a valid packet.

    Returns None rather than raising: the serial stream regularly contains
    partial or corrupted lines after a reset or a dropped byte, and one bad line
    must not interrupt the rest of the batch.
    """

    fields = line.split(',')
    if not fields:
        return None

    record_type = fields[0]
    field_count = len(fields)

    try:
        if record_type == 'I':
            if field_count == _IMU_FIELDS_WITH_GYRO:
                return _parse_imu_with_gyro(fields)
            if field_count == _IMU_FIELDS_LEGACY:
                return _parse_imu_legacy(fields)
        elif record_type == 'U' and field_count == _UWB_FIELDS:
            return _parse_uwb(fields)
    except (ValueError, IndexError):
        return None

    return None


# -----------------------------------------------------------------------------
# Serial Transport (Module 1)
# -----------------------------------------------------------------------------

class SerialStreamer:
    """Reads and parses packets from the ESP32 receiver's serial connection."""

    def __init__(self, port: str = 'COM3', baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connect()

    def connect(self):
        """Open the serial port, leaving self.ser as None if it is unavailable."""

        try:
            # A non-zero timeout lets readline() block until it sees '\n',
            # which is what guarantees whole lines rather than partial reads.
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(_BOARD_RESET_DELAY_S)
            self.ser.reset_input_buffer()
            print(f"[STREAMER] Connected to LIVE stream on {self.port}")
        except Exception as error:
            print(f"[STREAMER] Connection Error: {error}")
            self.ser = None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def read_new_packets(self) -> list[dict]:
        """
        Drain every complete line currently buffered and return parsed events.

        Only reads while bytes are already waiting, so a quiet stream returns
        immediately instead of blocking the caller's poll loop.
        """

        packets = []

        if not self.ser or not self.ser.is_open:
            return packets

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                packet = parse_packet_line(line)
                if packet is not None:
                    packets.append(packet)

        except Exception as error:
            print(f"[STREAMER] Read Error: {error}")

        return packets


# -----------------------------------------------------------------------------
# Link Statistics (For Module Testing)
# -----------------------------------------------------------------------------

class PacketStats:
    """
    Tracks receive rate and packet loss for one sensor stream.

    Loss is inferred from gaps in the sender's sequence numbers, so it counts
    packets dropped anywhere in the ESP-NOW to serial path.
    """

    def __init__(self, name: str):
        self.name = name

        self.total_received = 0
        self.total_expected = 0
        self.total_lost = 0

        self.last_packet_id = None
        self.first_ts = None
        self.last_ts = None

    def update(self, packet: dict):
        """Fold one received packet into the running counters."""

        packet_id = packet['packet_id']
        ts = packet['ts_hw']

        if self.first_ts is None:
            self.first_ts = ts
        self.last_ts = ts

        self.total_received += 1

        if self.last_packet_id is None:
            self.total_expected += 1
        else:
            sequence_gap = packet_id - self.last_packet_id
            if sequence_gap > 1:
                # Every id skipped between the last packet and this one was sent
                # but never arrived.
                self.total_lost += sequence_gap - 1
                self.total_expected += sequence_gap
            else:
                self.total_expected += 1

        self.last_packet_id = packet_id

    def get_summary(self) -> dict:
        """Return received/lost counts, their percentages, and the average rate."""

        if self.total_expected == 0:
            received_percent = 0
            loss_percent = 0
        else:
            received_percent = (self.total_received / self.total_expected) * 100
            loss_percent = (self.total_lost / self.total_expected) * 100

        hz = 0
        if self.first_ts is not None and self.last_ts != self.first_ts:
            elapsed_s = (self.last_ts - self.first_ts) / MICROSECONDS_PER_SECOND
            if elapsed_s > 0:
                hz = self.total_received / elapsed_s

        return {
            'received': self.total_received,
            'received_percent': received_percent,
            'lost': self.total_lost,
            'loss_percent': loss_percent,
            'hz': hz,
        }


# =============================================================================
# MODULE TESTING
#   Live serial dashboard: raw packets + link statistics
#   Fastest way to tell a wiring/firmware fault from a pipeline fault - if
#   packets are not clean here, nothing downstream can be trusted.
#
#   Run:  python -m background.pipelines.cleaner.unpacker
#
#   Reading the output:
#     - IMU Avg Hz should sit near cfg.imu.sample_rate_hz
#     - UWB Avg Hz should sit near cfg.uwb.rate_hz
#     - Sustained loss above a few percent points at the ESP-NOW link
# =============================================================================
if __name__ == '__main__':
    import os

    # 'LIVE' repaints a fixed dashboard; 'HISTORY' scrolls every packet.
    VIEW_MODE = 'LIVE'

    # Restrict the view to one sensor: 'BOTH', 'IMU', or 'UWB'.
    FILTER_MODE = 'BOTH'

    # Seconds between repaints. 0 renders as fast as packets arrive.
    DISPLAY_RATE_S = 0.1

    def _render_packet_line(packet: dict) -> str:
        if packet['sensor'] == 'IMU':
            return f"[IMU #{packet['packet_id']}] Acc: {packet['acc']}"
        distances = packet['dists']
        return (f">>> [UWB #{packet['packet_id']}] Dists: "
                f"{distances[0]:.2f}, {distances[1]:.2f}, "
                f"{distances[2]:.2f}, {distances[3]:.2f}")

    def _render_dashboard(latest: dict, imu_summary: dict, uwb_summary: dict) -> None:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"=========== STREAMER DEBUG ({DISPLAY_RATE_S}s) ==========")

        if FILTER_MODE in ('BOTH', 'IMU') and latest['IMU']:
            imu = latest['IMU']
            quaternion = imu['quat']
            print(f"\n[IMU #{imu['packet_id']}]")
            print(f"  Force: {imu['force']:.2f}")
            print(f"  Accel: {imu['acc'][0]:.2f}, {imu['acc'][1]:.2f}, {imu['acc'][2]:.2f}")
            if imu.get('gyro'):
                gyro = imu['gyro']
                print(f"  Gyro:  {gyro[0]:.4f}, {gyro[1]:.4f}, {gyro[2]:.4f}")
            print(f"  Quat:  {quaternion[0]:.2f}, {quaternion[1]:.2f}, "
                  f"{quaternion[2]:.2f}, {quaternion[3]:.2f}")

        if FILTER_MODE in ('BOTH', 'UWB') and latest['UWB']:
            uwb = latest['UWB']
            distances = uwb['dists']
            print(f"\n[UWB #{uwb['packet_id']}]")
            print(f"  Dists: {distances[0]:.2f}, {distances[1]:.2f}, "
                  f"{distances[2]:.2f}, {distances[3]:.2f}")

        print("\n=============== STATISTICS =================")

        if FILTER_MODE in ('BOTH', 'IMU') and latest['IMU']:
            print("\n[IMU]")
            print(f"  Received: {imu_summary['received']} ({imu_summary['received_percent']:.2f}%)")
            print(f"  Lost:     {imu_summary['lost']} ({imu_summary['loss_percent']:.2f}%)")
            print(f"  Avg Hz:   {imu_summary['hz']:.2f}")

        if FILTER_MODE in ('BOTH', 'UWB') and latest['UWB']:
            print("\n[UWB]")
            print(f"  Received: {uwb_summary['received']} ({uwb_summary['received_percent']:.2f}%)")
            print(f"  Lost:     {uwb_summary['lost']} ({uwb_summary['loss_percent']:.2f}%)")
            print(f"  Avg Hz:   {uwb_summary['hz']:.2f}")

        print("\n============================================")

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    if not streamer.ser:
        raise SystemExit(1)

    imu_stats = PacketStats('IMU')
    uwb_stats = PacketStats('UWB')

    latest_packets = {'IMU': None, 'UWB': None}
    last_render = 0.0

    try:
        while True:
            for new_packet in streamer.read_new_packets():
                sensor_name = new_packet['sensor']
                latest_packets[sensor_name] = new_packet
                (imu_stats if sensor_name == 'IMU' else uwb_stats).update(new_packet)

                if VIEW_MODE == 'HISTORY' and FILTER_MODE in ('BOTH', sensor_name):
                    print(_render_packet_line(new_packet))

            now = time.time()
            if VIEW_MODE == 'LIVE' and (now - last_render > DISPLAY_RATE_S):
                _render_dashboard(
                    latest_packets, imu_stats.get_summary(), uwb_stats.get_summary()
                )
                last_render = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.close()
