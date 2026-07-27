"""
PolyCast Async Stream CSV Recorder
==================================
Records the live asynchronous serial stream from the ESP32 WROOM receiver to a
CSV file for later offline replay (via _tx1replay.py) and pipeline testing.

Each incoming line is validated against the packet layouts that data_parser.py
expects, and anything malformed is counted and discarded rather than written:

    IMU:  I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
    UWB:  U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

All recordings are written into utils/recorded_data/; --output names the file
within that folder, it does not choose a location.

Usage:
    python _tx1logger.py                     # defaults: COM5, square_1.csv
    python _tx1logger.py --port COM3         # specify port
    python _tx1logger.py --output data.csv   # -> utils/recorded_data/data.csv

Press Ctrl+C to stop recording. Statistics are printed at the end.
"""

import os
import sys
import time
import argparse
import serial


# -----------------------------------------------------------------------------
# Configuration
#
# Default capture target. MODE/TEST_NAME
# -----------------------------------------------------------------------------
DEFAULT_PORT = 'COM5'
DEFAULT_BAUD = 115200

MODE      = "sample"
TEST_NAME = "1"

# Capture Data Location
RECORDED_DATA_DIR = 'utils/recorded_data'

DEFAULT_DATASET_FILENAME = f"{MODE}_{TEST_NAME}.csv"

# Packet field counts accepted by data_parser.py. IMU has two valid shapes:
# the 11-field legacy layout and the 14-field extended layout carrying gyro.
IMU_FIELD_COUNTS = (11, 14)
UWB_FIELD_COUNT = 7

# Rows between flush + progress print.
FLUSH_INTERVAL_PACKETS = 500


# -----------------------------------------------------------------------------
# Recorder
# -----------------------------------------------------------------------------
class AsyncCsvRecorder:
    """Records validated async stream lines from serial to a CSV file."""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, output_name=None):
        self.port = port
        self.baud = baud
        self.output_path = self._resolve_output_path(
            output_name or self._default_filename())
        self.serial_connection = None

        self.imu_packet_count = 0
        self.uwb_packet_count = 0
        self.skipped_line_count = 0
        self.start_time = None

    @staticmethod
    def _default_filename():
        """Generate a timestamped CSV filename for an unnamed capture."""

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        return f'async_{timestamp}.csv'

    @staticmethod
    def _resolve_output_path(output_name):
        """
        Place a capture inside RECORDED_DATA_DIR, creating the folder if needed.

        Only the basename is kept, so a stray path in --output cannot write the
        recording somewhere outside the recorded-data folder.
        """

        os.makedirs(RECORDED_DATA_DIR, exist_ok=True)
        return os.path.join(RECORDED_DATA_DIR, os.path.basename(output_name))

    def connect(self) -> bool:
        """Open the serial port. Returns True on success, False on failure."""

        try:
            self.serial_connection = serial.Serial(self.port, self.baud, timeout=1)

            # Opening the port resets the Arduino; give it time to boot before
            # reading, otherwise the first packets are bootloader garbage.
            time.sleep(2)
            print(f"Connected to {self.port} at {self.baud} baud")
            return True
        except Exception as error:
            print(f"Connection failed: {error}")
            return False

    def _is_valid_packet(self, parts) -> bool:
        """Check a split line against the IMU/UWB layouts data_parser.py accepts."""

        if parts[0] == 'I':
            return len(parts) in IMU_FIELD_COUNTS
        if parts[0] == 'U':
            return len(parts) == UWB_FIELD_COUNT
        return False

    def _count_packet(self, packet_type):
        if packet_type == 'I':
            self.imu_packet_count += 1
        else:
            self.uwb_packet_count += 1

    def _print_progress(self):
        elapsed_seconds = time.time() - self.start_time
        imu_rate_hz = self.imu_packet_count / elapsed_seconds if elapsed_seconds > 0 else 0
        uwb_rate_hz = self.uwb_packet_count / elapsed_seconds if elapsed_seconds > 0 else 0
        print(f"  [{elapsed_seconds:.0f}s] IMU: {self.imu_packet_count} "
              f"({imu_rate_hz:.0f} Hz)  |  UWB: {self.uwb_packet_count} "
              f"({uwb_rate_hz:.0f} Hz)  |  Skipped: {self.skipped_line_count}")

    def record(self):
        """
        Read lines from serial, validate their format, and write them to CSV.
        Runs until interrupted with Ctrl+C.
        """

        self.start_time = time.time()
        print(f"Recording to: {self.output_path}")
        print("Press Ctrl+C to stop.\n")

        with open(self.output_path, 'w') as output_file:
            try:
                while True:
                    if not self.serial_connection or self.serial_connection.in_waiting == 0:
                        # Yield the CPU briefly instead of busy-spinning on an
                        # empty input buffer.
                        time.sleep(0.001)
                        continue

                    raw_line = self.serial_connection.readline()
                    line = raw_line.decode('utf-8', errors='replace').strip()
                    if not line:
                        continue

                    parts = line.split(',')
                    if not self._is_valid_packet(parts):
                        self.skipped_line_count += 1
                        continue

                    output_file.write(line + '\n')
                    self._count_packet(parts[0])

                    total_packets = self.imu_packet_count + self.uwb_packet_count
                    if total_packets % FLUSH_INTERVAL_PACKETS == 0:
                        output_file.flush()
                        self._print_progress()

            except KeyboardInterrupt:
                output_file.flush()
                print("\nRecording stopped.")

    def close(self):
        """Close the serial port if it is open."""

        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None

    def print_summary(self):
        """Print packet counts, duration, and average rates for the session."""

        elapsed_seconds = time.time() - self.start_time if self.start_time else 0
        print(f"\n{'='*55}")
        print(f"  ASYNC CSV RECORDER - SESSION SUMMARY")
        print(f"{'='*55}")
        print(f"  Output file    : {self.output_path}")
        print(f"  Duration       : {elapsed_seconds:.1f} s")
        print(f"  IMU packets    : {self.imu_packet_count}")
        print(f"  UWB packets    : {self.uwb_packet_count}")
        print(f"  Skipped lines  : {self.skipped_line_count}")
        if elapsed_seconds > 0:
            print(f"  Avg IMU rate   : {self.imu_packet_count / elapsed_seconds:.1f} Hz")
            print(f"  Avg UWB rate   : {self.uwb_packet_count / elapsed_seconds:.1f} Hz")
        print(f"{'='*55}\n")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def parse_arguments():
    """Parse command-line arguments for port, baud rate, and output path."""

    parser = argparse.ArgumentParser(
        description='PolyCast Async Stream CSV Recorder')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--output', default=DEFAULT_DATASET_FILENAME,
                        help=f'Output CSV filename, written to {RECORDED_DATA_DIR}/ '
                             f'(default: {DEFAULT_DATASET_FILENAME})')
    return parser.parse_args()


def main():
    args = parse_arguments()

    recorder = AsyncCsvRecorder(
        port=args.port,
        baud=args.baud,
        output_name=args.output,
    )

    if not recorder.connect():
        sys.exit(1)

    try:
        recorder.record()
    finally:
        recorder.close()
        recorder.print_summary()


if __name__ == '__main__':
    main()
