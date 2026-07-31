"""
PolyCast Log Replayer
=====================
Streams a previously recorded dataset (via serial_recorder.py) out over a virtual
COM port so the downstream pipeline can be exercised without the prototype hardware.

Each line is re-parsed, validated against the IMU/UWB layouts the receiver
expects, and re-emitted as a strict comma-separated packet. 

Playback is paced using the hardware timestamps embedded in the log so the receiver
have roughly the same packet timing as possible in the original capture.

Expected Dataset Loacation and Format:
    test/_datasets/<pattern>_<version>.csv.

Usage:
    python -m utils.serial_replayer                               # sample_1.csv by default
    python -m utils.serial_replayer --pattern square --version 1
    python -m utils.serial_replayer --list                        # show datasets
"""

import os
import re
import sys
import time
import argparse
import serial


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
VIRTUAL_COM_PORT = 'COM19'
BAUD_RATE        = 921600

# Datasets are expected named "<pattern>_<version>.csv" (e.g. sample_1.csv)
DEFAULT_PATTERN = "sample"
DEFAULT_VERSION = "1"

# Datasets live at the repository root, one level above this script:
#    - (location) "test/_datasets" by default
#
# Anchored to this file so the replayer works regardless of the directory it is launched from.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASETS_DIR  = os.path.join(_PROJECT_ROOT, 'test', '_datasets')

# Packet field counts accepted by the receiver's unpacker. 
# IMU has two valid shapes: the 11-field legacy layout and the 14-field extended layout with gyro.
IMU_FIELD_COUNTS = (11, 14)
UWB_FIELD_COUNT  = 7

# Logged timestamps are microseconds. 
# Deltas outside this range indicate a gap between capture sessions or a counter wrap
MAX_REPLAY_DELAY_MICROSECONDS = 1_000_000

PROGRESS_INTERVAL_PACKETS = 50


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def parse_packet_fields(line):
    """
    Split a log line into packet fields, returning None if it is not a valid
    IMU or UWB packet.

    Logs may be delimited by tabs, spaces, or commas in any combination, so
    empty fragments from repeated separators are dropped before validating.
    """

    fields = [field for field in re.split(r'[\t, ]+', line) if field]
    if not fields:
        return None

    packet_type = fields[0]
    if packet_type == 'I' and len(fields) in IMU_FIELD_COUNTS:
        return fields
    if packet_type == 'U' and len(fields) == UWB_FIELD_COUNT:
        return fields

    # Headers and malformed rows are ignored entirely.
    return None


def sleep_for_timestamp_delta(previous_timestamp, current_timestamp):
    """Pause for the gap between two hardware timestamps, if it is plausible."""

    if previous_timestamp is None:
        return

    delay_microseconds = current_timestamp - previous_timestamp
    if 0 < delay_microseconds < MAX_REPLAY_DELAY_MICROSECONDS:
        time.sleep(delay_microseconds / 1_000_000.0)


def replay_log_file(serial_connection, log_path) -> int:
    """
    Stream every valid packet in the log to the serial port, paced by the
    hardware timestamps. Returns the number of packets sent.
    """

    packets_sent = 0
    previous_timestamp = None

    with open(log_path, mode='r', encoding='utf-8') as log_file:
        try:
            for raw_line in log_file:
                line = raw_line.strip()
                if not line:
                    continue

                fields = parse_packet_fields(line)
                if fields is None:
                    continue

                try:
                    current_timestamp = int(fields[-1])
                except ValueError:
                    continue

                sleep_for_timestamp_delta(previous_timestamp, current_timestamp)
                previous_timestamp = current_timestamp

                packet = ",".join(fields) + "\n"

                # Flush per packet so the receiver sees the intended pacing
                # instead of buffered bursts.
                serial_connection.write(packet.encode('utf-8'))
                serial_connection.flush()
                packets_sent += 1

                if packets_sent % PROGRESS_INTERVAL_PACKETS == 0:
                    print(f" -> Streamed {packets_sent} packets...", end='\r')

        except KeyboardInterrupt:
            print("\n\n[*] Replay stopped manually.")

    return packets_sent


# -----------------------------------------------------------------------------
# Dataset selection
# -----------------------------------------------------------------------------
def available_datasets():
    """List the dataset names present in DATASETS_DIR, without the .csv suffix."""

    if not os.path.isdir(DATASETS_DIR):
        return []

    names = [os.path.splitext(entry)[0]
             for entry in os.listdir(DATASETS_DIR)
             if entry.endswith('.csv')]
    return sorted(names)


def resolve_dataset_path(pattern, version):
    """Build the "<pattern>_<version>.csv" path inside DATASETS_DIR."""

    return os.path.join(DATASETS_DIR, f'{pattern}_{version}.csv')


def parse_arguments():
    """Parse the dataset selection and serial port arguments."""

    parser = argparse.ArgumentParser(
        description='PolyCast Log Replayer - stream a recorded dataset over a '
                    'virtual COM port.')
    parser.add_argument('--pattern', default=DEFAULT_PATTERN,
                        help=f'Test data pattern, e.g. circle, square '
                             f'(default: {DEFAULT_PATTERN})')
    parser.add_argument('--version', default=DEFAULT_VERSION,
                        help=f'Dataset version number (default: {DEFAULT_VERSION})')
    parser.add_argument('--port', default=VIRTUAL_COM_PORT,
                        help=f'Virtual COM port (default: {VIRTUAL_COM_PORT})')
    parser.add_argument('--baud', type=int, default=BAUD_RATE,
                        help=f'Baud rate (default: {BAUD_RATE})')
    parser.add_argument('--list', action='store_true',
                        help='List the available datasets and exit')
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_arguments()

    if args.list:
        print(f"[*] Datasets in {DATASETS_DIR}:")
        for name in available_datasets():
            print(f"      {name}")
        return

    log_file = resolve_dataset_path(args.pattern, args.version)

    if not os.path.exists(log_file):
        # Show what is actually on disk; a typo in --pattern/--version is the
        # likeliest cause and the valid names are not guessable otherwise.
        print(f"[!] Error: Dataset '{args.pattern}_{args.version}' "
              f"not found in {DATASETS_DIR}.")
        print("[!] Available datasets:")
        for name in available_datasets():
            print(f"      {name}")
        sys.exit(1)

    print(f"[*] Opening Virtual COM Port {args.port}...")
    try:
        serial_connection = serial.Serial(args.port, args.baud)
    except Exception as error:
        print(f"[!] Error opening port: {error}")
        sys.exit(1)

    print(f"[*] Reading '{log_file}' and streaming to {args.port}...")
    print("[*] Switch to your receiver window (COM20) now!\n")

    packets_sent = replay_log_file(serial_connection, log_file)

    print(f"\n\n[*] Replay Complete. Total Packets Sent: {packets_sent}")
    serial_connection.close()


if __name__ == "__main__":
    main()
