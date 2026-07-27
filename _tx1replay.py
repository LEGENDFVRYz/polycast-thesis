"""
PolyCast Log Replayer
=====================
Streams a previously recorded dataset (via _tx1logger.py) out over a virtual COM
port so the downstream pipeline can be exercised without the prototype hardware.

Each line is re-parsed, validated against the IMU/UWB layouts the receiver
expects, and re-emitted as a strict comma-separated packet. 

Playback is paced using the hardware timestamps embedded in the log so the receiver 
have roughly the same packet timing as possible in the original capture.
"""

import os
import re
import time
import serial


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
VIRTUAL_COM_PORT = 'COM19'
BAUD_RATE        = 921600
MODE             = "circle"
TEST_NAME        = "2"
LOG_FILE         = f'test/_datasets/{MODE}_{TEST_NAME}.csv'

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
# Main
# -----------------------------------------------------------------------------
def main():
    if not os.path.exists(LOG_FILE):
        print(f"[!] Error: Could not find {LOG_FILE}.")
        print("Please check the filename and try again.")
        exit()

    print(f"[*] Opening Virtual COM Port {VIRTUAL_COM_PORT}...")
    try:
        serial_connection = serial.Serial(VIRTUAL_COM_PORT, BAUD_RATE)
    except Exception as error:
        print(f"[!] Error opening port: {error}")
        exit()

    print(f"[*] Reading '{LOG_FILE}' and streaming to {VIRTUAL_COM_PORT}...")
    print("[*] Switch to your receiver window (COM20) now!\n")

    packets_sent = replay_log_file(serial_connection, LOG_FILE)

    print(f"\n\n[*] Replay Complete. Total Packets Sent: {packets_sent}")
    serial_connection.close()


if __name__ == "__main__":
    main()
