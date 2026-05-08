import sys
import os
import serial
import time

from kuru_method.asynchronous_stream.data_parser import AsyncDataParser

# ==============================================================================
# CONFIGURATION
# ==============================================================================
VIRTUAL_COM_PORT    = 'COM19'
BAUD_RATE           = 921600
MODE                = "abcde1"
TESTNAME            = "1"
LOG_FILE            = f'logs/datasets/v4/{MODE}_{TESTNAME}.csv'

# ==============================================================================
# MAIN REPLAY LOOP
# ==============================================================================
if __name__ == "__main__":
    if not os.path.exists(LOG_FILE):
        print(f"[!] Error: Could not find {LOG_FILE}.")
        print("Please check the filename and try again.")
        exit()

    print(f"[*] Opening Virtual COM Port {VIRTUAL_COM_PORT}...")
    try:
        ser = serial.Serial(VIRTUAL_COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"[!] Error opening port: {e}")
        exit()

    print(f"[*] Reading '{LOG_FILE}' and streaming to {VIRTUAL_COM_PORT}...")
    print("[*] Switch to your receiver window (COM20) now!\n")

    parser = AsyncDataParser(csv_path=os.path.abspath(LOG_FILE))
    if not parser.connect():
        ser.close()
        exit()

    packets_sent = 0
    last_hw_ts = None

    try:
        while True:
            pkt = parser.get_packet()
            if pkt == 'EOF':
                break
            if not pkt:
                continue

            # Hardware timestamp replay timing
            current_hw_ts = pkt['ts']
            if last_hw_ts is not None:
                delay_us = current_hw_ts - last_hw_ts
                if 0 < delay_us < 1_000_000:
                    time.sleep(delay_us / 1_000_000.0)
            last_hw_ts = current_hw_ts

            # Reconstruct strict comma-separated CSV string from parsed packet
            if pkt['type'] == 'imu':
                qx, qy, qz, qw = pkt['quat']
                ax, ay, az = pkt['acc']
                if pkt['format'] == 'imu_v2_gyro':
                    gx, gy, gz = pkt['gyro']
                    csv_string = f"I,{pkt['seq']},{qx},{qy},{qz},{qw},{ax},{ay},{az},{gx},{gy},{gz},{pkt['force']},{pkt['ts']}\n"
                else:  # imu_v1_legacy
                    csv_string = f"I,{pkt['seq']},{qx},{qy},{qz},{qw},{ax},{ay},{az},{pkt['force']},{pkt['ts']}\n"
            elif pkt['type'] == 'uwb':
                d0, d1, d2, d3 = pkt['dists']
                csv_string = f"U,{pkt['seq']},{d0},{d1},{d2},{d3},{pkt['ts']}\n"
            else:
                continue

            ser.write(csv_string.encode('utf-8'))
            ser.flush()
            packets_sent += 1

            if packets_sent % 50 == 0:
                print(f" -> Streamed {packets_sent} packets...", end='\r')

    except KeyboardInterrupt:
        print("\n\n[*] Replay stopped manually.")

    parser.summary()
    parser.close()
    ser.close()
    print(f"\n[*] Replay Complete. Total Packets Sent: {packets_sent}")
