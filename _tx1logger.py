"""
csv_recorder.py  —  PolyCast Async Stream CSV Recorder
======================================================
Records the live asynchronous serial stream from the ESP32 WROOM receiver

The output format matches data_parser.py expectations exactly:
    IMU:  I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
    UWB:  U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

Usage
-----
    python csv_recorder.py                     # defaults: COM5, output to timestamped file
    python csv_recorder.py --port COM3         # specify port
    python csv_recorder.py --output data.csv   # specify output file

Press Ctrl+C to stop recording. Statistics are printed at the end.
"""

import os
import sys
import time
import argparse
import serial


class AsyncCSVRecorder:
    """Records validated async stream lines from serial to a CSV file."""

    def __init__(self, port='COM5', baud=115200, output_path=None):
        self.port        = port
        self.baud        = baud
        self.output_path = output_path or self._default_filename()
        self.ser         = None

        # Statistics
        self.imu_count   = 0
        self.uwb_count   = 0
        self.skipped     = 0
        self.start_time  = None

    @staticmethod
    def _default_filename():
        """Generate a timestamped filename in the script's directory."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        datasets_dir = os.path.join(script_dir, 'datasets')
        os.makedirs(datasets_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        return os.path.join(datasets_dir, f'async_{ts}.csv')

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)   # wait for Arduino reset
            print(f"Connected to {self.port} at {self.baud} baud")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def record(self):
        """
        Read lines from serial, validate format, write to CSV.
        Runs until Ctrl+C.
        """
        self.start_time = time.time()
        print(f"Recording to: {self.output_path}")
        print("Press Ctrl+C to stop.\n")

        with open(self.output_path, 'w') as f:
            try:
                while True:
                    if not self.ser or self.ser.in_waiting == 0:
                        time.sleep(0.001)
                        continue

                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue

                    parts = line.split(',')

                    # Validate IMU line (legacy 11-field or extended 14-field with gyro)
                    if parts[0] == 'I' and len(parts) in (11, 14):
                        f.write(line + '\n')
                        self.imu_count += 1

                    # Validate UWB line: U,seq,d0,d1,d2,d3,ts
                    elif parts[0] == 'U' and len(parts) == 7:
                        f.write(line + '\n')
                        self.uwb_count += 1

                    else:
                        self.skipped += 1
                        continue

                    # Periodic flush and status
                    total = self.imu_count + self.uwb_count
                    if total % 500 == 0:
                        f.flush()
                        elapsed = time.time() - self.start_time
                        imu_hz = self.imu_count / elapsed if elapsed > 0 else 0
                        uwb_hz = self.uwb_count / elapsed if elapsed > 0 else 0
                        print(f"  [{elapsed:.0f}s] IMU: {self.imu_count} "
                              f"({imu_hz:.0f} Hz)  |  UWB: {self.uwb_count} "
                              f"({uwb_hz:.0f} Hz)  |  Skipped: {self.skipped}")

            except KeyboardInterrupt:
                f.flush()
                print("\nRecording stopped.")

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def print_summary(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        print(f"\n{'='*55}")
        print(f"  ASYNC CSV RECORDER — SESSION SUMMARY")
        print(f"{'='*55}")
        print(f"  Output file    : {self.output_path}")
        print(f"  Duration       : {elapsed:.1f} s")
        print(f"  IMU packets    : {self.imu_count}")
        print(f"  UWB packets    : {self.uwb_count}")
        print(f"  Skipped lines  : {self.skipped}")
        if elapsed > 0:
            print(f"  Avg IMU rate   : {self.imu_count / elapsed:.1f} Hz")
            print(f"  Avg UWB rate   : {self.uwb_count / elapsed:.1f} Hz")
        print(f"{'='*55}\n")


def main():
    parser = argparse.ArgumentParser(
        description='PolyCast Async Stream CSV Recorder')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (default: COM5)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--output', default=None,
                        help='Output CSV path (default: auto-timestamped)')
    args = parser.parse_args()

    MODE        = "triangle"
    TESTNAME    = "1"
    
    recorder = AsyncCSVRecorder(
        port="COM3", 
        baud=921600,
        output_path=f"test/raw/{MODE}_{TESTNAME}.csv"
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
