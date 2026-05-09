"""
data_parser.py  —  PolyCast Async Stream Parser
================================================
Parses interleaved IMU (I,...) and UWB (U,...) CSV lines from either
a live serial port or a saved CSV file.

CSV format (from receiver.ino)
------------------------------
    IMU v2: I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<force>,<ts>
    IMU v1: I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>  (legacy)
    UWB:    U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

The v2 IMU packet adds calibrated gyroscope data.  Parsed packets always
include a `gyro` key and a `gyro_valid` flag so older consumers keep working
while newer code can opt into gyro use.

Usage
-----
    parser = AsyncDataParser(port='COM5')        # live serial
    parser = AsyncDataParser(csv_path='data.csv') # file playback

    parser.connect()

    while True:
        pkt = parser.get_packet()
        if pkt is None:
            continue
        if pkt == 'EOF':
            break

        if pkt['type'] == 'imu':
            ekf.predict(pkt['quat'], pkt['acc'], pkt['ts'])
        elif pkt['type'] == 'uwb':
            ekf.update(pkt['dists'], pkt['ts'])

    parser.close()
"""

import os
import sys
try:
    import serial
except ImportError:   # Allows CSV playback/tests on machines without pyserial.
    serial = None
import time
from datetime import datetime
import kuru_method.asynchronous_stream.config as config


class AsyncDataParser:
    """
    Dual-mode parser for the PolyCast async sensor stream.

    Supports:
        live  — reads from serial port (ESP32 WROOM receiver)
        csv   — replays a previously saved dataset file

    Tracks per-stream packet loss via sequence number gaps.
    """

    def __init__(self, port=config.SERIAL_PORT, baud=config.BAUD_RATE, csv_path=''):
        self.port     = port
        self.baud     = baud
        self.csv_path = csv_path

        self.mode     = 'csv' if csv_path.strip() else 'live'
        self.ser      = None
        self.csv_file = None

        # Per-stream packet loss tracking
        self._imu_last_seq = None
        self._uwb_last_seq = None
        self.imu_received  = 0
        self.imu_lost      = 0
        self.uwb_received  = 0
        self.uwb_lost      = 0

    # ── Connection ────────────────────────────────────────────────────

    def connect(self) -> bool:
        if self.mode == 'live':
            if serial is None:
                print("Connection failed: pyserial is not installed; CSV mode still works.")
                return False
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2)   # wait for Arduino reset
                print(f"Connected to LIVE stream on {self.port}")
                return True
            except Exception as e:
                print(f"Connection failed: {e}")
                return False
        else:
            try:
                path = self.csv_path
                self.csv_file = open(path, 'r')

                # Skip optional header line (non-data lines starting with #)
                first_pos  = self.csv_file.tell()
                first_line = self.csv_file.readline().strip()
                if first_line and first_line[0] in ('I', 'U'):
                    self.csv_file.seek(first_pos)   # data line — rewind

                print(f"Opened CSV dataset: {path}")
                return True
            except Exception as e:
                print(f"Failed to open CSV: {e}")
                return False

    # ── Helpers ───────────────────────────────────────────────────────

    def data_available(self) -> bool:
        """True if live serial buffer has bytes waiting."""
        return self.mode == 'live' and bool(self.ser and self.ser.in_waiting > 0)

    # ── Core read / parse ─────────────────────────────────────────────

    def get_packet(self):
        """
        Read one CSV line, parse, and return a structured dict.
        """
        line = self._read_line()
        
        if line is None:
            return None
        if line == 'EOF':          # Explicit EOF flag from _read_line
            return 'EOF'
        if line == '':
            return None            # Safely ignore blank lines / boot messages

        parts = line.split(',')
        if not parts:
            return None

        try:
            # ── IMU line, v2 current receiver format ──────────────────
            # I,seq,qx,qy,qz,qw,ax,ay,az,gx,gy,gz,force,ts
            if parts[0] == 'I' and len(parts) == 14:
                seq = int(parts[1])
                self._track_loss('imu', seq)
                return {
                    'type'      : 'imu',
                    'seq'       : seq,
                    'quat'      : (float(parts[2]), float(parts[3]),
                                   float(parts[4]), float(parts[5])),
                    'acc'       : (float(parts[6]), float(parts[7]),
                                   float(parts[8])),
                    'gyro'      : (float(parts[9]), float(parts[10]),
                                   float(parts[11])),
                    'gyro_valid': True,
                    'force'     : float(parts[12]),
                    'ts'        : int(parts[13]),
                    'format'    : 'imu_v2_gyro',
                }

            # ── IMU line, v1 legacy format ────────────────────────────
            # I,seq,qx,qy,qz,qw,ax,ay,az,force,ts
            elif parts[0] == 'I' and len(parts) == 11:
                seq = int(parts[1])
                self._track_loss('imu', seq)
                return {
                    'type'      : 'imu',
                    'seq'       : seq,
                    'quat'      : (float(parts[2]), float(parts[3]),
                                   float(parts[4]), float(parts[5])),
                    'acc'       : (float(parts[6]), float(parts[7]),
                                   float(parts[8])),
                    # Legacy files have no gyro channel.  Keep the key present
                    # so downstream code can safely do pkt.get('gyro') or
                    # pkt['gyro'] while checking gyro_valid.
                    'gyro'      : (float('nan'), float('nan'), float('nan')),
                    'gyro_valid': False,
                    'force'     : float(parts[9]),
                    'ts'        : int(parts[10]),
                    'format'    : 'imu_v1_legacy',
                }

            # ── UWB line: U,seq,d0,d1,d2,d3,ts ──────────────────────
            elif parts[0] == 'U' and len(parts) == 7:
                seq = int(parts[1])
                self._track_loss('uwb', seq)
                return {
                    'type' : 'uwb',
                    'seq'  : seq,
                    'dists': (float(parts[2]), float(parts[3]),
                              float(parts[4]), float(parts[5])),
                    'ts'   : int(parts[6]),
                    'format': 'uwb_v1',
                }

            else:
                return None     # unrecognised line (boot messages, etc.)

        except (ValueError, IndexError):
            return None         # corrupt line — silently skip

    # ── Internal ──────────────────────────────────────────────────────

    def _read_line(self):
        """
        Read one line from serial or file.
        """
        if self.mode == 'live':
            if not self.ser or self.ser.in_waiting == 0:
                return None
            try:
                return self.ser.readline().decode('utf-8', errors='replace').strip()
            except Exception:
                return None
        else:
            if not self.csv_file:
                return None
            line = self.csv_file.readline()
            if not line:
                return 'EOF'       # Explicitly return 'EOF' string, not ''
            return line.strip()

    def _track_loss(self, stream, seq):
        """Track per-stream packet loss via sequence number gaps."""
        if stream == 'imu':
            self.imu_received += 1
            if self._imu_last_seq is not None:
                gap = seq - self._imu_last_seq - 1
                if gap > 0:
                    self.imu_lost += gap
            self._imu_last_seq = seq
        else:
            self.uwb_received += 1
            if self._uwb_last_seq is not None:
                gap = seq - self._uwb_last_seq - 1
                if gap > 0:
                    self.uwb_lost += gap
            self._uwb_last_seq = seq

    # ── Properties ────────────────────────────────────────────────────

    @property
    def imu_loss_rate(self) -> float:
        """IMU packet loss rate as a fraction [0.0, 1.0]."""
        total = self.imu_received + self.imu_lost
        return self.imu_lost / total if total > 0 else 0.0

    @property
    def uwb_loss_rate(self) -> float:
        """UWB packet loss rate as a fraction [0.0, 1.0]."""
        total = self.uwb_received + self.uwb_lost
        return self.uwb_lost / total if total > 0 else 0.0

    # ── Cleanup ───────────────────────────────────────────────────────

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

    def summary(self):
        """Print per-stream statistics."""
        print(f"\n{'='*50}")
        print(f"  ASYNC STREAM — SESSION SUMMARY")
        print(f"{'='*50}")
        print(f"  IMU received : {self.imu_received}")
        print(f"  IMU lost     : {self.imu_lost}  ({self.imu_loss_rate*100:.1f}%)")
        print(f"  UWB received : {self.uwb_received}")
        print(f"  UWB lost     : {self.uwb_lost}  ({self.uwb_loss_rate*100:.1f}%)")
        print(f"{'='*50}\n")
        
if __name__ == '__main__':

    def print_pretty_packet(pkt):
        """Formats the dictionary into a clean string with a system timestamp."""
        # Capture current system time (HH:MM:SS.ms)
        sys_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        if pkt['type'] == 'imu':
            qx, qy, qz, qw = pkt['quat']
            ax, ay, az = pkt['acc']
            gx, gy, gz = pkt['gyro']
            state = f"FSR: {pkt['force']:.0f}"
            gyro_state = (f"| 🧭 G({gx:>7.4f}, {gy:>7.4f}, {gz:>7.4f}) "
                          if pkt.get('gyro_valid') else "| 🧭 G(legacy: n/a) ")
            
            # Prepend [sys_time] to the output
            print(f"[{sys_time}] 🔵 IMU | Seq: {pkt['seq']:<6} | TS: {pkt['ts']:<10} "
                f"| 🔄 Q({qx:>7.4f}, {qy:>7.4f}, {qz:>7.4f}, {qw:>7.4f}) "
                f"| 🚀 A({ax:>7.4f}, {ay:>7.4f}, {az:>7.4f}) "
                f"{gyro_state}"
                f"| {state}")

        elif pkt['type'] == 'uwb':
            d0, d1, d2, d3 = pkt['dists']
            
            # Prepend [sys_time] to the output
            print(f"[{sys_time}] 🟢 UWB | Seq: {pkt['seq']:<6} | TS: {pkt['ts']:<10} "
                f"| 📏 Dists: ({d0:>7.4f}m, {d1:>7.4f}m, {d2:>7.4f}m, {d3:>7.4f}m)")

    # ---------------------------------------------------------
    # Main Execution
    # ---------------------------------------------------------
    parser = AsyncDataParser()
    
    if not parser.connect():
        sys.exit(1)

    print("🎧 Listening for PolyCast stream... (Press Ctrl+C to stop)\n")
    
    try:
        while True:
            pkt = parser.get_packet()
            
            # Check if we hit the end of the file/stream first!
            if pkt == 'EOF':
                print("\n📁 Reached end of data stream.")
                break
                
            # If it's a valid dictionary packet, print it
            if pkt:
                print_pretty_packet(pkt)
                
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
    finally:
        parser.summary()
        parser.close()
