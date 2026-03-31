"""
data_stream.py  —  PolyCast Data Ingestion Layer
=================================================
Supports two modes:
    live — reads from ESP32 WROOM over serial
    csv  — replays a previously-recorded dataset file

get_packet() now returns 'uwb_weights' in the packet dict so that
main.py can forward them into FusionEngine for IRLS weighting.
"""

import os
import serial
import time
from preprocessor import UWBPreprocessor, IMUPreprocessor


class DataStream:
    def __init__(self, port, baud_rate, dataset_filename=""):
        self.port             = port
        self.baud_rate        = baud_rate
        self.dataset_filename = dataset_filename

        self.mode     = "csv" if dataset_filename.strip() else "live"
        self.ser      = None
        self.csv_file = None

        self.uwb_cleaner = UWBPreprocessor(offsets=(-0.19, -0.20, -0.15, -0.40))
        self.imu_cleaner = IMUPreprocessor()

    # ── connection ────────────────────────────────────────────────────

    def connect(self):
        if self.mode == "live":
            try:
                self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
                time.sleep(2)               # wait for Arduino reset
                print(f"✅ Connected to LIVE stream on {self.port}")
                return True
            except Exception as e:
                print(f"❌ Connection Failed: {e}")
                return False
        else:
            try:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                full_path  = os.path.join(script_dir, self.dataset_filename)
                self.csv_file = open(full_path, 'r')

                # Auto-detect and skip optional CSV header row
                first_pos  = self.csv_file.tell()
                first_line = self.csv_file.readline()
                try:
                    float(first_line.split(',')[0])
                    self.csv_file.seek(first_pos)   # numeric first column → no header
                except ValueError:
                    pass                            # non-numeric → header, already consumed

                print(f"✅ Opened CSV dataset: {full_path}")
                return True
            except Exception as e:
                print(f"❌ Failed to open CSV: {e}")
                return False

    # ── helpers ───────────────────────────────────────────────────────

    def data_available(self):
        """True if live serial buffer has bytes waiting."""
        return self.mode == "live" and bool(self.ser and self.ser.in_waiting > 0)

    # ── core read / parse ─────────────────────────────────────────────

    def get_packet(self):
        """
        Read one CSV line, parse, preprocess, and return a structured dict.

        Returns
        -------
        dict  — successfully parsed packet
        "EOF" — end of CSV file (csv mode only)
        None  — no data yet / line corrupt / wrong column count
        """
        # 1. Read one line
        line = ""
        if self.mode == "live":
            if not self.ser or self.ser.in_waiting == 0:
                return None
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
            except Exception:
                return None
        else:
            if not self.csv_file:
                return None
            line = self.csv_file.readline().strip()
            if not line:
                return "EOF"

        if not line:
            return None

        # 2. Parse
        try:
            parts = line.split(',')

            # Expected layout:
            #   3 headers  +  4 UWB  +  (9 fields × 5 samples)  =  52 columns
            if len(parts) != 52:
                return None

            # ── Headers ───────────────────────────────────────────────
            seq      = int(parts[0])
            batch_ts = int(parts[1])
            uwb_ts   = int(parts[2])

            # ── UWB distances ─────────────────────────────────────────
            raw_d0 = float(parts[3])
            raw_d1 = float(parts[4])
            raw_d2 = float(parts[5])
            raw_d3 = float(parts[6])

            # Preprocessor now returns (filtered_dists, quality_weights)
            (d0, d1, d2, d3), (w0, w1, w2, w3) = self.uwb_cleaner.process(
                raw_d0, raw_d1, raw_d2, raw_d3
            )

            # ── IMU batch (5 samples × 9 fields) ─────────────────────
            imu_batch = []
            start_idx = 7
            for i in range(5):
                off = start_idx + i * 9
                qx, qy, qz, qw_ = (float(parts[off]),   float(parts[off+1]),
                                    float(parts[off+2]), float(parts[off+3]))
                ax, ay, az       = (float(parts[off+4]), float(parts[off+5]),
                                    float(parts[off+6]))
                force = float(parts[off+7])
                ts    = int(parts[off+8])

                clean = self.imu_cleaner.process_sample(qx, qy, qz, qw_, ax, ay, az)

                imu_batch.append({
                    'quat' : clean[0:4],
                    'acc'  : clean[4:7],
                    'force': force,
                    'ts'   : ts,
                })

            return {
                'seq'        : seq,
                'batch_ts'   : batch_ts,
                'uwb_ts'     : uwb_ts,
                'uwb'        : (d0, d1, d2, d3),           # filtered distances
                'uwb_raw'    : (raw_d0, raw_d1, raw_d2, raw_d3),
                'uwb_weights': (w0, w1, w2, w3),            # ← NEW: quality weights
                'imu'        : imu_batch,
            }

        except (ValueError, IndexError):
            return None             # corrupt line — silently skip

    # ── cleanup ───────────────────────────────────────────────────────

    def close(self):
        if self.ser:
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()