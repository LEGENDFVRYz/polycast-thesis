"""
validate_uwb.py — PolyCast UWB Baseline Validation
==================================================
Isolates the UWB multilateration logic to verify absolute positioning
accuracy without EKF or IMU interference.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from config import SERIAL_PORT, BAUD_RATE, ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from data_parser import AsyncDataParser
from preprocessor import UWBPreprocessor
from force_detector import ForceContactDetector
from fusion_engine import IRLSTrilateration

# Set to your dataset or leave empty for live serial
DATASET_FILENAME = '' 

def main():
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path=DATASET_FILENAME)
    if not parser.connect():
        sys.exit(1)

    uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
    contact_det = ForceContactDetector()
  
    # Two separate IRLS solvers so their 'last_pos' guesses don't cross-contaminate
    bmin = [-0.5, -0.5, -0.5]
    bmax = [ 2.0,  2.0,  1.0]
    irls_raw  = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)
    irls_prep = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    raw_x, raw_y   = [], []
    prep_x, prep_y = [], []
    is_writing     = False

    print("Listening for stream... (Press Ctrl+C to stop and plot)")

    try:
        while True:
            pkt = parser.get_packet()
            if pkt == 'EOF':
                break
            if not pkt:
                continue

            # 1. Update writing state using the IMU's force sensor
            if pkt['type'] == 'imu':
                state, _ = contact_det.process(pkt['force'])
                is_writing = bool(state)

            # 2. Process UWB distances if we are actively writing
            elif pkt['type'] == 'uwb' and is_writing:
                # get filtered (median+EMA) and ekf_dists (raw + offset + gate)
                filtered, _, ekf_dists = uwb_cleaner.process(*pkt['dists'])

                # Solve Raw
                pos_raw, _ = irls_raw.solve(ekf_dists)
                raw_x.append(pos_raw[0])
                raw_y.append(pos_raw[1])

                # Solve Preprocessed
                pos_prep, _ = irls_prep.solve(filtered)
                prep_x.append(pos_prep[0])
                prep_y.append(pos_prep[1])

    except KeyboardInterrupt:
        print("\nPlotting results...")
    finally:
        parser.close()

    # -- Plotting --
    plt.figure(figsize=(10, 10))
    plt.title("UWB-Only Trilateration Validation (Writing Strokes Only)")
    
    # Plot anchors
    plt.scatter(ANCHORS[:, 0], ANCHORS[:, 1], c='red', marker='s', s=100, label='Anchors')
    for i, a in enumerate(ANCHORS):
        plt.text(a[0], a[1] + 0.05, f'A{i}', color='red', ha='center', fontweight='bold')

    # Plot traces
    plt.plot(raw_x, raw_y, '.', color='gray', alpha=0.3, label='Raw (Offset + Gate)')
    plt.plot(prep_x, prep_y, '-', color='blue', alpha=0.8, linewidth=1.5, label='Preprocessed (Median + EMA)')

    plt.xlim(-0.3, 1.55)
    plt.ylim(-0.3, 1.55)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()