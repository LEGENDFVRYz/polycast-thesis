"""
uwb_raw_processed.py — PolyCast UWB Baseline Validation (Live Animation)
========================================================================
Isolates the UWB multilateration logic to verify absolute positioning
accuracy without EKF or IMU interference, rendered in real-time.
"""

import os
import sys
from pathlib import Path
import numpy as np

# Headless backend setup for CI/Saving
if os.environ.get('SAVE_PNG') or os.environ.get('UWB_RAW_HEADLESS'):
    import matplotlib
    matplotlib.use('Agg')
    
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from kuru_method.asynchronous_stream.config import SERIAL_PORT, BAUD_RATE, ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from kuru_method.asynchronous_stream.data_parser import AsyncDataParser
from kuru_method.asynchronous_stream.preprocessor import UWBPreprocessor
from kuru_method.asynchronous_stream.force_detector import ForceContactDetector
from kuru_method.asynchronous_stream.fusion_engine import IRLSTrilateration

# Set to your dataset or leave empty for live serial.
DATASET_FILENAME = ''  # '' = live serial;  'datasets/data.csv' = playback


class LiveUWBValidator:
    def __init__(self, parser):
        self.parser = parser
        self.uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
        self.contact_det = ForceContactDetector()
        
        # Two separate IRLS solvers so their 'last_pos' guesses don't cross-contaminate
        bmin = [-0.5, -0.5, -0.5]
        bmax = [ 2.0,  2.0,  1.0]
        self.irls_raw  = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)
        self.irls_prep = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

        # Data arrays
        self.raw_x, self.raw_y   = [], []
        self.prep_x, self.prep_y = [], []
        self.is_writing = False

        # --- Setup Matplotlib Figure ---
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("UWB-Only Trilateration Validation (Live Writing Strokes)")
        
        # Plot anchors
        self.ax.scatter(ANCHORS[:, 0], ANCHORS[:, 1], c='red', marker='s', s=100, label='Anchors')
        for i, a in enumerate(ANCHORS):
            self.ax.text(a[0], a[1] + 0.05, f'A{i}', color='red', ha='center', fontweight='bold')

        # Initialize empty lines for animation
        self.line_raw, = self.ax.plot([], [], '.', color='gray', alpha=0.3, label='Raw (Offset + Gate)')
        self.line_prep, = self.ax.plot([], [], '-', color='blue', alpha=0.8, linewidth=1.5, label='Preprocessed (Median + EMA)')

        # Grid and Axis config
        self.ax.set_xlim(-0.3, 1.55)
        self.ax.set_ylim(-0.3, 1.55)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, alpha=0.4)
        self.ax.legend()

    def update(self, frame):
        # Drain up to 50 packets per visual frame to keep the UI from freezing
        # (At 50Hz UWB and 200Hz IMU, 50 packets easily clears the async queue)
        for _ in range(50):
            pkt = self.parser.get_packet()
            
            if pkt == 'EOF':
                break  # End of dataset
            if not pkt:
                break  # Queue is empty, break inner loop to render the frame

            # 1. Update writing state using the IMU's force sensor
            if pkt['type'] == 'imu':
                state, _ = self.contact_det.process(pkt['force'])
                self.is_writing = bool(state)

            # 2. Process UWB distances if we are actively writing
            elif pkt['type'] == 'uwb' and self.is_writing:
                # get filtered (median+EMA) and ekf_dists (raw + offset + gate)
                filtered, _, ekf_dists = self.uwb_cleaner.process(*pkt['dists'])

                # Solve Raw
                pos_raw, _ = self.irls_raw.solve(ekf_dists)
                self.raw_x.append(pos_raw[0])
                self.raw_y.append(pos_raw[1])

                # Solve Preprocessed
                pos_prep, _ = self.irls_prep.solve(filtered)
                self.prep_x.append(pos_prep[0])
                self.prep_y.append(pos_prep[1])

        # Apply new data to the plot lines
        self.line_raw.set_data(self.raw_x, self.raw_y)
        self.line_prep.set_data(self.prep_x, self.prep_y)
        
        return self.line_raw, self.line_prep


def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else DATASET_FILENAME
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path=dataset)
    
    if not parser.connect():
        sys.exit(1)

    print("🚀 Starting Live Stream Animation... (Close the window to exit)")
    
    # Initialize the validator class
    validator = LiveUWBValidator(parser)
    
    # Run the real-time animation loop (refreshing roughly every 16ms / ~60 FPS)
    ani = FuncAnimation(validator.fig, validator.update, interval=16, blit=True, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nPlotting interrupted.")
    finally:
        parser.close()
        
        # Optional: Print final R95 stat when you close the window
        if validator.raw_x:
            raw_arr  = np.column_stack([validator.raw_x, validator.raw_y])
            prep_arr = np.column_stack([validator.prep_x, validator.prep_y])
            def _r95(a):
                c = a.mean(0)
                return float(np.quantile(np.linalg.norm(a - c, axis=1), 0.95))
            
            stem = Path(dataset).stem if dataset else 'live'
            print(f'[{stem}] R95 raw={_r95(raw_arr):.4f} m  prep={_r95(prep_arr):.4f} m  n={len(validator.raw_x)}')

if __name__ == '__main__':
    main()