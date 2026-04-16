"""
Module 5b — UWB Trilateration Solver

Input  (from range preprocessor): Cleaned distances and valid mask
Output (Position event): Raw (x,y) intersection and solve_error
"""

import os
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import numpy as np
from scipy.optimize import least_squares
from background.pipelines.config import cfg


class UWBSolver:
    def __init__(self):
        # Anchor configurations
        self.anchors = np.array(cfg.anchors.positions)
        self.board_width = cfg.anchors.board_size_x
        self.board_height = getattr(cfg.anchors, 'board_size_y', 1.24) 
        
        # Issue 5: Z-plane assumption. Pen is at Z=0.0. 
        # The anchors are at Z=0.07. The 3D distance math inherently handles this offset!
        self.pen_z = 0.0  
        
        # Bounds and initial guess
        self.bounds_min = [-0.10, -0.10]
        self.bounds_max = [self.board_width + 0.10, self.board_height + 0.10] 
        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])

    def _residuals(self, guess_xy, distances, valid_anchors):
        """Calculate the error between the guess and the actual measured ranges."""
        guess_xyz = np.array([guess_xy[0], guess_xy[1], self.pen_z])
        calc_dists = np.linalg.norm(valid_anchors - guess_xyz, axis=1)
        return calc_dists - distances

    def process_one(self, ev: dict) -> dict | None:
        if ev.get('sensor') != 'UWB' or 'clean_dists' not in ev:
            return None

        # Issue 1: IGNORING valid_mask (FIXED)
        clean_dists = np.array(ev['clean_dists'])
        valid_mask = np.array(ev.get('valid_mask', [True] * 4), dtype=bool)
        
        valid_dists = clean_dists[valid_mask]
        valid_anchors = self.anchors[valid_mask]

        # Trilateration fundamentally requires at least 3 valid spheres to intersect
        if len(valid_dists) < 3:
            return None
        
        try:
            res = least_squares(
                self._residuals,
                self._guess,
                bounds=(self.bounds_min, self.bounds_max),
                args=(valid_dists, valid_anchors),
                loss='soft_l1',
                f_scale=0.1
            )
            
            raw_x, raw_y = res.x
            cost = res.cost
            
            # Issue 2: No confidence gating (FIXED)
            rms_error = float(np.sqrt(np.mean(res.fun ** 2)))
            if rms_error > cfg.uwb.trilat_max_residual:
                return None  # Math converged, but to a garbage location
            
            self._guess = res.x

            return {
                'sensor':      'POSITION',
                'ts_hw':       ev['ts_hw'],
                'packet_id':   ev['packet_id'],
                'pos_raw':     (round(raw_x, 4), round(raw_y, 4)),
                'solve_error': round(cost, 6)
            }

        except Exception:
            return None

    def reset(self):
        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])



# ==============================================================================
# HARDWARE DATA LOGGING & REPORTING (Module 5b: Trilateration Solver)
#   - Goal: Validate the pure mathematical intersection of the UWB ranges.
#   - Expectation: The 2D plot will show the natural "raw scatter" of UWB.
#                  The Error plot will prove if the ranges intersect cleanly.
# ==============================================================================
if __name__ == '__main__':
    import time
    import os
    import csv
    import matplotlib.pyplot as plt
    
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.config import cfg

    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    DISPLAY_RATE = 0.2

    # Initialize up to Solver
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    uwb_offsets = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offsets)
    
    solver = UWBSolver()

    print("=" * 60)
    print(f"  [TEST] PURE MATH: Trilateration Solver: {SERIAL_PORT}")
    print("  Press Ctrl+C to stop and generate raw mathematical reports.")
    print("=" * 60)

    event_log = []
    last_print_time = 0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                events = norm.normalize(raw_packets)
                clean_ranges = range_prep.feed(events)
                
                for cr in clean_ranges:
                    pos_event = solver.process_one(cr)
                    
                    if pos_event:
                        event_log.append(pos_event)
                        
                        current_time = time.time()
                        if current_time - last_print_time >= DISPLAY_RATE:
                            os.system('cls' if os.name == 'nt' else 'clear')
                            print(f"========= RAW TRILATERATION MATH ({DISPLAY_RATE}s) =========")
                            print(f"  Pkt ID     : {pos_event['packet_id']}")
                            print(f"  Solve Cost : {pos_event['solve_error']:.6f} (Math Confidence)")
                            print(f"  RAW POS    : X: {pos_event['pos_raw'][0]:6.3f} m  |  Y: {pos_event['pos_raw'][1]:6.3f} m")
                            print("==========================================================")
                            last_print_time = current_time
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            exit()

        # --- CSV EXPORT ---
        csv_filename = "trilateration_math_report.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ts_hw', 'packet_id', 'raw_x', 'raw_y', 'solve_error'])
            for ev in event_log:
                writer.writerow([ev['ts_hw'], ev['packet_id'], ev['pos_raw'][0], ev['pos_raw'][1], ev['solve_error']])
        print(f"[EXPORT] Saved to {csv_filename}")

        # --- PLOT REPORT ---
        print("[PLOT] Rendering Math Report...")
        try:
            raw_x = [ev['pos_raw'][0] for ev in event_log]
            raw_y = [ev['pos_raw'][1] for ev in event_log]
            errors = [ev['solve_error'] for ev in event_log]
            t_sec = [(ev['ts_hw'] - event_log[0]['ts_hw']) / 1_000_000.0 for ev in event_log]

            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            fig.suptitle('Module 5b: Pure Trilateration Math', fontsize=16, fontweight='bold')

            # Subplot 1: Raw Scatter
            board_w = getattr(cfg.anchors, 'board_size_m', 1.25)
            board_h = getattr(cfg.anchors, 'board_size_y', 1.24)
            ax1.add_patch(plt.Rectangle((0, 0), board_w, board_h, fill=False, edgecolor='black', linestyle='--', lw=2))
            ax1.scatter([0, board_w, board_w, 0], [0, 0, board_h, board_h], c='red', s=100, marker='s', label='Anchors')
            ax1.plot(raw_x, raw_y, label='Raw Solved Coordinates', color='red', alpha=0.5, marker='.', linestyle='none')
            ax1.set_xlim(-0.2, board_w + 0.2)
            ax1.set_ylim(-0.2, board_h + 0.2)
            ax1.set_aspect('equal')
            ax1.set_title('Raw Geometric Intersections')
            ax1.grid(True, linestyle=':', alpha=0.7)
            ax1.legend()

            # Subplot 2: Solve Error
            ax2.plot(t_sec, errors, color='purple', linewidth=1.5)
            ax2.axhline(y=getattr(cfg.uwb, 'trilat_max_residual', 0.15), color='red', linestyle='--', label='Rejection Threshold')
            ax2.set_title('Least-Squares Optimization Cost (Confidence)')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Residual Error (Lower = Better geometric fit)')
            ax2.grid(True, linestyle='--', alpha=0.5)
            ax2.legend()

            plt.tight_layout()
            
            # Save the PNG first, just in case the UI crashes!
            plot_filename = "trilateration_math_report.png"
            plt.savefig(plot_filename, dpi=300)
            print(f"[EXPORT] Plot saved successfully to {plot_filename}")
            
            # Now attempt to show the interactive UI
            plt.show(block=True)
            
        except KeyboardInterrupt:
            # Ignore ghost Ctrl+C signals sent to the Qt GUI
            print("\n[INFO] Matplotlib UI interrupted. Image was saved to disk.")
        except Exception as e:
            print(f"\n[ERROR] Plotting failed: {e}")