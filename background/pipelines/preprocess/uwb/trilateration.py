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
        # The anchors are at Z=0.01 (per AnchorConfig). The 3D distance math inherently handles this offset!
        self.pen_z = 0.0

        # Bounds and initial guess
        self.bounds_min = [-0.10, -0.10]
        self.bounds_max = [self.board_width + 0.10, self.board_height + 0.10]
        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])

        # Stale-guess recovery: track last successful solve timestamp
        self._last_valid_ts: int | None = None
        self._stale_timeout_us: int = cfg.uwb.stale_guess_timeout_us

    def _residuals(self, guess_xy, distances, valid_anchors, weights):
        """Weighted residuals for WLS: multiplying by sqrt(w) makes least_squares minimize Σ w·r²."""
        guess_xyz = np.array([guess_xy[0], guess_xy[1], self.pen_z])
        calc_dists = np.linalg.norm(valid_anchors - guess_xyz, axis=1)
        return (calc_dists - distances) * np.sqrt(weights)

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

        # Stale-guess recovery: if pen was lifted/relocated, re-seed from anchor centroid
        ts = ev['ts_hw']
        if (self._last_valid_ts is None or
                (ts - self._last_valid_ts) > self._stale_timeout_us):
            self._guess = np.mean(valid_anchors[:, :2], axis=0)

        try:
            # IDW weights: closer anchors trusted more (shorter range = less multipath opportunity)
            raw_w = 1.0 / (valid_dists ** cfg.uwb.wls_power + cfg.uwb.wls_epsilon)
            weights = raw_w * (len(raw_w) / raw_w.sum())  # normalize so mean weight = 1

            res = least_squares(
                self._residuals,
                self._guess,
                bounds=(self.bounds_min, self.bounds_max),
                args=(valid_dists, valid_anchors, weights),
                loss='soft_l1',
                f_scale=0.1
            )

            raw_x, raw_y = res.x

            # Compute RMS on unweighted geometric residuals so trilat_max_residual keeps its
            # physical meaning in metres (res.fun contains weighted residuals after WLS).
            unweighted = np.linalg.norm(
                valid_anchors - np.array([raw_x, raw_y, self.pen_z]), axis=1
            ) - valid_dists
            rms_error = float(np.sqrt(np.mean(unweighted ** 2)))
            if rms_error > cfg.uwb.trilat_max_residual:
                return None  # Math converged, but to a garbage location

            low_confidence = rms_error > 0.08  # warning band: 0.08–0.12 m

            self._guess = res.x
            self._last_valid_ts = ts

            return {
                'sensor':          'POSITION',
                'ts_hw':           ev['ts_hw'],
                'packet_id':       ev['packet_id'],
                'pos_raw':         (round(raw_x, 4), round(raw_y, 4)),
                'solve_error':     round(rms_error, 6),
                'low_confidence':  low_confidence,
            }

        except Exception:
            return None

    def reset(self):
        self._guess = np.array([self.board_width / 2.0, self.board_height / 2.0])
        self._last_valid_ts = None



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
    import math
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

    # --- REVISION: Ground Truth Prompt ---
    ground_truth = None
    print("Do you want to test accuracy against a specific known coordinate? (y/n)")
    if input().strip().lower() == 'y':
        try:
            gt_x = float(input("  Enter expected X coordinate (m): "))
            gt_y = float(input("  Enter expected Y coordinate (m): "))
            ground_truth = (gt_x, gt_y)
            print(f"  [SET] Target ground truth: X={gt_x:.3f}, Y={gt_y:.3f}")
        except ValueError:
            print("  [ERROR] Invalid input. Proceeding without ground truth.")
    print("=" * 60)
    # -------------------------------------

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
                            
                            # --- REVISION: Live Error Display ---
                            if ground_truth:
                                dist_err = math.hypot(pos_event['pos_raw'][0] - ground_truth[0], pos_event['pos_raw'][1] - ground_truth[1])
                                print(f"  POS ERROR  : {dist_err:.4f} m from target")
                            # ------------------------------------
                            
                            print("==========================================================")
                            last_print_time = current_time
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            exit()

        # --- REVISION: Final Average Error Calculation ---
        if ground_truth:
            errors_m = [math.hypot(ev['pos_raw'][0] - ground_truth[0], ev['pos_raw'][1] - ground_truth[1]) for ev in event_log]
            avg_error = sum(errors_m) / len(errors_m)
            print("\n" + "=" * 60)
            print(f"  [ACCURACY REPORT]")
            print(f"  Target Coordinate : X={ground_truth[0]:.3f}, Y={ground_truth[1]:.3f}")
            print(f"  Samples Evaluated : {len(errors_m)}")
            print(f"  Average Error     : {avg_error:.4f} meters ({avg_error * 100:.2f} cm)")
            print("=" * 60 + "\n")
        # -------------------------------------------------

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
            
            # --- REVISION: Plot Ground Truth if available ---
            if ground_truth:
                ax1.plot(ground_truth[0], ground_truth[1], marker='X', color='blue', markersize=12, label='Ground Truth Target')
            # ------------------------------------------------
            
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