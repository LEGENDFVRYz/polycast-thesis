"""
Module 5c — Position-Level Smoother

Input  (from trilateration): Raw (x, y) coordinates
Output (Cleaned Position event): Clamped, EMA-smoothed coordinates
"""

import os
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import math
from background.pipelines.config import cfg

class UWBPositionFilter:
    def __init__(self):
        self.board_width = cfg.anchors.board_size_x
        self.board_height = cfg.anchors.board_size_y
        self.max_speed_ms = cfg.uwb.outlier_speed_limit_ms

        # Alpha-Beta filter gains
        self._alpha = cfg.uwb.pos_alpha
        self._beta  = cfg.uwb.pos_beta

        # Alpha-Beta filter state
        self._est_x: float | None = None
        self._est_y: float | None = None
        self._vel_x: float = 0.0
        self._vel_y: float = 0.0

        # Used by the velocity-outlier gate (pre-filter) and for dt
        self._prev_pos = None
        self._prev_ts = None

    def process_one(self, ev: dict) -> dict | None:
        if ev.get('sensor') != 'POSITION' or 'pos_raw' not in ev:
            return None

        raw_x, raw_y = ev['pos_raw']
        ts = ev['ts_hw']

        # ── 1. Velocity Consistency Check (Issue 6 FIXED) ──
        speed_flag = False
        if self._prev_pos is not None and self._prev_ts is not None:
            dt_s = (ts - self._prev_ts) / 1_000_000.0
            if dt_s > 0:
                dist = math.hypot(raw_x - self._prev_pos[0], raw_y - self._prev_pos[1])
                speed = dist / dt_s

                if speed > self.max_speed_ms:
                    speed_flag = True
                    if cfg.uwb.drop_speed_outliers:
                        return None

        # ── 2. Boundary Clamping ──
        clamped_x = max(0.0, min(self.board_width, raw_x))
        clamped_y = max(0.0, min(self.board_height, raw_y))

        # ── 3. Alpha-Beta Filter ──
        dt_s = (ts - self._prev_ts) / 1_000_000.0 if self._prev_ts is not None else 1.0 / cfg.uwb.rate_hz
        if dt_s <= 0:
            dt_s = 1.0 / cfg.uwb.rate_hz

        if self._est_x is None:
            # First sample: seed the estimate directly
            self._est_x, self._est_y = clamped_x, clamped_y
        else:
            # Predict
            pred_x = self._est_x + self._vel_x * dt_s
            pred_y = self._est_y + self._vel_y * dt_s
            # Residual
            err_x = clamped_x - pred_x
            err_y = clamped_y - pred_y
            # Update position and velocity
            self._est_x = pred_x + self._alpha * err_x
            self._est_y = pred_y + self._alpha * err_y
            self._vel_x = self._vel_x + (self._beta * err_x) / dt_s
            self._vel_y = self._vel_y + (self._beta * err_y) / dt_s

        # When geometry is bad or the point was clamped, the velocity the
        # alpha-beta filter learned is unreliable. Decay it so the filter
        # doesn't coast on a phantom trajectory between UWB corrections.
        low_confidence = ev.get('low_confidence', False)
        was_clamped_early = (
            max(0.0, min(self.board_width,  raw_x)) != raw_x or
            max(0.0, min(self.board_height, raw_y)) != raw_y
        )
        if low_confidence or was_clamped_early:
            self._vel_x *= 0.2
            self._vel_y *= 0.2

        clean_x, clean_y = self._est_x, self._est_y

        self._prev_pos = (clamped_x, clamped_y)
        self._prev_ts = ts

        # --- THE FIX: Explicit Axis Definition ---
        # We keep 'pos_clean' untouched so your hardware plotting script doesn't break
        ev['pos_clean']  = (round(clean_x, 4), round(clean_y, 4))
        ev['speed_flag'] = speed_flag

        # We add explicitly labeled axes for Module 7 (The Fusion Node)
        ev['mapped_position'] = {
            'board_width_x': round(clean_x, 4),
            'board_height_y': round(clean_y, 4),
            'depth_z': 0.07  # The physical offset of the anchors from the whiteboard
        }
        ev['coordinate_frame'] = 'UWB_BOARD_XY'

        # Quality metadata propagated to ESKF for adaptive R inflation
        was_clamped = (clamped_x != raw_x) or (clamped_y != raw_y)
        ev['uwb_quality'] = {
            'solve_error':    ev.get('solve_error', 0.0),
            'speed_flag':     speed_flag,
            'was_clamped':    was_clamped,
            'low_confidence': ev.get('low_confidence', False),
        }
        
        return ev

    def reset(self):
        self._est_x = None
        self._est_y = None
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._prev_pos = None
        self._prev_ts = None



# ==============================================================================
# HARDWARE DATA LOGGING & REPORTING (Module 5c: Position Smoother)
#   - Goal: Validate trajectory smoothing, speed-limit jumps, and board clamping.
#   - Expectation: Blue line is smooth and stays strictly inside the board bounds,
#                  even if the red line (raw math) jumps wildly outside.
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
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.config import cfg

    SERIAL_PORT = cfg.serial.port
    BAUD_RATE = cfg.serial.baud
    DISPLAY_RATE = 0.2

    # Initialize the FULL UWB Pipeline
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    uwb_offsets = cfg.uwb.range_offsets_m
    range_prep = UWBRangePreprocessor(offsets=uwb_offsets)
    solver = UWBSolver()
    
    # Initialize the module we are testing
    pos_filter = UWBPositionFilter()

    print("=" * 60)
    print(f"  [TEST] PHYSICS POLICE: Position Filter: {SERIAL_PORT}")
    print("  Press Ctrl+C to stop and generate Trajectory Smoothing reports.")
    print("=" * 60)

    # --- REVISION: Ground Truth Prompt ---
    ground_truth = None
    print("Do you want to test clean position accuracy against a specific known coordinate? (y/n)")
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
                    raw_pos_event = solver.process_one(cr)
                    
                    if raw_pos_event:
                        # Pass raw math into the Position Filter
                        clean_pos_event = pos_filter.process_one(raw_pos_event)
                        
                        if clean_pos_event:
                            event_log.append(clean_pos_event)
                            
                            current_time = time.time()
                            if current_time - last_print_time >= DISPLAY_RATE:
                                os.system('cls' if os.name == 'nt' else 'clear')
                                print(f"========= TRAJECTORY SMOOTHER ({DISPLAY_RATE}s) =========")
                                print(f"  Pkt ID     : {clean_pos_event['packet_id']}")
                                print(f"  Raw Input  : X: {clean_pos_event['pos_raw'][0]:6.3f} m  |  Y: {clean_pos_event['pos_raw'][1]:6.3f} m")
                                print(f"  CLEAN OUT  : X: {clean_pos_event['pos_clean'][0]:6.3f} m  |  Y: {clean_pos_event['pos_clean'][1]:6.3f} m")
                                
                                # --- REVISION: Live Error Display ---
                                if ground_truth:
                                    dist_err = math.hypot(clean_pos_event['pos_clean'][0] - ground_truth[0], clean_pos_event['pos_clean'][1] - ground_truth[1])
                                    print(f"  POS ERROR  : {dist_err:.4f} m from target (Cleaned)")
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
            errors_m = [math.hypot(ev['pos_clean'][0] - ground_truth[0], ev['pos_clean'][1] - ground_truth[1]) for ev in event_log]
            avg_error = sum(errors_m) / len(errors_m)
            print("\n" + "=" * 60)
            print(f"  [ACCURACY REPORT (CLEANED POSITION)]")
            print(f"  Target Coordinate : X={ground_truth[0]:.3f}, Y={ground_truth[1]:.3f}")
            print(f"  Samples Evaluated : {len(errors_m)}")
            print(f"  Average Error     : {avg_error:.4f} meters ({avg_error * 100:.2f} cm)")
            print("=" * 60 + "\n")
        # -------------------------------------------------

        # --- CSV EXPORT ---
        csv_filename = "position_filter_report.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ts_hw', 'packet_id', 'raw_x', 'raw_y', 'clean_x', 'clean_y'])
            for ev in event_log:
                writer.writerow([ev['ts_hw'], ev['packet_id'], ev['pos_raw'][0], ev['pos_raw'][1], ev['pos_clean'][0], ev['pos_clean'][1]])
        print(f"[EXPORT] Saved to {csv_filename}")

        # --- PLOT REPORT ---
        print("[PLOT] Rendering Trajectory Report...")
        raw_x = [ev['pos_raw'][0] for ev in event_log]
        raw_y = [ev['pos_raw'][1] for ev in event_log]
        clean_x = [ev['pos_clean'][0] for ev in event_log]
        clean_y = [ev['pos_clean'][1] for ev in event_log]
        t_sec = [(ev['ts_hw'] - event_log[0]['ts_hw']) / 1_000_000.0 for ev in event_log]

        # 3-Panel Plot to prove Physics, Clamping, and Smoothing
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle('Module 5c: Position Filtering & Boundary Clamping', fontsize=16, fontweight='bold')

        # Main Plot: 2D Spatial Board
        ax1 = plt.subplot(2, 1, 1)
        board_w = cfg.anchors.board_size_x
        board_h = cfg.anchors.board_size_y
        ax1.add_patch(plt.Rectangle((0, 0), board_w, board_h, fill=False, edgecolor='black', linestyle='--', lw=2))
        ax1.scatter([0, board_w, board_w, 0], [0, 0, board_h, board_h], c='red', s=100, marker='s', label='Anchors')
        
        ax1.plot(raw_x, raw_y, label='Raw Math Trajectory (Can exit bounds)', color='red', alpha=0.3, linewidth=1, marker='.')
        ax1.plot(clean_x, clean_y, label='Filtered Trajectory (Clamped & Smoothed)', color='blue', linewidth=2)
        
        # --- REVISION: Plot Ground Truth if available ---
        if ground_truth:
            ax1.plot(ground_truth[0], ground_truth[1], marker='X', color='green', markersize=12, label='Ground Truth Target')
        # ------------------------------------------------
        
        ax1.set_xlim(-0.2, board_w + 0.2)
        ax1.set_ylim(-0.2, board_h + 0.2)
        ax1.set_aspect('equal')
        ax1.set_title('2D Board Tracking')
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend()

        # Subplot: X-Axis over Time
        ax2 = plt.subplot(2, 2, 3)
        ax2.plot(t_sec, raw_x, color='red', alpha=0.3, label='Raw X')
        ax2.plot(t_sec, clean_x, color='blue', linewidth=2, label='Clean X')
        ax2.axhline(y=0, color='black', linestyle='--')
        ax2.axhline(y=board_w, color='black', linestyle='--')
        ax2.set_title('X-Axis Smoothing & Clamping')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('X Position (m)')
        ax2.grid(True, linestyle='--', alpha=0.5)

        # Subplot: Y-Axis over Time
        ax3 = plt.subplot(2, 2, 4, sharex=ax2)
        ax3.plot(t_sec, raw_y, color='red', alpha=0.3, label='Raw Y')
        ax3.plot(t_sec, clean_y, color='blue', linewidth=2, label='Clean Y')
        ax3.axhline(y=0, color='black', linestyle='--')
        ax3.axhline(y=board_h, color='black', linestyle='--')
        ax3.set_title('Y-Axis Smoothing & Clamping')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Y Position (m)')
        ax3.grid(True, linestyle='--', alpha=0.5)

        plt.tight_layout()
        plt.savefig("position_filter_report.png", dpi=300)
        plt.show()