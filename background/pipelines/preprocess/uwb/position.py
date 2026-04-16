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
        
        # Issue 3: Hardcoded board height (FIXED)
        self.board_height = cfg.anchors.board_size_y
        
        # Issue 4: Position EMA too strong (FIXED)
        self.pos_ema_alpha = cfg.uwb.pos_ema_alpha
        
        # Issue 6: Velocity limit
        self.max_speed_ms = cfg.uwb.outlier_speed_limit_ms
        
        self._prev_pos = None
        self._prev_ts = None

    def process_one(self, ev: dict) -> dict | None:
        if ev.get('sensor') != 'POSITION' or 'pos_raw' not in ev:
            return None

        raw_x, raw_y = ev['pos_raw']
        ts = ev['ts_hw']

        # ── 1. Velocity Consistency Check (Issue 6 FIXED) ──
        if self._prev_pos is not None and self._prev_ts is not None:
            dt_s = (ts - self._prev_ts) / 1_000_000.0
            if dt_s > 0:
                dist = math.hypot(raw_x - self._prev_pos[0], raw_y - self._prev_pos[1])
                speed = dist / dt_s
                
                if speed > self.max_speed_ms:
                    # Physically impossible jump! Drop the event entirely.
                    return None

        # ── 2. Boundary Clamping ──
        clamped_x = max(0.0, min(self.board_width, raw_x))
        clamped_y = max(0.0, min(self.board_height, raw_y))

        # ── 3. EMA Smoothing ──
        if self._prev_pos is None:
            clean_x, clean_y = clamped_x, clamped_y
        else:
            clean_x = (self.pos_ema_alpha * clamped_x) + ((1 - self.pos_ema_alpha) * self._prev_pos[0])
            clean_y = (self.pos_ema_alpha * clamped_y) + ((1 - self.pos_ema_alpha) * self._prev_pos[1])
            
        self._prev_pos = (clean_x, clean_y)
        self._prev_ts = ts

        # Inject the clean position into the same event payload
        ev['pos_clean'] = (round(clean_x, 4), round(clean_y, 4))
        return ev

    def reset(self):
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