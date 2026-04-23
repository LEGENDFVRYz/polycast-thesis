"""
Module 7 — Baseline Sensor Fusion v1

Implements a Complementary Filter (Filter-and-Correct).
- IMU drives fast, local, high-resolution kinematic integration (Velocity -> Position).
- UWB provides slow, absolute global coordinate corrections to kill IMU drift.
- Contact State Detector gates when strokes are actively "drawn".
"""

import math
from background.pipelines.config import cfg

class FusionEngine:
    def __init__(self, fusion_alpha: float = 0.15):
        # ── State Variables ──
        self.pos_x = getattr(cfg.anchors, 'board_size_m', 1.25) / 2.0
        self.pos_y = getattr(cfg.anchors, 'board_size_y', 1.24) / 2.0
        
        self.vel_x = 0.0
        self.vel_y = 0.0
        
        # ── Tuning ──
        self.fusion_alpha = fusion_alpha  # How strongly to trust UWB
        
        # ── Tracking ──
        self.last_ts = None
        
        # We store the last known clean UWB coordinate for reference
        self.last_uwb_x = self.pos_x
        self.last_uwb_y = self.pos_y

    def process_event(self, ev: dict) -> dict | None:
        ts = ev.get('ts_hw')
        if ts is None:
            return None

        # Calculate dt (Time Delta)
        if self.last_ts is not None:
            dt_s = (ts - self.last_ts) / 1_000_000.0
            if dt_s <= 0 or dt_s > 0.5:
                dt_s = 0.02
        else:
            dt_s = 0.02
            
        self.last_ts = ts

        # ==========================================
        # PATH A: IMU UPDATE (The Driver)
        # ==========================================
        if ev['sensor'] == 'IMU':
            # 1. Apply ZUPT (The Brakes)
            if ev.get('is_static', False):
                self.vel_x = 0.0
                self.vel_y = 0.0
            else:
                # 2. Integrate Acceleration -> Velocity
                acc_x, acc_y = ev.get('acc_board', (0.0, 0.0))
                self.vel_x += acc_x * dt_s
                self.vel_y += acc_y * dt_s

            # 3. Integrate Velocity -> Position (Relative Displacement)
            self.pos_x += self.vel_x * dt_s
            self.pos_y += self.vel_y * dt_s
            
            state_str = ev.get('stroke_state', 'UNKNOWN')
            stroke_id = ev.get('stroke_id', 0)
            is_active = ev.get('stroke_active', False)

        # ==========================================
        # PATH B: UWB UPDATE (The Corrector)
        # ==========================================
        elif ev['sensor'] == 'POSITION':
            # 1. Extract the smoothed, clamped coordinate from Module 5c
            uwb_x, uwb_y = ev.get('pos_clean', (self.pos_x, self.pos_y))
            self.last_uwb_x, self.last_uwb_y = uwb_x, uwb_y
            
            # 2. Apply Complementary Correction (Drift Killer)
            self.pos_x = self.pos_x + self.fusion_alpha * (uwb_x - self.pos_x)
            self.pos_y = self.pos_y + self.fusion_alpha * (uwb_y - self.pos_y)
            
            # Pass-through generic states since UWB doesn't know about ink
            state_str = 'UWB_CORRECTION'
            stroke_id = 0
            is_active = False

        else:
            return None

        # ==========================================
        # BOUNDARY CLAMPING
        # ==========================================
        board_w = getattr(cfg.anchors, 'board_size_m', 1.25)
        board_h = getattr(cfg.anchors, 'board_size_y', 1.24)
        
        self.pos_x = max(0.0, min(board_w, self.pos_x))
        self.pos_y = max(0.0, min(board_h, self.pos_y))

        return {
            'ts_hw': ts,
            'source': ev['sensor'],
            'fused_x': self.pos_x,
            'fused_y': self.pos_y,
            'uwb_x': self.last_uwb_x,  # Keep for plotting comparison
            'uwb_y': self.last_uwb_y,
            'state': state_str,
            'stroke_id': stroke_id,
            'stroke_active': is_active
        }


# ==============================================================================
# LIVE HARDWARE VALIDATION (Master Fusion Pipeline)
#   - Connects: Stream -> Time Aligner -> (IMU/UWB Preprocessors) -> Fusion
# ==============================================================================
if __name__ == '__main__':
    import time
    import os
    import csv
    import matplotlib.pyplot as plt

    # --- WINDOWS SCIPY/QT CRASH FIX ---
    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

    # IMPORT THE ENTIRE PIPELINE
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.time_alignment import TimeAlignLayer
    
    # IMU Branch
    from background.pipelines.preprocess.imu import IMUPreprocessor
    
    # UWB Branch (The Split Modules)
    from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
    from background.pipelines.preprocess.uwb.trilateration import UWBSolver
    from background.pipelines.preprocess.uwb.position import UWBPositionFilter
    
    from background.pipelines.preprocess.contact import ContactStateDetector
    
    
    SERIAL_PORT = getattr(cfg.serial, 'port', 'COM20')
    BAUD_RATE = getattr(cfg.serial, 'baud', 115200)
    DISPLAY_RATE = 0.1

    # 1. Initialize System
    streamer = SerialStreamer(port=SERIAL_PORT, baud=BAUD_RATE)
    norm = StreamNormalizer()
    aligner = TimeAlignLayer(buffer_size=500)
    
    # 2. Initialize Subsystems
    imu_prep = IMUPreprocessor()
    state_detector = ContactStateDetector()
    
    uwb_offsets = getattr(cfg.uwb, 'range_offsets_m', (0.0, 0.0, 0.0, 0.0))
    range_prep = UWBRangePreprocessor(offsets=uwb_offsets)
    trilat = UWBSolver()
    pos_filter = UWBPositionFilter()
    
    # 3. Initialize Fusion
    fusion = FusionEngine(fusion_alpha=0.15)

    print("=" * 60)
    print(f"  [TEST] MODULE 7: MASTER FUSION ENGINE: {SERIAL_PORT}")
    print("  Ready for Tests F1, F2, F3, and F4. Press Ctrl+C to stop.")
    print("=" * 60)

    event_log = []
    last_print_time = 0

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                events = norm.normalize(raw_packets)
                aligner.add_events(events)
                
                # Fetch time-aligned, sorted events
                sorted_events = aligner.get_all_sorted()
                aligner.clear()
                
                for ev in sorted_events:
                    
                    # --- IMU BRANCH ---
                    if ev['sensor'] == 'IMU':
                        p_imu = imu_prep.process_one(ev)
                        if p_imu:
                            s_imu = state_detector.process_one(p_imu)
                            fused = fusion.process_event(s_imu)
                            if fused: event_log.append(fused)
                    
                    # --- UWB BRANCH ---
                    elif ev['sensor'] == 'UWB':
                        # We must feed it as a list because range_prep.feed takes lists
                        ranges = range_prep.feed([ev]) 
                        for r_ev in ranges:
                            raw_pos = trilat.process_one(r_ev)
                            if raw_pos:
                                clean_pos = pos_filter.process_one(raw_pos)
                                if clean_pos:
                                    fused = fusion.process_event(clean_pos)
                                    if fused: event_log.append(fused)

            # Live Dashboard (Uses last event in log if available)
            current_time = time.time()
            if event_log and (current_time - last_print_time >= DISPLAY_RATE):
                latest = event_log[-1]
                os.system('cls' if os.name == 'nt' else 'clear')
                print(f"========= LIVE FUSION ({DISPLAY_RATE}s) =========")
                print(f"  State      : {latest['state']}")
                print(f"  Stroke ID  : {latest['stroke_id']} (Active: {latest['stroke_active']})")
                print("-" * 50)
                print(f"  Fused Pos  : X: {latest['fused_x']:6.3f} m | Y: {latest['fused_y']:6.3f} m")
                print(f"  UWB Anchor : X: {latest['uwb_x']:6.3f} m | Y: {latest['uwb_y']:6.3f} m")
                print("=================================================")
                last_print_time = current_time

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            exit()

        # --- EXPORT ---
        csv_filename = "fusion_baseline_report.csv"
        with open(csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ts_hw', 'source', 'fused_x', 'fused_y', 'uwb_x', 'uwb_y', 'state', 'stroke_id', 'stroke_active'])
            for ev in event_log:
                writer.writerow([ev['ts_hw'], ev['source'], ev['fused_x'], ev['fused_y'], ev['uwb_x'], ev['uwb_y'], ev['state'], ev['stroke_id'], int(ev['stroke_active'])])
        print(f"[EXPORT] Saved to {csv_filename}")

        # --- PLOT REPORT ---
        print("[PLOT] Rendering Fusion Performance Report...")
        try:
            # Filter only for active drawing strokes to see the final "ink"
            ink_events = [ev for ev in event_log if ev['stroke_active']]
            
            if not ink_events:
                print("\n[WARNING] No active drawing strokes detected. Showing all movement instead.")
                ink_events = event_log

            f_x = [ev['fused_x'] for ev in ink_events]
            f_y = [ev['fused_y'] for ev in ink_events]
            
            # Extract UWB updates (since IMU events just copy the last UWB state)
            uwb_events = [ev for ev in ink_events if ev['source'] == 'POSITION']
            u_x = [ev['uwb_x'] for ev in uwb_events]
            u_y = [ev['uwb_y'] for ev in uwb_events]

            fig = plt.figure(figsize=(12, 12))
            fig.suptitle('Module 7: IMU + UWB Sensor Fusion (Baseline v1)', fontsize=16, fontweight='bold')

            board_w = getattr(cfg.anchors, 'board_size_m', 1.25)
            board_h = getattr(cfg.anchors, 'board_size_y', 1.24)

            ax = plt.subplot(1, 1, 1)
            ax.add_patch(plt.Rectangle((0, 0), board_w, board_h, fill=False, edgecolor='black', linestyle='--', lw=2))
            
            # Plot UWB "Anchor" Path
            ax.plot(u_x, u_y, label='UWB Reference (Slower, Jagged)', color='blue', alpha=0.4, marker='.', linestyle='dashed')
            
            # Plot Fused "Ink" Path
            ax.plot(f_x, f_y, label='Fused Kinematic Output (Smooth, Corrected)', color='orange', linewidth=2.5)

            ax.set_xlim(-0.1, board_w + 0.1)
            ax.set_ylim(-0.1, board_h + 0.1)
            ax.set_aspect('equal')
            ax.set_title('Spatial Trajectory Validation')
            ax.grid(True, linestyle=':', alpha=0.7)
            ax.legend(loc='upper right')

            plt.tight_layout()
            plt.savefig("fusion_baseline_report.png", dpi=300)
            plt.show(block=True)

        except Exception as e:
            print(f"\n[ERROR] Plotting failed: {e}")