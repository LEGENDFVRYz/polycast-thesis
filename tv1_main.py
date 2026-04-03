import os
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import time
import numpy as np
import matplotlib.pyplot as plt
import signal

# --- NEW PIPELINE IMPORTS ---
from tv1_unpacker import SerialStreamer
from tv1_preprocess import IMUCleaner, TimeSyncer, UWBCleaner

# ==============================================================================
# HELPER: CLOCK ALIGNER (Essential for Pipeline)
# ==============================================================================
class ClockAligner:
    def __init__(self):
        self.offset = None
    
    def get_synced_time(self, hw_micros):
        # Convert Hardware Microseconds -> System Seconds
        hw_sec = hw_micros / 1_000_000.0
        if self.offset is None:
            self.offset = time.time() - hw_sec
        return hw_sec + self.offset

# ==============================================================================
# UPDATED STROKE TRACKER CLASS
# (Retains the "Imitation Filter" logic from fusion_v401)
# ==============================================================================
class StrokeTracker:
    def __init__(self):
        # History for plotting
        self.raw_x = []         # Output from Pipeline (Geometric Solve)
        self.raw_y = []
        self.filtered_x = []    # Output after EMA Filter
        self.filtered_y = []
        
        # EMA Filter State (The "Imitation" Logic)
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False

    def update(self, pos_x, pos_y, status):
        """
        Input: Clean Position from UWBCleaner
        """
        # 1. Store the "Raw" Pipeline output (Yellow Dotted Line)
        self.raw_x.append(pos_x)
        self.raw_y.append(pos_y)

        # 2. Apply the EMA Filter (Red Crosses)
        # This logic mimics the "0.8*old + 0.2*new" from your old code
        if status == "VALID":
            if not self.sim_initialized:
                # Snap to first point
                self.sim_filter_x = pos_x
                self.sim_filter_y = pos_y
                self.sim_initialized = True
            else:
                # Apply Smoothing (Heavy filtering)
                self.sim_filter_x = (0.80 * self.sim_filter_x) + (0.20 * pos_x)
                self.sim_filter_y = (0.80 * self.sim_filter_y) + (0.20 * pos_y)

            self.filtered_x.append(self.sim_filter_x)
            self.filtered_y.append(self.sim_filter_y)
        else:
            # If rejected (Ghost/Speed), we just copy the last known good position
            # to keep the line continuous, or you can choose to skip adding points.
            if self.sim_initialized:
                self.filtered_x.append(self.sim_filter_x)
                self.filtered_y.append(self.sim_filter_y)
        
        # Keep buffer small for performance
        if len(self.raw_x) > 100:
            self.raw_x.pop(0); self.raw_y.pop(0)
            self.filtered_x.pop(0); self.filtered_y.pop(0)

# ==============================================================================
# MAIN VISUALIZATION LOOP
# ==============================================================================
if __name__ == "__main__":
    # 1. SETUP HARDWARE STREAM
    streamer = SerialStreamer(port='COM20', baud=115200)
    
    # 2. SETUP PIPELINE MODULES
    imu_cleaner = IMUCleaner()
    time_syncer = TimeSyncer()
    uwb_cleaner = UWBCleaner()
    clock = ClockAligner()
    
    # 3. SETUP TRACKER (The State Manager)
    tracker = StrokeTracker()
    
    # 4. SETUP MATPLOTLIB (Same style as fusion_v401)
    print("Initializing Visualization...")
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Yellow: The "Pipeline Raw" (Geometry Solved + Physics Gated)
    line_raw, = ax.plot([], [], linestyle=':', marker='o', markersize=4, 
                        alpha=0.4, color="#E6AC3F", label='Pipeline Output')
    
    # Red: The "Stroke Filtered" (Your EMA Logic)
    line_comp, = ax.plot([], [], linestyle='-', marker='x', markersize=6, 
                         alpha=0.8, color="#E63F3F", label='Stroke Filtered')

    ax.set_title("TV1 Stroke Engine (Pipeline Powered)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    # Set bounds to your whiteboard size + margin
    ax.set_xlim(-0.5, 2.0)
    ax.set_ylim(-0.5, 2.0)
    ax.grid(True)
    ax.legend()
    
    # Graceful Exit
    is_running = True
    def signal_handler(sig, frame):
        global is_running
        is_running = False
    signal.signal(signal.SIGINT, signal_handler)

    print("System Ready. Draw on the board!")
    
    try:
        while is_running:
            # A. READ NEW PACKETS
            packets = streamer.read_new_packets()
            
            # B. PROCESS PIPELINE
            for pkt in packets:
                
                # --- IMU PATH (Clean Physics) ---
                if pkt['type'] == 'IMU':
                    raw_sample = pkt['samples'][-1]
                    ts = clock.get_synced_time(raw_sample['ts'])
                    
                    # Clean & Sync
                    clean_acc = imu_cleaner.process(raw_sample)
                    time_syncer.push(ts, clean_acc)
                    
                # --- UWB PATH (Clean Geometry) ---
                elif pkt['type'] == 'UWB':
                    if 'dists' in pkt:
                        # Get Timestamp
                        if 'ts' in pkt:
                            ts = clock.get_synced_time(pkt['ts'])
                        else:
                            ts = time.time()
                            
                        # Pipeline Process
                        # (This replaces the old trilaterate_wls logic)
                        final_pos, status = uwb_cleaner.process(pkt['dists'], ts, time_syncer)
                        
                        # C. UPDATE TRACKER
                        # Pass the clean pipeline data to your Stroke Class
                        tracker.update(final_pos[0], final_pos[1], status)

            # D. UPDATE PLOT (Throttle to prevent lag)
            # Only draw if we have new data
            line_raw.set_xdata(tracker.raw_x)
            line_raw.set_ydata(tracker.raw_y)
            
            line_comp.set_xdata(tracker.filtered_x)
            line_comp.set_ydata(tracker.filtered_y)
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            # Tiny sleep to let UI breathe

    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing Stream...")
        streamer.close()
        plt.ioff()
        plt.show()