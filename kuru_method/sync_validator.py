import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_stream import DataStream

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
DATASET_FILENAME = 'datasets/abcde_letters.csv' # Leave empty "" for live mode, or provide path to CSV dataset

# Expected hardware timing
EXPECTED_IMU_HZ = 100
EXPECTED_DT_US = 1000000 / EXPECTED_IMU_HZ  # 10,000 microseconds
PACKET_DURATION_US = EXPECTED_DT_US * 5     # 50,000 microseconds per ESP-NOW packet

# Display settings
MAX_SAMPLES = 500   # How many IMU samples to show on the live plot
MAX_PACKETS = 100   # How many packets to show on the UWB timeline plot

class ValidatorDashboard:
    def __init__(self, stream):
        self.stream = stream
        
        # --- GLOBAL STATISTICS (Tracks the entire session) ---
        self.total_packets = 0
        self.first_ts = None
        self.last_ts = None
        
        self.all_dt_ms = []
        self.max_spike_ms = 0.0
        
        self.total_uwb_updates = 0
        self.current_stale_count = 0
        self.max_stale_count = 0
        self.last_uwb_raw = None

        self.last_imu_ts = None
        self.playback_finished = False
        
        # --- ROLLING BUFFERS (For the live plots) ---
        self.plot_dt_ms = []
        self.plot_uwb_timeline = [] # 1 for update, 0 for stale

        # --- SETUP UI ---
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 9))
        self.fig.canvas.manager.set_window_title('Live PolyCast Hardware Diagnostics')
        
        # Plot 1: IMU Timing Spikes
        self.ax1 = plt.subplot(2, 2, 1)
        self.line_dt, = self.ax1.plot([], [], color='cyan', alpha=0.8, linewidth=1.5)
        self.ax1.axhline(10.0, color='lime', linestyle='--', label='Target (10ms)')
        self.ax1.set_title("Live IMU Inter-Sample Time (Δt)", fontsize=12, weight='bold')
        self.ax1.set_ylabel("Milliseconds (ms)")
        self.ax1.set_xlim(0, MAX_SAMPLES)
        self.ax1.set_ylim(5, 15) # Will auto-scale if spikes are larger
        self.ax1.grid(True, alpha=0.2)

        # Plot 2: Jitter Distribution (Histogram)
        self.ax2 = plt.subplot(2, 2, 2)
        self.ax2.set_title("Global IMU Jitter Distribution", fontsize=12, weight='bold')
        self.ax2.set_xlabel("Δt (ms)")
        self.ax2.set_ylabel("Frequency")
        self.ax2.grid(True, alpha=0.2)

        # Plot 3: UWB Update Timeline
        self.ax3 = plt.subplot(2, 1, 2)
        self.line_uwb, = self.ax3.step([], [], color='yellow', linewidth=2, where='pre')
        self.ax3.fill_between([], [], color='yellow', step='pre', alpha=0.3) # Initialized empty
        self.ax3.set_title("Live UWB Refresh Timeline (1 = New Data, 0 = Stale)", fontsize=12, weight='bold')
        self.ax3.set_xlim(0, MAX_PACKETS)
        self.ax3.set_ylim(-0.2, 1.2)
        self.ax3.set_yticks([0, 1])
        self.ax3.set_yticklabels(["Stale Data", "New Data"])
        self.ax3.grid(True, alpha=0.2)
        
        # Text Overlay for Live Stats
        self.stats_text = self.fig.text(0.5, 0.97, "", ha='center', va='top', fontsize=10, color='white', weight='bold', 
                                        bbox=dict(facecolor='black', alpha=0.7, edgecolor='white', boxstyle='round,pad=0.5'))
        
        plt.tight_layout(rect=[0, 0, 1, 0.92]) # Leave room for stats

    def process_packet(self, packet):
        self.total_packets += 1
        
        # 1. IMU Timing Extraction
        for i in range(5):
            current_ts = packet['imu'][i]['ts']
            
            if self.first_ts is None:
                self.first_ts = current_ts
            self.last_ts = current_ts
            
            if self.last_imu_ts is not None:
                dt_us = current_ts - self.last_imu_ts
                dt_ms = dt_us / 1000.0
                
                # Update Stats
                self.all_dt_ms.append(dt_ms)
                if dt_ms > self.max_spike_ms:
                    self.max_spike_ms = dt_ms
                    
                # Update Buffer
                self.plot_dt_ms.append(dt_ms)
                if len(self.plot_dt_ms) > MAX_SAMPLES:
                    self.plot_dt_ms.pop(0)
                    
            self.last_imu_ts = current_ts

        # 2. UWB Staleness Extraction
        current_uwb_raw = packet['uwb_raw']
        is_updated = 1
        
        if self.last_uwb_raw is not None:
            if current_uwb_raw == self.last_uwb_raw:
                is_updated = 0
                self.current_stale_count += 1
                if self.current_stale_count > self.max_stale_count:
                    self.max_stale_count = self.current_stale_count
            else:
                self.current_stale_count = 0
                
        self.total_uwb_updates += is_updated
        self.last_uwb_raw = current_uwb_raw
        
        # Update Buffer
        self.plot_uwb_timeline.append(is_updated)
        if len(self.plot_uwb_timeline) > MAX_PACKETS:
            self.plot_uwb_timeline.pop(0)

    def update_plot(self, frame):
        # --- NEW: Freeze the plot if the file is completely read ---
        if self.playback_finished:
            return self.line_dt, self.line_uwb

        packets_processed_this_frame = 0
        
        # Fetch Data based on mode
        if self.stream.mode == "live":
            while self.stream.data_available():
                packet = self.stream.get_packet()
                if packet:
                    self.process_packet(packet)
                    packets_processed_this_frame += 1
        else:
            # Process up to 10 CSV lines per frame for fast playback
            for _ in range(10): 
                packet = self.stream.get_packet()
                if packet == "EOF":
                    self.playback_finished = True
                    self.stream.close()
                    print("\n✅ End of dataset reached. Final statistics are on screen.")
                    return self.line_dt, self.line_uwb
                if packet:
                    self.process_packet(packet)
                    packets_processed_this_frame += 1

        if packets_processed_this_frame == 0:
            return self.line_dt, self.line_uwb

        # --- CALCULATE LIVE STATISTICS ---
        if len(self.all_dt_ms) > 0 and self.first_ts and self.last_ts:
            mean_dt = np.mean(self.all_dt_ms)
            jitter = np.std(self.all_dt_ms)
            
            total_time_us = self.last_ts - self.first_ts
            expected_packets = total_time_us / PACKET_DURATION_US if total_time_us > 0 else 1
            pdr = min((self.total_packets / expected_packets) * 100, 100.0)
            
            uwb_rate = (self.total_uwb_updates / self.total_packets) * 100
            max_blind_ms = self.max_stale_count * (PACKET_DURATION_US / 1000.0)
            
            stats_str = (f"Pkts: {self.total_packets} | PDR: {pdr:.1f}% | "
                         f"Mean Δt: {mean_dt:.2f}ms | Jitter: ±{jitter:.2f}ms | Max Spike: {self.max_spike_ms:.1f}ms\n"
                         f"UWB Refresh Rate: {uwb_rate:.1f}% | Max Blind Spot: {max_blind_ms:.1f}ms ({self.max_stale_count} pkts)")
            self.stats_text.set_text(stats_str)

        # --- UPDATE PLOTS ---
        # 1. IMU Line Plot
        self.line_dt.set_data(range(len(self.plot_dt_ms)), self.plot_dt_ms)
        if self.max_spike_ms > 15:
            self.ax1.set_ylim(0, self.max_spike_ms + 2)

        # 2. Histogram (Redrawn completely to track global distribution)
        if len(self.all_dt_ms) > 0 and frame % 5 == 0:
            self.ax2.clear()
            self.ax2.hist(self.all_dt_ms, bins=50, color='magenta', alpha=0.8, edgecolor='white')
            self.ax2.axvline(10.0, color='lime', linestyle='--', linewidth=2)
            self.ax2.set_title("Global IMU Jitter Distribution", fontsize=12, weight='bold')
            self.ax2.set_xlabel("Δt (ms)")
            self.ax2.set_ylabel("Frequency")
            self.ax2.grid(True, alpha=0.2)
            self.ax2.set_xlim(8, 12)

        # 3. UWB Timeline
        x_data = range(len(self.plot_uwb_timeline))
        self.line_uwb.set_data(x_data, self.plot_uwb_timeline)
        
        # To fill the area under the step plot dynamically, remove old collections then draw new ones
        for coll in self.ax3.collections:
            coll.remove()
        self.ax3.fill_between(x_data, self.plot_uwb_timeline, color='yellow', step='pre', alpha=0.3)

        return self.line_dt, self.line_uwb

def main():
    stream = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
    if not stream.connect():
        sys.exit(1)

    dashboard = ValidatorDashboard(stream)
    
    print(f"🚀 Dashboard Running in {stream.mode.upper()} mode... (Close window to stop)")
    
    # Run the animation loop
    ani = FuncAnimation(dashboard.fig, dashboard.update_plot, interval=50, blit=False, cache_frame_data=False)
    
    plt.show()

    # Cleanup after closing
    stream.close()

if __name__ == "__main__":
    main()