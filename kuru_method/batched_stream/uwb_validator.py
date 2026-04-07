import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_stream import DataStream

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
# For variance testing, use a dataset where the tag is completely stationary
# For SNR testing, use a dataset with actual writing/movement
DATASET_FILENAME = '../datasets_v2/big_circle.csv'  # Leave empty "" for live mode, or provide path to CSV dataset

MAX_SAMPLES = 200  # Number of points visible on the scrolling live graph

class UWBFilterDashboard:
    def __init__(self, stream):
        self.stream = stream
        self.playback_finished = False

        # --- GLOBAL DATA BUFFERS (For final statistics) ---
        self.global_raw = {0: [], 1: [], 2: [], 3: []}
        self.global_filtered = {0: [], 1: [], 2: [], 3: []}

        # --- ROLLING BUFFERS (For live plotting) ---
        self.plot_raw = {0: [], 1: [], 2: [], 3: []}
        self.plot_filtered = {0: [], 1: [], 2: [], 3: []}

        # --- SETUP UI ---
        plt.style.use('dark_background')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
        self.fig.canvas.manager.set_window_title('UWB Spatial Filter Validation')
        self.ax_list = self.axes.flatten()
        
        self.lines_raw = []
        self.lines_filtered = []
        self.text_overlays = []

        anchor_colors = ['#00FFFF', '#FF00FF', '#FFFF00', '#00FF00']

        for i in range(4):
            ax = self.ax_list[i]
            
            # Plot lines
            line_r, = ax.plot([], [], color='white', alpha=0.3, linewidth=1, label='Raw Input')
            line_f, = ax.plot([], [], color=anchor_colors[i], linewidth=2.5, label='Filtered Output')
            
            self.lines_raw.append(line_r)
            self.lines_filtered.append(line_f)
            
            # Added real-time text overlay positioned relative to the axes
            txt = ax.text(0.02, 0.88, "Waiting for data...", transform=ax.transAxes, 
                          fontsize=11, weight='bold', color='white',
                          bbox=dict(facecolor='black', alpha=0.6, edgecolor='none', boxstyle='round,pad=0.3'))
            self.text_overlays.append(txt)

            ax.set_title(f"Anchor {i} (Dist{i})", fontsize=11, weight='bold')
            ax.set_ylabel("Distance (m)")
            ax.set_xlim(0, MAX_SAMPLES)
            ax.grid(True, alpha=0.2)
            ax.legend(loc='upper right', fontsize=9)

        self.ax_list[2].set_xlabel("Sample Index")
        self.ax_list[3].set_xlabel("Sample Index")
        plt.tight_layout()

    def process_packet(self, packet):
        raw_dists = packet['uwb_raw']
        filt_dists = packet['uwb']

        for i in range(4):
            r_val = raw_dists[i]
            f_val = filt_dists[i]

            # Update Global
            self.global_raw[i].append(r_val)
            self.global_filtered[i].append(f_val)

            # Update Plot Buffers
            self.plot_raw[i].append(r_val)
            self.plot_filtered[i].append(f_val)
            
            if len(self.plot_raw[i]) > MAX_SAMPLES:
                self.plot_raw[i].pop(0)
                self.plot_filtered[i].pop(0)

    def update_plot(self, frame):
        if self.playback_finished:
            return self.lines_raw + self.lines_filtered + self.text_overlays

        packets_processed_this_frame = 0
        
        if self.stream.mode == "live":
            while self.stream.data_available():
                packet = self.stream.get_packet()
                if packet:
                    self.process_packet(packet)
                    packets_processed_this_frame += 1
        else:
            for _ in range(5): # Playback speed
                packet = self.stream.get_packet()
                if packet == "EOF":
                    self.playback_finished = True
                    self.stream.close()
                    return self.lines_raw + self.lines_filtered + self.text_overlays
                if packet:
                    self.process_packet(packet)
                    packets_processed_this_frame += 1

        if packets_processed_this_frame == 0:
            return self.lines_raw + self.lines_filtered + self.text_overlays

        # Update lines
        x_data = range(len(self.plot_raw[0]))
        for i in range(4):
            self.lines_raw[i].set_data(x_data, self.plot_raw[i])
            self.lines_filtered[i].set_data(x_data, self.plot_filtered[i])
            
            # Dynamic Y-Axis scaling based on recent data
            current_vals = self.plot_raw[i] + self.plot_filtered[i]
            if current_vals:
                min_y = max(0, min(current_vals) - 0.2)
                max_y = max(current_vals) + 0.2
                self.ax_list[i].set_ylim(min_y, max_y)
                
                # Update the live text overlay with the latest measurements
                latest_r = self.plot_raw[i][-1]
                latest_f = self.plot_filtered[i][-1]
                self.text_overlays[i].set_text(f"Raw: {latest_r:.3f}m | Filt: {latest_f:.3f}m")

        return self.lines_raw + self.lines_filtered + self.text_overlays

    def print_final_summary(self):
        print("\n" + "="*88)
        print(" UWB STABILIZATION & FILTER PERFORMANCE SUMMARY")
        print("="*88)
        
        if len(self.global_raw[0]) == 0:
            print("No data processed.")
            return

        print(f"{'Metric':<15} | {'Anchor 0':<15} | {'Anchor 1':<15} | {'Anchor 2':<15} | {'Anchor 3':<15}")
        print("-" * 88)

        # 1. Variance
        var_raw = [np.var(self.global_raw[i]) for i in range(4)]
        var_filt = [np.var(self.global_filtered[i]) for i in range(4)]
        
        print(f"{'Raw Var (σ²)':<15} | {var_raw[0]:<15.6f} | {var_raw[1]:<15.6f} | {var_raw[2]:<15.6f} | {var_raw[3]:<15.6f}")
        print(f"{'Filt Var (σ²)':<15} | {var_filt[0]:<15.6f} | {var_filt[1]:<15.6f} | {var_filt[2]:<15.6f} | {var_filt[3]:<15.6f}")
        print("-" * 88)

        # 2. Standard Deviation
        std_raw = [np.std(self.global_raw[i]) for i in range(4)]
        std_filt = [np.std(self.global_filtered[i]) for i in range(4)]

        print(f"{'Raw StdDev (σ)':<15} | {std_raw[0]:<14.4f}m | {std_raw[1]:<14.4f}m | {std_raw[2]:<14.4f}m | {std_raw[3]:<14.4f}m")
        print(f"{'Filt StdDev (σ)':<15} | {std_filt[0]:<14.4f}m | {std_filt[1]:<14.4f}m | {std_filt[2]:<14.4f}m | {std_filt[3]:<14.4f}m")
        print("-" * 88)

        # 3. Signal-to-Noise Ratio (SNR)
        snr_vals = []
        for i in range(4):
            raw_arr = np.array(self.global_raw[i])
            filt_arr = np.array(self.global_filtered[i])
            noise = raw_arr - filt_arr
            
            var_signal = np.var(filt_arr)
            var_noise = np.var(noise)
            
            if var_noise == 0:
                snr = float('inf')
            elif var_signal == 0:
                snr = 0.0
            else:
                snr = 10 * np.log10(var_signal / var_noise)
            snr_vals.append(snr)

        print(f"{'Estimated SNR':<15} | {snr_vals[0]:<11.2f} dB  | {snr_vals[1]:<11.2f} dB  | {snr_vals[2]:<11.2f} dB  | {snr_vals[3]:<11.2f} dB")
        print("="*88 + "\n")

def main():
    stream = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
    if not stream.connect():
        sys.exit(1)

    dashboard = UWBFilterDashboard(stream)
    
    print(f"🚀 UWB Filter Validator Running... (Close window to generate statistics)")
    
    ani = FuncAnimation(dashboard.fig, dashboard.update_plot, interval=30, blit=False, cache_frame_data=False)
    plt.show()

    stream.close()
    dashboard.print_final_summary()

if __name__ == "__main__":
    main()