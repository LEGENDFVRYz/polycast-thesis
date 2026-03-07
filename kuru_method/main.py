import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_stream import DataStream
from fusion_engine import FusionEngine

# --- CONFIG ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
DATASET_FILENAME = ''  # Leave empty "" for live mode, or provide path to CSV dataset
MAX_HISTORY = 500

# --- GLOBAL OBJECTS ---
stream = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
engine = FusionEngine()

# Plot Data Buffers
raw_x, raw_y = [], []
clean_x, clean_y = [], []

def update(frame):
    packet = None
    
    # 1. Fetch Data specific to the mode
    if stream.mode == "live":
        # Loop to clear the serial buffer so we don't lag behind real-time
        while stream.data_available():
            temp_packet = stream.get_packet()
            if temp_packet:
                packet = temp_packet  # Keep the freshest packet
    else:
        # In CSV mode, read exactly one line per frame to animate it properly
        temp_packet = stream.get_packet()
        if temp_packet == "EOF":
            return raw_scatter, clean_line, current_pos_dot
        if temp_packet:
            packet = temp_packet
            
    if not packet:
        return raw_scatter, clean_line, current_pos_dot

    # 2. Extract Data
    raw_dists = packet.get('uwb_raw', packet['uwb']) 
    clean_dists = packet['uwb']
    
    # 3. Calculate Positions
    pos_raw, pos_clean = engine.update(raw_dists, clean_dists)

    # 4. Update Buffers
    raw_x.append(pos_raw[0])
    raw_y.append(pos_raw[1])
    clean_x.append(pos_clean[0])
    clean_y.append(pos_clean[1])
    
    if len(raw_x) > MAX_HISTORY:
        raw_x.pop(0); raw_y.pop(0)
        clean_x.pop(0); clean_y.pop(0)

    # 5. Update Plot Artists
    raw_scatter.set_data(raw_x, raw_y)
    clean_line.set_data(clean_x, clean_y)
    current_pos_dot.set_data([pos_clean[0]], [pos_clean[1]])

    return raw_scatter, clean_line, current_pos_dot

def main():
    if not stream.connect():
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Static Elements
    ax_x = engine.anchors[:, 0]
    ax_y = engine.anchors[:, 1]
    ax.scatter(ax_x, ax_y, c='red', marker='s', s=100, label='Anchors')
    
    for i, anchor in enumerate(engine.anchors):
        x, y, z = anchor
        ax.text(x, y + 0.1, f"A{i}", color='red', fontsize=12, ha='center')

    # Dynamic Elements
    global raw_scatter, clean_line, current_pos_dot
    
    raw_scatter, = ax.plot([], [], 'o', color='lightgray', alpha=0.5, markersize=5, label='Raw UWB')
    clean_line, = ax.plot([], [], '-', color='blue', linewidth=2, label='Filtered Path')
    current_pos_dot, = ax.plot([], [], 'o', color='blue', markersize=10)

    # Styling
    ax.set_xlim(-0.5, 2.0) 
    ax.set_ylim(-0.5, 2.0)
    ax.set_xlabel("X Meters")
    ax.set_ylabel("Y Meters")
    ax.legend(loc='upper right')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box') 
    
    # Start Animation
    ani = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
    
    mode_text = "CSV Playback" if stream.mode == "csv" else "Live Serial"
    print(f"🚀 Plotter Running in {mode_text} mode... (Close window to stop)")
    plt.show()

    # Cleanup after window closes
    stream.close()

if __name__ == "__main__":
    main()