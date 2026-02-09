import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_stream import DataStream
from fusion_engine import FusionEngine

# --- CONFIG ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
MAX_HISTORY = 100 # Keep last 100 points to keep plot fast

# --- GLOBAL OBJECTS ---
stream = DataStream(SERIAL_PORT, BAUD_RATE)
engine = FusionEngine()

# Plot Data Buffers
raw_x, raw_y = [], []
clean_x, clean_y = [], []

def update(frame):
    """
    This function is called automatically by Matplotlib.
    It fetches data, updates arrays, and returns the artists to redraw.
    """
    # 1. Fetch Data
    # We loop to clear the serial buffer so we don't lag behind real-time
    packet = None
    while stream.ser.in_waiting > 0:
        temp_packet = stream.get_packet()
        if temp_packet:
            packet = temp_packet  # Keep the freshest packet
            
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
    
    # Trim history to prevent memory leak / slow down
    if len(raw_x) > MAX_HISTORY:
        raw_x.pop(0); raw_y.pop(0)
        clean_x.pop(0); clean_y.pop(0)

    # 5. Update Plot Artists
    raw_scatter.set_data(raw_x, raw_y)
    clean_line.set_data(clean_x, clean_y)
    current_pos_dot.set_data([pos_clean[0]], [pos_clean[1]])

    return raw_scatter, clean_line, current_pos_dot

def main():
    # 1. Connect
    if not stream.connect():
        sys.exit(1)

    # 2. Setup Plot
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Static Elements (Anchors) - Drawn once
    ax_x = engine.anchors[:, 0]
    ax_y = engine.anchors[:, 1]
    ax.scatter(ax_x, ax_y, c='red', marker='s', s=100, label='Anchors')
    
    for i, anchor in enumerate(engine.anchors):
        x, y, z = anchor
        ax.text(x, y + 0.1, f"A{i}", color='red', fontsize=12, ha='center')

    # Dynamic Elements (Initialized empty)
    # Note: We declare these global so 'update' can see them
    global raw_scatter, clean_line, current_pos_dot
    
    raw_scatter, = ax.plot([], [], 'o', color='lightgray', alpha=0.5, markersize=5, label='Raw UWB')
    clean_line, = ax.plot([], [], '-', color='blue', linewidth=2, label='Filtered Path')
    current_pos_dot, = ax.plot([], [], 'o', color='blue', markersize=10)

    # Styling
    ax.set_xlim(-0.5, 2.0) # Set these wider than your anchor area so you can see
    ax.set_ylim(-0.5, 2.0)
    ax.set_xlabel("X Meters")
    ax.set_ylabel("Y Meters")
    ax.legend(loc='upper right')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box') # Ensures 1 meter looks like 1 meter
    
    # 3. Start Animation
    ani = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
    
    print("🚀 Plotter Running... (Close window to stop)")
    plt.show()

    # Cleanup after window closes
    if stream.ser:
        stream.ser.close()

if __name__ == "__main__":
    main()