"""
main.py  —  PolyCast Live Position Visualiser
=============================================
Reads from DataStream, passes quality weights into FusionEngine,
and feeds the solved position back into UWBPreprocessor so the
per-anchor NLOS/innovation detection improves each cycle.
"""

import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_stream  import DataStream
from fusion_engine import FusionEngine

# ── Configuration ─────────────────────────────────────────────────────
SERIAL_PORT      = 'COM5'
BAUD_RATE        = 115200
DATASET_FILENAME = ''       # '' = live serial;  'mydata.csv' = playback
MAX_HISTORY      = 500      # trail length (packets)

# ── Global objects ────────────────────────────────────────────────────
stream = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
engine = FusionEngine()

raw_x,   raw_y   = [], []
clean_x, clean_y = [], []


# ─────────────────────────────────────────────────────────────────────
#  ANIMATION CALLBACK
# ─────────────────────────────────────────────────────────────────────
def update(frame):
    packet = None

    if stream.mode == "live":
        # Drain the serial buffer to stay current; keep only freshest packet
        while stream.data_available():
            tmp = stream.get_packet()
            if tmp:
                packet = tmp
    else:
        tmp = stream.get_packet()
        if tmp == "EOF":
            return raw_scatter, clean_line, current_pos_dot
        if tmp:
            packet = tmp

    if not packet:
        return raw_scatter, clean_line, current_pos_dot

    # ── Extract data ──────────────────────────────────────────────────
    raw_dists     = packet.get('uwb_raw', packet['uwb'])
    clean_dists   = packet['uwb']
    quality_w     = packet.get('uwb_weights')           # may be None for old CSVs
    batch_ts      = packet.get('batch_ts')

    # ── Solve positions ───────────────────────────────────────────────
    pos_raw, pos_clean = engine.update(
        raw_dists,
        clean_dists,
        quality_weights = quality_w,
        packet_ts       = batch_ts,
    )

    # ── Feed position back → improves NLOS detection next cycle ───────
    stream.uwb_cleaner.update_predictions(pos_clean, engine.anchors)

    # ── Update trail buffers ──────────────────────────────────────────
    raw_x.append(pos_raw[0]);   raw_y.append(pos_raw[1])
    clean_x.append(pos_clean[0]); clean_y.append(pos_clean[1])

    if len(raw_x) > MAX_HISTORY:
        raw_x.pop(0);   raw_y.pop(0)
        clean_x.pop(0); clean_y.pop(0)

    # ── Update plot artists ───────────────────────────────────────────
    raw_scatter.set_data(raw_x, raw_y)
    clean_line.set_data(clean_x, clean_y)
    current_pos_dot.set_data([pos_clean[0]], [pos_clean[1]])

    return raw_scatter, clean_line, current_pos_dot


# ─────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────
def main():
    if not stream.connect():
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Static: anchor markers
    ax_x = engine.anchors[:, 0]
    ax_y = engine.anchors[:, 1]
    ax.scatter(ax_x, ax_y, c='red', marker='s', s=120, zorder=5, label='Anchors')
    for i, anchor in enumerate(engine.anchors):
        ax.text(anchor[0], anchor[1] + 0.07, f"A{i}",
                color='red', fontsize=12, ha='center', fontweight='bold')

    # Dynamic artists
    global raw_scatter, clean_line, current_pos_dot
    raw_scatter,    = ax.plot([], [], 'o', color='lightgray', alpha=0.4,
                               markersize=4, label='Raw IRLS')
    clean_line,     = ax.plot([], [], '-', color='royalblue', linewidth=2,
                               label='IRLS + KF (clean)')
    current_pos_dot,= ax.plot([], [], 'o', color='royalblue', markersize=11,
                               zorder=6)

    ax.set_xlim(-0.5, 2.0)
    ax.set_ylim(-0.5, 2.0)
    ax.set_xlabel("X  (metres)")
    ax.set_ylabel("Y  (metres)")
    ax.set_title("PolyCast — Live UWB Position")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.4)
    ax.set_aspect('equal', adjustable='box')

    ani = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)

    mode_str = "CSV Playback" if stream.mode == "csv" else "Live Serial"
    print(f"🚀 Plotter running in {mode_str} mode  (close window to stop)")
    plt.show()

    stream.close()


if __name__ == "__main__":
    main()