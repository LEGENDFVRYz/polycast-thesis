"""
main_ekf.py  —  PolyCast EKF Live Visualiser
=============================================
Runs the full UWB + IMU tightly-coupled EKF and displays:

    Blue line   — EKF position estimate (drawing mode ON)
    Gray dots   — EKF position (pen lifted — tracked but not drawn)
    Orange ×    — UWB anchors rejected by chi²(1) gate this packet
    Velocity    — short arrow from current position (optional, toggle V key)
    Console     — per-second diagnostics (gate rate, bias, heading, speed)

Keyboard shortcuts
------------------
    V      — toggle velocity arrow display
    R      — reset EKF (re-initialise from next IRLS fix)
    C      — clear drawing trail
    Q/Esc  — quit

Configuration
-------------
    Edit SERIAL_PORT, BAUD_RATE, DATASET_FILENAME as before.
    All tuning constants live in ekf_fusion.py and imu_integrator.py.
"""

import sys
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from data_stream  import DataStream
from ekf_fusion   import EKFFusionEngine


# ── Causal output smoother ────────────────────────────────────────────
class TrailSmoother:
    """Weighted moving average over the last N positions (causal, no lookahead)."""

    _WEIGHTS = np.array([0.05, 0.10, 0.20, 0.30, 0.35])

    def __init__(self):
        n = len(self._WEIGHTS)
        self._buf_x = deque(maxlen=n)
        self._buf_y = deque(maxlen=n)

    def push(self, x: float, y: float):
        self._buf_x.append(x)
        self._buf_y.append(y)

    def get(self):
        n = len(self._buf_x)
        if n == 0:
            return 0.0, 0.0
        if n == 1:
            return self._buf_x[0], self._buf_y[0]
        w = self._WEIGHTS[-n:]
        w = w / w.sum()
        sx = sum(w[i] * self._buf_x[i] for i in range(n))
        sy = sum(w[i] * self._buf_y[i] for i in range(n))
        return float(sx), float(sy)

    def reset(self):
        self._buf_x.clear()
        self._buf_y.clear()

# ── Configuration ──────────────────────────────────────────────────────
SERIAL_PORT      = 'COM5'
BAUD_RATE        = 115200
DATASET_FILENAME = '../datasets_v2/big_triangle.csv'   # '' = live serial

MAX_TRAIL        = 600    # maximum position samples in the drawing trail
SHOW_VELOCITY    = True   # initial state; toggle with V key
VEL_SCALE        = 0.3    # arrow length multiplier
DIAG_INTERVAL_S  = 1.0    # console diagnostic print interval

# ── Global state ───────────────────────────────────────────────────────
stream   = DataStream(SERIAL_PORT, BAUD_RATE, DATASET_FILENAME)
engine   = EKFFusionEngine()
smoother = TrailSmoother()

# Drawing trail — only positions where is_writing == True
draw_x, draw_y   = [], []
# Lifted trail — positions where is_writing == False (faint)
lift_x, lift_y   = [], []
# Rejected anchor scatter
rej_x, rej_y     = [], []

_show_vel        = SHOW_VELOCITY
_last_diag_time  = 0.0
_prev_writing    = False   # tracks previous contact state to detect lift events
_eof_handled     = False   # ensures RTS smoother runs once at end of CSV


def _trim(lst, maxlen):
    if len(lst) > maxlen:
        del lst[:len(lst) - maxlen]


# Enable RTS history recording for CSV playback
if stream.mode == 'csv':
    engine.ekf.record_history = True


def _run_rts_smoother():
    """Run RTS backward smoother at end of CSV and overlay smoothed trajectory."""
    if not engine.ekf.record_history or len(engine.ekf._history) < 10:
        print("[RTS] Not enough history to smooth.")
        return
    print(f"[RTS] Running backward smoother on {len(engine.ekf._history)} steps…")
    smoothed = engine.ekf.rts_smooth()
    # Overlay smoothed trajectory in green (toggle with 'S' key)
    ax = line_draw.axes
    ax.plot(smoothed[:, 0], smoothed[:, 1], '-', color='limegreen', lw=1.5,
            alpha=0.8, label='RTS smoothed', zorder=5)
    ax.legend(loc='upper right', fontsize=9)
    ax.figure.canvas.draw_idle()
    print(f"[RTS] Smoothed trajectory overlaid ({len(smoothed)} points).")


# ── Animation callback ─────────────────────────────────────────────────
def update(frame):
    global _last_diag_time

    packet = None

    global _eof_handled

    if stream.mode == 'live':
        while stream.data_available():
            tmp = stream.get_packet()
            if tmp:
                packet = tmp
    else:
        tmp = stream.get_packet()
        if tmp == 'EOF':
            if not _eof_handled:
                _eof_handled = True
                _run_rts_smoother()
            return artists
        if tmp:
            packet = tmp

    if not packet:
        return artists

    # ── EKF update ────────────────────────────────────────────────────
    pos, vel, is_writing, accepted, rejected = engine.update(
        imu_batch       = packet['imu'],
        filtered_dists  = packet['uwb_ekf'],   # offset+despike, no EMA
        quality_weights = packet.get('uwb_weights'),
        packet_ts       = packet.get('batch_ts'),
        uwb_ts          = packet.get('uwb_ts'),
    )

    # ── Feed-back: update NLOS predictions in preprocessor ────────────
    if pos is not None:
        stream.uwb_cleaner.update_predictions(
            np.append(pos, engine.ekf._tag_z), engine.anchors)

    if pos is None:
        return artists

    # ── Trail management ──────────────────────────────────────────────
    global _prev_writing

    if is_writing:
        smoother.push(pos[0], pos[1])
        sx, sy = smoother.get()
        draw_x.append(sx);  draw_y.append(sy)
        _trim(draw_x, MAX_TRAIL); _trim(draw_y, MAX_TRAIL)
    else:
        # Insert a NaN break the moment the pen lifts so that the next
        # writing stroke starts a fresh line segment, never connected to
        # the previous one.
        if _prev_writing:
            draw_x.append(float('nan'))
            draw_y.append(float('nan'))
            smoother.reset()
        lift_x.append(pos[0]);  lift_y.append(pos[1])
        _trim(lift_x, 100); _trim(lift_y, 100)

    _prev_writing = is_writing

    # Mark rejected anchors by their known positions
    for i in rejected:
        rej_x.append(engine.anchors[i, 0])
        rej_y.append(engine.anchors[i, 1])
    _trim(rej_x, 40); _trim(rej_y, 40)

    # ── Plot update ───────────────────────────────────────────────────
    line_draw.set_data(draw_x, draw_y)
    scat_lift.set_data(lift_x, lift_y)
    scat_rej.set_data(rej_x, rej_y)
    dot_cur.set_data([pos[0]], [pos[1]])

    # Velocity arrow
    if _show_vel and vel is not None:
        spd = float(np.linalg.norm(vel))
        if spd > 0.01:
            vel_arrow.set_offsets([[pos[0], pos[1]]])
            vel_arrow.set_UVC(
                [vel[0] * VEL_SCALE],
                [vel[1] * VEL_SCALE],
            )
        else:
            vel_arrow.set_offsets([[pos[0], pos[1]]])
            vel_arrow.set_UVC([0], [0])

    # ── Console diagnostics ───────────────────────────────────────────
    now = time.time()
    if now - _last_diag_time >= DIAG_INTERVAL_S:
        _last_diag_time = now
        diag = engine.diagnostics
        bias = diag.get('bias')
        bstr = f"({bias[0]:+.3f}, {bias[1]:+.3f})" if bias is not None else "—"
        spd  = float(np.linalg.norm(vel)) if vel is not None else 0.0
        hl   = "YES" if diag.get('heading_locked') else "NO"
        print(f"[EKF] pos=({pos[0]:.3f},{pos[1]:.3f})  "
              f"speed={spd:.2f}m/s  "
              f"bias={bstr}  "
              f"accepted={len(accepted)}/4  "
              f"outage={diag['consec_outage']}  "
              f"heading_locked={hl}")

    return artists


# ── Keyboard handler ──────────────────────────────────────────────────
def on_key(event):
    global _show_vel
    if event.key == 'v':
        _show_vel = not _show_vel
        print(f"[UI] Velocity arrow: {'ON' if _show_vel else 'OFF'}")
    elif event.key == 'c':
        draw_x.clear(); draw_y.clear()
        lift_x.clear(); lift_y.clear()
        rej_x.clear();  rej_y.clear()
        smoother.reset()
        _prev_writing = False
        print("[UI] Trail cleared.")
    elif event.key == 'r':
        engine.ekf._x = None   # force re-initialisation
        print("[UI] EKF reset — will re-initialise from next IRLS fix.")
    elif event.key in ('q', 'escape'):
        plt.close('all')


# ── Entry point ───────────────────────────────────────────────────────
def main():
    global artists, line_draw, scat_lift, scat_rej, dot_cur, vel_arrow

    if not stream.connect():
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(9, 9))
    fig.canvas.manager.set_window_title('PolyCast — EKF Live Position')
    fig.canvas.mpl_connect('key_press_event', on_key)

    # Anchors (static)
    ax.scatter(engine.anchors[:, 0], engine.anchors[:, 1],
               c='red', marker='s', s=120, zorder=5)
    for i, a in enumerate(engine.anchors):
        ax.text(a[0], a[1] + 0.07, f'A{i}',
                color='red', fontsize=12, ha='center', fontweight='bold')

    # Artists
    line_draw, = ax.plot([], [], '-', color='royalblue', lw=2.0,
                         label='EKF position (writing)', zorder=4)
    scat_lift, = ax.plot([], [], '.', color='#888888', ms=4, alpha=0.5,
                         label='Lifted (tracked, not drawn)', zorder=3)
    scat_rej,  = ax.plot([], [], 'x', color='orange', ms=8,
                         markeredgewidth=1.8, alpha=0.7,
                         label='Anchor gate reject', zorder=4)
    dot_cur,   = ax.plot([], [], 'o', color='royalblue', ms=12, zorder=6)
    vel_arrow  = ax.quiver([], [], [], [], color='cyan', scale=1,
                           scale_units='xy', angles='xy',
                           width=0.003, zorder=7)

    ax.set_xlim(-0.3, 1.55)
    ax.set_ylim(-0.3, 1.55)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X  (metres)')
    ax.set_ylabel('Y  (metres)')
    ax.set_title('PolyCast — UWB + IMU EKF Fusion')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Info text (bottom)
    ax.text(0.01, 0.01,
            'V = velocity arrow   C = clear trail   R = reset EKF',
            transform=ax.transAxes, fontsize=8, color='gray', va='bottom')

    artists = (line_draw, scat_lift, scat_rej, dot_cur)

    ani = FuncAnimation(fig, update, interval=20, blit=False,
                        cache_frame_data=False)

    mode = 'CSV Playback' if stream.mode == 'csv' else 'Live Serial'
    print(f'[EKF] Running in {mode} mode  (V/C/R keys, Q to quit)')

    plt.show()
    stream.close()


if __name__ == '__main__':
    main()