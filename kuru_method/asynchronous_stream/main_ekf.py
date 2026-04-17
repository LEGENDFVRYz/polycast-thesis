"""
main_ekf.py  —  PolyCast EKF Live Visualiser (Async Stream)
============================================================
Event-driven sensor fusion visualiser for the decoupled async stream.

    IMU packet -> EKF predict  (~100 Hz)
    UWB packet -> EKF update   (~10 Hz)

Display:
    Blue line   — EKF position estimate (drawing mode ON)
    Gray dots   — EKF position (pen lifted — tracked but not drawn)
    Orange x    — UWB anchors rejected by chi^2(1) gate
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
    Edit SERIAL_PORT, BAUD_RATE, DATASET_FILENAME below.
    All tuning constants live in ekf_fusion.py and imu_integrator.py.
"""

import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from data_parser    import AsyncDataParser
from preprocessor   import UWBPreprocessor, IMUPreprocessor
from ekf_fusion     import AsyncEKFFusionEngine
from trail_smoother import TrailSmoother


# -- Configuration ----------------------------------------------------------
from config import SERIAL_PORT, BAUD_RATE, UWB_OFFSETS
DATASET_FILENAME = 'datasets_a3_ls/SQUARE-.csv'   # '' = live serial;  'datasets/data.csv' = playback

MAX_TRAIL        = 1000  # maximum position samples in the drawing trail
SHOW_VELOCITY    = True  # initial state; toggle with V key
VEL_SCALE        = 0.3   # arrow length multiplier
DIAG_INTERVAL_S  = 1.0   # console diagnostic print interval

# Trail subsampling: at 100 Hz IMU, record every Nth predict to avoid
# overwhelming matplotlib.  3 = ~33 Hz display rate.
TRAIL_SUBSAMPLE  = 3

# CSV playback: packets to process per animation frame.
# 10 packets at ~10ms each = ~100ms real data per 20ms frame = ~5x speed.
CSV_PACKETS_PER_FRAME = 10


# -- Global state -----------------------------------------------------------
parser      = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE,
                              csv_path=DATASET_FILENAME)
engine      = AsyncEKFFusionEngine()
uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
imu_cleaner = IMUPreprocessor()
smoother    = TrailSmoother()

# Drawing trail — only positions where is_writing == True
draw_x, draw_y   = [], []
# Lifted trail — positions where is_writing == False (faint)
lift_x, lift_y   = [], []
# Rejected anchor scatter
rej_x, rej_y     = [], []

_show_vel        = SHOW_VELOCITY
_last_diag_time  = 0.0
_prev_writing    = False
_eof_handled     = False
_imu_subsample   = 0       # counter for trail subsampling

# Latest position/velocity for artist updates (persists across packets)
_last_pos = None
_last_vel = None


def _trim(lst, maxlen):
    if len(lst) > maxlen:
        del lst[:len(lst) - maxlen]


# Enable RTS history recording for CSV playback
if parser.mode == 'csv':
    engine.ekf.record_history = True


def _run_rts_smoother():
    """Run RTS backward smoother at end of CSV and overlay smoothed trajectory."""
    if not engine.ekf.record_history or len(engine.ekf._history) < 10:
        print("[RTS] Not enough history to smooth.")
        return
    print(f"[RTS] Running backward smoother on {len(engine.ekf._history)} steps...")
    smoothed = engine.ekf.rts_smooth()
    ax = line_draw.axes
    ax.plot(smoothed[:, 0], smoothed[:, 1], '-', color='limegreen', lw=1.5,
            alpha=0.8, label='RTS smoothed', zorder=5)
    ax.legend(loc='upper right', fontsize=9)
    ax.figure.canvas.draw_idle()
    print(f"[RTS] Smoothed trajectory overlaid ({len(smoothed)} points).")


# -- Animation callback -----------------------------------------------------
def update(frame):
    global _last_diag_time, _prev_writing, _eof_handled
    global _imu_subsample, _last_pos, _last_vel

    # Drain available packets
    packets = []
    if parser.mode == 'live':
        while parser.data_available():
            pkt = parser.get_packet()
            if pkt and pkt != 'EOF':
                packets.append(pkt)
    else:
        for _ in range(CSV_PACKETS_PER_FRAME):
            pkt = parser.get_packet()
            if pkt == 'EOF':
                if not _eof_handled:
                    _eof_handled = True
                    _run_rts_smoother()
                    parser.summary()
                return artists
            if pkt:
                packets.append(pkt)

    if not packets:
        return artists

    for pkt in packets:

        # -- IMU packet: preprocess + predict ------------------------------
        if pkt['type'] == 'imu':
            # Quaternion normalization
            clean = imu_cleaner.process_sample(
                *pkt['quat'], *pkt['acc'])
            pkt['quat'] = clean[0:4]
            pkt['acc']  = clean[4:7]

            pos, vel, is_writing = engine.process_imu(pkt)

            if pos is None:
                continue

            _last_pos = pos
            _last_vel = vel

            # Trail management (subsampled)
            _imu_subsample += 1
            if _imu_subsample >= TRAIL_SUBSAMPLE:
                _imu_subsample = 0

                if is_writing:
                    smoother.push(pos[0], pos[1])
                    sx, sy = smoother.get()
                    draw_x.append(sx)
                    draw_y.append(sy)
                    _trim(draw_x, MAX_TRAIL)
                    _trim(draw_y, MAX_TRAIL)
                else:
                    # NaN break on pen lift transition
                    if _prev_writing:
                        draw_x.append(float('nan'))
                        draw_y.append(float('nan'))
                        smoother.reset()
                    lift_x.append(pos[0])
                    lift_y.append(pos[1])
                    _trim(lift_x, 100)
                    _trim(lift_y, 100)

                _prev_writing = is_writing

        # -- UWB packet: preprocess + update -------------------------------
        elif pkt['type'] == 'uwb':
            raw = pkt['dists']
            filtered, weights, despiked = uwb_cleaner.process(*raw)

            pos, vel, accepted, rejected = engine.process_uwb(
                despiked, weights)

            if pos is not None:
                _last_pos = pos
                _last_vel = vel

                # Feed back predictions for NLOS tracking
                uwb_cleaner.update_predictions(
                    np.append(pos, engine.ekf._tag_z), engine.anchors)

            # Mark rejected anchors by their known positions
            for i in rejected:
                rej_x.append(engine.anchors[i, 0])
                rej_y.append(engine.anchors[i, 1])
            _trim(rej_x, 40)
            _trim(rej_y, 40)

    # -- Plot update --------------------------------------------------------
    if _last_pos is not None:
        line_draw.set_data(draw_x, draw_y)
        scat_lift.set_data(lift_x, lift_y)
        scat_rej.set_data(rej_x, rej_y)
        dot_cur.set_data([_last_pos[0]], [_last_pos[1]])

        # Velocity arrow
        if _show_vel and _last_vel is not None:
            spd = float(np.linalg.norm(_last_vel))
            if spd > 0.01:
                vel_arrow.set_offsets([[_last_pos[0], _last_pos[1]]])
                vel_arrow.set_UVC(
                    [_last_vel[0] * VEL_SCALE],
                    [_last_vel[1] * VEL_SCALE],
                )
            else:
                vel_arrow.set_offsets([[_last_pos[0], _last_pos[1]]])
                vel_arrow.set_UVC([0], [0])

    # -- Console diagnostics -----------------------------------------------
    now = time.time()
    if now - _last_diag_time >= DIAG_INTERVAL_S and _last_pos is not None:
        _last_diag_time = now
        diag = engine.diagnostics
        bias = diag.get('bias')
        bstr = f"({bias[0]:+.3f}, {bias[1]:+.3f})" if bias is not None else "--"
        spd  = float(np.linalg.norm(_last_vel)) if _last_vel is not None else 0.0
        hl   = "YES" if diag.get('heading_locked') else "NO"
        print(f"[EKF] pos=({_last_pos[0]:.3f},{_last_pos[1]:.3f})  "
              f"speed={spd:.2f}m/s  "
              f"bias={bstr}  "
              f"imu_steps={diag['imu_steps']}  "
              f"uwb_acc={diag['uwb_accepted']}  "
              f"uwb_rej={diag['uwb_rejected']}  "
              f"outage={diag['consec_outage']}  "
              f"heading={hl}")

    return artists


# -- Keyboard handler -------------------------------------------------------
def on_key(event):
    global _show_vel, _prev_writing
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
        engine._cold_buf.clear()
        print("[UI] EKF reset — will re-initialise from next IRLS fix.")
    elif event.key in ('q', 'escape'):
        plt.close('all')


# -- Entry point ------------------------------------------------------------
def main():
    global artists, line_draw, scat_lift, scat_rej, dot_cur, vel_arrow

    if not parser.connect():
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(9, 9))
    fig.canvas.manager.set_window_title('PolyCast — EKF Live Position (Async)')
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
    ax.set_title('PolyCast — UWB + IMU EKF Fusion (Async Stream)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Info text (bottom)
    ax.text(0.01, 0.01,
            'V = velocity arrow   C = clear trail   R = reset EKF',
            transform=ax.transAxes, fontsize=8, color='gray', va='bottom')

    artists = (line_draw, scat_lift, scat_rej, dot_cur)

    ani = FuncAnimation(fig, update, interval=20, blit=False,
                        cache_frame_data=False)

    mode = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'[EKF] Running in {mode} mode — Async Stream')
    print(f'[EKF] Trail subsample: every {TRAIL_SUBSAMPLE} IMU predicts '
          f'(~{100/TRAIL_SUBSAMPLE:.0f} Hz display)')
    print(f'[EKF] Keys: V/C/R, Q to quit')

    plt.show()
    parser.close()


if __name__ == '__main__':
    main()
