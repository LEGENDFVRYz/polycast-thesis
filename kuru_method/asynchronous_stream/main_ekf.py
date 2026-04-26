"""
main_ekf.py  —  PolyCast EKF Live Visualiser (Async Stream)
============================================================
Event-driven sensor fusion visualiser for the decoupled async stream.

    IMU packet -> EKF predict  (~200 Hz)
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

import argparse
import atexit
import os
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
from note_smoother  import NoteSmoother
from imu_calibrate  import trigger_dcd_save


# -- Configuration ----------------------------------------------------------
from config import SERIAL_PORT, BAUD_RATE, UWB_OFFSETS
DATASET_FILENAME = ''   # '' = live; 'path/to/data.csv' = playback
MAX_TRAIL        = 1000  # maximum position samples in the drawing trail
SHOW_VELOCITY    = True  # initial state; toggle with V key
VEL_SCALE        = 0.3   # arrow length multiplier
DIAG_INTERVAL_S  = 1.0   # console diagnostic print interval

# Nominal IMU rate — used for display-rate reporting only; dt in the EKF
# comes from packet timestamps, not from this constant.
IMU_RATE_HZ      = 200

# Trail subsampling: at 200 Hz IMU, record every Nth predict to avoid
# overwhelming matplotlib.  3 = ~67 Hz display rate.
TRAIL_SUBSAMPLE  = 3

# CSV playback: packets to process per animation frame.
# At 200 Hz IMU (~5 ms cadence), 20 packets per 20 ms frame = ~5x speed.
CSV_PACKETS_PER_FRAME = 20


# -- Global state -----------------------------------------------------------
parser      = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE,
                              csv_path=DATASET_FILENAME)
engine      = AsyncEKFFusionEngine()
uwb_cleaner = UWBPreprocessor(offsets=UWB_OFFSETS)
imu_cleaner = IMUPreprocessor()
smoother    = TrailSmoother()
note_smooth = NoteSmoother()    # Phase 8 Step 4b Pattern A: per-stroke RTS


# -- Phase 9 Step 1: live auto-recording ------------------------------------
# Every live session is auto-recorded as a fresh timestamped CSV under
# datasets_str_50hz/ in the same I,…/U,… format that data_parser.py reads.
# This makes any live failure replayable through CSV mode.
_live_csv_file = None
_live_csv_path = None
_live_packet_count = 0


def _live_recorder_open() -> None:
    """Open the timestamped live capture file. Called once at module load
    when parser.mode == 'live'."""
    global _live_csv_file, _live_csv_path
    if parser.mode != 'live':
        return
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir    = os.path.join(script_dir, 'datasets_str_50hz')
    os.makedirs(out_dir, exist_ok=True)
    _live_csv_path = os.path.join(
        out_dir, f'live_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    _live_csv_file = open(_live_csv_path, 'w', encoding='utf-8', newline='')
    print(f'[live-rec] Recording to {_live_csv_path}')
    atexit.register(_live_recorder_close)


def _live_recorder_close() -> None:
    """Flush + close the live capture file. Idempotent."""
    global _live_csv_file
    if _live_csv_file is None:
        return
    try:
        _live_csv_file.flush()
        _live_csv_file.close()
    except Exception:
        pass
    print(f'[live-rec] Closed {_live_csv_path}  ({_live_packet_count} packets)')
    _live_csv_file = None


def _live_recorder_write(pkt: dict) -> None:
    """Append one packet to the live CSV in csv_recorder.py-compatible
    format. Reconstructed from parsed fields (data_parser drops the raw
    line); precision is preserved via Python's default float repr."""
    global _live_packet_count
    if _live_csv_file is None or pkt is None:
        return
    if pkt['type'] == 'imu':
        qx, qy, qz, qw = pkt['quat']
        ax, ay, az     = pkt['acc']
        line = (f"I,{pkt['seq']},{qx},{qy},{qz},{qw},"
                f"{ax},{ay},{az},{pkt['force']},{pkt['ts']}\n")
    elif pkt['type'] == 'uwb':
        d0, d1, d2, d3 = pkt['dists']
        line = f"U,{pkt['seq']},{d0},{d1},{d2},{d3},{pkt['ts']}\n"
    else:
        return
    _live_csv_file.write(line)
    _live_packet_count += 1
    # Periodic flush so a Ctrl+C / window-close can't lose more than ~500 pkts.
    if _live_packet_count % 500 == 0:
        _live_csv_file.flush()


_live_recorder_open()

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

# Phase 8 Step 4b Pattern A — "snap-to-RTS on stroke completion".
# When the FSR contact edge falls (stroke ends) we re-render the just-
# completed stroke using NoteSmoother (offline RTS) instead of leaving
# the causal 5-point moving-average trail. The active stroke remains
# causal during writing (cannot smooth the future), so this adds zero
# live-latency — the stroke just "snaps cleaner" the instant the pen lifts.
_stroke_history_start: int | None = None   # index into engine.ekf._history
_stroke_draw_start:    int | None = None   # index into draw_x

# Phase 8 Step 4b — parallel per-IMU-step streams used by the per-stroke
# green RTS overlay at end-of-CSV (Layer-10-style slicing).
_is_writing_stream:  list[bool] = []
_history_idx_stream: list[int]  = []


def _trim(lst, maxlen):
    if len(lst) > maxlen:
        del lst[:len(lst) - maxlen]


# Enable RTS history recording in BOTH modes — live mode needs it for the
# Step 4b snap-to-RTS pass on stroke completion.
engine.ekf.record_history = True
# Soft cap to bound memory in long live sessions (~200 Hz × 50 B/record).
_HISTORY_SOFT_CAP = 60_000   # ≈ 5 min @ 200 Hz before trim
_HISTORY_KEEP     = 30_000


def _snap_stroke_to_rts() -> None:
    """At lift edge: replace the most-recent stroke's draw_x/draw_y with
    the RTS-smoothed XY from the EKF history slice that covers it."""
    global _stroke_history_start, _stroke_draw_start
    if _stroke_history_start is None or _stroke_draw_start is None:
        return
    history = engine.ekf._history
    if not (0 <= _stroke_history_start < len(history)):
        _stroke_history_start = None
        _stroke_draw_start    = None
        return
    slice_ = history[_stroke_history_start:]
    if len(slice_) >= 2:
        smoothed = note_smooth.smooth_stroke(slice_)
        sub      = smoothed[::TRAIL_SUBSAMPLE]
        n_replace = len(draw_x) - _stroke_draw_start
        n_use     = min(len(sub), n_replace)
        for i in range(n_use):
            draw_x[_stroke_draw_start + i] = float(sub[i, 0])
            draw_y[_stroke_draw_start + i] = float(sub[i, 1])
    _stroke_history_start = None
    _stroke_draw_start    = None
    # Trim history if it has grown past the soft cap (live mode only —
    # CSV mode runs its own end-of-replay RTS overlay and needs the full
    # history; we still trim, but only past _HISTORY_KEEP).
    if len(history) > _HISTORY_SOFT_CAP:
        del history[:len(history) - _HISTORY_KEEP]


def _run_rts_smoother():
    """End-of-CSV per-stroke RTS overlay (matches Layer 10 logic).

    Phase 8 Step 4b edit 1: previously this plotted the *full history* RTS
    as one continuous line, which included every hover/relocate path and
    rendered as a tangled blob. Now it slices `engine.ekf._history` by
    the parallel `_is_writing_stream` flags and renders each stroke as a
    separate green line — same algorithm Layer 10 uses, just overlaid on
    the live axes.
    """
    if not engine.ekf.record_history or len(engine.ekf._history) < 10:
        print("[RTS] Not enough history to smooth.")
        return
    history = engine.ekf._history
    strokes_hist: list[list[dict]] = []
    cur: list[dict] = []
    for w, idx in zip(_is_writing_stream, _history_idx_stream):
        if 0 <= idx < len(history):
            if w:
                cur.append(history[idx])
            elif cur:
                strokes_hist.append(cur); cur = []
    if cur:
        strokes_hist.append(cur)

    print(f"[RTS] Per-stroke smoother on {len(strokes_hist)} strokes "
          f"({sum(len(s) for s in strokes_hist)} writing samples).")

    ax = line_draw.axes
    first = True
    for s in strokes_hist:
        if len(s) < 2:
            continue
        xy = note_smooth.smooth_stroke(s)
        ax.plot(xy[:, 0], xy[:, 1], '-', color='limegreen', lw=1.5,
                alpha=0.85,
                label='RTS smoothed (per-stroke)' if first else None,
                zorder=5)
        first = False
    ax.legend(loc='upper right', fontsize=9)
    ax.figure.canvas.draw_idle()
    print(f"[RTS] Per-stroke trajectories overlaid.")


# -- Animation callback -----------------------------------------------------
def update(frame):
    global _last_diag_time, _prev_writing, _eof_handled
    global _imu_subsample, _last_pos, _last_vel
    global _stroke_history_start, _stroke_draw_start

    # Drain available packets
    packets = []
    if parser.mode == 'live':
        while parser.data_available():
            pkt = parser.get_packet()
            if pkt and pkt != 'EOF':
                # Phase 9 Step 1: append to live capture before processing
                # so even a Python exception downstream still leaves the
                # raw stream on disk for diagnosis.
                _live_recorder_write(pkt)
                packets.append(pkt)
    else:
        for _ in range(CSV_PACKETS_PER_FRAME):
            pkt = parser.get_packet()
            if pkt == 'EOF':
                if not _eof_handled:
                    _eof_handled = True
                    # Edit 2: if the CSV ended mid-stroke (no fall edge ever
                    # fired), snap that final in-progress stroke now so it
                    # gets the same RTS treatment as every completed stroke.
                    _snap_stroke_to_rts()
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

            # Pen-tip position (Item A): subtract the tip-to-tag offset
            # projected onto the whiteboard plane.  Falls back to the tag
            # position before heading lock.
            tip = engine.tip_position
            draw_pt = pos

            _last_pos = draw_pt
            _last_vel = vel

            # Per-IMU-step streams for the per-stroke RTS overlay (edit 1).
            if engine.ekf.initialized:
                _is_writing_stream.append(bool(is_writing))
                _history_idx_stream.append(len(engine.ekf._history) - 1)

            # Trail management (subsampled)
            _imu_subsample += 1
            if _imu_subsample >= TRAIL_SUBSAMPLE:
                _imu_subsample = 0

                # Phase 8 Step 4b — track stroke boundaries so the snap-to-RTS
                # pass can replace just-written points with the offline RTS XY
                # the instant the FSR fall edge fires.
                rising  = is_writing and not _prev_writing
                falling = (not is_writing) and _prev_writing
                if rising:
                    _stroke_history_start = len(engine.ekf._history)
                    _stroke_draw_start    = len(draw_x)

                if is_writing:
                    smoother.push(draw_pt[0], draw_pt[1])
                    sx, sy = smoother.get()
                    draw_x.append(sx)
                    draw_y.append(sy)
                    # Edit 3: in CSV-playback mode keep every stroke visible.
                    # Live mode still trims to bound memory.
                    if parser.mode != 'csv':
                        _trim(draw_x, MAX_TRAIL)
                        _trim(draw_y, MAX_TRAIL)
                else:
                    # NaN break on pen lift transition
                    if falling:
                        # Snap-to-RTS BEFORE inserting the NaN separator,
                        # so the just-completed stroke's points get rewritten
                        # in place with the offline-smoothed XY.
                        _snap_stroke_to_rts()
                        draw_x.append(float('nan'))
                        draw_y.append(float('nan'))
                        smoother.reset()
                    lift_x.append(draw_pt[0])
                    lift_y.append(draw_pt[1])
                    _trim(lift_x, 100)
                    _trim(lift_y, 100)

                _prev_writing = is_writing

        # -- UWB packet: preprocess + update -------------------------------
        elif pkt['type'] == 'uwb':
            raw = pkt['dists']
            # filtered = viz-only (median + EMA). ekf_dists = offset+gate only.
            filtered, weights, ekf_dists = uwb_cleaner.process(*raw)

            pos, vel, accepted, rejected = engine.process_uwb(
                ekf_dists, weights, ts=pkt.get('ts'))

            if pos is not None:
                # Tip-aware drawing point (Item A) — see IMU branch above.
                tip = engine.tip_position
                _last_pos = tip if tip is not None else pos
                _last_vel = vel

                # Feed back predictions for NLOS tracking using the tilt-
                # aware tag z (engine.tag_z_wb), not the static MARKER_LENGTH.
                uwb_cleaner.update_predictions(
                    np.append(pos, engine.tag_z_wb), engine.anchors)

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
        engine.imu_integrator.reset_heading()
        print("[UI] EKF reset — will re-initialise from next IRLS fix.")
    elif event.key in ('q', 'escape'):
        plt.close('all')


# -- Entry point ------------------------------------------------------------
def _parse_cli():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibrate", action="store_true",
                   help="Send CAL command to persist BNO085 DCD, then exit "
                        "(Item D — sh2_saveDcdNow on the marker).")
    return p.parse_args()


def main():
    global artists, line_draw, scat_lift, scat_rej, dot_cur, vel_arrow

    args = _parse_cli()
    if args.calibrate:
        rc = trigger_dcd_save()
        sys.exit(0 if rc == 0 else 1)

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
                         label='Pen tip (writing)', zorder=4)
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
          f'(~{IMU_RATE_HZ/TRAIL_SUBSAMPLE:.0f} Hz display)')
    print(f'[EKF] Keys: V/C/R, Q to quit')

    plt.show()
    parser.close()


if __name__ == '__main__':
        main()
