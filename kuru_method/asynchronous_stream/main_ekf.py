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
from collections import deque
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
from config import SERIAL_PORT, BAUD_RATE, UWB_OFFSETS, TIP_OFFSET_FROM_TAG_M
DATASET_FILENAME = 'datasets_str_50hz/ABC_b2.csv'   # '' = live; 'path/to/data.csv' = playback
MAX_TRAIL        = 1000  # maximum position samples in the drawing trail
SHOW_VELOCITY    = True  # initial state; toggle with V key
VEL_SCALE        = 0.3   # arrow length multiplier
DIAG_INTERVAL_S  = 1.0   # console diagnostic print interval
DEBUG_ASSISTANT  = True  # show rule-based live debugging panel
SHOW_RTS         = False # start hidden; toggle with T. RTS is diagnostic/experimental.
# Visual-only relocation break guard. This does not change the EKF state; it only
# inserts NaN separators in the drawn/saved trail when FSR appears to stay
# pressed during a reposition. Toggle at runtime with the B key.
AUTO_BREAK_RELOCATION = True
RELOC_SEGMENT_M       = 0.045  # one display-sample jump > 4.5 cm -> break
RELOC_SPEED_MPS       = 0.80   # fast move while contact is still true
RELOC_SPEED_SEG_M     = 0.025  # and at least 2.5 cm since last drawn point
USE_ONLINE_TRAIL_SMOOTHER = False  # False = draw raw causal EKF tip, True = 5-point causal display smoother

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
        gyro = pkt.get('gyro')
        gyro_valid = bool(pkt.get('gyro_valid', gyro is not None))
        if gyro_valid and gyro is not None:
            gx, gy, gz = gyro
            # New receiver format: preserve calibrated gyroscope data.
            line = (f"I,{pkt['seq']},{qx},{qy},{qz},{qw},"
                    f"{ax},{ay},{az},{gx},{gy},{gz},{pkt['force']},{pkt['ts']}\n")
        else:
            # Backward-compatible legacy write for old datasets without gyro.
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

# Writing trails
# draw_x/draw_y = causal pen-tip trace used for comparison.
draw_x, draw_y       = [], []
# tag_draw_x/tag_draw_y = fused UWB tag XY trace.
tag_draw_x, tag_draw_y = [], []
# Lifted trail — positions where is_writing == False (faint pen-tip trace)
lift_x, lift_y       = [], []
# Rejected anchor scatter
rej_x, rej_y         = [], []
# RTS-smoothed per-stroke overlay. Kept separate from the causal EKF trace.
rts_x, rts_y         = [], []

_show_vel        = SHOW_VELOCITY
_show_rts        = SHOW_RTS
_auto_break_relocation = AUTO_BREAK_RELOCATION
_last_diag_time  = 0.0
_prev_writing    = False
_eof_handled     = False
_imu_subsample   = 0       # counter for trail subsampling

# Latest positions/velocity for artist updates (persists across packets)
_last_pos = None          # pen-tip position kept for compatibility
_last_tip_pos = None
_last_tag_pos = None
_last_vel = None
_last_draw_tip = None
_last_draw_tag = None

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
# Axis sample aligned with each EKF history record. Used to convert RTS tag XY to pen-tip XY.
_axis_by_history_idx: list[np.ndarray] = []
_last_rts_stats: dict = {}


def _safe_axis_for_history(i: int) -> np.ndarray:
    if 0 <= i < len(_axis_by_history_idx):
        return _axis_by_history_idx[i]
    return np.array([0.0, 0.0, 1.0], dtype=float)


def _tag_xy_to_tip_xy(tag_xy: np.ndarray, axes_wb: np.ndarray) -> np.ndarray:
    axes = np.asarray(axes_wb, dtype=float)
    if axes.ndim == 1:
        axes = np.tile(axes, (len(tag_xy), 1))
    if len(axes) < len(tag_xy):
        pad = np.tile(axes[-1] if len(axes) else np.array([0.0, 0.0, 1.0]),
                      (len(tag_xy) - len(axes), 1))
        axes = np.vstack([axes, pad])
    return np.asarray(tag_xy, dtype=float) - TIP_OFFSET_FROM_TAG_M * axes[:len(tag_xy), :2]


class DebugAssistant:
    """Rule-based live diagnostic layer for main_ekf.py.

    This is diagnostic-only: it never changes fusion behavior. It keeps
    both the latest UWB update and the latest PEN_DOWN UWB update so the
    panel remains useful after the pen enters hover_long.
    """

    RTS_SUSPECT_DELTA_M = 0.05

    def __init__(self, maxlen: int = 250):
        self.maxlen = maxlen
        self.uwb_accept_counts = deque(maxlen=maxlen)
        self.uwb_reject_counts = deque(maxlen=maxlen)
        self.dropout_counts = deque(maxlen=maxlen)
        self.quality = deque(maxlen=maxlen)
        self.last_warnings: list[str] = []
        self.last_rts = {}
        self.last_mode = 'unknown'
        self.last_contact = False
        self.current_anchor_rows: list[str] = []
        self.pen_down_anchor_rows: list[str] = []
        self.pen_down_summary = 'No pen-down UWB update captured yet.'
        self.last_pen_down_range_kf_lag_m = 0.0
        # Contact/FSR diagnostics: this is now the main suspect when UWB is healthy.
        self.imu_samples = 0
        self.contact_samples = 0
        self.stroke_count = 0
        self._prev_contact_dbg = False
        self._stroke_start_ts = None
        self._stroke_pts = []
        self.last_stroke_summary = 'No completed stroke yet.'
        self.last_force_raw = float('nan')
        self.last_force_smooth = float('nan')
        self.last_force_pre = 0
        self.motion_breaks = 0
        self.last_motion_break = 'none'
        self.completed_strokes = deque(maxlen=12)

    def ingest_imu(self, is_writing: bool, raw_force=None, ts=None, tip_pos=None) -> None:
        contact = bool(is_writing)
        self.imu_samples += 1
        if contact:
            self.contact_samples += 1
        self.last_contact = contact
        try:
            self.last_mode = engine._pen_mode.mode.value
        except Exception:
            self.last_mode = 'unknown'
        try:
            self.last_force_raw = float(raw_force) if raw_force is not None else float('nan')
            self.last_force_smooth = float(engine._contact._ema) if engine._contact._ema is not None else float('nan')
            self.last_force_pre = int(engine._contact._raw_classify)
        except Exception:
            pass

        # Track stroke boundaries and stroke size. Large continuous strokes are
        # strong evidence that FSR/contact stayed down during a lift/reposition.
        if contact and not self._prev_contact_dbg:
            self.stroke_count += 1
            self._stroke_start_ts = ts
            self._stroke_pts = []
        if contact and tip_pos is not None:
            try:
                self._stroke_pts.append(np.asarray(tip_pos, dtype=float).copy())
            except Exception:
                pass
        if (not contact) and self._prev_contact_dbg:
            if self._stroke_pts:
                pts = np.asarray(self._stroke_pts, dtype=float)
                mn = np.nanmin(pts, axis=0); mx = np.nanmax(pts, axis=0)
                w = float(mx[0] - mn[0]); h = float(mx[1] - mn[1])
                dur = 0.0
                if ts is not None and self._stroke_start_ts is not None:
                    dur = max(0.0, (int(ts) - int(self._stroke_start_ts)) / 1_000_000.0)
                diffs = np.diff(pts[:, :2], axis=0) if len(pts) > 1 else np.empty((0, 2))
                segs = np.linalg.norm(diffs, axis=1) if len(diffs) else np.array([], dtype=float)
                path_m = float(np.nansum(segs)) if len(segs) else 0.0
                max_jump_m = float(np.nanmax(segs)) if len(segs) else 0.0
                straight_m = float(np.linalg.norm(pts[-1, :2] - pts[0, :2])) if len(pts) > 1 else 0.0
                straightness = straight_m / max(path_m, 1e-9)
                self.last_stroke_summary = (f'last stroke: n={len(pts)} dur={dur:.2f}s '
                                            f'box={w*100:.1f}x{h*100:.1f}cm '
                                            f'path={path_m*100:.1f}cm jump={max_jump_m*100:.1f}cm')
                self.completed_strokes.append({
                    'id': self.stroke_count,
                    'n': len(pts),
                    'dur': dur,
                    'w': w,
                    'h': h,
                    'path': path_m,
                    'max_jump': max_jump_m,
                    'straightness': straightness,
                })
            self._stroke_pts = []
            self._stroke_start_ts = None
        self._prev_contact_dbg = contact

    @staticmethod
    def _fmt(v, width=5, prec=2):
        try:
            f = float(v)
        except Exception:
            return ' ' * max(0, width - 2) + '--'
        if not np.isfinite(f):
            return ' ' * max(0, width - 2) + '--'
        return f'{f:{width}.{prec}f}'

    def _make_anchor_rows(self, ekf_dists, weights, accepted, rejected,
                          pre_pos=None, pre_P=None, pre_tag_z=None):
        rows = []
        z_in = np.asarray(ekf_dists, dtype=float)
        qs = np.asarray(weights, dtype=float)
        filt = np.asarray(engine.last_filtered_ranges, dtype=float)
        try:
            used = np.asarray(engine.last_ekf_ranges_used, dtype=float)
        except Exception:
            used = filt
        accepted_set = set(accepted)
        rejected_set = set(rejected)

        for i, a in enumerate(engine.anchors):
            mute_rem = int(engine.ekf._mute_remain[i]) if hasattr(engine.ekf, '_mute_remain') else 0
            if i >= len(z_in) or not np.isfinite(z_in[i]):
                status = 'MISS'
            elif mute_rem > 0 and i in rejected_set:
                status = 'MUTE'
            elif i in accepted_set:
                status = 'ACC '
            elif i in rejected_set:
                status = 'REJ '
            else:
                status = '----'

            z_kf = float(filt[i]) if i < len(filt) else float('nan')
            z_used = float(used[i]) if i < len(used) else z_kf
            pred = np.nan
            innov = np.nan
            nis_g = np.nan

            if pre_pos is not None and pre_P is not None and pre_tag_z is not None and i < len(z_in) and np.isfinite(z_in[i]):
                dx = float(pre_pos[0] - a[0])
                dy = float(pre_pos[1] - a[1])
                dz = float(pre_tag_z - a[2])
                pred = float(np.sqrt(dx*dx + dy*dy + dz*dz))
                if np.isfinite(z_used) and pred > 1e-9:
                    innov = z_used - pred
                    H = np.array([[dx / pred, dy / pred, 0.0, 0.0, 0.0, 0.0]], dtype=float)
                    try:
                        hpht = float((H @ pre_P @ H.T).item())
                        sg = hpht + engine.ekf.GATE_SIGMA_UWB ** 2
                        if sg > 1e-12:
                            nis_g = (innov * innov) / sg
                    except Exception:
                        nis_g = np.nan

            q_i = qs[i] if i < len(qs) else np.nan
            row = (
                f'A{i} {status} q={self._fmt(q_i,4,2)}  in={self._fmt(z_in[i] if i < len(z_in) else np.nan,5,2)} '
                f'use={self._fmt(z_used,5,2)} kf={self._fmt(z_kf,5,2)}\n'
                f'   pr={self._fmt(pred,5,2)}  ν={self._fmt(innov,6,3)}  NIS={self._fmt(nis_g,5,2)}'
            )
            if mute_rem > 0:
                row += f' mute={mute_rem:02d}'
            rows.append(row)
        return rows

    def ingest_uwb(self, raw, ekf_dists, weights, accepted, rejected,
                   pre_pos=None, pre_P=None, pre_tag_z=None,
                   is_pen_down: bool = False) -> None:
        ekf_arr = np.asarray(ekf_dists, dtype=float)
        weights_arr = np.asarray(weights, dtype=float)
        self.uwb_accept_counts.append(len(accepted))
        self.uwb_reject_counts.append(len(rejected))
        self.dropout_counts.append(int(np.sum(~np.isfinite(ekf_arr))))
        if len(weights_arr):
            self.quality.append(float(np.nanmean(weights_arr)))

        rows = self._make_anchor_rows(ekf_dists, weights, accepted, rejected,
                                      pre_pos=pre_pos, pre_P=pre_P, pre_tag_z=pre_tag_z)
        self.current_anchor_rows = rows

        # Preserve the most recent writing-time anchor diagnostics so after
        # lift/hover we can still inspect what happened during the stroke.
        if is_pen_down:
            self.pen_down_anchor_rows = list(rows)
            acc = len(accepted)
            rej = len(rejected)
            qmean = float(np.nanmean(weights_arr)) if len(weights_arr) else float('nan')
            try:
                kf = np.asarray(engine.last_filtered_ranges, dtype=float)
                zin = np.asarray(ekf_dists, dtype=float)
                self.last_pen_down_range_kf_lag_m = float(np.nanmax(np.abs(kf - zin)))
            except Exception:
                self.last_pen_down_range_kf_lag_m = 0.0
            self.pen_down_summary = (f'Last pen-down UWB: acc={acc}, rej={rej}, q_mean={qmean:.2f}, '
                                     f'max|kf-in|={self.last_pen_down_range_kf_lag_m*100:.1f}cm')

    def ingest_motion_break(self, seg_m: float, speed_mps: float, reason: str) -> None:
        self.motion_breaks += 1
        self.last_motion_break = f'#{self.motion_breaks}: {reason}, seg={seg_m*100:.1f}cm, speed={speed_mps:.2f}m/s'

    def ingest_rts(self, stats: dict) -> None:
        self.last_rts = dict(stats)

    def _window_reject_rate(self) -> float:
        acc = float(np.sum(self.uwb_accept_counts))
        rej = float(np.sum(self.uwb_reject_counts))
        return rej / max(1.0, acc + rej)

    def warnings(self) -> list[str]:
        diag = engine.diagnostics
        w = []
        bias = diag.get('bias')
        vel = diag.get('velocity')
        speed = float(np.linalg.norm(vel)) if vel is not None else 0.0
        pos = _last_tip_pos if _last_tip_pos is not None else _last_pos
        heading_locked = bool(diag.get('heading_locked'))
        outage = int(diag.get('consec_outage', 0))
        rej_rate = self._window_reject_rate()
        q_mean = float(np.mean(self.quality)) if self.quality else 1.0
        drop_recent = int(np.sum(self.dropout_counts)) if self.dropout_counts else 0

        if not heading_locked and diag.get('imu_steps', 0) > 100:
            w.append('Heading still unlocked after >100 IMU predicts.')
        if outage >= engine.ekf.MAX_CONSEC_OUTAGE:
            w.append(f'UWB outage={outage}: EKF is mostly propagating on IMU.')
        if rej_rate > 0.60 and len(self.uwb_reject_counts) > 20:
            w.append(f'High UWB gate rejection window ({rej_rate*100:.0f}%).')
        if q_mean < 0.35 and len(self.quality) > 20:
            w.append(f'Low mean anchor quality ({q_mean:.2f}): likely NLOS / multipath.')
        if drop_recent > 0:
            w.append(f'Recent missing/invalid UWB anchor samples: {drop_recent}.')
        if bias is not None and np.max(np.abs(bias)) > 0.095:
            w.append('Accel bias is near clamp (±0.10 m/s²).')
        if speed > 0.95 * engine.ekf.MAX_WRITING_SPEED:
            w.append('Velocity is near hard cap: inspect dt / IMU spikes / UWB pull.')
        if pos is not None:
            x_max = float(np.max(engine.anchors[:, 0]))
            y_max = float(np.max(engine.anchors[:, 1]))
            if pos[0] < -0.10 or pos[0] > x_max + 0.10 or pos[1] < -0.10 or pos[1] > y_max + 0.10:
                w.append('Displayed tip is outside board margin.')
        if self.last_rts and self.last_rts.get('max_delta_m', 0.0) > self.RTS_SUSPECT_DELTA_M:
            w.append(f'RTS SUSPECT: moved last stroke by {self.last_rts["max_delta_m"]*100:.1f} cm max.')
        if self.last_pen_down_range_kf_lag_m > 0.04:
            w.append(f'Range-KF lag suspect: max |kf-in| during pen-down was {self.last_pen_down_range_kf_lag_m*100:.1f} cm.')
        if self.motion_breaks > 0:
            w.append(f'Display motion-breaks inserted: {self.motion_breaks}; last {self.last_motion_break}.')
        if self.completed_strokes:
            worst = max(self.completed_strokes, key=lambda s: max(s['w'], s['h']))
            if max(worst['w'], worst['h']) > 0.18 or worst['max_jump'] > 0.05:
                w.append(f'Stroke #{worst["id"]} suspect: box={worst["w"]*100:.1f}x{worst["h"]*100:.1f}cm, jump={worst["max_jump"]*100:.1f}cm.')
        # Warn when one contact segment is big enough to look like multiple letters/lines.
        try:
            if 'box=' in self.last_stroke_summary:
                import re as _re
                m = _re.search(r'box=([0-9.]+)x([0-9.]+)cm', self.last_stroke_summary)
                if m and (float(m.group(1)) > 30.0 or float(m.group(2)) > 22.0):
                    w.append('Contact segmentation suspect: one pen-down stroke spans a large area; inspect FSR thresholds/debounce.')
        except Exception:
            pass
        self.last_warnings = w[:7]
        return self.last_warnings

    def text(self) -> str:
        diag = engine.diagnostics
        bias = diag.get('bias')
        vel = diag.get('velocity')
        speed = float(np.linalg.norm(vel)) if vel is not None else 0.0
        bstr = f'{bias[0]:+.3f},{bias[1]:+.3f}' if bias is not None else '--'
        acc_win = float(np.mean(self.uwb_accept_counts)) if self.uwb_accept_counts else 0.0
        rej_win = float(np.mean(self.uwb_reject_counts)) if self.uwb_reject_counts else 0.0
        q_mean = float(np.mean(self.quality)) if self.quality else 1.0
        tip_tag_delta = None
        if _last_tip_pos is not None and _last_tag_pos is not None:
            tip_tag_delta = _last_tip_pos - _last_tag_pos

        contact_duty = (self.contact_samples / self.imu_samples * 100.0) if self.imu_samples else 0.0
        lines = [
            'POLYCAST DEBUG ASSISTANT',
            'FILTER',
            f'mode={self.last_mode} contact={int(self.last_contact)} heading={"LOCK" if diag.get("heading_locked") else "NO"}',
            f'tip=({_last_tip_pos[0]:+.3f},{_last_tip_pos[1]:+.3f})  tag=({_last_tag_pos[0]:+.3f},{_last_tag_pos[1]:+.3f})' if (_last_tip_pos is not None and _last_tag_pos is not None) else 'tip/tag=--',
            (f'tip-tag=({tip_tag_delta[0]:+.3f},{tip_tag_delta[1]:+.3f}) |d|={np.linalg.norm(tip_tag_delta):.3f}m'
             if tip_tag_delta is not None else 'tip-tag=--'),
            f'speed={speed:.2f}m/s bias={bstr}',
            f'uwb win acc/rej={acc_win:.1f}/{rej_win:.1f} outage={diag.get("consec_outage", 0)} q={q_mean:.2f}',
            '',
            'CONTACT / FSR',
            f'force raw={self._fmt(self.last_force_raw,5,0)} ema={self._fmt(self.last_force_smooth,5,0)} pre={self.last_force_pre} deb={int(self.last_contact)}',
            f'strokes={self.stroke_count} duty={contact_duty:.1f}%  {self.last_stroke_summary}',
            f'motion-breaks={self.motion_breaks} auto={"ON" if _auto_break_relocation else "OFF"} last={self.last_motion_break}',
        ]
        if self.completed_strokes:
            recent = list(self.completed_strokes)[-4:]
            lines.append('recent strokes: ' + ' | '.join([
                f'#{s["id"]} {s["dur"]:.1f}s {s["w"]*100:.0f}x{s["h"]*100:.0f}cm j{s["max_jump"]*100:.1f}'
                for s in recent
            ]))
        if self.last_rts:
            label = 'RTS SUSPECT' if self.last_rts.get('max_delta_m', 0.0) > self.RTS_SUSPECT_DELTA_M else 'RTS'
            lines.append(f'{label}: mean={self.last_rts.get("mean_delta_m", 0)*100:.1f}cm max={self.last_rts.get("max_delta_m", 0)*100:.1f}cm')
        lines.extend(['', 'CURRENT UWB', 'A STAT q     in   use    kf / pred, innov, NIS'])
        lines.extend(self.current_anchor_rows or ['-- no current UWB update --'])
        lines.extend(['', 'LAST PEN-DOWN UWB', self.pen_down_summary, 'A STAT q     in   use    kf / pred, innov, NIS'])
        lines.extend(self.pen_down_anchor_rows or ['-- waiting for pen-down UWB --'])
        warns = self.warnings()
        lines.append('')
        if warns:
            lines.append('WARNINGS')
            lines.extend([f'• {x}' for x in warns])
        else:
            lines.append('WARNINGS: none')
        lines.append('')
        lines.append('Legend: in=input, use=EKF range, kf=range-KF, pr=pred, ν=use-pr')
        return '\n'.join(lines)


debugger = DebugAssistant()


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
    """At lift edge: compute RTS for the just-completed stroke and append it
    to a separate overlay trace. The causal EKF trace is intentionally left
    unchanged so smoothed vs. unsmoothed output can be compared."""
    global _stroke_history_start, _stroke_draw_start, _last_rts_stats
    if _stroke_history_start is None:
        return
    history = engine.ekf._history
    if not (0 <= _stroke_history_start < len(history)):
        _stroke_history_start = None
        _stroke_draw_start    = None
        return

    hist_slice = history[_stroke_history_start:]
    if len(hist_slice) >= 2:
        smoothed_tag = note_smooth.smooth_stroke(hist_slice)
        axes = np.array([_safe_axis_for_history(i)
                         for i in range(_stroke_history_start,
                                        _stroke_history_start + len(smoothed_tag))])
        smoothed_tip = _tag_xy_to_tip_xy(smoothed_tag, axes)
        sub = smoothed_tip[::TRAIL_SUBSAMPLE]

        # Compare RTS to the causal EKF samples from the same stroke.
        if _stroke_draw_start is not None and _stroke_draw_start < len(draw_x):
            causal = np.column_stack([
                np.asarray(draw_x[_stroke_draw_start:], dtype=float),
                np.asarray(draw_y[_stroke_draw_start:], dtype=float),
            ])
            causal = causal[np.all(np.isfinite(causal), axis=1)]
            n = min(len(causal), len(sub))
            if n > 0:
                delta = np.linalg.norm(causal[:n] - sub[:n], axis=1)
                _last_rts_stats = {
                    'n': int(n),
                    'mean_delta_m': float(np.mean(delta)),
                    'max_delta_m': float(np.max(delta)),
                }
                debugger.ingest_rts(_last_rts_stats)

        for x, y in sub:
            rts_x.append(float(x))
            rts_y.append(float(y))
        rts_x.append(float('nan'))
        rts_y.append(float('nan'))

        if parser.mode != 'csv':
            _trim(rts_x, MAX_TRAIL)
            _trim(rts_y, MAX_TRAIL)

    _stroke_history_start = None
    _stroke_draw_start    = None

    # Trim history and aligned axis cache together in long live sessions.
    if len(history) > _HISTORY_SOFT_CAP:
        n_drop = len(history) - _HISTORY_KEEP
        del history[:n_drop]
        del _axis_by_history_idx[:min(n_drop, len(_axis_by_history_idx))]


def _run_rts_smoother():
    """End-of-CSV per-stroke RTS overlay (matches Layer 10 logic).

    Phase 8 Step 4b edit 1: previously this plotted the *full history* RTS
    as one continuous line, which included every hover/relocate path and
    rendered as a tangled blob. Now it slices `engine.ekf._history` by
    the parallel `_is_writing_stream` flags and renders each stroke as a
    separate green line — same algorithm Layer 10 uses, just overlaid on
    the live axes.
    """
    # In the debug visualizer, completed strokes are already overlaid at each
    # FSR fall edge. Avoid duplicating them at EOF; this function is only a
    # fallback for older CSVs or cases where no live fall edge was observed.
    if rts_x:
        return
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

    for s in strokes_hist:
        if len(s) < 2:
            continue
        xy_tag = note_smooth.smooth_stroke(s)
        xy_tip = xy_tag  # End-of-CSV fallback: exact per-sample axes may already be live-overlaid.
        for x, y in xy_tip[::TRAIL_SUBSAMPLE]:
            rts_x.append(float(x)); rts_y.append(float(y))
        rts_x.append(float('nan')); rts_y.append(float('nan'))
    line_rts.set_data(rts_x, rts_y)
    line_rts.set_visible(_show_rts)
    line_draw.axes.figure.canvas.draw_idle()
    print(f"[RTS] Per-stroke trajectories overlaid.")


def _should_break_relocation(draw_pt, tag_pt, vel, rising: bool) -> tuple[bool, str, float, float]:
    """Visual-only guard against false connectors when FSR contact sticks
    through a reposition. Returns (break?, reason, segment_m, speed_mps)."""
    global _last_draw_tip
    if (not _auto_break_relocation) or rising or _last_draw_tip is None:
        return False, '', 0.0, float(np.linalg.norm(vel)) if vel is not None else 0.0
    try:
        pt = np.asarray(draw_pt, dtype=float)[:2]
        prev = np.asarray(_last_draw_tip, dtype=float)[:2]
        seg = float(np.linalg.norm(pt - prev))
        spd = float(np.linalg.norm(vel)) if vel is not None else 0.0
    except Exception:
        return False, '', 0.0, 0.0
    if seg > RELOC_SEGMENT_M:
        return True, 'large display jump', seg, spd
    if spd > RELOC_SPEED_MPS and seg > RELOC_SPEED_SEG_M:
        return True, 'fast pen-down move', seg, spd
    return False, '', seg, spd

# -- Animation callback -----------------------------------------------------
def update(frame):
    global _last_diag_time, _prev_writing, _eof_handled
    global _imu_subsample, _last_pos, _last_tip_pos, _last_tag_pos, _last_vel
    global _last_draw_tip, _last_draw_tag
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
            # projected onto the whiteboard plane. Falls back to the tag
            # position before heading lock.
            tip = engine.tip_position
            draw_pt = tip if tip is not None else pos

            _last_tag_pos = np.asarray(pos, dtype=float).copy()
            _last_tip_pos = np.asarray(draw_pt, dtype=float).copy()
            _last_pos = _last_tip_pos
            _last_vel = vel
            debugger.ingest_imu(is_writing, raw_force=pkt.get('force'), ts=pkt.get('ts'), tip_pos=_last_tip_pos)

            # Per-IMU-step streams for the per-stroke RTS overlay (edit 1).
            if engine.ekf.initialized and len(engine.ekf._history) > 0:
                hist_idx = len(engine.ekf._history) - 1
                _is_writing_stream.append(bool(is_writing))
                _history_idx_stream.append(hist_idx)
                while len(_axis_by_history_idx) <= hist_idx:
                    _axis_by_history_idx.append(engine.marker_axis_wb.copy())
                _axis_by_history_idx[hist_idx] = engine.marker_axis_wb.copy()

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
                    _last_draw_tip = None
                    _last_draw_tag = None

                if is_writing:
                    if USE_ONLINE_TRAIL_SMOOTHER:
                        smoother.push(draw_pt[0], draw_pt[1])
                        sx, sy = smoother.get()
                    else:
                        sx, sy = float(draw_pt[0]), float(draw_pt[1])
                    break_now, reason, seg_m, spd_mps = _should_break_relocation(
                        np.array([sx, sy], dtype=float), pos, vel, rising)
                    if break_now:
                        draw_x.append(float('nan'))
                        draw_y.append(float('nan'))
                        tag_draw_x.append(float('nan'))
                        tag_draw_y.append(float('nan'))
                        _last_draw_tip = None
                        _last_draw_tag = None
                        smoother.reset()
                        debugger.ingest_motion_break(seg_m, spd_mps, reason)
                    draw_x.append(sx)
                    draw_y.append(sy)
                    tag_draw_x.append(float(pos[0]))
                    tag_draw_y.append(float(pos[1]))
                    _last_draw_tip = np.array([sx, sy], dtype=float)
                    _last_draw_tag = np.asarray(pos[:2], dtype=float).copy()
                    # In CSV-playback mode keep every stroke visible.
                    # Live mode still trims to bound memory.
                    if parser.mode != 'csv':
                        _trim(draw_x, MAX_TRAIL)
                        _trim(draw_y, MAX_TRAIL)
                        _trim(tag_draw_x, MAX_TRAIL)
                        _trim(tag_draw_y, MAX_TRAIL)
                else:
                    # NaN break on pen lift transition
                    if falling:
                        _snap_stroke_to_rts()
                        draw_x.append(float('nan'))
                        draw_y.append(float('nan'))
                        tag_draw_x.append(float('nan'))
                        tag_draw_y.append(float('nan'))
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

            pre_pos = engine.ekf.position if engine.ekf.initialized else None
            pre_P = engine.ekf._P.copy() if engine.ekf.initialized and engine.ekf._P is not None else None
            pre_tag_z = engine.tag_z_wb if engine.ekf.initialized else None
            was_pen_down = bool(engine.is_writing)

            pos, vel, accepted, rejected = engine.process_uwb(
                ekf_dists, weights, ts=pkt.get('ts'))
            debugger.ingest_uwb(raw, ekf_dists, weights, accepted, rejected,
                                pre_pos=pre_pos, pre_P=pre_P,
                                pre_tag_z=pre_tag_z, is_pen_down=was_pen_down)

            if pos is not None:
                # Tip-aware drawing point (Item A) — see IMU branch above.
                tip = engine.tip_position
                _last_tag_pos = np.asarray(pos, dtype=float).copy()
                _last_tip_pos = np.asarray(tip if tip is not None else pos, dtype=float).copy()
                _last_pos = _last_tip_pos
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
        line_tag.set_data(tag_draw_x, tag_draw_y)
        line_rts.set_data(rts_x, rts_y)
        line_rts.set_visible(_show_rts)
        scat_lift.set_data(lift_x, lift_y)
        scat_rej.set_data(rej_x, rej_y)
        if _last_tip_pos is not None:
            dot_tip.set_data([_last_tip_pos[0]], [_last_tip_pos[1]])
        if _last_tag_pos is not None:
            dot_tag.set_data([_last_tag_pos[0]], [_last_tag_pos[1]])
        if DEBUG_ASSISTANT:
            debug_text.set_text(debugger.text())

        # Velocity arrow
        if _show_vel and _last_vel is not None and _last_tip_pos is not None:
            spd = float(np.linalg.norm(_last_vel))
            if spd > 0.01:
                vel_arrow.set_offsets([[_last_tip_pos[0], _last_tip_pos[1]]])
                vel_arrow.set_UVC(
                    [_last_vel[0] * VEL_SCALE],
                    [_last_vel[1] * VEL_SCALE],
                )
            else:
                vel_arrow.set_offsets([[_last_tip_pos[0], _last_tip_pos[1]]])
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
    global _show_vel, _show_rts, _auto_break_relocation, _prev_writing, DEBUG_ASSISTANT, _last_draw_tip, _last_draw_tag
    if event.key == 'v':
        _show_vel = not _show_vel
        print(f"[UI] Velocity arrow: {'ON' if _show_vel else 'OFF'}")
    elif event.key == 't':
        _show_rts = not _show_rts
        try:
            line_rts.set_visible(_show_rts)
        except NameError:
            pass
        print(f"[UI] RTS overlay: {'ON' if _show_rts else 'OFF'}")
    elif event.key == 'd':
        DEBUG_ASSISTANT = not DEBUG_ASSISTANT
        try:
            debug_text.set_visible(DEBUG_ASSISTANT)
        except NameError:
            pass
        print(f"[UI] Debug assistant: {'ON' if DEBUG_ASSISTANT else 'OFF'}")
    elif event.key == 'c':
        draw_x.clear(); draw_y.clear()
        tag_draw_x.clear(); tag_draw_y.clear()
        rts_x.clear();  rts_y.clear()
        lift_x.clear(); lift_y.clear()
        rej_x.clear();  rej_y.clear()
        smoother.reset()
        _last_draw_tip = None
        _last_draw_tag = None
        _prev_writing = False
        print("[UI] Trail cleared.")
    elif event.key == 'r':
        engine.ekf._x = None   # force re-initialisation
        engine._cold_buf.clear()
        engine.imu_integrator.reset_heading()
        engine.ekf._history.clear()
        _axis_by_history_idx.clear()
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
    global artists, line_draw, line_tag, line_rts, scat_lift, scat_rej, dot_tip, dot_tag, vel_arrow, debug_text

    args = _parse_cli()
    if args.calibrate:
        rc = trigger_dcd_save()
        sys.exit(0 if rc == 0 else 1)

    if not parser.connect():
        sys.exit(1)

    fig = plt.figure(figsize=(18, 9.5), facecolor='#F8FAFC')
    gs = fig.add_gridspec(1, 2, width_ratios=[3.0, 2.1], wspace=0.05)
    ax = fig.add_subplot(gs[0, 0])
    ax_dbg = fig.add_subplot(gs[0, 1])
    fig.canvas.manager.set_window_title('PolyCast — EKF Live Position (Async)')
    fig.canvas.mpl_connect('key_press_event', on_key)
    ax.set_facecolor('white')
    ax_dbg.set_facecolor('#F8FAFC')
    ax_dbg.set_xticks([]); ax_dbg.set_yticks([])
    for spine in ax_dbg.spines.values():
        spine.set_visible(False)
    ax_dbg.set_xlim(0, 1); ax_dbg.set_ylim(0, 1)

    # Anchors (static)
    ax.scatter(engine.anchors[:, 0], engine.anchors[:, 1],
               c='red', marker='s', s=120, zorder=5)
    for i, a in enumerate(engine.anchors):
        ax.text(a[0], a[1] + 0.07, f'A{i}',
                color='red', fontsize=12, ha='center', fontweight='bold')

    # Artists
    line_draw, = ax.plot([], [], '-', color='royalblue', lw=2.0,
                         label='Pen tip XY (causal fused)', zorder=5)
    line_tag,  = ax.plot([], [], '--', color='purple', lw=1.5, alpha=0.82,
                         label='UWB tag XY', zorder=4)
    line_rts,  = ax.plot([], [], '-', color='limegreen', lw=2.0, alpha=0.90,
                         label='RTS smoothed pen tip (experimental)', zorder=6)
    scat_lift, = ax.plot([], [], '.', color='#888888', ms=4, alpha=0.5,
                         label='Lifted tip (tracked, not drawn)', zorder=3)
    scat_rej,  = ax.plot([], [], 'x', color='orange', ms=8,
                         markeredgewidth=1.8, alpha=0.7,
                         label='Anchor gate reject', zorder=4)
    dot_tip,   = ax.plot([], [], 'o', color='royalblue', ms=10, zorder=7)
    dot_tag,   = ax.plot([], [], 's', color='purple', ms=7, alpha=0.9, zorder=7)
    vel_arrow  = ax.quiver([], [], [], [], color='cyan', scale=1,
                           scale_units='xy', angles='xy',
                           width=0.003, zorder=8)

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
            'Blue=tip  Purple=tag  Green=RTS experimental  T=RTS  B=break-guard  V=velocity  C=clear  R=reset  D=debug',
            transform=ax.transAxes, fontsize=8, color='gray', va='bottom')

    debug_text = ax_dbg.text(0.02, 0.98, 'DEBUG ASSISTANT\nwaiting for data...',
                             transform=ax_dbg.transAxes, fontsize=6.8, color='#111827',
                             va='top', ha='left', family='monospace', clip_on=True,
                             bbox=dict(facecolor='white', alpha=0.96, edgecolor='#CBD5E1',
                                       boxstyle='round,pad=0.5'))
    debug_text.set_visible(DEBUG_ASSISTANT)
    line_rts.set_visible(_show_rts)

    artists = (line_draw, line_tag, line_rts, scat_lift, scat_rej, dot_tip, dot_tag, debug_text)

    ani = FuncAnimation(fig, update, interval=20, blit=False,
                        cache_frame_data=False)

    mode = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'[EKF] Running in {mode} mode — Async Stream')
    print(f'[EKF] Trail subsample: every {TRAIL_SUBSAMPLE} IMU predicts '
          f'(~{IMU_RATE_HZ/TRAIL_SUBSAMPLE:.0f} Hz display)')
    print(f'[EKF] Keys: T/B/V/C/R/D, Q to quit')

    plt.show()
    parser.close()


if __name__ == '__main__':
        main()
