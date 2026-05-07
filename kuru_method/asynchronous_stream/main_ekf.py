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
import csv
import json
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
from recognition_preprocessor import NormalizedStrokeExtractor
from trace_quality_diagnostics import TraceQualityDiagnostics


# -- Configuration ----------------------------------------------------------
from config import SERIAL_PORT, BAUD_RATE, UWB_OFFSETS, TIP_OFFSET_FROM_TAG_M
DATASET_FILENAME = 'datasets_whiteboard/abcde1_1.csv'   # '' = live; 'path/to/data.csv' = playback
MAX_TRAIL        = 2000  # maximum position samples in the drawing trail
SHOW_VELOCITY    = True  # initial state; toggle with V key
VEL_SCALE        = 0.3   # arrow length multiplier
DIAG_INTERVAL_S  = 1.0   # console diagnostic print interval
DEBUG_ASSISTANT  = True  # show rule-based live debugging panel
SHOW_RTS         = False # start hidden; toggle with T. RTS is diagnostic/experimental.
# Manual offline NoteSmoother overlay. Press O after/during playback to rebuild
# the full-note RTS overlay from all EKF history available so far. This is
# display-only and does not feed back into the EKF/recognition pipeline.
NOTE_SMOOTHER_SPLINE = False
NOTE_SMOOTHER_RESAMPLE_POINTS = None

# Session-integrity guard. This does not change the EKF numerics. It only
# protects display/export/recognition buffers from being stitched across
# discontinuities such as a UWB outage, EKF runaway soft re-init, or a single
# impossible UWB-position jump. Toggle with I.
SESSION_INTEGRITY_GUARD = True
INTEGRITY_REINIT_JUMP_M = 0.20      # one UWB update jump >20 cm means break the note/export trail
INTEGRITY_OUTAGE_MARKERS = True     # insert a boundary when prolonged UWB outage begins
INTEGRITY_MARKER_SIZE = 10
# Visual-only relocation break guard. This does not change the EKF state; it only
# inserts NaN separators in the drawn/saved trail when FSR appears to stay
# pressed during a reposition. Toggle at runtime with the B key.
AUTO_BREAK_RELOCATION = True
RELOC_SEGMENT_M       = 0.045  # one display-sample jump > 4.5 cm -> break
RELOC_SPEED_MPS       = 0.80   # fast move while contact is still true
RELOC_SPEED_SEG_M     = 0.025  # and at least 2.5 cm since last drawn point
USE_ONLINE_TRAIL_SMOOTHER = False  # False = draw raw causal EKF tip, True = 5-point causal display smoother
SHOW_RECOGNITION_TRACE = True       # cyan recognition-ready post-processed stroke trace
SHOW_NORMALIZED_PREVIEW = True       # black normalized preview overlaid in the same board-space axes
NORMALIZED_PREVIEW_MODE = 'raw'       # 'raw' actual extracted path; also 'chars', 'cells', 'lines', 'full', or 'strokes'
RECOG_MIN_POINT_STEP_M = 0.0015     # decimate tiny sub-mm jitter in recognition trace
RECOG_SMOOTHING_MODE = 'light'      # 'raw', 'light', or 'normal'. Toggle with S.
NORMALIZED_CANVAS_SIZE = 64         # export/preview canvas size in pixels

# Tip-offset calibration is display/export-only.  It does not alter the EKF
# state or UWB update.  Use it to test whether the tip projection/heading is
# causing slant/shift before editing config.py permanently.
TIP_OFFSET_CAL_DELTA_M = 0.0
TIP_OFFSET_CAL_ANGLE_DEG = 0.0
TIP_OFFSET_CAL_STEP_M = 0.002       # 2 mm physical marker-length trim per keypress
TIP_OFFSET_CAL_ANGLE_STEP_DEG = 2.0 # 2 deg projected-axis trim per keypress
# Safety rails for display/export-only tip calibration.  These are physical
# tag-to-tip lengths, not the 2-D projected on-board offset.  The projected
# offset is usually only a few cm because the marker is mostly normal to the board.
TIP_OFFSET_CAL_MIN_M = max(0.0, float(TIP_OFFSET_FROM_TAG_M) - 0.050)
TIP_OFFSET_CAL_MAX_M = float(TIP_OFFSET_FROM_TAG_M) + 0.050
SHOW_TIP_VECTOR = True

# Stroke-gap markers show large same-line pen-up gaps in the board-space view.
SHOW_STROKE_GAPS = True
STROKE_GAP_MARKER_THRESHOLD_M = 0.08  # 8 cm same-line gap marker

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
note_smooth = NoteSmoother(spline=NOTE_SMOOTHER_SPLINE, n_resample=NOTE_SMOOTHER_RESAMPLE_POINTS)


class RecognitionTraceFilter:
    """Low-latency display/output smoother for handwriting recognition.

    This is deliberately separate from the EKF and from RTS. It does not change
    the fused state. It only creates a cleaner per-stroke path from the causal
    pen-tip samples so OCR/recognition can consume a less jittery trajectory.

    Behaviour:
      * strong smoothing for tiny UWB/IMU jitter,
      * lighter smoothing on larger intentional motion,
      * corner preservation when the stroke direction changes sharply,
      * small-step decimation so repeated near-identical points do not bloat
        recognition input.
    """

    def __init__(self, min_step_m: float = RECOG_MIN_POINT_STEP_M, mode: str = RECOG_SMOOTHING_MODE):
        self.min_step_m = float(min_step_m)
        self.mode = str(mode or 'light').lower()
        if self.mode not in ('raw', 'light', 'normal'):
            self.mode = 'light'
        self.reset()

    def reset(self) -> None:
        self._out = None
        self._last_raw = None
        self._last_vec = None
        self._last_emitted = None
        self.samples_in = 0
        self.samples_out = 0

    def set_mode(self, mode: str) -> None:
        mode = str(mode or 'light').lower()
        if mode not in ('raw', 'light', 'normal'):
            mode = 'light'
        if mode != self.mode:
            self.mode = mode
            self.reset()

    def cycle_mode(self) -> str:
        order = ['raw', 'light', 'normal']
        try:
            idx = order.index(self.mode)
        except ValueError:
            idx = 0
        self.set_mode(order[(idx + 1) % len(order)])
        return self.mode

    @staticmethod
    def _angle_cos(a: np.ndarray, b: np.ndarray) -> float:
        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na < 1e-9 or nb < 1e-9:
            return 1.0
        return float(np.dot(a, b) / (na * nb))

    def update(self, x: float, y: float):
        raw = np.array([float(x), float(y)], dtype=float)
        self.samples_in += 1

        if self._out is None:
            self._out = raw.copy()
            self._last_raw = raw.copy()
            self._last_emitted = raw.copy()
            self.samples_out += 1
            return float(raw[0]), float(raw[1])

        raw_step = raw - self._last_raw
        step_len = float(np.linalg.norm(raw_step))

        # Raw mode is the most unbiased export/display path: it only decimates
        # tiny duplicate samples but does not smooth or move the blue trace.
        if self.mode == 'raw':
            self._last_raw = raw.copy()
            if self._last_emitted is not None:
                emit_step = float(np.linalg.norm(raw - self._last_emitted))
                if emit_step < self.min_step_m:
                    return None
            self._out = raw.copy()
            if step_len > 1e-6:
                self._last_vec = raw_step.copy()
            self._last_emitted = raw.copy()
            self.samples_out += 1
            return float(raw[0]), float(raw[1])

        # Adaptive smoothing. Small steps are likely quantisation/jitter; large
        # steps are intentional pen motion and should be followed more closely.
        if self.mode == 'light':
            if step_len < 0.003:
                alpha = 0.50
            elif step_len < 0.010:
                alpha = 0.65
            elif step_len < 0.025:
                alpha = 0.78
            else:
                alpha = 0.90
        else:  # normal, previous behaviour
            if step_len < 0.003:
                alpha = 0.18
            elif step_len < 0.010:
                alpha = 0.32
            elif step_len < 0.025:
                alpha = 0.52
            else:
                alpha = 0.72

        # Preserve corners: if the raw direction turns sharply, raise alpha so
        # the filtered line does not round letter corners too much.
        if self._last_vec is not None and step_len > 0.004:
            c = self._angle_cos(self._last_vec, raw_step)
            if c < 0.35:       # roughly >70 degree turn
                alpha = max(alpha, 0.68)
            elif c < 0.0:      # reversal / cusp
                alpha = max(alpha, 0.80)

        out = alpha * raw + (1.0 - alpha) * self._out
        self._last_raw = raw.copy()
        if step_len > 1e-6:
            self._last_vec = raw_step.copy()
        self._out = out

        if self._last_emitted is not None:
            emit_step = float(np.linalg.norm(out - self._last_emitted))
            if emit_step < self.min_step_m:
                return None

        self._last_emitted = out.copy()
        self.samples_out += 1
        return float(out[0]), float(out[1])


recog_filter = RecognitionTraceFilter()
normalized_strokes = NormalizedStrokeExtractor(canvas_size=NORMALIZED_CANVAS_SIZE)
trace_quality = TraceQualityDiagnostics()
_last_recognition_export = {}


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
# rec_x/rec_y = recognition-ready post-processed pen-tip trace.
rec_x, rec_y         = [], []
# Lifted trail — positions where is_writing == False (faint pen-tip trace)
lift_x, lift_y       = [], []
# Rejected anchor scatter
rej_x, rej_y         = [], []
# RTS-smoothed per-stroke overlay. Kept separate from the causal EKF trace.
rts_x, rts_y         = [], []
# Session-integrity markers: red diamonds where the visual/export trail was
# forcibly broken because EKF continuity was unsafe.
integrity_x, integrity_y = [], []
integrity_labels: list[str] = []

_show_vel        = SHOW_VELOCITY
_show_rts        = SHOW_RTS
_show_recognition = SHOW_RECOGNITION_TRACE
_show_normalized_preview = SHOW_NORMALIZED_PREVIEW
_normalized_preview_mode = NORMALIZED_PREVIEW_MODE
_auto_break_relocation = AUTO_BREAK_RELOCATION
_tip_offset_cal_delta_m = TIP_OFFSET_CAL_DELTA_M
_tip_offset_cal_angle_deg = TIP_OFFSET_CAL_ANGLE_DEG
_show_stroke_gaps = SHOW_STROKE_GAPS
_session_integrity_guard = SESSION_INTEGRITY_GUARD
_session_outage_boundary_active = False
_last_session_integrity_event = 'none'
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
    """Convert smoothed EKF tag XY to displayed pen-tip XY.

    This is used by the offline NoteSmoother/RTS overlay. Keep it aligned with
    the live display/export calibration:
      * use the physical tag-to-tip length plus the keyboard trim,
      * use only the marker-axis XY projection (do not normalize it),
      * apply the optional keyboard angle trim.

    It is intentionally display-only; it never changes the EKF state.
    """
    tag_xy = np.asarray(tag_xy, dtype=float)
    if tag_xy.size == 0:
        return np.zeros((0, 2), dtype=float)
    tag_xy = tag_xy.reshape((-1, 2))

    axes = np.asarray(axes_wb, dtype=float)
    if axes.ndim == 1:
        axes = np.tile(axes.reshape(1, -1), (len(tag_xy), 1))
    if len(axes) < len(tag_xy):
        fallback = axes[-1] if len(axes) else np.array([0.0, 0.0, 1.0], dtype=float)
        axes = np.vstack([axes, np.tile(fallback, (len(tag_xy) - len(axes), 1))])

    tip_xy = tag_xy.copy()
    eff = _tip_offset_effective_m()
    for i in range(len(tag_xy)):
        e = np.asarray(axes[i], dtype=float).reshape(-1)
        if len(e) < 2 or not np.all(np.isfinite(e[:2])):
            continue
        e2 = np.asarray(e[:2], dtype=float)
        # Preserve projected magnitude. Only clip malformed vectors.
        n2 = float(np.linalg.norm(e2))
        if n2 > 1.0:
            e2 = e2 / n2
        if abs(float(_tip_offset_cal_angle_deg)) > 1e-9 and n2 > 1e-9:
            e2 = _rot2(e2, _tip_offset_cal_angle_deg)
        tip_xy[i, :] = tag_xy[i, :] - eff * e2
    return tip_xy


def _writing_history_index_slices(include_active: bool = True) -> list[list[int]]:
    """Return EKF-history index slices for each pen-down contact segment.

    The NoteSmoother needs EKF history records, not just plotted XY samples.
    This reconstructs completed strokes from the parallel writing/history index
    streams collected in the IMU branch. It preserves real pen lifts by keeping
    each contact segment separate.
    """
    history_len = len(engine.ekf._history)
    strokes: list[list[int]] = []
    cur: list[int] = []
    last_idx = None
    for w, idx in zip(_is_writing_stream, _history_idx_stream):
        valid_idx = False
        try:
            idx = int(idx)
            valid_idx = (0 <= idx < history_len)
        except Exception:
            idx = -1
            valid_idx = False
        # Integrity boundaries may be stored as invalid/negative false markers.
        # They must still terminate the current slice so offline NoteSmoother
        # never smooths across an EKF reset/outage boundary.
        if not valid_idx:
            if (not w) and cur:
                strokes.append(cur)
                cur = []
                last_idx = None
            continue
        if w:
            # Avoid duplicate consecutive indices if a backend reuses a history sample.
            if idx != last_idx:
                cur.append(idx)
                last_idx = idx
        else:
            if cur:
                strokes.append(cur)
                cur = []
            last_idx = None
    if include_active and cur:
        strokes.append(cur)
    return strokes


def _rebuild_offline_note_smoother_overlay(include_active: bool = True, show: bool = True) -> None:
    """Manual full-note offline smoother overlay.

    Press O to run this after CSV playback, or during live testing. It rebuilds
    the green overlay from all pen-down EKF history available so far. This uses
    the offline NoteSmoother after the points already exist, so it is suitable
    for saved-note quality inspection without adding live latency.
    """
    global _show_rts, _last_rts_stats
    history = engine.ekf._history
    idx_slices = _writing_history_index_slices(include_active=include_active)

    rts_x.clear()
    rts_y.clear()
    _last_rts_stats = {}

    total_points = 0
    kept_strokes = 0
    for idxs in idx_slices:
        if len(idxs) < 2:
            continue
        hist_slice = [history[i] for i in idxs]
        smoothed_tag = note_smooth.smooth_stroke(hist_slice)
        if smoothed_tag is None or len(smoothed_tag) == 0:
            continue
        axes = np.array([_safe_axis_for_history(i) for i in idxs[:len(smoothed_tag)]], dtype=float)
        smoothed_tip = _tag_xy_to_tip_xy(smoothed_tag, axes)
        for x, y in smoothed_tip:
            rts_x.append(float(x))
            rts_y.append(float(y))
        rts_x.append(float('nan'))
        rts_y.append(float('nan'))
        total_points += int(len(smoothed_tip))
        kept_strokes += 1

    _last_rts_stats = {
        'manual_offline': True,
        'strokes': int(kept_strokes),
        'points': int(total_points),
        'spline': bool(note_smooth.spline),
    }
    debugger.ingest_rts(_last_rts_stats)

    _show_rts = bool(show and kept_strokes > 0)
    try:
        line_rts.set_data(rts_x, rts_y)
        line_rts.set_visible(_show_rts)
        line_rts.axes.figure.canvas.draw_idle()
    except NameError:
        pass

    if kept_strokes:
        print(f"[NOTE] Offline NoteSmoother rebuilt: {kept_strokes} strokes, {total_points} points, "
              f"spline={'ON' if note_smooth.spline else 'OFF'}; overlay {'ON' if _show_rts else 'OFF'}")
    else:
        print("[NOTE] Offline NoteSmoother: no pen-down EKF history available yet.")



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
        self.integrity_events = 0
        self.last_integrity_event = 'none'

    def ingest_integrity_event(self, reason: str, pos=None) -> None:
        self.integrity_events += 1
        if pos is not None:
            try:
                p = np.asarray(pos, dtype=float).reshape(-1)
                self.last_integrity_event = f'#{self.integrity_events}: {reason} @({p[0]:.2f},{p[1]:.2f})'
            except Exception:
                self.last_integrity_event = f'#{self.integrity_events}: {reason}'
        else:
            self.last_integrity_event = f'#{self.integrity_events}: {reason}'

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
        if bool(diag.get('runaway_deferred', False)):
            w.append(f'Runaway re-init DEFERRED while writing: mismatch={float(diag.get("runaway_deferred_mismatch_m", 0.0))*100:.1f}cm; waiting for pen lift + stable UWB.')
        if self.integrity_events > 0:
            w.append(f'Session-integrity breaks inserted: {self.integrity_events}; last {self.last_integrity_event}.')
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
            f'uwb policy={diag.get("uwb_policy", "--")} R×={diag.get("uwb_r_scale", 1.0):.1f} gateχ²={diag.get("uwb_gate_chi2", 0.0):.2f}',
            f'write-UWB drift mode={"ON" if diag.get("writing_uwb_drift_mode", False) else "OFF"} penDownR×={diag.get("pen_down_uwb_r_scale", 1.0):.1f}',
            f'reinit guard={diag.get("runaway_reinit_policy", "--")} pending={int(bool(diag.get("runaway_deferred", False)))} stable={diag.get("runaway_deferred_stable", 0)}/{diag.get("runaway_deferred_required", 0)} mismatch={float(diag.get("runaway_deferred_mismatch_m", 0.0))*100:.1f}cm last={diag.get("runaway_last_event", "none")}',
            f'tipCal phys={_tip_offset_effective_m():.3f}m proj={_tip_offset_projected_m()*100:.1f}cm Δ={_tip_offset_cal_delta_m*1000:+.0f}mm angle={_tip_offset_cal_angle_deg:+.1f}°',
            f'recog trace={"ON" if _show_recognition else "OFF"} smooth={recog_filter.mode} in/out={recog_filter.samples_in}/{recog_filter.samples_out}',
            f'extract overlay={"ON" if _show_normalized_preview else "OFF"} mode={_normalized_preview_mode} gaps={"ON" if _show_stroke_gaps else "OFF"} {normalized_strokes.summary()}',
            trace_quality.quick_summary(),
            '',
            'CONTACT / FSR',
            f'force raw={self._fmt(self.last_force_raw,5,0)} ema={self._fmt(self.last_force_smooth,5,0)} pre={self.last_force_pre} deb={int(self.last_contact)}',
            f'strokes={self.stroke_count} duty={contact_duty:.1f}%  {self.last_stroke_summary}',
            f'motion-breaks={self.motion_breaks} auto={"ON" if _auto_break_relocation else "OFF"} last={self.last_motion_break}',
            f'integrity breaks={self.integrity_events} guard={"ON" if _session_integrity_guard else "OFF"} last={self.last_integrity_event}',
        ]
        if self.completed_strokes:
            recent = list(self.completed_strokes)[-4:]
            lines.append('recent strokes: ' + ' | '.join([
                f'#{s["id"]} {s["dur"]:.1f}s {s["w"]*100:.0f}x{s["h"]*100:.0f}cm j{s["max_jump"]*100:.1f}'
                for s in recent
            ]))
        if self.last_rts:
            if self.last_rts.get('manual_offline'):
                lines.append(f'NOTE SMOOTHER: strokes={self.last_rts.get("strokes", 0)} pts={self.last_rts.get("points", 0)} spline={int(bool(self.last_rts.get("spline", False)))}')
            else:
                label = 'RTS SUSPECT' if self.last_rts.get('max_delta_m', 0.0) > self.RTS_SUSPECT_DELTA_M else 'RTS'
                lines.append(f'{label}: mean={self.last_rts.get("mean_delta_m", 0)*100:.1f}cm max={self.last_rts.get("max_delta_m", 0)*100:.1f}cm')
        lines.extend(['', 'CURRENT UWB', 'A STAT q     in   use    kf / pred, innov, NIS'])
        lines.extend(self.current_anchor_rows or ['-- no current UWB update --'])
        lines.extend(['', 'LAST PEN-DOWN UWB', self.pen_down_summary, 'A STAT q     in   use    kf / pred, innov, NIS'])
        lines.extend(self.pen_down_anchor_rows or ['-- waiting for pen-down UWB --'])
        if _last_recognition_export:
            lines.extend(['', 'LAST RECOGNITION EXPORT', _last_recognition_export.get('json', '--'), _last_recognition_export.get('csv', '--')])
            if _last_recognition_export.get('quality_json'):
                lines.extend(['TRACE QUALITY', _last_recognition_export.get('quality_json', '--')])
            if _last_recognition_export.get('trace_paths_csv'):
                lines.extend(['TRACE PATHS', _last_recognition_export.get('trace_paths_csv', '--')])
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
    """End-of-CSV offline NoteSmoother overlay fallback.

    Manual rebuild is available with the O key. At EOF, only auto-build if the
    overlay has not already been populated by lift-edge smoothing.
    """
    if rts_x:
        return
    _rebuild_offline_note_smoother_overlay(include_active=True, show=_show_rts)





def _rot2(v: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate a 2D vector by angle_deg."""
    v = np.asarray(v, dtype=float).reshape(2)
    a = np.deg2rad(float(angle_deg))
    c = float(np.cos(a)); q = float(np.sin(a))
    return np.array([c * v[0] - q * v[1], q * v[0] + c * v[1]], dtype=float)


def _tip_offset_effective_m() -> float:
    """Physical tag-to-tip length used for display/export calibration."""
    return float(np.clip(float(TIP_OFFSET_FROM_TAG_M) + float(_tip_offset_cal_delta_m),
                         float(TIP_OFFSET_CAL_MIN_M), float(TIP_OFFSET_CAL_MAX_M)))


def _projected_tip_axis_2d() -> np.ndarray:
    """Return the marker axis projected into the whiteboard XY plane.

    Important: do NOT normalize this 2-D vector.  ``engine.marker_axis_wb`` is a
    3-D unit vector.  Its XY part is the projected direction/magnitude on the
    board.  Normalizing the XY part turns a few-cm projection into the full
    physical 21.5 cm marker length, which caused the huge arcs in the previous
    patch.
    """
    try:
        e3 = np.asarray(engine.marker_axis_wb, dtype=float).reshape(-1)
        if len(e3) < 2 or not np.all(np.isfinite(e3[:2])):
            return np.zeros(2, dtype=float)
        e2 = np.asarray(e3[:2], dtype=float)
        # ``engine.marker_axis_wb`` should already be unit length, but clip the
        # projected magnitude defensively in case an upstream bad quaternion leaks in.
        n2 = float(np.linalg.norm(e2))
        if n2 > 1.0:
            e2 = e2 / n2
        if abs(float(_tip_offset_cal_angle_deg)) > 1e-9 and n2 > 1e-9:
            e2 = _rot2(e2, _tip_offset_cal_angle_deg)
        return e2
    except Exception:
        return np.zeros(2, dtype=float)


def _calibrated_tip_from_tag(tag_pos) -> np.ndarray:
    """Return display/export pen tip from tag using live calibration offsets.

    This intentionally does not change the EKF state.  It is display/export only.
    The key fix versus the previous patch is preserving the 2-D projection
    magnitude of the marker axis instead of normalizing it.
    """
    tag = np.asarray(tag_pos, dtype=float)[:2]
    try:
        e2 = _projected_tip_axis_2d()
        if float(np.linalg.norm(e2)) < 1e-9:
            return np.asarray(engine.tip_position if engine.tip_position is not None else tag, dtype=float)[:2]
        return tag - _tip_offset_effective_m() * e2
    except Exception:
        return np.asarray(engine.tip_position if engine.tip_position is not None else tag, dtype=float)[:2]


def _tip_offset_projected_m() -> float:
    """Current XY projected tag-to-tip length in metres for debug display."""
    try:
        return float(_tip_offset_effective_m() * np.linalg.norm(_projected_tip_axis_2d()))
    except Exception:
        return float('nan')


def _nan_segments(xs, ys):
    """Yield NaN-separated segments from x/y lists."""
    seg = []
    for x, y in zip(list(xs), list(ys)):
        try:
            xf = float(x); yf = float(y)
        except Exception:
            continue
        if not (np.isfinite(xf) and np.isfinite(yf)):
            if seg:
                yield seg
                seg = []
            continue
        seg.append((xf, yf))
    if seg:
        yield seg


def _export_trace_paths(base_path: str) -> dict[str, str]:
    """Export blue raw, cyan recognition, and black extraction paths as-is."""
    base = os.fspath(base_path)
    csv_path = base + '_trace_paths.csv'
    json_path = base + '_trace_paths.json'
    # Prefer the NaN-segmented cyan display buffers.  If the user cycled
    # smoothing mode after playback, older versions cleared rec_x/rec_y even
    # though trace_quality still retained the recognition samples; recover from
    # that so trace_paths never exports an empty cyan trace while the diagnostics
    # have recognition points.
    if len(rec_x) and len(rec_y):
        cyan_x, cyan_y = list(rec_x), list(rec_y)
    else:
        pts = getattr(trace_quality, 'rec_points', [])
        cyan_x = [float(p[0]) for p in pts if len(p) >= 2]
        cyan_y = [float(p[1]) for p in pts if len(p) >= 2]
    trace_defs = [
        ('blue_raw_tip', list(draw_x), list(draw_y)),
        ('cyan_recognition', cyan_x, cyan_y),
        ('black_extraction_raw', *normalized_strokes.preview_overlay(include_active=False, mode='raw')),
    ]
    summary = {
        'created_unix_s': time.time(),
        'purpose': 'unbiased trace path export; no letter completion or character bias',
        'recognition_smoothing_mode': recog_filter.mode,
        'tip_offset_calibration': {
            'base_m': float(TIP_OFFSET_FROM_TAG_M),
            'delta_m': float(_tip_offset_cal_delta_m),
            'effective_m': float(_tip_offset_effective_m()),
            'projected_xy_m': float(_tip_offset_projected_m()),
            'projected_axis_norm': float(np.linalg.norm(_projected_tip_axis_2d())),
            'angle_deg': float(_tip_offset_cal_angle_deg),
        },
        'traces': {},
    }
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['trace_type', 'segment_id', 'point_index', 'x_m', 'y_m'])
        for name, xs, ys in trace_defs:
            seg_count = 0
            point_count = 0
            for seg_id, seg in enumerate(_nan_segments(xs, ys), start=1):
                seg_count += 1
                for i, (x, y) in enumerate(seg):
                    point_count += 1
                    w.writerow([name, seg_id, i, x, y])
            summary['traces'][name] = {'segments': seg_count, 'points': point_count}
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
    return {'trace_paths_csv': csv_path, 'trace_paths_json': json_path}


def _gap_overlay_points():
    """Return (link_x, link_y, marker_x, marker_y) for large same-line gaps."""
    if not _show_stroke_gaps or normalized_strokes.completed_count < 2:
        return [], [], [], []
    try:
        line_map = {}
        for g in normalized_strokes.line_groups():
            for sid in g.get('stroke_ids', []):
                line_map[int(sid)] = int(g.get('line_id', 0))
        strokes = sorted(normalized_strokes.strokes, key=lambda st: int(st.stroke_id))
        lx, ly, mx, my = [], [], [], []
        for a, b in zip(strokes[:-1], strokes[1:]):
            if line_map.get(int(a.stroke_id), 0) != line_map.get(int(b.stroke_id), 0):
                continue
            pa = np.asarray(a.resampled_points or a.raw_points, dtype=float)
            pb = np.asarray(b.resampled_points or b.raw_points, dtype=float)
            if len(pa) == 0 or len(pb) == 0:
                continue
            p0 = pa[-1, :2]; p1 = pb[0, :2]
            gap = float(np.linalg.norm(p1 - p0))
            if gap < STROKE_GAP_MARKER_THRESHOLD_M:
                continue
            mid = 0.5 * (p0 + p1)
            lx.extend([float(p0[0]), float(p1[0]), float('nan')])
            ly.extend([float(p0[1]), float(p1[1]), float('nan')])
            mx.append(float(mid[0])); my.append(float(mid[1]))
        return lx, ly, mx, my
    except Exception:
        return [], [], [], []

def _recognition_export_base_path() -> str:
    """Return a timestamped base path for normalized recognition exports."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(script_dir, 'datasets_str_50hz', 'recognition_exports')
    os.makedirs(out_dir, exist_ok=True)
    if parser.mode == 'csv' and getattr(parser, 'csv_path', None):
        try:
            src = os.path.splitext(os.path.basename(parser.csv_path))[0]
        except Exception:
            src = 'csv'
    else:
        src = 'live'
    return os.path.join(out_dir, f'{src}_recognition_{time.strftime("%Y%m%d_%H%M%S")}')


def _export_recognition_strokes(include_active: bool = False) -> None:
    """Export completed normalized strokes plus trace-quality diagnostics."""
    global _last_recognition_export
    try:
        if normalized_strokes.completed_count == 0 and not (include_active and normalized_strokes.active_count):
            print('[REC] No normalized strokes to export yet.')
            return
        base = _recognition_export_base_path()
        paths = normalized_strokes.export(base, include_active=include_active)
        qpaths = trace_quality.export(base, normalized_strokes)
        paths.update(qpaths)
        paths.update(_export_trace_paths(base))
        _last_recognition_export = dict(paths)
        print(f"[REC] Exported normalized strokes: {paths.get('json')} and {paths.get('csv')}")
        print(f"[TRACEQ] Exported trace-quality diagnostics: {paths.get('quality_json')} and {paths.get('quality_csv')}")
        print(f"[TRACE] Exported raw/smoothed trace paths: {paths.get('trace_paths_csv')}")
    except Exception as exc:
        print(f'[REC] Export failed: {exc}')



def _append_nan_breaks(include_rts: bool = True) -> None:
    """Insert visual/export separators in every path that is segment-based."""
    for xs, ys in ((draw_x, draw_y), (tag_draw_x, tag_draw_y), (rec_x, rec_y)):
        # Avoid growing repeated NaNs during a long outage.
        if xs and ys and not (np.isfinite(xs[-1]) and np.isfinite(ys[-1])):
            continue
        xs.append(float('nan'))
        ys.append(float('nan'))
    if include_rts:
        if not (rts_x and rts_y and not (np.isfinite(rts_x[-1]) and np.isfinite(rts_y[-1]))):
            rts_x.append(float('nan'))
            rts_y.append(float('nan'))


def _append_history_boundary_marker() -> None:
    """Break offline NoteSmoother history slices at a session-integrity event."""
    try:
        idx = len(engine.ekf._history) - 1
    except Exception:
        idx = -1
    _is_writing_stream.append(False)
    _history_idx_stream.append(idx if idx >= 0 else -1)


def _force_session_integrity_boundary(reason: str, ts=None, pos=None) -> None:
    """Protect live/export buffers from being stitched across an EKF discontinuity.

    This is deliberately visual/export-side only. It does not reset the EKF and
    does not edit any past samples. It closes the current recognition stroke,
    inserts NaN segment breaks, resets causal display smoothers, and stores a
    red diamond marker so the user can see exactly where a UWB outage or soft
    re-init contaminated the note.
    """
    global _prev_writing, _last_draw_tip, _last_draw_tag
    global _stroke_history_start, _stroke_draw_start, _last_session_integrity_event

    if not _session_integrity_guard:
        return

    # End active recognition segment so exports do not merge across the event.
    try:
        if getattr(normalized_strokes, 'active_count', 0):
            normalized_strokes.end_stroke(ts=ts, reason=f'integrity:{reason}')
    except Exception:
        try:
            normalized_strokes.force_finish_active(reason=f'integrity:{reason}')
        except Exception:
            pass

    _append_nan_breaks(include_rts=True)
    _append_history_boundary_marker()

    smoother.reset()
    recog_filter.reset()
    _last_draw_tip = None
    _last_draw_tag = None
    _stroke_history_start = None
    _stroke_draw_start = None
    # Force a clean rising edge if the physical FSR/contact is still down on
    # the next IMU packet.
    _prev_writing = False

    marker = pos
    if marker is None:
        marker = _last_tip_pos if _last_tip_pos is not None else _last_pos
    if marker is not None:
        try:
            p = np.asarray(marker, dtype=float).reshape(-1)
            if len(p) >= 2 and np.all(np.isfinite(p[:2])):
                integrity_x.append(float(p[0]))
                integrity_y.append(float(p[1]))
                integrity_labels.append(str(reason))
                _trim(integrity_x, 80)
                _trim(integrity_y, 80)
                if len(integrity_labels) > 80:
                    del integrity_labels[:len(integrity_labels) - 80]
        except Exception:
            pass

    _last_session_integrity_event = str(reason)
    try:
        debugger.ingest_integrity_event(str(reason), marker)
    except Exception:
        pass
    print(f'[INTEGRITY] Boundary inserted: {reason}')

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
    global _session_outage_boundary_active

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
                    normalized_strokes.force_finish_active(reason='csv_eof')
                    _run_rts_smoother()
                    if normalized_strokes.completed_count:
                        _export_recognition_strokes(include_active=False)
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
            draw_pt = _calibrated_tip_from_tag(pos)

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
                    recog_filter.reset()
                    normalized_strokes.begin_stroke(ts=pkt.get('ts'))

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
                        rec_x.append(float('nan'))
                        rec_y.append(float('nan'))
                        _last_draw_tip = None
                        _last_draw_tag = None
                        smoother.reset()
                        recog_filter.reset()
                        normalized_strokes.end_stroke(ts=pkt.get('ts'), reason=f'motion_break:{reason}')
                        normalized_strokes.begin_stroke(ts=pkt.get('ts'))
                        debugger.ingest_motion_break(seg_m, spd_mps, reason)
                    draw_x.append(sx)
                    draw_y.append(sy)
                    tag_draw_x.append(float(pos[0]))
                    tag_draw_y.append(float(pos[1]))
                    _last_draw_tip = np.array([sx, sy], dtype=float)
                    _last_draw_tag = np.asarray(pos[:2], dtype=float).copy()
                    trace_quality.add_tip_tag(_last_draw_tip, _last_draw_tag, ts=pkt.get('ts'))

                    normalized_strokes.add_point(float(sx), float(sy), ts=pkt.get('ts'))

                    # Recognition trace: post-process the same causal pen-tip
                    # sample used for the blue trace.  This is display/output
                    # only; it does not feed back into the EKF.  The previous
                    # patch created the line and toggle but forgot to feed this
                    # filter, so the debug counter stayed at in/out=0/0.
                    rec_pt = recog_filter.update(float(sx), float(sy))
                    if rec_pt is not None:
                        rx, ry = rec_pt
                        rec_x.append(rx)
                        rec_y.append(ry)
                        trace_quality.add_recognition_pair([sx, sy], [rx, ry], ts=pkt.get('ts'))
                    # In CSV-playback mode keep every stroke visible.
                    # Live mode still trims to bound memory.
                    if parser.mode != 'csv':
                        _trim(draw_x, MAX_TRAIL)
                        _trim(draw_y, MAX_TRAIL)
                        _trim(tag_draw_x, MAX_TRAIL)
                        _trim(tag_draw_y, MAX_TRAIL)
                        _trim(rec_x, MAX_TRAIL)
                        _trim(rec_y, MAX_TRAIL)
                else:
                    # NaN break on pen lift transition
                    if falling:
                        _snap_stroke_to_rts()
                        draw_x.append(float('nan'))
                        draw_y.append(float('nan'))
                        tag_draw_x.append(float('nan'))
                        tag_draw_y.append(float('nan'))
                        rec_x.append(float('nan'))
                        rec_y.append(float('nan'))
                        normalized_strokes.end_stroke(ts=pkt.get('ts'), reason='lift')
                        recog_filter.reset()
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
            try:
                pre_outage = int(engine.ekf.consecutive_outage)
            except Exception:
                pre_outage = 0

            pos, vel, accepted, rejected = engine.process_uwb(
                ekf_dists, weights, ts=pkt.get('ts'))
            debugger.ingest_uwb(raw, ekf_dists, weights, accepted, rejected,
                                pre_pos=pre_pos, pre_P=pre_P,
                                pre_tag_z=pre_tag_z, is_pen_down=was_pen_down)

            # Session-integrity guard: detect EKF continuity breaks that should
            # never be stitched into one handwriting stroke/note. This catches
            # UWB outage onset and soft re-init/runaway jumps without changing
            # the EKF itself.
            post_pos = engine.ekf.position if engine.ekf.initialized else None
            try:
                post_outage = int(engine.ekf.consecutive_outage)
            except Exception:
                post_outage = 0
            try:
                max_outage = int(engine.ekf.MAX_CONSEC_OUTAGE)
            except Exception:
                max_outage = 8

            if INTEGRITY_OUTAGE_MARKERS and post_outage >= max_outage and pre_outage < max_outage:
                _force_session_integrity_boundary(
                    f'UWB outage start ({post_outage} consecutive)',
                    ts=pkt.get('ts'), pos=_last_tip_pos if _last_tip_pos is not None else post_pos)
                _session_outage_boundary_active = True
            elif post_outage == 0:
                _session_outage_boundary_active = False

            if pre_pos is not None and post_pos is not None:
                try:
                    jump_m = float(np.linalg.norm(np.asarray(post_pos, dtype=float)[:2] -
                                                  np.asarray(pre_pos, dtype=float)[:2]))
                except Exception:
                    jump_m = 0.0
                if jump_m > INTEGRITY_REINIT_JUMP_M:
                    _force_session_integrity_boundary(
                        f'EKF discontinuity / soft re-init jump={jump_m*100:.1f}cm',
                        ts=pkt.get('ts'), pos=post_pos)

            if pos is not None:
                # Tip-aware drawing point (Item A) — see IMU branch above.
                _last_tag_pos = np.asarray(pos, dtype=float).copy()
                _last_tip_pos = np.asarray(_calibrated_tip_from_tag(pos), dtype=float).copy()
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
        line_rec.set_data(rec_x, rec_y)
        line_rec.set_visible(_show_recognition)
        line_rts.set_data(rts_x, rts_y)
        line_rts.set_visible(_show_rts)
        if _show_normalized_preview:
            nx, ny = normalized_strokes.preview_overlay(include_active=True, mode=_normalized_preview_mode)
            line_norm.set_data(nx, ny)
        line_norm.set_visible(_show_normalized_preview)
        gx, gy, gm_x, gm_y = _gap_overlay_points()
        line_gap_links.set_data(gx, gy)
        scat_gap.set_data(gm_x, gm_y)
        line_gap_links.set_visible(_show_stroke_gaps)
        scat_gap.set_visible(_show_stroke_gaps)
        scat_integrity.set_data(integrity_x, integrity_y)
        if SHOW_TIP_VECTOR and _last_tip_pos is not None and _last_tag_pos is not None:
            line_tipvec.set_data([_last_tag_pos[0], _last_tip_pos[0]], [_last_tag_pos[1], _last_tip_pos[1]])
        else:
            line_tipvec.set_data([], [])
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
    global _show_vel, _show_rts, _show_recognition, _show_normalized_preview, _normalized_preview_mode, _auto_break_relocation, _prev_writing, DEBUG_ASSISTANT, _last_draw_tip, _last_draw_tag, _tip_offset_cal_delta_m, _tip_offset_cal_angle_deg, _show_stroke_gaps, _session_integrity_guard
    if event.key == 'v':
        _show_vel = not _show_vel
        print(f"[UI] Velocity arrow: {'ON' if _show_vel else 'OFF'}")
    elif event.key == 't':
        _show_rts = not _show_rts
        try:
            line_rts.set_visible(_show_rts)
        except NameError:
            pass
        print(f"[UI] RTS/NoteSmoother overlay: {'ON' if _show_rts else 'OFF'}")
    elif event.key == 'o':
        _rebuild_offline_note_smoother_overlay(include_active=True, show=True)
    elif event.key == 'p':
        note_smooth.spline = not bool(note_smooth.spline)
        _rebuild_offline_note_smoother_overlay(include_active=True, show=True)
        print(f"[UI] Offline NoteSmoother spline: {'ON' if note_smooth.spline else 'OFF'}")
    elif event.key == 'y':
        _show_recognition = not _show_recognition
        try:
            line_rec.set_visible(_show_recognition)
        except NameError:
            pass
        print(f"[UI] Recognition trace: {'ON' if _show_recognition else 'OFF'}")
    elif event.key == 's':
        mode = recog_filter.cycle_mode()
        # Do not clear the existing cyan trace.  Clearing after CSV EOF made
        # trace_paths export an empty cyan_recognition even though the displayed
        # and diagnostic recognition samples existed.  The new mode applies to
        # future samples; press R/C and replay to regenerate the whole trace.
        print(f"[UI] Recognition smoothing mode: {mode.upper()} (applies to future samples; replay to regenerate existing trace)")
    elif event.key == ',':
        _tip_offset_cal_delta_m -= TIP_OFFSET_CAL_STEP_M
        eff = _tip_offset_effective_m()
        _tip_offset_cal_delta_m = eff - float(TIP_OFFSET_FROM_TAG_M)
        print(f"[UI] Tip offset physical length: {eff:.3f} m; projected XY={_tip_offset_projected_m()*100:.1f} cm (delta {_tip_offset_cal_delta_m*1000:+.0f} mm)")
    elif event.key == '.':
        _tip_offset_cal_delta_m += TIP_OFFSET_CAL_STEP_M
        eff = _tip_offset_effective_m()
        _tip_offset_cal_delta_m = eff - float(TIP_OFFSET_FROM_TAG_M)
        print(f"[UI] Tip offset physical length: {eff:.3f} m; projected XY={_tip_offset_projected_m()*100:.1f} cm (delta {_tip_offset_cal_delta_m*1000:+.0f} mm)")
    elif event.key == ';':
        _tip_offset_cal_angle_deg -= TIP_OFFSET_CAL_ANGLE_STEP_DEG
        print(f"[UI] Tip offset angle: {_tip_offset_cal_angle_deg:+.1f} deg")
    elif event.key in ("'", '"'):
        _tip_offset_cal_angle_deg += TIP_OFFSET_CAL_ANGLE_STEP_DEG
        print(f"[UI] Tip offset angle: {_tip_offset_cal_angle_deg:+.1f} deg")
    elif event.key == 'g':
        _show_stroke_gaps = not _show_stroke_gaps
        print(f"[UI] Stroke gap markers: {'ON' if _show_stroke_gaps else 'OFF'}")
    elif event.key == 'n':
        # Cycle overlay modes.  The black normalized preview is drawn in the
        # same board-space axes as the original writing, not in a separate
        # coordinate plane.  'chars' is anchored to actual writing; 'cells' shows the older equal-cell layout.
        modes = ['raw', 'chars', 'cells', 'lines', 'full', 'strokes']
        if not _show_normalized_preview:
            _show_normalized_preview = True
            _normalized_preview_mode = 'raw'
        else:
            try:
                idx = modes.index(_normalized_preview_mode)
            except ValueError:
                idx = 0
            if idx >= len(modes) - 1:
                _show_normalized_preview = False
            else:
                _normalized_preview_mode = modes[idx + 1]
        try:
            line_norm.set_visible(_show_normalized_preview)
        except NameError:
            pass
        print(f"[UI] Recognition/extraction overlay: {'ON' if _show_normalized_preview else 'OFF'} mode={_normalized_preview_mode}")
    elif event.key == 'e':
        _export_recognition_strokes(include_active=True)
    elif event.key == 'w':
        try:
            engine.ekf.writing_uwb_drift_mode = not engine.ekf.writing_uwb_drift_mode
            print(f"[UI] Writing UWB drift correction: {'ON' if engine.ekf.writing_uwb_drift_mode else 'OFF'} "
                  f"(pen-down R×={engine.ekf.pen_down_uwb_r_scale:.1f})")
        except Exception as exc:
            print(f"[UI] Could not toggle writing UWB drift mode: {exc}")
    elif event.key == '[':
        try:
            engine.ekf.pen_down_uwb_r_scale = max(1.0, engine.ekf.pen_down_uwb_r_scale / 1.5)
            print(f"[UI] Pen-down UWB R scale: {engine.ekf.pen_down_uwb_r_scale:.2f}")
        except Exception as exc:
            print(f"[UI] Could not reduce pen-down UWB R scale: {exc}")
    elif event.key == ']':
        try:
            engine.ekf.pen_down_uwb_r_scale = min(81.0, engine.ekf.pen_down_uwb_r_scale * 1.5)
            print(f"[UI] Pen-down UWB R scale: {engine.ekf.pen_down_uwb_r_scale:.2f}")
        except Exception as exc:
            print(f"[UI] Could not increase pen-down UWB R scale: {exc}")
    elif event.key == 'i':
        _session_integrity_guard = not _session_integrity_guard
        print(f"[UI] Session integrity guard: {'ON' if _session_integrity_guard else 'OFF'}")
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
        rec_x.clear(); rec_y.clear(); recog_filter.reset(); normalized_strokes.clear(); trace_quality.clear()
        rts_x.clear();  rts_y.clear()
        integrity_x.clear(); integrity_y.clear(); integrity_labels.clear()
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
        integrity_x.clear(); integrity_y.clear(); integrity_labels.clear()
        trace_quality.clear()
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
    global artists, line_draw, line_tag, line_rec, line_rts, line_norm, line_gap_links, scat_gap, scat_integrity, line_tipvec, scat_lift, scat_rej, dot_tip, dot_tag, vel_arrow, debug_text

    args = _parse_cli()
    if args.calibrate:
        rc = trigger_dcd_save()
        sys.exit(0 if rc == 0 else 1)

    if not parser.connect():
        sys.exit(1)

    fig = plt.figure(figsize=(18, 9.0), facecolor='#F8FAFC')
    gs = fig.add_gridspec(1, 2, width_ratios=[3.1, 2.1], wspace=0.05)
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
    line_rec,  = ax.plot([], [], '-', color='cyan', lw=3.2, alpha=0.98,
                         label='Recognition trace (post-processed)', zorder=9)
    line_rts,  = ax.plot([], [], '-', color='limegreen', lw=2.0, alpha=0.90,
                         label='Offline NoteSmoother / RTS overlay', zorder=6)
    line_norm, = ax.plot([], [], '-', color='black', lw=2.2, alpha=0.88,
                         label='Recognition/extraction overlay', zorder=10)
    line_gap_links, = ax.plot([], [], '--', color='red', lw=1.3, alpha=0.55,
                              label='Large same-line stroke gap', zorder=11)
    scat_gap, = ax.plot([], [], 'o', color='red', ms=7, alpha=0.85, zorder=12)
    scat_integrity, = ax.plot([], [], 'D', color='crimson', ms=INTEGRITY_MARKER_SIZE, alpha=0.95,
                              markeredgecolor='white', markeredgewidth=0.8,
                              label='EKF reset / outage boundary', zorder=13)
    line_tipvec, = ax.plot([], [], '-', color='#F97316', lw=1.5, alpha=0.75,
                           label='Tip-tag vector', zorder=8)
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

    ax.set_xlim(-0.3, 2.20)
    ax.set_ylim(-0.3, 1.55)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X  (metres)')
    ax.set_ylabel('Y  (metres)')
    ax.set_title('PolyCast — UWB + IMU EKF Fusion (Async Stream)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Info text (bottom)
    ax.text(0.01, 0.01,
            "Blue=tip Purple=tag Cyan=recog Green=offline-note Black=extract Red=gap Orange=tip-vector O=note P=spline I=integrity Y=recog S=smooth N=overlay G=gaps E=export W=write-UWB [/]=Rscale ,/.=tipLen ;/'=tipAng T/B/V/C/R/D",
            transform=ax.transAxes, fontsize=8, color='gray', va='bottom')

    debug_text = ax_dbg.text(0.02, 0.98, 'DEBUG ASSISTANT\nwaiting for data...',
                             transform=ax_dbg.transAxes, fontsize=6.4, color='#111827',
                             va='top', ha='left', family='monospace', clip_on=True,
                             bbox=dict(facecolor='white', alpha=0.96, edgecolor='#CBD5E1',
                                       boxstyle='round,pad=0.5'))
    debug_text.set_visible(DEBUG_ASSISTANT)
    line_rts.set_visible(_show_rts)
    line_rec.set_visible(_show_recognition)
    line_norm.set_visible(_show_normalized_preview)
    line_gap_links.set_visible(_show_stroke_gaps)
    scat_gap.set_visible(_show_stroke_gaps)

    artists = (line_draw, line_tag, line_rec, line_rts, line_norm, line_gap_links, scat_gap, scat_integrity, line_tipvec, scat_lift, scat_rej, dot_tip, dot_tag, debug_text)

    ani = FuncAnimation(fig, update, interval=20, blit=False,
                        cache_frame_data=False)

    mode = 'CSV Playback' if parser.mode == 'csv' else 'Live Serial'
    print(f'[EKF] Running in {mode} mode — Async Stream')
    print(f'[EKF] Trail subsample: every {TRAIL_SUBSAMPLE} IMU predicts '
          f'(~{IMU_RATE_HZ/TRAIL_SUBSAMPLE:.0f} Hz display)')
    print("[EKF] Keys: O=offline-note P=spline Y/S/N/G/E/W/[ ]/, ./; \'/T/B/V/C/R/D, Q to quit")

    plt.show()
    parser.close()


if __name__ == '__main__':
        main()
