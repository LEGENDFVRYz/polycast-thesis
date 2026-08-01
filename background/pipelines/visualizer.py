"""
Live ESKF pipeline visualization and debug dashboard.
Applies docs/bg_rules.md (Pipelines / Sensor Fusion).

Renderer: PyQtGraph (Qt-native, ~10x faster than matplotlib ion-mode).
Falls back gracefully: if PyQtGraph is unavailable the script prints an
error and exits instead of silently producing a broken window.

Four stroke layers:
    Black  (solid,  z=5) fused stroke in CONTACT_DRAWING state (actual ink)
    Gray   (a=0.55, z=4) fused position in non-contact / air movement
    Orange (a=0.55, z=3) UWB clean positions (pos_clean from position.py)
    Blue   (a=0.55, z=2) IMU dead-reckoning only (forward Euler integrator)

Outputs:
    Live PyQtGraph canvas at ~20 Hz (Qt event loop, no blocking pause)
    CSV written continuously (line-buffered) -> visualizer_log.csv
    PNG board snapshot on Ctrl+C             -> visualizer_output.png

Usage:
    python -m background.pipelines.visualizer
"""

import csv
import math
import os
import signal
import sys
import time
from collections import deque

os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

# PyQtGraph / Qt
try:
    import pyqtgraph as pg
    import pyqtgraph.exporters
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
except ImportError as import_error:
    sys.exit(
        f"[ERROR] PyQtGraph not installed: {import_error}\n"
        "Run:  python -m pip install pyqtgraph PyQt5"
    )

import numpy as np

# Pipeline imports
from background.pipelines.cleaner.normalizer import StreamNormalizer
from background.pipelines.cleaner.time_alignment import TimeAlignLayer
from background.pipelines.cleaner.unpacker import SerialStreamer
from background.pipelines.config import cfg
from background.pipelines.fusion.eskf import ESKF
from background.pipelines.module_output import ModuleRunOutput
from background.pipelines.preprocess.contact import ContactStateDetector
from background.pipelines.preprocess.imu import IMUPreprocessor
from background.pipelines.preprocess.uwb.position import UWBPositionFilter
from background.pipelines.preprocess.uwb.range import UWBRangePreprocessor
from background.pipelines.preprocess.uwb.trilateration import UWBSolver
from background.pipelines.reconstruct import StrokeReconstructor

# -----------------------------------------------------------------------------
# Layout / rolling-buffer constants
# -----------------------------------------------------------------------------
RENDER_HZ = 20           # target refresh rate
POLL_MS = 4              # serial-poll interval (QTimer)
CSV_FILENAME = 'visualizer_log.csv'
PNG_FILENAME = 'visualizer_output.png'

# Wall-clock ceiling on how long one _poll_pipeline() call may spend running
# events through the fusion/reconstruction/CSV path, checked between events
# rather than as a fixed event count: profiled per-event cost varies with
# what an event does (a plain IMU sample vs. one that closes a stroke and
# triggers a pyqtgraph plot add), so a count-based cap either blocks too long
# on expensive batches or under-uses idle capacity on cheap ones. Measured
# per-event cost on this pipeline is ~0.5 ms mean / ~3 ms p99, so 12 ms keeps
# a single tick's blocking time below where Windows starts treating the
# window as unresponsive, while still processing enough events per tick
# (~20+ at the measured mean cost) to drain a backlog well ahead of the
# ~260 Hz combined IMU+UWB arrival rate.
MAX_POLL_BLOCK_S = 0.012

# One output directory per launch, shared by the streaming CSV and the final
# PNG so both land in the same timestamped run folder. Created on first use
# rather than at import, so merely importing this module writes nothing.
_RUN_OUTPUT = None


def run_output() -> ModuleRunOutput:
    """Return this session's output directory, creating it on first call."""

    global _RUN_OUTPUT
    if _RUN_OUTPUT is None:
        _RUN_OUTPUT = ModuleRunOutput('visualizer')
    return _RUN_OUTPUT


BOARD_WIDTH = cfg.anchors.board_size_x
BOARD_HEIGHT = cfg.anchors.board_size_y

AIR_TRAIL = 3000
IMU_TRAIL = 3000
UWB_TRAIL = 800

# Maximum allowed displacement between consecutive ink points (metres).
# Jumps larger than this indicate a covariance reset or NLOS spike; the
# current polyline is broken and a fresh segment begins at the new point.
_MAX_INK_JUMP_M = 0.15

_ANCHOR_XS = [position[0] for position in cfg.anchors.positions]
_ANCHOR_YS = [position[1] for position in cfg.anchors.positions]

# PyQtGraph colour helpers (R, G, B, A  0-255)
_COLOR_INK = (10, 10, 10, 255)
_COLOR_AIR = (130, 130, 130, 140)
_COLOR_UWB = (230, 140, 20, 140)
_COLOR_IMU = (80, 130, 220, 140)
_COLOR_ANCHOR = (220, 30, 30, 255)
_COLOR_BOARD = (60, 60, 60, 200)

# Mode strip colours (R, G, B, A)
_MODE_COLORS = {
    'CONTACT_DRAWING': (50, 200, 80, 220),
    'DRAWING_FAST':    (240, 180, 0, 220),
    'AIR_MOVE':        (140, 140, 140, 160),
    'IDLE':            (80, 120, 220, 200),
}
_MODE_STRIP_LEN = 2000   # rolling samples shown in the strip


# -----------------------------------------------------------------------------
# IMU dead-reckoning integrator (blue layer)
# -----------------------------------------------------------------------------
class _IMUTrack:
    """
    Forward Euler integrator - no UWB correction, shows raw drift.

    On IMU preprocessor reset (process_one returns None due to a large dt gap),
    the track snaps back to board center so successive strokes start from a
    known reference and drift is visible relative to center rather than
    accumulating from an arbitrary off-screen position.
    """

    __slots__ = ('position', 'velocity', '_ts')

    def __init__(self):
        self.position = [BOARD_WIDTH / 2.0, BOARD_HEIGHT / 2.0]
        self.velocity = [0.0, 0.0]
        self._ts = None

    def reset(self) -> tuple:
        """Snap to board center - called when IMUPreprocessor.reset() fires."""

        self.position[0] = BOARD_WIDTH / 2.0
        self.position[1] = BOARD_HEIGHT / 2.0
        self.velocity[0] = 0.0
        self.velocity[1] = 0.0
        self._ts = None
        return (self.position[0], self.position[1])

    def update(self, event: dict) -> tuple:
        ts = event['ts_hw']
        if self._ts is None:
            self._ts = ts
            return (self.position[0], self.position[1])

        dt = (ts - self._ts) / 1_000_000.0
        self._ts = ts
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / cfg.imu.sample_rate_hz

        if event.get('is_static', False):
            self.velocity[0] = self.velocity[1] = 0.0
        else:
            # acc_board_tip: rigid-body corrected to pen nib, not sensor-end.
            acc_x, acc_y = event.get('acc_board_tip', event.get('acc_board', (0.0, 0.0)))
            self.velocity[0] += acc_x * dt
            self.velocity[1] += acc_y * dt

        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        return (self.position[0], self.position[1])


# -----------------------------------------------------------------------------
# Debug text formatter
# -----------------------------------------------------------------------------
def _format_debug(latest_fused, latest_uwb_fused, latest_imu_event, latest_uwb_event,
                   imu_count, uwb_count, closed_count) -> str:
    lines = []
    divider = '-' * 28

    lines += ['==============================',
              '  POLYCAST  PIPELINE DEBUG',
              '==============================']

    # ESKF state (general - from latest event, IMU or UWB)
    lines += ['', divider, '  ESKF / FUSION']
    if latest_fused:
        diagnostics = latest_fused.get('eskf', {})
        accel_bias = diagnostics.get('b_a', (0.0, 0.0))
        turn_tag = ' [TURN]' if diagnostics.get('turn_flag') else ''
        uwb_x = latest_fused.get('uwb_x') or 0.0
        uwb_y = latest_fused.get('uwb_y') or 0.0
        lines += [
            f"  Fused   X: {latest_fused['fused_x']:6.3f}  Y: {latest_fused['fused_y']:6.3f}",
            f"  UWB ref X: {uwb_x:6.3f}  Y: {uwb_y:6.3f}",
            f"  P trace  : {diagnostics.get('P_pos_trace', 0):.4f} m",
            f"  omega plane : {diagnostics.get('omega_in_plane', 0):.3f} r/s{turn_tag}",
            f"  Bias b_a : ({accel_bias[0]:+.4f}, {accel_bias[1]:+.4f})",
        ]
    else:
        lines.append('  (waiting for fusion data)')

    # UWB Kalman diagnostics - only valid on POSITION events, tracked separately
    lines += ['', divider, '  ESKF / UWB KALMAN']
    if latest_uwb_fused:
        uwb_diagnostics = latest_uwb_fused.get('eskf', {})
        accepted_count = uwb_diagnostics.get('uwb_accepted', 0)
        rejected_count = uwb_diagnostics.get('uwb_rejected', 0)
        total_count = accepted_count + rejected_count
        rejected_pct = 100.0 * rejected_count / total_count if total_count else 0.0
        lines += [
            f"  K_pos    : {uwb_diagnostics.get('K_pos_diag', 0):.4f}  (target 0.10-0.65)",
            f"  |innov|  : {uwb_diagnostics.get('innovation_norm', 0):.4f} m",
            f"  R scale  : {uwb_diagnostics.get('r_scale', 1.0):.2f}",
            f"  RMS err  : {uwb_diagnostics.get('uwb_residual_rms', 0):.4f} m",
            f"  b_a norm : {uwb_diagnostics.get('b_a_norm', 0):.4f} m/s^2",
            f"  Accepted : {accepted_count}  Rejected: {rejected_count} ({rejected_pct:.1f}%)",
        ]
    else:
        lines.append('  (no UWB correction yet)')

    # Stroke / IMU state
    lines += ['', divider, '  STROKE & IMU']
    if latest_imu_event and latest_fused:
        active_tag = 'YES' if latest_fused.get('stroke_active') else 'NO '
        static_tag = 'YES' if latest_imu_event.get('is_static') else 'NO '
        contact_tag = 'YES' if latest_imu_event.get('contact') else 'NO '
        lines += [
            f"  State   : {latest_fused.get('state', '?')}",
            f"  Stroke  : ID={latest_fused.get('stroke_id', 0):>3}  Active: {active_tag}",
            f"  Force   : {latest_imu_event.get('force', 0):.1f}",
            f"  Jerk    : {latest_imu_event.get('jerk', 0):.2f} m/s^3",
            f"  Static  : {static_tag}   Contact: {contact_tag}",
        ]
    else:
        lines.append('  (waiting for IMU data)')

    # UWB state
    lines += ['', divider, '  UWB']
    if latest_uwb_event:
        clean_position = latest_uwb_event.get('pos_clean', (0.0, 0.0))
        raw_position = latest_uwb_event.get('pos_raw', (0.0, 0.0))
        solve_error = latest_uwb_event.get('solve_error', 0.0)
        speed_flag = 'YES' if latest_uwb_event.get('speed_flag') else 'NO '
        lines += [
            f"  Clean   X: {clean_position[0]:6.3f}  Y: {clean_position[1]:6.3f}",
            f"  Raw     X: {raw_position[0]:6.3f}  Y: {raw_position[1]:6.3f}",
            f"  SolErr   : {solve_error:.4f}",
            f"  SpeedFlg : {speed_flag}",
        ]
    else:
        lines.append('  (waiting for UWB data)')

    # Fusion mode panel
    lines += ['', divider, '  FUSION MODE']
    if latest_fused:
        mode = latest_fused.get('fusion_mode', '?')
        mode_diagnostics = latest_fused.get('eskf', {})
        frames_contact = mode_diagnostics.get('frames_contact', 0)
        frames_fast = mode_diagnostics.get('frames_fast', 0)
        frames_air = mode_diagnostics.get('frames_air', 0)
        frames_static = mode_diagnostics.get('frames_static', 0)
        total_frames = frames_contact + frames_fast + frames_air + frames_static

        def frame_pct(count):
            return 100 * count / total_frames if total_frames else 0

        avg_gain_contact = mode_diagnostics.get('avg_K_contact', 0.0)
        avg_gain_fast = mode_diagnostics.get('avg_K_fast', 0.0)
        avg_gain_air = mode_diagnostics.get('avg_K_air', 0.0)
        fast_arm_count = mode_diagnostics.get('fast_arm_count', 0)
        fast_burst_count = mode_diagnostics.get('fast_burst_count', 0)
        fast_arm_frames = cfg.fusion_eskf.drawing_fast_min_frames
        stroke_start_snaps = mode_diagnostics.get('stroke_start_snaps', 0)
        bias_mag_m = mode_diagnostics.get('b_p_mag', 0.0)
        bias_xy = mode_diagnostics.get('b_p', (0.0, 0.0))
        velocity_pseudo_applied = mode_diagnostics.get('vel_pseudo_applied', False)
        velocity_pseudo_dx_pos = mode_diagnostics.get('vel_pseudo_dx_pos', (0.0, 0.0))
        velocity_pseudo_dx_vel = mode_diagnostics.get('vel_pseudo_dx_vel', (0.0, 0.0))
        guard_fired = mode_diagnostics.get('active_uwb_guard_fired', False)
        guard_correction_mm = mode_diagnostics.get('active_uwb_guard_correction', 0.0) * 1000.0
        lock_active = mode_diagnostics.get('tip_lock_active', False)
        lock_candidate = mode_diagnostics.get('tip_lock_candidate', False)
        lock_correction_mm = mode_diagnostics.get('tip_lock_correction', 0.0) * 1000.0
        lock_count = mode_diagnostics.get('tip_lock_count', 0)
        lock_reason = mode_diagnostics.get('tip_lock_reason', 'OFF')
        is_fast = mode_diagnostics.get('drawing_fast', False)
        fast_tag = ' *FAST' if is_fast else ''
        stroke_age_s = mode_diagnostics.get('stroke_age_s', 0.0)
        age_gain_mult = mode_diagnostics.get('age_cap_mult', 1.0)
        base_gain_cap = (
            cfg.fusion_eskf.modes.drawing_fast if is_fast else cfg.fusion_eskf.modes.drawing
        ).pos_gain_cap
        effective_gain_cap = min(base_gain_cap * age_gain_mult, 1.0)
        lines += [
            f"  Mode     : {mode}{fast_tag}",
            f"  F_draw   : {frames_contact:4d} ({frame_pct(frames_contact):3.0f}%)  Kavg={avg_gain_contact:.4f}",
            f"  F_fast   : {frames_fast:4d} ({frame_pct(frames_fast):3.0f}%)  Kavg={avg_gain_fast:.4f}",
            f"  F_air    : {frames_air:4d} ({frame_pct(frames_air):3.0f}%)  Kavg={avg_gain_air:.4f}",
            f"  F_static : {frames_static:4d} ({frame_pct(frames_static):3.0f}%)",
            f"  FastArm  : {fast_arm_count}/{fast_arm_frames}  Burst: {fast_burst_count}",
            f"  SnapCount: {stroke_start_snaps}",
            f"  StrokeAge: {stroke_age_s:.2f} s",
            f"  AgeMult  : {age_gain_mult:.3f}x",
            f"  KcapEff  : {effective_gain_cap:.4f}",
            f"  b_p      : ({bias_xy[0]:+.4f}, {bias_xy[1]:+.4f})  |{bias_mag_m*1000:.1f} mm|",
            f"  VelPseudo: {'ON' if velocity_pseudo_applied else 'OFF'}  "
            f"dP=({velocity_pseudo_dx_pos[0]*1000:+.1f},{velocity_pseudo_dx_pos[1]*1000:+.1f})mm  "
            f"dV=({velocity_pseudo_dx_vel[0]:+.2f},{velocity_pseudo_dx_vel[1]:+.2f})",
            f"  UWBGuard : {'ON' if guard_fired else 'OFF'}  corr={guard_correction_mm:.1f} mm",
            f"  TipLock  : {'ON' if lock_active else ('CAND' if lock_candidate else 'OFF')}  "
            f"n={lock_count}  corr={lock_correction_mm:.1f} mm  {lock_reason}",
        ]
    else:
        lines.append('  (waiting for fusion data)')

    # IMU curve diagnostics panel
    lines += ['', divider, '  IMU CURVE']
    if latest_imu_event:
        acc_hp_tip = latest_imu_event.get('acc_board_hp_tip', (0.0, 0.0))
        acc_tip = latest_imu_event.get('acc_board_tip', (0.0, 0.0))
        omega_world = latest_imu_event.get('omega_world', (0.0, 0.0, 0.0))
        alpha_world = latest_imu_event.get('alpha_world', (0.0, 0.0, 0.0))
        jerk = latest_imu_event.get('jerk', 0.0)
        acc_hp_mag = math.hypot(float(acc_hp_tip[0]), float(acc_hp_tip[1]))
        acc_tip_mag = math.hypot(float(acc_tip[0]), float(acc_tip[1]))
        omega_mag = math.sqrt(sum(float(component) ** 2 for component in omega_world))
        alpha_mag = math.sqrt(sum(float(component) ** 2 for component in alpha_world))
        lines += [
            f"  acc_hp_tip : {float(acc_hp_tip[0]):+.3f} {float(acc_hp_tip[1]):+.3f}  |{acc_hp_mag:.3f}| m/s^2",
            f"  acc_tip    : {float(acc_tip[0]):+.3f} {float(acc_tip[1]):+.3f}  |{acc_tip_mag:.3f}| m/s^2",
            f"  omega      : |{omega_mag:.3f}| rad/s",
            f"  alpha      : |{alpha_mag:.3f}| rad/s^2",
            f"  jerk       : {jerk:.2f} m/s^3",
        ]
    else:
        lines.append('  (waiting for IMU data)')

    # Session counts
    lines += [
        '', divider, '  SESSION',
        f"  IMU events  : {imu_count}",
        f"  UWB events  : {uwb_count}",
        f"  Closed strk : {closed_count}",
    ]

    return '\n'.join(lines)


def _format_postprocess_debug(postprocess: dict) -> str:
    """Compact postprocess block appended to the debug label."""

    divider = '-' * 28
    lines = ['', divider, '  POSTPROCESS']
    if not postprocess or not postprocess.get('applied'):
        reason = (postprocess.get('reason', '-') if postprocess else 'no stroke yet')
        lines.append(f"  PP: OFF  ({reason})")
        return '\n'.join(lines)

    centroid_align = postprocess.get('centroid_align', {})
    min_jerk = postprocess.get('min_jerk', {})

    align_applied = centroid_align.get('applied', False)
    if align_applied:
        correction_mm = centroid_align.get('delta_m', 0.0) * 1000.0
        rotation_deg = centroid_align.get('rotation_deg', 0.0)
        scale = centroid_align.get('scale', 1.0)
        uwb_used = centroid_align.get('uwb_used', centroid_align.get('uwb_count', 0))
        rotation_meta = centroid_align.get('rotation', {})
        rotation_method = rotation_meta.get('method', '-')
        rmse = rotation_meta.get('rmse_after_m')
        rmse_text = f"  rmse:{float(rmse)*1000:.1f}mm" if isinstance(rmse, (int, float)) else ""
        lines += [
            f"  PP: ON   pts:{postprocess.get('n_points','?')}  uwb:{uwb_used}",
            f"  align: {correction_mm:+.1f}mm  rot:{rotation_deg:+.1f}deg  sc:{scale:.3f}",
            f"  method: {rotation_method}{rmse_text}",
        ]
    else:
        lines += [
            f"  PP: ON  align: OFF ({centroid_align.get('reason','?')})",
        ]

    jerk_applied = min_jerk.get('applied', False)
    if jerk_applied:
        lines.append(
            f"  jerk: ON  wp:{min_jerk.get('waypoints','?')}  blend:{min_jerk.get('shape_blend',0):.2f}"
        )
    else:
        lines.append(f"  jerk: OFF ({min_jerk.get('reason','?')})")

    return '\n'.join(lines)


def _imu_curve_csv(imu_event: dict) -> list:
    """
    Return IMU curve diagnostic columns as formatted strings.

    Columns: acc_hp_tip_x/y/mag, acc_tip_x/y/mag, omega_mag, alpha_mag
    """

    acc_hp_tip = imu_event.get('acc_board_hp_tip', (0.0, 0.0))
    acc_tip = imu_event.get('acc_board_tip', (0.0, 0.0))
    omega_world = imu_event.get('omega_world', (0.0, 0.0, 0.0))
    alpha_world = imu_event.get('alpha_world', (0.0, 0.0, 0.0))
    hp_x, hp_y = float(acc_hp_tip[0]), float(acc_hp_tip[1])
    tip_x, tip_y = float(acc_tip[0]), float(acc_tip[1])
    hp_mag = math.hypot(hp_x, hp_y)
    tip_mag = math.hypot(tip_x, tip_y)
    omega_mag = math.sqrt(sum(float(component) ** 2 for component in omega_world))
    alpha_mag = math.sqrt(sum(float(component) ** 2 for component in alpha_world))
    return [
        f'{hp_x:.5f}', f'{hp_y:.5f}', f'{hp_mag:.5f}',
        f'{tip_x:.5f}', f'{tip_y:.5f}', f'{tip_mag:.5f}',
        f'{omega_mag:.5f}', f'{alpha_mag:.5f}',
    ]


# -----------------------------------------------------------------------------
# PyQtGraph application window
# -----------------------------------------------------------------------------
class VisualizerWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle(
            f'PolyCast Visualizer  -  fusion=eskf  -  {cfg.serial.port}'
        )
        self.resize(1280, 700)

        # Pipeline
        self.streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
        self.normalizer = StreamNormalizer()
        # Drained every poll, so this only has to absorb a GUI stall. At the
        # combined IMU+UWB rate 500 covered under two seconds, which a window
        # drag or a PNG export can exceed; 4000 buys about fifteen.
        self.aligner = TimeAlignLayer(buffer_size=4000)
        self.imu_prep = IMUPreprocessor()
        self.contact = ContactStateDetector()
        self.range_prep = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
        self.trilateration = UWBSolver()
        self.position_filter = UWBPositionFilter()
        self.reconstructor = StrokeReconstructor()
        self.imu_track = _IMUTrack()
        self.fusion = ESKF()

        # Data accumulators
        self.air_x = deque(maxlen=AIR_TRAIL)
        self.air_y = deque(maxlen=AIR_TRAIL)
        self.imu_x = deque(maxlen=IMU_TRAIL)
        self.imu_y = deque(maxlen=IMU_TRAIL)
        self.uwb_x = deque(maxlen=UWB_TRAIL)
        self.uwb_y = deque(maxlen=UWB_TRAIL)

        self.ink_strokes = []   # list of completed point lists [(x,y), ...]
        self.current_ink_x = []
        self.current_ink_y = []

        self.latest_fused = None
        self.latest_uwb_fused = None   # last fused event produced by a UWB correction
        self.latest_imu_event = None
        self.latest_uwb_event = None
        self.last_uwb_position = (BOARD_WIDTH / 2.0, BOARD_HEIGHT / 2.0)
        self.last_imu_position = (BOARD_WIDTH / 2.0, BOARD_HEIGHT / 2.0)
        self.imu_count = 0
        self.uwb_count = 0
        self.closed_count = 0
        self._prev_stroke_active = False
        self._stopping = False
        self._last_postprocess_meta: dict = {}

        # Events sorted off the aligner but not yet run through the fusion/
        # reconstruction/CSV path. _poll_pipeline() only processes up to
        # MAX_EVENTS_PER_POLL per tick; the remainder waits here so a backlog
        # drains across several ticks instead of one long call.
        self._pending_events = deque()

        # Mode strip rolling colour buffer (one entry per IMU sample)
        self._strip_colors = deque(maxlen=_MODE_STRIP_LEN)

        # CSV. Default-buffered rather than line-buffered: a flush syscall
        # per IMU sample (~200/s) was a meaningful share of the per-event
        # cost that let the poll backlog outgrow real time. Flushed
        # explicitly on the render tick and once more on shutdown instead.
        self._csv_file = open(run_output().path(CSV_FILENAME), 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'ts_hw', 'source',
            'imu_p_x', 'imu_p_y',
            'uwb_p_x', 'uwb_p_y',
            'fused_x', 'fused_y',
            'state', 'fusion_mode', 'stroke_id', 'stroke_active',
            'is_static', 'contact', 'contact_raw', 'ink_written', 'force', 'jerk',
            'P_pos_trace', 'innovation_norm', 'r_scale',
            'K_pos_diag', 'b_a_norm', 'uwb_residual_rms',
            'omega_in_plane', 'turn_flag', 'b_a_x', 'b_a_y',
            'b_p_x', 'b_p_y', 'b_p_mag',
            'p_nom_x', 'p_nom_y', 'uwb_tip_x', 'uwb_tip_y',
            'vel_pseudo_applied', 'vel_pseudo_dx_pos_x', 'vel_pseudo_dx_pos_y',
            'vel_pseudo_dx_vel_x', 'vel_pseudo_dx_vel_y',
            'active_uwb_guard_fired', 'active_uwb_guard_correction_m',
            'tip_lock_active', 'tip_lock_candidate', 'tip_lock_count',
            'tip_lock_correction_m', 'tip_lock_uwb_blend', 'tip_lock_reason',
            'stroke_age_s', 'age_cap_mult', 'kcap_eff',
            'acc_hp_tip_x', 'acc_hp_tip_y', 'acc_hp_tip_mag',
            'acc_tip_x', 'acc_tip_y', 'acc_tip_mag',
            'omega_mag', 'alpha_mag',
        ])

        # Qt layout
        pg.setConfigOptions(antialias=True, useOpenGL=False)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer_layout = QtWidgets.QHBoxLayout(central)
        outer_layout.setContentsMargins(6, 6, 6, 6)
        outer_layout.setSpacing(8)

        # Left column: board plot + mode strip (stacked vertically)
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(2)

        # Board plot
        self._plot_widget = pg.PlotWidget(title='Drawing Board - Live')
        self._plot_widget.setBackground('#fafafa')
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.setXRange(-0.06, BOARD_WIDTH + 0.06, padding=0)
        self._plot_widget.setYRange(-0.06, BOARD_HEIGHT + 0.06, padding=0)
        self._plot_widget.showGrid(x=True, y=True, alpha=0.20)
        self._plot_widget.getAxis('bottom').setLabel('Board X (m)')
        self._plot_widget.getAxis('left').setLabel('Board Y (m)')
        left_layout.addWidget(self._plot_widget, stretch=1)

        # Mode strip (time x mode colour, 38 px tall)
        self._strip_widget = pg.PlotWidget()
        self._strip_widget.setBackground('#1a1a1a')
        self._strip_widget.setFixedHeight(38)
        self._strip_widget.hideAxis('left')
        self._strip_widget.hideAxis('bottom')
        self._strip_widget.setMouseEnabled(x=False, y=False)
        self._strip_widget.setMenuEnabled(False)
        # Label inside the strip using a TextItem
        strip_label = pg.TextItem(
            '  FUSION MODE STRIP  -  green=CONTACT  yellow=FAST  gray=AIR',
            color=(200, 200, 200), anchor=(0, 0),
        )
        strip_label.setPos(0, 0.5)
        self._strip_widget.addItem(strip_label)
        self._strip_scatter = pg.ScatterPlotItem(size=6, pxMode=True)
        self._strip_widget.addItem(self._strip_scatter)
        self._strip_widget.setXRange(0, _MODE_STRIP_LEN, padding=0)
        self._strip_widget.setYRange(0, 1, padding=0)
        left_layout.addWidget(self._strip_widget, stretch=0)

        outer_layout.addWidget(left_widget, stretch=3)

        # Debug panel (right, 1/4 width)
        self._debug_label = QtWidgets.QLabel()
        self._debug_label.setFont(QtGui.QFont('Consolas', 8))
        self._debug_label.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft
        )
        self._debug_label.setStyleSheet(
            'background:#f0f0f0; border:1px solid #ccc; '
            'padding:6px; color:#111;'
        )
        self._debug_label.setWordWrap(False)
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidget(self._debug_label)
        scroll_area.setWidgetResizable(True)
        scroll_area.setMinimumWidth(280)
        scroll_area.setMaximumWidth(340)
        outer_layout.addWidget(scroll_area, stretch=1)

        # Static board elements (drawn once, never redrawn)
        view_box = self._plot_widget.getPlotItem().getViewBox()

        # Board outline rectangle
        board_rect = QtWidgets.QGraphicsRectItem(0, 0, BOARD_WIDTH, BOARD_HEIGHT)
        board_pen = pg.mkPen(color=_COLOR_BOARD, width=2, style=QtCore.Qt.PenStyle.DashLine)
        board_rect.setPen(board_pen)
        board_rect.setBrush(pg.mkBrush(None))
        view_box.addItem(board_rect)

        # Anchor markers
        anchor_plot = pg.ScatterPlotItem(
            x=_ANCHOR_XS, y=_ANCHOR_YS,
            symbol='s', size=12,
            pen=pg.mkPen(_COLOR_ANCHOR, width=1),
            brush=pg.mkBrush(_COLOR_ANCHOR),
        )
        self._plot_widget.addItem(anchor_plot)

        # Dynamic plot items (updated each frame)
        def _make_pen(color, width=1.5):
            return pg.mkPen(color=color, width=width)

        self._imu_curve = self._plot_widget.plot([], [], pen=_make_pen(_COLOR_IMU), name='IMU dead-reck.')
        self._uwb_curve = self._plot_widget.plot([], [], pen=_make_pen(_COLOR_UWB), name='UWB clean')
        self._air_curve = self._plot_widget.plot([], [], pen=_make_pen(_COLOR_AIR), name='Air (fused)')
        self._current_stroke_curve = self._plot_widget.plot(
            [], [], pen=_make_pen(_COLOR_INK, 2.2), name='Current stroke'
        )
        # Completed strokes stored as individual PlotDataItem objects
        self._ink_items = []

        # Legend
        legend = self._plot_widget.addLegend(offset=(10, 10))
        legend.addItem(self._imu_curve, 'IMU dead-reck.')
        legend.addItem(self._uwb_curve, 'UWB clean')
        legend.addItem(self._air_curve, 'Air (fused)')
        legend.addItem(self._current_stroke_curve, 'Open stroke')

        # QTimers
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self._poll_pipeline)
        self._poll_timer.start()

        self._render_timer = QtCore.QTimer(self)
        self._render_timer.setInterval(int(1000 / RENDER_HZ))
        self._render_timer.timeout.connect(self._refresh_display)
        self._render_timer.start()

        # Dirty flag - only redraw if new data arrived
        self._dirty = False

        print('=' * 62)
        print(f'  [VISUALIZER]  {cfg.serial.port}  fusion=eskf')
        print(f'  CSV  -> {run_output().path(CSV_FILENAME)}')
        print(f'  PNG  -> {PNG_FILENAME}  (on Ctrl+C / window close)')
        print('  Draw strokes. Close window or Ctrl+C to stop and export.')
        print('=' * 62)

    # -------------------------------------------------------------------------
    # Serial poll (called every POLL_MS)
    # -------------------------------------------------------------------------
    def _poll_pipeline(self):
        if self._stopping:
            return

        raw_packets = self.streamer.read_new_packets()
        if raw_packets:
            self.aligner.add_events(self.normalizer.normalize(raw_packets))
            self._pending_events.extend(self.aligner.get_all_sorted())
            self.aligner.clear()

        # Bounded by wall-clock time, not event count, so a backlog (a GUI
        # stall, a slow tick) drains across several ticks instead of one call
        # running long enough to starve the render timer and the Qt event
        # loop until the whole backlog clears - see MAX_POLL_BLOCK_S.
        deadline = time.perf_counter() + MAX_POLL_BLOCK_S
        while self._pending_events and time.perf_counter() < deadline:
            self._process_event(self._pending_events.popleft())

    def _process_event(self, event: dict) -> None:
        """Run one sorted IMU or UWB event through fusion/reconstruction/CSV."""

        if event['sensor'] == 'IMU':
            preprocessed = self.imu_prep.process_one(event)
            if not preprocessed:
                return

            self.latest_imu_event = preprocessed

            with_contact = self.contact.process_one(preprocessed)
            fused = self.fusion.process_event(with_contact)
            if not fused:
                return

            self.imu_count += 1
            self.latest_fused = fused
            self._dirty = True

            stroke_active_now = bool(fused.get('stroke_active', False))

            # Reset blue IMU track to board center on each new stroke start.
            # This makes each stroke's raw dead-reckoned shape start from
            # the same reference so drift is visible per-stroke, not
            # accumulated.
            if stroke_active_now and not self._prev_stroke_active:
                snap = self.imu_track.reset()
                # Insert NaN break so PyQtGraph does not draw a line from
                # the last stroke's endpoint to the new origin on reset.
                self.imu_x.append(float('nan'))
                self.imu_y.append(float('nan'))
                self.imu_x.append(snap[0])
                self.imu_y.append(snap[1])

            # Only integrate while the pen is confirmed writing - idle and
            # air-move frames are excluded so between-stroke drift does
            # not contaminate the blue layer.
            if stroke_active_now:
                self.last_imu_position = self.imu_track.update(preprocessed)
                self.imu_x.append(self.last_imu_position[0])
                self.imu_y.append(self.last_imu_position[1])

            self._prev_stroke_active = stroke_active_now

            # Mode strip - one entry per IMU sample (store mode name string)
            self._strip_colors.append(fused.get('fusion_mode', 'AIR_MOVE'))

            fused_x, fused_y = fused['fused_x'], fused['fused_y']
            if math.isfinite(fused_x) and math.isfinite(fused_y):
                if fused.get('stroke_active', False):
                    if self.current_ink_x:
                        jump = math.hypot(
                            fused_x - self.current_ink_x[-1], fused_y - self.current_ink_y[-1]
                        )
                        if jump > _MAX_INK_JUMP_M:
                            # Teleport guard: break the polyline so no line
                            # is drawn across the jump. NaN reads as a gap
                            # to PyQtGraph, which keeps the points already
                            # drawn on screen - clearing the list would
                            # erase the stroke still being written.
                            self.current_ink_x.append(float('nan'))
                            self.current_ink_y.append(float('nan'))
                    self.current_ink_x.append(fused_x)
                    self.current_ink_y.append(fused_y)
                else:
                    if self.air_x:
                        air_jump = math.hypot(fused_x - self.air_x[-1], fused_y - self.air_y[-1])
                        if air_jump > _MAX_INK_JUMP_M:
                            # Same teleport guard for the gray air trail.
                            self.air_x.append(float('nan'))
                            self.air_y.append(float('nan'))
                    self.air_x.append(fused_x)
                    self.air_y.append(fused_y)

            closed = self.reconstructor.process_event(fused)
            if closed:
                points = [(point[0], point[1]) for point in closed['points']]
                self._add_ink_stroke(points)
                self.current_ink_x.clear()
                self.current_ink_y.clear()
                self.closed_count += 1
                self._last_postprocess_meta = closed.get('postprocess', {})

            self._write_csv_row(fused, preprocessed, 'IMU')

        elif event['sensor'] == 'UWB':
            for ranged in self.range_prep.feed([event]):
                solved = self.trilateration.process_one(ranged)
                if not solved:
                    continue
                clean = self.position_filter.process_one(solved)
                if not clean:
                    continue

                clean_position = clean.get('pos_clean', (BOARD_WIDTH / 2, BOARD_HEIGHT / 2))
                self.last_uwb_position = clean_position
                self.uwb_x.append(clean_position[0])
                self.uwb_y.append(clean_position[1])
                self.latest_uwb_event = clean

                fused = self.fusion.process_event(clean)
                if fused:
                    self.uwb_count += 1
                    self.latest_fused = fused
                    self.latest_uwb_fused = fused
                    self._dirty = True

    # -------------------------------------------------------------------------
    # Render refresh (called every 1000/RENDER_HZ ms)
    # -------------------------------------------------------------------------
    def _refresh_display(self):
        if not self._dirty:
            return
        self._dirty = False

        # CSV rows accumulate in Python's file buffer between renders now
        # that the file isn't line-buffered; flush here so the file on disk
        # stays reasonably current without a syscall per row.
        self._csv_file.flush()

        # Update rolling curves
        if self.imu_x:
            self._imu_curve.setData(list(self.imu_x), list(self.imu_y))
        if self.uwb_x:
            self._uwb_curve.setData(list(self.uwb_x), list(self.uwb_y))
        if self.air_x:
            self._air_curve.setData(list(self.air_x), list(self.air_y))
        if self.current_ink_x:
            self._current_stroke_curve.setData(self.current_ink_x, self.current_ink_y)
        else:
            self._current_stroke_curve.setData([], [])

        # Mode strip - render per-sample colour band
        if self._strip_colors:
            if not hasattr(self, '_cached_brushes'):
                # Pre-build QBrush objects once; reuse per-frame (3 possible modes)
                self._cached_brushes = {
                    mode: pg.mkBrush(*color) for mode, color in _MODE_COLORS.items()
                }
                self._cached_brushes['_fallback'] = pg.mkBrush(100, 100, 100, 160)
            sample_count = len(self._strip_colors)
            xs = list(range(sample_count))
            ys = [0.5] * sample_count
            fallback_brush = self._cached_brushes['_fallback']
            brushes = [self._cached_brushes.get(mode, fallback_brush) for mode in self._strip_colors]
            self._strip_scatter.setData(
                x=xs, y=ys,
                brush=brushes,
                pen=pg.mkPen(None),
            )
            self._strip_widget.setXRange(max(0, sample_count - _MODE_STRIP_LEN), sample_count, padding=0)

        # Debug panel
        debug_text = _format_debug(
            self.latest_fused, self.latest_uwb_fused,
            self.latest_imu_event, self.latest_uwb_event,
            self.imu_count, self.uwb_count, self.closed_count,
        ) + _format_postprocess_debug(self._last_postprocess_meta)
        self._debug_label.setText(debug_text)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _add_ink_stroke(self, points: list):
        if not points:
            return
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        item = self._plot_widget.plot(
            xs, ys,
            pen=pg.mkPen(color=_COLOR_INK, width=2.2),
        )
        self._ink_items.append(item)
        self.ink_strokes.append(points)

    def _write_csv_row(self, fused: dict, imu_event: dict, source: str):
        diagnostics = fused.get('eskf', {})
        accel_bias = diagnostics.get('b_a', (0.0, 0.0))
        self._csv_writer.writerow([
            fused['ts_hw'], source,
            f'{self.last_imu_position[0]:.4f}', f'{self.last_imu_position[1]:.4f}',
            f'{self.last_uwb_position[0]:.4f}', f'{self.last_uwb_position[1]:.4f}',
            f'{fused["fused_x"]:.4f}', f'{fused["fused_y"]:.4f}',
            fused.get('state', ''),
            fused.get('fusion_mode', ''),
            fused.get('stroke_id', 0),
            int(fused.get('stroke_active', False)),
            int(imu_event.get('is_static', False)),
            int(imu_event.get('contact', False)),
            int(fused.get('contact_raw', True)),
            int(
                bool(fused.get('stroke_active', False)) and
                bool(fused.get('contact_raw', True)) and
                int(fused.get('stroke_id', 0)) != 0
            ),
            f'{imu_event.get("force", 0):.1f}',
            f'{imu_event.get("jerk", 0):.4f}',
            f'{diagnostics.get("P_pos_trace", 0):.5f}',
            f'{diagnostics.get("innovation_norm", 0):.4f}',
            f'{diagnostics.get("r_scale", 1):.3f}',
            f'{diagnostics.get("K_pos_diag", 0):.5f}',
            f'{diagnostics.get("b_a_norm", 0):.5f}',
            f'{diagnostics.get("uwb_residual_rms", 0):.5f}',
            f'{diagnostics.get("omega_in_plane", 0):.4f}',
            int(diagnostics.get('turn_flag', False)),
            f'{accel_bias[0]:.5f}',
            f'{accel_bias[1]:.5f}',
            f'{diagnostics.get("b_p", (0.0, 0.0))[0]:.5f}',
            f'{diagnostics.get("b_p", (0.0, 0.0))[1]:.5f}',
            f'{diagnostics.get("b_p_mag", 0.0):.5f}',
            f'{diagnostics.get("p_nominal", (0.0, 0.0))[0]:.5f}',
            f'{diagnostics.get("p_nominal", (0.0, 0.0))[1]:.5f}',
            f'{diagnostics.get("z_uwb_tip", (0.0, 0.0))[0]:.5f}',
            f'{diagnostics.get("z_uwb_tip", (0.0, 0.0))[1]:.5f}',
            int(diagnostics.get('vel_pseudo_applied', False)),
            f'{diagnostics.get("vel_pseudo_dx_pos", (0.0, 0.0))[0]:.5f}',
            f'{diagnostics.get("vel_pseudo_dx_pos", (0.0, 0.0))[1]:.5f}',
            f'{diagnostics.get("vel_pseudo_dx_vel", (0.0, 0.0))[0]:.5f}',
            f'{diagnostics.get("vel_pseudo_dx_vel", (0.0, 0.0))[1]:.5f}',
            int(diagnostics.get('active_uwb_guard_fired', False)),
            f'{diagnostics.get("active_uwb_guard_correction", 0.0):.5f}',
            int(diagnostics.get('tip_lock_active', False)),
            int(diagnostics.get('tip_lock_candidate', False)),
            int(diagnostics.get('tip_lock_count', 0)),
            f'{diagnostics.get("tip_lock_correction", 0.0):.5f}',
            f'{diagnostics.get("tip_lock_uwb_blend", 0.0):.3f}',
            diagnostics.get('tip_lock_reason', 'OFF'),
            f'{diagnostics.get("stroke_age_s", 0.0):.3f}',
            f'{diagnostics.get("age_cap_mult", 1.0):.4f}',
            f'{min(diagnostics.get("age_cap_mult", 1.0) * (cfg.fusion_eskf.modes.drawing_fast if diagnostics.get("drawing_fast") else cfg.fusion_eskf.modes.drawing).pos_gain_cap, 1.0):.5f}',
            # IMU curve diagnostics
            *_imu_curve_csv(imu_event),
        ])

    # -------------------------------------------------------------------------
    # Shutdown (window close or Ctrl+C)
    # -------------------------------------------------------------------------
    def closeEvent(self, event):
        self._shutdown()
        event.accept()

    def _shutdown(self):
        if self._stopping:
            return
        self._stopping = True

        self._poll_timer.stop()
        self._render_timer.stop()

        # Flush open stroke
        tail = self.reconstructor.flush()
        if tail:
            points = [(point[0], point[1]) for point in tail['points']]
            self._add_ink_stroke(points)
            self.closed_count += 1

        self.streamer.close()

        # Final refresh for PNG. Runs before closing the CSV file since it
        # flushes any rows still sitting in the file's write buffer.
        self._dirty = True
        self._refresh_display()
        self._csv_file.close()

        # Export PNG using pyqtgraph's exporter
        try:
            exporter = pg.exporters.ImageExporter(self._plot_widget.getPlotItem())
            exporter.parameters()['width'] = 1600
            exporter.export(run_output().path(PNG_FILENAME))
            run_output().record(PNG_FILENAME)
        except Exception as export_error:
            print(f'[WARN]  PNG export failed: {export_error}')

        run_output().record(CSV_FILENAME)
        run_output().finish()
        print(f'  Strokes   : {self.closed_count}')
        print(f'  IMU evts  : {self.imu_count}')
        print(f'  UWB evts  : {self.uwb_count}')


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
def main() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    window = VisualizerWindow()
    window.show()

    # Let Ctrl+C in the terminal trigger a clean shutdown
    signal.signal(
        signal.SIGINT,
        lambda *_: (window._shutdown(), app.quit()),
    )
    # Pulse the event loop so Python can check signals
    signal_timer = QtCore.QTimer()
    signal_timer.setInterval(200)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
