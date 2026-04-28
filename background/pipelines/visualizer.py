"""
visualizer.py — Live ESKF Pipeline Visualization & Debug Dashboard
Applies docs/bg_rules.md (Pipelines / Sensor Fusion).

Renderer: PyQtGraph (Qt-native, ~10× faster than matplotlib ion-mode).
Falls back gracefully: if PyQtGraph is unavailable the script prints an
error and exits instead of silently producing a broken window.

Four stroke layers:
    Black  (solid, z=5)  — fused stroke in CONTACT_DRAWING state (actual ink)
    Gray   (α=0.55, z=4) — fused position in non-contact / air movement
    Orange (α=0.55, z=3) — UWB clean positions (pos_clean from position.py)
    Blue   (α=0.55, z=2) — IMU dead-reckoning only (forward Euler integrator)

Outputs:
    • Live PyQtGraph canvas at ~20 Hz (Qt event loop, no blocking pause)
    • CSV written continuously (line-buffered) → visualizer_log.csv
    • PNG board snapshot on Ctrl+C                → visualizer_output.png

Usage:
    python -m background.pipelines.visualizer
    python -m background.pipelines.visualizer --fusion complementary
"""

import os
import sys
import time
import csv
import math
import signal
from collections import deque

os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

# ── PyQtGraph / Qt ────────────────────────────────────────────────────────────
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
except ImportError as _e:
    sys.exit(
        f"[ERROR] PyQtGraph not installed: {_e}\n"
        "Run:  python -m pip install pyqtgraph PyQt5"
    )

import numpy as np

# ── Pipeline imports ──────────────────────────────────────────────────────────
from background.pipelines.config import cfg
from background.pipelines.cleaner.unpacker              import SerialStreamer
from background.pipelines.cleaner.normalizer            import StreamNormalizer
from background.pipelines.cleaner.time_alignment        import TimeAlignLayer
from background.pipelines.preprocess.imu               import IMUPreprocessor
from background.pipelines.preprocess.contact           import ContactStateDetector
from background.pipelines.preprocess.uwb.range         import UWBRangePreprocessor
from background.pipelines.preprocess.uwb.trilateration import UWBSolver
from background.pipelines.preprocess.uwb.position      import UWBPositionFilter
from background.pipelines.fusion.eskf                  import ESKF
from background.pipelines.fusion.baseline              import FusionEngine
from background.pipelines.reconstruct                  import StrokeReconstructor


# ── Layout / rolling-buffer constants ────────────────────────────────────────
RENDER_HZ    = 20           # target refresh rate
POLL_MS      = 4            # serial-poll interval (QTimer)
CSV_FILENAME = 'visualizer_log.csv'
PNG_FILENAME = 'visualizer_output.png'

BW = cfg.anchors.board_size_x
BH = cfg.anchors.board_size_y

AIR_TRAIL = 3000
IMU_TRAIL = 3000
UWB_TRAIL = 800

_ANCHOR_XS = [p[0] for p in cfg.anchors.positions]
_ANCHOR_YS = [p[1] for p in cfg.anchors.positions]

# PyQtGraph colour helpers (R, G, B, A  0–255)
_C_INK    = (10,  10,  10,  255)
_C_AIR    = (130, 130, 130, 140)
_C_UWB    = (230, 140,  20, 140)
_C_IMU    = (80,  130, 220, 140)
_C_ANCHOR = (220,  30,  30, 255)
_C_BOARD  = (60,   60,  60, 200)

# Mode strip colours (R, G, B, A)
_MODE_COLORS = {
    'CONTACT_DRAWING': (50,  200,  80, 220),
    'DRAWING_FAST':    (240, 180,   0, 220),
    'AIR_MOVE':        (140, 140, 140, 160),
    'IDLE':            ( 80, 120, 220, 200),
}
_MODE_STRIP_LEN = 2000   # rolling samples shown in the strip


# ── IMU dead-reckoning integrator (blue layer) ────────────────────────────────
class _IMUTrack:
    """Forward Euler integrator — no UWB correction, shows raw drift."""

    __slots__ = ('p', 'v', '_ts')

    def __init__(self):
        self.p  = [BW / 2.0, BH / 2.0]
        self.v  = [0.0, 0.0]
        self._ts = None

    def update(self, ev: dict) -> tuple:
        ts = ev['ts_hw']
        if self._ts is None:
            self._ts = ts
            return (self.p[0], self.p[1])

        dt = (ts - self._ts) / 1_000_000.0
        self._ts = ts
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / cfg.imu.sample_rate_hz

        if ev.get('is_static', False):
            self.v[0] = self.v[1] = 0.0
        else:
            ax, ay = ev.get('acc_board', (0.0, 0.0))
            self.v[0] += ax * dt
            self.v[1] += ay * dt

        self.p[0] += self.v[0] * dt
        self.p[1] += self.v[1] * dt
        return (self.p[0], self.p[1])


# ── Debug text formatter ───────────────────────────────────────────────────────
def _format_debug(latest_fused, latest_uwb_fused, latest_imu, latest_uwb,
                  imu_count, uwb_count, closed_count) -> str:
    L = []
    D = '─' * 28

    L += ['══════════════════════════════',
          '  POLYCAST  PIPELINE DEBUG',
          '══════════════════════════════']

    # ESKF state (general — from latest event, IMU or UWB)
    L += ['', D, '  ESKF / FUSION']
    if latest_fused:
        e  = latest_fused.get('eskf', {})
        ba = e.get('b_a', (0.0, 0.0))
        turn_tag = ' [TURN]' if e.get('turn_flag') else ''
        ux = latest_fused.get('uwb_x') or 0.0
        uy = latest_fused.get('uwb_y') or 0.0
        L += [
            f"  Fused   X: {latest_fused['fused_x']:6.3f}  Y: {latest_fused['fused_y']:6.3f}",
            f"  UWB ref X: {ux:6.3f}  Y: {uy:6.3f}",
            f"  P trace  : {e.get('P_pos_trace', 0):.4f} m",
            f"  ω plane  : {e.get('omega_in_plane', 0):.3f} r/s{turn_tag}",
            f"  Bias b_a : ({ba[0]:+.4f}, {ba[1]:+.4f})",
        ]
    else:
        L.append('  (waiting for fusion data)')

    # UWB Kalman diagnostics — only valid on POSITION events, tracked separately
    L += ['', D, '  ESKF / UWB KALMAN']
    if latest_uwb_fused:
        e2 = latest_uwb_fused.get('eskf', {})
        acc_cnt = e2.get('uwb_accepted', 0)
        rej_cnt = e2.get('uwb_rejected', 0)
        total   = acc_cnt + rej_cnt
        rej_pct = 100.0 * rej_cnt / total if total else 0.0
        L += [
            f"  K_pos    : {e2.get('K_pos_diag', 0):.4f}  (target 0.10–0.65)",
            f"  |innov|  : {e2.get('innovation_norm', 0):.4f} m",
            f"  R scale  : {e2.get('r_scale', 1.0):.2f}",
            f"  RMS err  : {e2.get('uwb_residual_rms', 0):.4f} m",
            f"  b_a norm : {e2.get('b_a_norm', 0):.4f} m/s²",
            f"  Accepted : {acc_cnt}  Rejected: {rej_cnt} ({rej_pct:.1f}%)",
        ]
    else:
        L.append('  (no UWB correction yet)')

    # Stroke / IMU state
    L += ['', D, '  STROKE & IMU']
    if latest_imu and latest_fused:
        active_tag = 'YES' if latest_fused.get('stroke_active') else 'NO '
        static_tag = 'YES' if latest_imu.get('is_static')       else 'NO '
        contct_tag = 'YES' if latest_imu.get('contact')         else 'NO '
        L += [
            f"  State   : {latest_fused.get('state', '?')}",
            f"  Stroke  : ID={latest_fused.get('stroke_id', 0):>3}  Active: {active_tag}",
            f"  Force   : {latest_imu.get('force', 0):.1f}",
            f"  Jerk    : {latest_imu.get('jerk', 0):.2f} m/s³",
            f"  Static  : {static_tag}   Contact: {contct_tag}",
        ]
    else:
        L.append('  (waiting for IMU data)')

    # UWB state
    L += ['', D, '  UWB']
    if latest_uwb:
        pc = latest_uwb.get('pos_clean', (0.0, 0.0))
        pr = latest_uwb.get('pos_raw',   (0.0, 0.0))
        se = latest_uwb.get('solve_error', 0.0)
        sf = 'YES' if latest_uwb.get('speed_flag') else 'NO '
        L += [
            f"  Clean   X: {pc[0]:6.3f}  Y: {pc[1]:6.3f}",
            f"  Raw     X: {pr[0]:6.3f}  Y: {pr[1]:6.3f}",
            f"  SolErr   : {se:.4f}",
            f"  SpeedFlg : {sf}",
        ]
    else:
        L.append('  (waiting for UWB data)')

    # Fusion mode panel
    L += ['', D, '  FUSION MODE']
    if latest_fused:
        mode = latest_fused.get('fusion_mode', '?')
        e3   = latest_fused.get('eskf', {})
        fc   = e3.get('frames_contact', 0)
        ff   = e3.get('frames_fast', 0)
        fa   = e3.get('frames_air', 0)
        fs   = e3.get('frames_static', 0)
        total = fc + ff + fa + fs
        def pct(n): return 100 * n / total if total else 0
        avg_kc = e3.get('avg_K_contact', 0.0)
        avg_kf = e3.get('avg_K_fast', 0.0)
        avg_ka = e3.get('avg_K_air', 0.0)
        arm    = e3.get('fast_arm_count', 0)
        burst  = e3.get('fast_burst_count', 0)
        fast_f   = cfg.fusion_eskf.drawing_fast_min_frames
        snaps    = e3.get('stroke_start_snaps', 0)
        b_p_mag   = e3.get('b_p_mag', 0.0)
        b_p_xy    = e3.get('b_p', (0.0, 0.0))
        is_fast   = e3.get('drawing_fast', False)
        fast_tag  = ' ★FAST' if is_fast else ''
        age_s     = e3.get('stroke_age_s', 0.0)
        age_mult  = e3.get('age_cap_mult', 1.0)
        base_cap  = (cfg.fusion_eskf.modes.drawing_fast if is_fast else cfg.fusion_eskf.modes.drawing).pos_gain_cap
        kcap_eff  = min(base_cap * age_mult, 1.0)
        L += [
            f"  Mode     : {mode}{fast_tag}",
            f"  F_draw   : {fc:4d} ({pct(fc):3.0f}%)  K̄={avg_kc:.4f}",
            f"  F_fast   : {ff:4d} ({pct(ff):3.0f}%)  K̄={avg_kf:.4f}",
            f"  F_air    : {fa:4d} ({pct(fa):3.0f}%)  K̄={avg_ka:.4f}",
            f"  F_static : {fs:4d} ({pct(fs):3.0f}%)",
            f"  FastArm  : {arm}/{fast_f}  Burst: {burst}",
            f"  SnapCount: {snaps}",
            f"  StrokeAge: {age_s:.2f} s",
            f"  AgeMult  : {age_mult:.3f}x",
            f"  KcapEff  : {kcap_eff:.4f}",
            f"  b_p      : ({b_p_xy[0]:+.4f}, {b_p_xy[1]:+.4f})  |{b_p_mag*1000:.1f} mm|",
        ]
    else:
        L.append('  (waiting for fusion data)')

    # IMU Curve Diagnostics panel
    L += ['', D, '  IMU CURVE']
    if latest_imu:
        acc_hp_tip  = latest_imu.get('acc_board_hp_tip', (0.0, 0.0))
        acc_tip     = latest_imu.get('acc_board_tip', (0.0, 0.0))
        omega       = latest_imu.get('omega_world', (0.0, 0.0, 0.0))
        alpha       = latest_imu.get('alpha_world', (0.0, 0.0, 0.0))
        jerk        = latest_imu.get('jerk', 0.0)
        acc_hp_mag  = math.hypot(float(acc_hp_tip[0]), float(acc_hp_tip[1]))
        acc_tip_mag = math.hypot(float(acc_tip[0]),    float(acc_tip[1]))
        omega_mag   = math.sqrt(sum(float(x)**2 for x in omega))
        alpha_mag   = math.sqrt(sum(float(x)**2 for x in alpha))
        L += [
            f"  acc_hp_tip : {float(acc_hp_tip[0]):+.3f} {float(acc_hp_tip[1]):+.3f}  |{acc_hp_mag:.3f}| m/s²",
            f"  acc_tip    : {float(acc_tip[0]):+.3f} {float(acc_tip[1]):+.3f}  |{acc_tip_mag:.3f}| m/s²",
            f"  omega      : |{omega_mag:.3f}| rad/s",
            f"  alpha      : |{alpha_mag:.3f}| rad/s²",
            f"  jerk       : {jerk:.2f} m/s³",
        ]
    else:
        L.append('  (waiting for IMU data)')

    # Session counts
    L += [
        '', D, '  SESSION',
        f"  IMU events  : {imu_count}",
        f"  UWB events  : {uwb_count}",
        f"  Closed strk : {closed_count}",
    ]

    return '\n'.join(L)


def _imu_curve_csv(imu_ev: dict) -> list:
    """Return [acc_hp_tip_x, acc_hp_tip_y, acc_hp_tip_mag, omega_mag, alpha_mag] as formatted strings."""
    hp  = imu_ev.get('acc_board_hp_tip', (0.0, 0.0))
    om  = imu_ev.get('omega_world', (0.0, 0.0, 0.0))
    al  = imu_ev.get('alpha_world', (0.0, 0.0, 0.0))
    hx, hy = float(hp[0]), float(hp[1])
    hp_mag = math.hypot(hx, hy)
    om_mag = math.sqrt(sum(float(x)**2 for x in om))
    al_mag = math.sqrt(sum(float(x)**2 for x in al))
    return [
        f'{hx:.5f}', f'{hy:.5f}', f'{hp_mag:.5f}',
        f'{om_mag:.5f}', f'{al_mag:.5f}',
    ]


# ── PyQtGraph application window ──────────────────────────────────────────────
class VisualizerWindow(QtWidgets.QMainWindow):

    def __init__(self, fusion_mode: str):
        super().__init__()
        self.fusion_mode = fusion_mode
        self.setWindowTitle(
            f'PolyCast Visualizer  ·  fusion={fusion_mode}  ·  {cfg.serial.port}'
        )
        self.resize(1280, 700)

        # ── Pipeline ─────────────────────────────────────────────────────────
        self.streamer   = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
        self.norm       = StreamNormalizer()
        self.aligner    = TimeAlignLayer(buffer_size=500)
        self.imu_prep   = IMUPreprocessor()
        self.contact    = ContactStateDetector()
        self.range_prep = UWBRangePreprocessor(offsets=cfg.uwb.range_offsets_m)
        self.trilat     = UWBSolver()
        self.pos_filt   = UWBPositionFilter()
        self.rec        = StrokeReconstructor()
        self.imu_trk    = _IMUTrack()
        self.fusion     = ESKF() if fusion_mode == 'eskf' else FusionEngine(fusion_alpha=0.15)

        # ── Data accumulators ─────────────────────────────────────────────────
        self.air_x = deque(maxlen=AIR_TRAIL); self.air_y = deque(maxlen=AIR_TRAIL)
        self.imu_x = deque(maxlen=IMU_TRAIL); self.imu_y = deque(maxlen=IMU_TRAIL)
        self.uwb_x = deque(maxlen=UWB_TRAIL); self.uwb_y = deque(maxlen=UWB_TRAIL)

        self.ink_strokes   = []   # list of completed np arrays [(x,y), ...]
        self.cur_ink_x     = []
        self.cur_ink_y     = []

        self.latest_fused     = None
        self.latest_uwb_fused = None   # last fused event produced by a UWB correction
        self.latest_imu_ev = None
        self.latest_uwb_ev = None
        self.last_uwb_p    = (BW / 2.0, BH / 2.0)
        self.last_imu_p    = (BW / 2.0, BH / 2.0)
        self.imu_count     = 0
        self.uwb_count     = 0
        self.closed_count  = 0
        self._stopping     = False

        # Mode strip rolling colour buffer (one entry per IMU sample)
        self._strip_colors = deque(maxlen=_MODE_STRIP_LEN)

        # ── CSV ───────────────────────────────────────────────────────────────
        self._csvf   = open(CSV_FILENAME, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._csvf)
        self._writer.writerow([
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
            'stroke_age_s', 'age_cap_mult', 'kcap_eff',
            'acc_hp_tip_x', 'acc_hp_tip_y', 'acc_hp_tip_mag',
            'omega_mag', 'alpha_mag',
        ])

        # ── Qt layout ─────────────────────────────────────────────────────────
        pg.setConfigOptions(antialias=True, useOpenGL=False)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        hbox = QtWidgets.QHBoxLayout(central)
        hbox.setContentsMargins(6, 6, 6, 6)
        hbox.setSpacing(8)

        # Left column: board plot + mode strip (stacked vertically)
        left_widget = QtWidgets.QWidget()
        vbox_left = QtWidgets.QVBoxLayout(left_widget)
        vbox_left.setContentsMargins(0, 0, 0, 0)
        vbox_left.setSpacing(2)

        # Board plot
        self._plot_widget = pg.PlotWidget(title='Drawing Board — Live')
        self._plot_widget.setBackground('#fafafa')
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.setXRange(-0.06, BW + 0.06, padding=0)
        self._plot_widget.setYRange(-0.06, BH + 0.06, padding=0)
        self._plot_widget.showGrid(x=True, y=True, alpha=0.20)
        self._plot_widget.getAxis('bottom').setLabel('Board X (m)')
        self._plot_widget.getAxis('left').setLabel('Board Y (m)')
        vbox_left.addWidget(self._plot_widget, stretch=1)

        # Mode strip (time × mode colour, 38 px tall)
        self._strip_widget = pg.PlotWidget()
        self._strip_widget.setBackground('#1a1a1a')
        self._strip_widget.setFixedHeight(38)
        self._strip_widget.hideAxis('left')
        self._strip_widget.hideAxis('bottom')
        self._strip_widget.setMouseEnabled(x=False, y=False)
        self._strip_widget.setMenuEnabled(False)
        # Label inside the strip using a TextItem
        _strip_lbl = pg.TextItem(
            '  FUSION MODE STRIP  ·  green=CONTACT  yellow=FAST  gray=AIR',
            color=(200, 200, 200), anchor=(0, 0),
        )
        _strip_lbl.setPos(0, 0.5)
        self._strip_widget.addItem(_strip_lbl)
        self._strip_scatter = pg.ScatterPlotItem(size=6, pxMode=True)
        self._strip_widget.addItem(self._strip_scatter)
        self._strip_widget.setXRange(0, _MODE_STRIP_LEN, padding=0)
        self._strip_widget.setYRange(0, 1, padding=0)
        vbox_left.addWidget(self._strip_widget, stretch=0)

        hbox.addWidget(left_widget, stretch=3)

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
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(self._debug_label)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(280)
        scroll.setMaximumWidth(340)
        hbox.addWidget(scroll, stretch=1)

        # ── Static board elements (drawn once, never redrawn) ─────────────────
        vb = self._plot_widget.getPlotItem().getViewBox()

        # Board outline rectangle
        board_rect = QtWidgets.QGraphicsRectItem(0, 0, BW, BH)
        pen = pg.mkPen(color=_C_BOARD, width=2, style=QtCore.Qt.PenStyle.DashLine)
        board_rect.setPen(pen)
        board_rect.setBrush(pg.mkBrush(None))
        vb.addItem(board_rect)

        # Anchor markers
        anchor_plot = pg.ScatterPlotItem(
            x=_ANCHOR_XS, y=_ANCHOR_YS,
            symbol='s', size=12,
            pen=pg.mkPen(_C_ANCHOR, width=1),
            brush=pg.mkBrush(_C_ANCHOR),
        )
        self._plot_widget.addItem(anchor_plot)

        # ── Dynamic plot items (updated each frame) ───────────────────────────
        def _pen(c, w=1.5): return pg.mkPen(color=c, width=w)

        self._imu_curve  = self._plot_widget.plot([], [], pen=_pen(_C_IMU),  name='IMU dead-reck.')
        self._uwb_curve  = self._plot_widget.plot([], [], pen=_pen(_C_UWB),  name='UWB clean')
        self._air_curve  = self._plot_widget.plot([], [], pen=_pen(_C_AIR),  name='Air (fused)')
        self._cur_curve  = self._plot_widget.plot([], [], pen=_pen(_C_INK, 2.2), name='Current stroke')
        # Completed strokes stored as individual PlotDataItem objects
        self._ink_items  = []

        # Legend
        legend = self._plot_widget.addLegend(offset=(10, 10))
        legend.addItem(self._imu_curve,  'IMU dead-reck.')
        legend.addItem(self._uwb_curve,  'UWB clean')
        legend.addItem(self._air_curve,  'Air (fused)')
        legend.addItem(self._cur_curve,  'Open stroke')

        # ── QTimers ───────────────────────────────────────────────────────────
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self._poll_pipeline)
        self._poll_timer.start()

        self._render_timer = QtCore.QTimer(self)
        self._render_timer.setInterval(int(1000 / RENDER_HZ))
        self._render_timer.timeout.connect(self._refresh_display)
        self._render_timer.start()

        # dirty flag — only redraw if new data arrived
        self._dirty = False

        print('=' * 62)
        print(f'  [VISUALIZER]  {cfg.serial.port}  fusion={fusion_mode}')
        print(f'  CSV  → {CSV_FILENAME}')
        print(f'  PNG  → {PNG_FILENAME}  (on Ctrl+C / window close)')
        print('  Draw strokes. Close window or Ctrl+C to stop and export.')
        print('=' * 62)

    # ── Serial poll (called every POLL_MS) ────────────────────────────────────
    def _poll_pipeline(self):
        if self._stopping:
            return

        raw = self.streamer.read_new_packets()
        if not raw:
            return

        evs = self.norm.normalize(raw)
        self.aligner.add_events(evs)
        sorted_evs = self.aligner.get_all_sorted()
        self.aligner.clear()

        for ev in sorted_evs:

            if ev['sensor'] == 'IMU':
                p = self.imu_prep.process_one(ev)
                if not p:
                    continue

                self.last_imu_p = self.imu_trk.update(p)
                self.imu_x.append(self.last_imu_p[0])
                self.imu_y.append(self.last_imu_p[1])
                self.latest_imu_ev = p

                s     = self.contact.process_one(p)
                fused = self.fusion.process_event(s)
                if not fused:
                    continue

                self.imu_count   += 1
                self.latest_fused = fused
                self._dirty       = True

                # Mode strip — one entry per IMU sample (store mode name string)
                self._strip_colors.append(fused.get('fusion_mode', 'AIR_MOVE'))

                fx, fy = fused['fused_x'], fused['fused_y']
                if math.isfinite(fx) and math.isfinite(fy):
                    if fused.get('stroke_active', False):
                        self.cur_ink_x.append(fx)
                        self.cur_ink_y.append(fy)
                    else:
                        self.air_x.append(fx)
                        self.air_y.append(fy)

                closed = self.rec.process_event(fused)
                if closed:
                    pts = [(pt[0], pt[1]) for pt in closed['points']]
                    self._add_ink_stroke(pts)
                    self.cur_ink_x.clear()
                    self.cur_ink_y.clear()
                    self.closed_count += 1

                self._write_csv_row(fused, p, 'IMU')

            elif ev['sensor'] == 'UWB':
                for r in self.range_prep.feed([ev]):
                    rp = self.trilat.process_one(r)
                    if not rp:
                        continue
                    clean = self.pos_filt.process_one(rp)
                    if not clean:
                        continue

                    pc = clean.get('pos_clean', (BW / 2, BH / 2))
                    self.last_uwb_p = pc
                    self.uwb_x.append(pc[0])
                    self.uwb_y.append(pc[1])
                    self.latest_uwb_ev = clean

                    fused = self.fusion.process_event(clean)
                    if fused:
                        self.uwb_count        += 1
                        self.latest_fused      = fused
                        self.latest_uwb_fused  = fused
                        self._dirty            = True

    # ── Render refresh (called every 1000/RENDER_HZ ms) ───────────────────────
    def _refresh_display(self):
        if not self._dirty:
            return
        self._dirty = False

        # Update rolling curves
        if self.imu_x:
            self._imu_curve.setData(list(self.imu_x), list(self.imu_y))
        if self.uwb_x:
            self._uwb_curve.setData(list(self.uwb_x), list(self.uwb_y))
        if self.air_x:
            self._air_curve.setData(list(self.air_x), list(self.air_y))
        if self.cur_ink_x:
            self._cur_curve.setData(self.cur_ink_x, self.cur_ink_y)
        else:
            self._cur_curve.setData([], [])

        # Mode strip — render per-sample colour band
        if self._strip_colors:
            if not hasattr(self, '_cached_brushes'):
                # Pre-build QBrush objects once; reuse per-frame (3 possible modes)
                self._cached_brushes = {k: pg.mkBrush(*v) for k, v in _MODE_COLORS.items()}
                self._cached_brushes['_fallback'] = pg.mkBrush(100, 100, 100, 160)
            n = len(self._strip_colors)
            xs = list(range(n))
            ys = [0.5] * n
            fb = self._cached_brushes['_fallback']
            brushes = [self._cached_brushes.get(m, fb) for m in self._strip_colors]
            self._strip_scatter.setData(
                x=xs, y=ys,
                brush=brushes,
                pen=pg.mkPen(None),
            )
            self._strip_widget.setXRange(max(0, n - _MODE_STRIP_LEN), n, padding=0)

        # Debug panel
        txt = _format_debug(
            self.latest_fused, self.latest_uwb_fused,
            self.latest_imu_ev, self.latest_uwb_ev,
            self.imu_count, self.uwb_count, self.closed_count,
        )
        self._debug_label.setText(txt)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _add_ink_stroke(self, pts: list):
        if not pts:
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        item = self._plot_widget.plot(
            xs, ys,
            pen=pg.mkPen(color=_C_INK, width=2.2),
        )
        self._ink_items.append(item)
        self.ink_strokes.append(pts)

    def _write_csv_row(self, fused: dict, imu_ev: dict, source: str):
        e  = fused.get('eskf', {})
        ba = e.get('b_a', (0.0, 0.0))
        self._writer.writerow([
            fused['ts_hw'], source,
            f'{self.last_imu_p[0]:.4f}', f'{self.last_imu_p[1]:.4f}',
            f'{self.last_uwb_p[0]:.4f}', f'{self.last_uwb_p[1]:.4f}',
            f'{fused["fused_x"]:.4f}', f'{fused["fused_y"]:.4f}',
            fused.get('state', ''),
            fused.get('fusion_mode', ''),
            fused.get('stroke_id', 0),
            int(fused.get('stroke_active', False)),
            int(imu_ev.get('is_static', False)),
            int(imu_ev.get('contact', False)),
            int(fused.get('contact_raw', True)),
            int(
                bool(fused.get('stroke_active', False)) and
                bool(fused.get('contact_raw', True)) and
                int(fused.get('stroke_id', 0)) != 0
            ),
            f'{imu_ev.get("force", 0):.1f}',
            f'{imu_ev.get("jerk", 0):.4f}',
            f'{e.get("P_pos_trace", 0):.5f}',
            f'{e.get("innovation_norm", 0):.4f}',
            f'{e.get("r_scale", 1):.3f}',
            f'{e.get("K_pos_diag", 0):.5f}',
            f'{e.get("b_a_norm", 0):.5f}',
            f'{e.get("uwb_residual_rms", 0):.5f}',
            f'{e.get("omega_in_plane", 0):.4f}',
            int(e.get('turn_flag', False)),
            f'{ba[0]:.5f}',
            f'{ba[1]:.5f}',
            f'{e.get("b_p", (0.0, 0.0))[0]:.5f}',
            f'{e.get("b_p", (0.0, 0.0))[1]:.5f}',
            f'{e.get("b_p_mag", 0.0):.5f}',
            f'{e.get("stroke_age_s", 0.0):.3f}',
            f'{e.get("age_cap_mult", 1.0):.4f}',
            f'{min(e.get("age_cap_mult", 1.0) * (cfg.fusion_eskf.modes.drawing_fast if e.get("drawing_fast") else cfg.fusion_eskf.modes.drawing).pos_gain_cap, 1.0):.5f}',
            # IMU curve diagnostics
            *_imu_curve_csv(imu_ev),
        ])

    # ── Shutdown (window close or Ctrl+C) ─────────────────────────────────────
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
        tail = self.rec.flush()
        if tail:
            pts = [(pt[0], pt[1]) for pt in tail['points']]
            self._add_ink_stroke(pts)
            self.closed_count += 1

        self.streamer.close()
        self._csvf.close()

        # Final refresh for PNG
        self._dirty = True
        self._refresh_display()

        # Export PNG using pyqtgraph's exporter
        try:
            exporter = pg.exporters.ImageExporter(self._plot_widget.getPlotItem())
            exporter.parameters()['width'] = 1600
            exporter.export(PNG_FILENAME)
            print(f'[EXPORT] PNG  → {PNG_FILENAME}')
        except Exception as exc:
            print(f'[WARN]  PNG export failed: {exc}')

        print(f'[EXPORT] CSV  → {CSV_FILENAME}')
        print(f'  Strokes   : {self.closed_count}')
        print(f'  IMU evts  : {self.imu_count}')
        print(f'  UWB evts  : {self.uwb_count}')


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    fusion_mode = 'eskf'
    for i, arg in enumerate(sys.argv):
        if arg == '--fusion' and i + 1 < len(sys.argv):
            fusion_mode = sys.argv[i + 1]
        elif arg.startswith('--fusion='):
            fusion_mode = arg.split('=', 1)[1]

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    win = VisualizerWindow(fusion_mode)
    win.show()

    # Let Ctrl+C in the terminal trigger a clean shutdown
    signal.signal(
        signal.SIGINT,
        lambda *_: (win._shutdown(), app.quit()),
    )
    # Pulse the event loop so Python can check signals
    sig_timer = QtCore.QTimer()
    sig_timer.setInterval(200)
    sig_timer.timeout.connect(lambda: None)
    sig_timer.start()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
