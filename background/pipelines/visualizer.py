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

# ── Odometry helper ───────────────────────────────────────────────────────────
def _extract_odom_frame(imu_ev: dict) -> np.ndarray:
    """Pack acc_board_tip + quat into the 6-feature vector expected by OdometryNet."""
    ax, ay = imu_ev.get('acc_board_tip', (0.0, 0.0))
    qx, qy, qz, qw = imu_ev.get('quat', (0.0, 0.0, 0.0, 1.0))
    return np.array([ax, ay, qw, qx, qy, qz], dtype=np.float32)


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
from background.pipelines.fusion.learned_odometry      import IMUOdometryBuffer, OdometryInferer, OdometryDataCollector
from background.pipelines.reconstruct                  import StrokeReconstructor
from background.pipelines.camera                       import MarkerCapDetector


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
_C_RAW    = (180, 100,  30,  90)   # faint amber overlay for pre-snap raw polyline
_C_AIR    = (130, 130, 130, 140)
_C_UWB    = (230, 140,  20, 140)
_C_IMU    = (80,  130, 220, 140)
_C_ODOM   = (50,  210,  90, 180)   # green — learned odometry (NN) overlay
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
                  imu_count, uwb_count, closed_count,
                  odom_buf_fill: float = 0.0,
                  odom_last_delta: tuple = (0.0, 0.0),
                  odom_ready: bool = False) -> str:
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
        p_nom = e.get('p_nominal', (0.0, 0.0))
        z_tip = e.get('z_uwb_tip', (0.0, 0.0))
        dxp = e.get('vel_pseudo_dx_pos', (0.0, 0.0))
        dxv = e.get('vel_pseudo_dx_vel', (0.0, 0.0))
        p_nom = e.get('p_nominal', (0.0, 0.0))
        z_tip = e.get('z_uwb_tip', (0.0, 0.0))
        dxp = e.get('vel_pseudo_dx_pos', (0.0, 0.0))
        dxv = e.get('vel_pseudo_dx_vel', (0.0, 0.0))
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
        vp_on     = e3.get('vel_pseudo_applied', False)
        vp_dxp    = e3.get('vel_pseudo_dx_pos', (0.0, 0.0))
        vp_dxv    = e3.get('vel_pseudo_dx_vel', (0.0, 0.0))
        guard_on  = e3.get('active_uwb_guard_fired', False)
        guard_mm  = e3.get('active_uwb_guard_correction', 0.0) * 1000.0
        lock_on   = e3.get('tip_lock_active', False)
        lock_cand = e3.get('tip_lock_candidate', False)
        lock_mm   = e3.get('tip_lock_correction', 0.0) * 1000.0
        lock_n    = e3.get('tip_lock_count', 0)
        lock_r    = e3.get('tip_lock_reason', 'OFF')
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
            f"  VelPseudo: {'ON' if vp_on else 'OFF'}  dP=({vp_dxp[0]*1000:+.1f},{vp_dxp[1]*1000:+.1f})mm  dV=({vp_dxv[0]:+.2f},{vp_dxv[1]:+.2f})",
            f"  UWBGuard : {'ON' if guard_on else 'OFF'}  corr={guard_mm:.1f} mm",
            f"  TipLock  : {'ON' if lock_on else ('CAND' if lock_cand else 'OFF')}  n={lock_n}  corr={lock_mm:.1f} mm  {lock_r}",
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

    # Odometry panel
    L += ['', D, '  ODOMETRY (NN)']
    status = 'READY' if odom_ready else 'NO MODEL'
    dx, dy = odom_last_delta
    L += [
        f"  Status  : {status}",
        f"  Buf fill: {odom_buf_fill * 100:.0f}%",
        f"  Last Δ  : ({dx*1000:+.1f}, {dy*1000:+.1f}) mm",
    ]

    # Session counts
    L += [
        '', D, '  SESSION',
        f"  IMU events  : {imu_count}",
        f"  UWB events  : {uwb_count}",
        f"  Closed strk : {closed_count}",
    ]

    return '\n'.join(L)


def _format_pp_debug(pp: dict) -> str:
    """Compact postprocess block appended to the debug label."""
    D = '─' * 28
    L = ['', D, '  POSTPROCESS']
    if not pp or not pp.get('applied'):
        reason = (pp.get('reason', '—') if pp else 'no stroke yet')
        L.append(f"  PP: OFF  ({reason})")
        return '\n'.join(L)

    ca = pp.get('centroid_align', {})
    mj = pp.get('min_jerk', {})

    align_on = ca.get('applied', False)
    if align_on:
        corr_mm  = ca.get('delta_m', 0.0) * 1000.0
        rot_deg  = ca.get('rotation_deg', 0.0)
        sc       = ca.get('scale', 1.0)
        uwb_used = ca.get('uwb_used', ca.get('uwb_count', 0))
        rot_meta = ca.get('rotation', {})
        rot_method = rot_meta.get('method', '—')
        rmse = rot_meta.get('rmse_after_m')
        rmse_txt = f"  rmse:{float(rmse)*1000:.1f}mm" if isinstance(rmse, (int, float)) else ""
        L += [
            f"  PP: ON   pts:{pp.get('n_points','?')}  uwb:{uwb_used}",
            f"  align: {corr_mm:+.1f}mm  rot:{rot_deg:+.1f}°  sc:{sc:.3f}",
            f"  method: {rot_method}{rmse_txt}",
        ]
    else:
        L += [
            f"  PP: ON  align: OFF ({ca.get('reason','?')})",
        ]

    jerk_on = mj.get('applied', False)
    if jerk_on:
        L.append(f"  jerk: ON  wp:{mj.get('waypoints','?')}  blend:{mj.get('shape_blend',0):.2f}")
    else:
        L.append(f"  jerk: OFF ({mj.get('reason','?')})")

    return '\n'.join(L)


def _imu_curve_csv(imu_ev: dict) -> list:
    """Return IMU curve diagnostic columns as formatted strings.

    Columns: acc_hp_tip_x/y/mag, acc_tip_x/y/mag, omega_mag, alpha_mag
    """
    hp  = imu_ev.get('acc_board_hp_tip', (0.0, 0.0))
    tip = imu_ev.get('acc_board_tip',    (0.0, 0.0))
    om  = imu_ev.get('omega_world', (0.0, 0.0, 0.0))
    al  = imu_ev.get('alpha_world', (0.0, 0.0, 0.0))
    hx, hy = float(hp[0]),  float(hp[1])
    tx, ty = float(tip[0]), float(tip[1])
    hp_mag  = math.hypot(hx, hy)
    tip_mag = math.hypot(tx, ty)
    om_mag  = math.sqrt(sum(float(x)**2 for x in om))
    al_mag  = math.sqrt(sum(float(x)**2 for x in al))
    return [
        f'{hx:.5f}', f'{hy:.5f}', f'{hp_mag:.5f}',
        f'{tx:.5f}', f'{ty:.5f}', f'{tip_mag:.5f}',
        f'{om_mag:.5f}', f'{al_mag:.5f}',
    ]


# ── PyQtGraph application window ──────────────────────────────────────────────
class VisualizerWindow(QtWidgets.QMainWindow):

    def __init__(self, fusion_mode: str, collect_mode: bool = False,
                 collect_path: str = "odometry_training_data.npz"):
        super().__init__()
        self.fusion_mode  = fusion_mode
        self.collect_mode = collect_mode
        self.collect_path = collect_path
        self.setWindowTitle(
            f'PolyCast Visualizer  ·  fusion={fusion_mode}'
            + ('  ·  [COLLECTING DATA]' if collect_mode else '')
            + f'  ·  {cfg.serial.port}'
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

        # ── Learned odometry (overlay only — does not affect ESKF state) ─────
        self._odom_buf   = IMUOdometryBuffer(cfg.odometry.window_size, cfg.odometry.stride)
        self._odom_infer = OdometryInferer()
        self._odom_pos   = [BW / 2.0, BH / 2.0]
        self._odom_last_delta = (0.0, 0.0)
        self._prev_stroke_active = False

        # ── Data collector (only active with --collect) ───────────────────────
        self._collector: "OdometryDataCollector | None" = (
            OdometryDataCollector(
                window_size=cfg.odometry.window_size,
                stride=cfg.odometry.window_size,
            ) if collect_mode else None
        )

        # ── Camera detector (only when enabled in config) ─────────────────
        self._cam_cap    = None
        self._cam_det: "MarkerCapDetector | None" = None
        self._cam_timer  = None
        self._latest_imu_quat = None
        if cfg.camera.enabled:
            try:
                import cv2
                self._cam_det = MarkerCapDetector()
                if self._cam_det.calibrated:
                    import time as _ct
                    self._cam_cap = cv2.VideoCapture(cfg.camera.camera_index)
                    _ct.sleep(2.0)   # Windows MSMF init delay
                    self._cam_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg.camera.frame_width)
                    self._cam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.camera.frame_height)
                    self._cam_timer = QtCore.QTimer(self)
                    self._cam_timer.setInterval(int(1000 / cfg.camera.fps))
                    self._cam_timer.timeout.connect(self._poll_camera)
                    self._cam_timer.start()
                    print(f"[Camera] Started on index {cfg.camera.camera_index}")
                else:
                    print("[Camera] Not calibrated — run calibrate_camera.py first.")
            except Exception as exc:
                print(f"[Camera] Failed to start: {exc}")

        # ── Data accumulators ─────────────────────────────────────────────────
        self.air_x  = deque(maxlen=AIR_TRAIL); self.air_y  = deque(maxlen=AIR_TRAIL)
        self.imu_x  = deque(maxlen=IMU_TRAIL); self.imu_y  = deque(maxlen=IMU_TRAIL)
        self.uwb_x  = deque(maxlen=UWB_TRAIL); self.uwb_y  = deque(maxlen=UWB_TRAIL)
        self.odom_x = deque(maxlen=3000);      self.odom_y = deque(maxlen=3000)

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
        self._last_pp_meta: dict = {}

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
            'odom_x', 'odom_y',
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

        self._imu_curve  = self._plot_widget.plot([], [], pen=_pen(_C_IMU),      name='IMU dead-reck.')
        self._uwb_curve  = self._plot_widget.plot([], [], pen=_pen(_C_UWB),      name='UWB clean')
        self._air_curve  = self._plot_widget.plot([], [], pen=_pen(_C_AIR),      name='Air (fused)')
        self._odom_curve = self._plot_widget.plot([], [], pen=_pen(_C_ODOM),     name='Odometry (NN)')
        self._cur_curve  = self._plot_widget.plot([], [], pen=_pen(_C_INK, 2.2), name='Current stroke')
        # Completed strokes stored as individual PlotDataItem objects
        self._ink_items  = []

        # Legend
        legend = self._plot_widget.addLegend(offset=(10, 10))
        legend.addItem(self._imu_curve,  'IMU dead-reck.')
        legend.addItem(self._uwb_curve,  'UWB clean')
        legend.addItem(self._air_curve,  'Air (fused)')
        legend.addItem(self._odom_curve, 'Odometry (NN)')
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
        if collect_mode:
            import os as _os
            _abs = _os.path.abspath(collect_path)
            print(f'  [COLLECT]  Output → {_abs}')
            print('  Draw real strokes on the board. More strokes = better model.')
            print('  Target: 2000+ samples. Close window or Ctrl+C when done.')
        else:
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
                self._latest_imu_quat = p.get('quat')

                s     = self.contact.process_one(p)
                fused = self.fusion.process_event(s)
                if not fused:
                    continue

                self.imu_count   += 1
                self.latest_fused = fused
                self._dirty       = True

                # ── Learned odometry overlay (read-only, non-blocking) ────────
                if cfg.odometry.enabled:
                    odom_frame = _extract_odom_frame(p)
                    window = self._odom_buf.push_packet([odom_frame])
                    if window is not None:
                        delta = self._odom_infer.predict(window)
                        if delta is not None:
                            self._odom_pos[0] += float(delta[0])
                            self._odom_pos[1] += float(delta[1])
                            self._odom_last_delta = (float(delta[0]), float(delta[1]))
                            self.odom_x.append(self._odom_pos[0])
                            self.odom_y.append(self._odom_pos[1])

                    # Reset buffer on stroke end to avoid cross-stroke contamination
                    stroke_now = fused.get('stroke_active', False)
                    if self._prev_stroke_active and not stroke_now:
                        self._odom_buf.reset()
                        if self._collector:
                            self._collector.on_stroke_end()
                    self._prev_stroke_active = stroke_now

                # ── Data collection: IMU frame feed ───────────────────────────
                if self._collector:
                    odom_frame = _extract_odom_frame(p)
                    stroke_now = fused.get('stroke_active', False)
                    self._collector.on_imu_frame(odom_frame, stroke_now)

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
                    pp  = closed.get('postprocess', {})
                    raw_pts = None
                    if pp.get('shape', {}).get('applied'):
                        raw = closed.get('raw_points') or []
                        if raw:
                            raw_pts = [(pt[0], pt[1]) for pt in raw]
                    self._add_ink_stroke(pts, raw_pts=raw_pts)
                    self.cur_ink_x.clear()
                    self.cur_ink_y.clear()
                    self.closed_count += 1
                    self._last_pp_meta = pp

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

                    # ── Data collection: UWB ground-truth label ───────────────
                    if self._collector:
                        uwb_xy = np.array(pc[:2], dtype=np.float32)
                        stroke_now = (fused.get('stroke_active', False)
                                      if fused else False)
                        self._collector.on_uwb_fix(uwb_xy, stroke_now)

    # ── Camera poll (called every 1000/fps ms) ────────────────────────────────
    def _poll_camera(self):
        if self._stopping or self._cam_cap is None or self._cam_det is None:
            return
        ret, frame = self._cam_cap.read()
        if not ret:
            return

        result = self._cam_det.process_frame(frame, self._latest_imu_quat)
        if result is None:
            return

        # Build a camera event and feed into ESKF
        import time
        cam_ev = {
            'sensor':     'CAMERA',
            'ts_hw':      int(time.time() * 1_000_000),
            'tip_board':  result['tip_board'],
            'confidence': result['confidence'],
        }
        fused = self.fusion.process_event(cam_ev)
        if fused:
            self.latest_fused = fused
            self._dirty = True

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
        if self.odom_x:
            self._odom_curve.setData(list(self.odom_x), list(self.odom_y))
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
            odom_buf_fill=self._odom_buf.fill_ratio(),
            odom_last_delta=self._odom_last_delta,
            odom_ready=self._odom_infer._ready,
        ) + _format_pp_debug(self._last_pp_meta)
        self._debug_label.setText(txt)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _add_ink_stroke(self, pts: list, raw_pts: list = None):
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
        # Faint amber overlay showing the pre-snap raw polyline when shape
        # correction was applied — lets the user see what got autocorrected.
        if raw_pts:
            rxs = [p[0] for p in raw_pts]
            rys = [p[1] for p in raw_pts]
            raw_item = self._plot_widget.plot(
                rxs, rys,
                pen=pg.mkPen(color=_C_RAW, width=1.2, style=pg.QtCore.Qt.DashLine),
            )
            self._ink_items.append(raw_item)

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
            f'{e.get("p_nominal", (0.0, 0.0))[0]:.5f}',
            f'{e.get("p_nominal", (0.0, 0.0))[1]:.5f}',
            f'{e.get("z_uwb_tip", (0.0, 0.0))[0]:.5f}',
            f'{e.get("z_uwb_tip", (0.0, 0.0))[1]:.5f}',
            int(e.get('vel_pseudo_applied', False)),
            f'{e.get("vel_pseudo_dx_pos", (0.0, 0.0))[0]:.5f}',
            f'{e.get("vel_pseudo_dx_pos", (0.0, 0.0))[1]:.5f}',
            f'{e.get("vel_pseudo_dx_vel", (0.0, 0.0))[0]:.5f}',
            f'{e.get("vel_pseudo_dx_vel", (0.0, 0.0))[1]:.5f}',
            int(e.get('active_uwb_guard_fired', False)),
            f'{e.get("active_uwb_guard_correction", 0.0):.5f}',
            int(e.get('tip_lock_active', False)),
            int(e.get('tip_lock_candidate', False)),
            int(e.get('tip_lock_count', 0)),
            f'{e.get("tip_lock_correction", 0.0):.5f}',
            f'{e.get("tip_lock_uwb_blend", 0.0):.3f}',
            e.get('tip_lock_reason', 'OFF'),
            f'{e.get("stroke_age_s", 0.0):.3f}',
            f'{e.get("age_cap_mult", 1.0):.4f}',
            f'{min(e.get("age_cap_mult", 1.0) * (cfg.fusion_eskf.modes.drawing_fast if e.get("drawing_fast") else cfg.fusion_eskf.modes.drawing).pos_gain_cap, 1.0):.5f}',
            # IMU curve diagnostics
            *_imu_curve_csv(imu_ev),
            # Odometry overlay
            f'{self._odom_pos[0]:.4f}', f'{self._odom_pos[1]:.4f}',
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
        if self._cam_cap is not None:
            self._cam_cap.release()
        self._csvf.close()

        # Save collected training data (if in collect mode)
        if self._collector:
            self._collector.save(self.collect_path)

        # Final refresh for PNG
        self._dirty = True
        self._refresh_display()

        # Export PNG using pyqtgraph's exporter
        try:
            try:
                from pyqtgraph.exporters import ImageExporter
            except ImportError:
                ImageExporter = pg.exporters.ImageExporter
            exporter = ImageExporter(self._plot_widget.getPlotItem())
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
    import os
    _HERE = os.path.dirname(os.path.abspath(__file__))
    _PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..'))

    fusion_mode   = 'eskf'
    collect_mode  = False
    collect_path  = os.path.join(_PROJECT_ROOT, 'data', 'odometry_training_data.npz')

    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == '--fusion' and i + 1 < len(sys.argv):
            fusion_mode = sys.argv[i + 1]; i += 2
        elif arg.startswith('--fusion='):
            fusion_mode = arg.split('=', 1)[1]; i += 1
        elif arg == '--collect':
            collect_mode = True; i += 1
        elif arg.startswith('--collect='):
            collect_mode = True
            collect_path = arg.split('=', 1)[1]; i += 1
        else:
            i += 1

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    win = VisualizerWindow(fusion_mode, collect_mode=collect_mode,
                           collect_path=collect_path)
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
