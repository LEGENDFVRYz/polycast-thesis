"""
test/batch/_batch_utils.py — Shared utilities for the batch replay/plot flow.

Provides:
  - _FIELDNAMES / _row(ev)        CSV schema + row extractor
  - compute_metrics(results)      summary metrics dict (fusion health + geometry)
  - _ink_rows / _per_stroke       stroke helpers
  - _straightness / _closure / _bbox / _path_length / _pct  geometry
  - run_one(tag, dataset, overrides, raw_dir, out_dir)       headless run + CSV write
  - write_summary(rows, path, fields)                        generic CSV writer
  - plot_zoom / plot_full / plot_imu_diag / plot_compare     matplotlib plots
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# Ensure project root is on sys.path
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from test.tuning.headless_runner import run_dataset, parse_dataset
from test.tuning.config_patcher   import patch_cfg, reset_cfg
from background.pipelines.preprocess.imu     import IMUPreprocessor
from background.pipelines.preprocess.contact import ContactStateDetector
from background.pipelines.reconstruct        import StrokeReconstructor
from background.pipelines.config import cfg as _cfg

BOARD_W = 1.25
BOARD_H = 1.24

_COL_FUSED = '#1a1a1a'
_COL_AIR   = '#888888'
_COL_UWB   = '#e07b00'

# -----------------------------------------------------------------------------
# CSV Column Schema
# -----------------------------------------------------------------------------

_FIELDNAMES = [
    'ts_hw', 'source',
    'fused_x', 'fused_y',
    'uwb_x', 'uwb_y',
    'state', 'fusion_mode', 'stroke_id', 'stroke_active',
    'contact_raw', 'ink_written',
    'P_pos_trace', 'innovation_norm', 'r_scale', 'K_pos_diag',
    'stroke_age_s', 'age_cap_mult', 'kcap_eff',
    'b_a_x', 'b_a_y', 'b_a_norm',
    'b_p_x', 'b_p_y', 'b_p_mag',
    'frames_contact', 'frames_fast', 'frames_air', 'frames_static',
    'avg_K_contact', 'avg_K_fast', 'avg_K_air',
    'fast_arm_count', 'fast_burst_count',
    'uwb_residual_rms', 'uwb_accepted', 'uwb_rejected',
    'acc_hp_tip_x', 'acc_hp_tip_y', 'acc_hp_tip_mag',
    'acc_tip_x', 'acc_tip_y', 'acc_tip_mag',
    'omega_mag', 'alpha_mag', 'jerk',
]


def _row(ev: dict) -> dict:
    e   = ev.get('eskf', {})
    ba  = e.get('b_a', (0.0, 0.0))
    bp  = e.get('b_p', (0.0, 0.0))

    imu_ref  = ev.get('_imu_ev', {})
    hp_raw   = imu_ref.get('acc_board_hp_tip', (0.0, 0.0))
    tip_raw  = imu_ref.get('acc_board_tip',    (0.0, 0.0))
    om_raw   = imu_ref.get('omega_world', (0.0, 0.0, 0.0))
    al_raw   = imu_ref.get('alpha_world', (0.0, 0.0, 0.0))
    jerk     = imu_ref.get('jerk', 0.0)
    hx, hy   = float(hp_raw[0]),  float(hp_raw[1])
    tx, ty   = float(tip_raw[0]), float(tip_raw[1])
    om_mag   = math.sqrt(sum(float(x)**2 for x in om_raw))
    al_mag   = math.sqrt(sum(float(x)**2 for x in al_raw))

    is_ink = (
        bool(ev.get('stroke_active', False)) and
        bool(ev.get('contact_raw', True)) and
        int(ev.get('stroke_id', 0)) != 0
    )

    mode_p_cap = (
        _cfg.fusion_eskf.modes.drawing_fast if e.get('drawing_fast') else _cfg.fusion_eskf.modes.drawing
    ).pos_gain_cap
    kcap_eff = min(e.get('age_cap_mult', 1.0) * mode_p_cap, 1.0)

    return {
        'ts_hw':           ev.get('ts_hw', 0),
        'source':          ev.get('source', ''),
        'fused_x':         f"{ev.get('fused_x', 0):.5f}",
        'fused_y':         f"{ev.get('fused_y', 0):.5f}",
        'uwb_x':           f"{ev.get('uwb_x', 0):.5f}",
        'uwb_y':           f"{ev.get('uwb_y', 0):.5f}",
        'state':           ev.get('state', ''),
        'fusion_mode':     ev.get('fusion_mode', ''),
        'stroke_id':       ev.get('stroke_id', 0),
        'stroke_active':   int(ev.get('stroke_active', False)),
        'contact_raw':     int(ev.get('contact_raw', True)),
        'ink_written':     int(is_ink),
        'P_pos_trace':     f"{e.get('P_pos_trace', 0):.5f}",
        'innovation_norm': f"{e.get('innovation_norm', 0):.4f}",
        'r_scale':         f"{e.get('r_scale', 1):.3f}",
        'K_pos_diag':      f"{e.get('K_pos_diag', 0):.5f}",
        'stroke_age_s':    f"{e.get('stroke_age_s', 0.0):.3f}",
        'age_cap_mult':    f"{e.get('age_cap_mult', 1.0):.4f}",
        'kcap_eff':        f"{kcap_eff:.5f}",
        'b_a_x':           f"{float(ba[0]):.5f}",
        'b_a_y':           f"{float(ba[1]):.5f}",
        'b_a_norm':        f"{e.get('b_a_norm', 0):.5f}",
        'b_p_x':           f"{float(bp[0]):.5f}",
        'b_p_y':           f"{float(bp[1]):.5f}",
        'b_p_mag':         f"{e.get('b_p_mag', 0):.5f}",
        'frames_contact':  e.get('frames_contact', 0),
        'frames_fast':     e.get('frames_fast', 0),
        'frames_air':      e.get('frames_air', 0),
        'frames_static':   e.get('frames_static', 0),
        'avg_K_contact':   f"{e.get('avg_K_contact', 0):.5f}",
        'avg_K_fast':      f"{e.get('avg_K_fast', 0):.5f}",
        'avg_K_air':       f"{e.get('avg_K_air', 0):.5f}",
        'fast_arm_count':  e.get('fast_arm_count', 0),
        'fast_burst_count':e.get('fast_burst_count', 0),
        'uwb_residual_rms':f"{e.get('uwb_residual_rms', 0):.5f}",
        'uwb_accepted':    e.get('uwb_accepted', 0),
        'uwb_rejected':    e.get('uwb_rejected', 0),
        'acc_hp_tip_x':    f"{hx:.5f}",
        'acc_hp_tip_y':    f"{hy:.5f}",
        'acc_hp_tip_mag':  f"{math.hypot(hx, hy):.5f}",
        'acc_tip_x':       f"{tx:.5f}",
        'acc_tip_y':       f"{ty:.5f}",
        'acc_tip_mag':     f"{math.hypot(tx, ty):.5f}",
        'omega_mag':       f"{om_mag:.5f}",
        'alpha_mag':       f"{al_mag:.5f}",
        'jerk':            f"{float(jerk):.4f}",
    }


# -----------------------------------------------------------------------------
# Geometry Helpers
# -----------------------------------------------------------------------------

def _ink_rows(results: list[dict]) -> list[dict]:
    return [r for r in results if r.get('stroke_active') and r.get('contact_raw', True)
            and r.get('stroke_id', 0) != 0]


def _per_stroke(results: list[dict]) -> dict:
    strokes: dict[int, list] = {}
    for r in results:
        sid = r.get('stroke_id', 0)
        if sid and r.get('stroke_active') and r.get('contact_raw', True):
            strokes.setdefault(sid, []).append(r)
    return strokes


def _bbox(pts) -> tuple:
    if not pts:
        return 0.0, 0.0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (max(xs) - min(xs)) * 1000, (max(ys) - min(ys)) * 1000


def _closure(pts) -> float:
    if len(pts) < 2:
        return 0.0
    return math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) * 1000


def _straightness(pts) -> float:
    if len(pts) < 2:
        return 1.0
    chord = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    arc = sum(math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
              for i in range(1, len(pts)))
    return min(chord / arc, 1.0) if arc > 0 else 1.0


def _path_length(pts) -> float:
    return sum(math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
               for i in range(1, len(pts))) * 1000


def _angle_deg(pts) -> float:
    if len(pts) < 2:
        return 0.0
    return math.degrees(math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0]))


def _pct(vals: list, p: float) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    idx = int(len(s) * p / 100)
    return s[min(idx, len(s) - 1)]


def _mean(vals: list) -> float:
    return sum(vals) / len(vals) if vals else 0.0


# -----------------------------------------------------------------------------
# Postprocess Stroke Collection
# -----------------------------------------------------------------------------

def collect_strokes(results: list[dict]) -> list[dict]:
    """Run StrokeReconstructor over fused results and return closed stroke dicts.

    Each returned stroke has a 'postprocess' key populated by StrokePostprocessor
    (centroid alignment + min-jerk) when cfg.postprocess.enabled is True.
    """

    rec = StrokeReconstructor()
    strokes: list[dict] = []
    for ev in results:
        closed = rec.process_event(ev)
        if closed:
            strokes.append(closed)
    last = rec.flush()
    if last:
        strokes.append(last)
    return strokes


def _compute_pp_metrics(strokes: list[dict]) -> dict:
    """Aggregate postprocess correction stats across all closed strokes."""

    empty = {
        'pp_strokes_total':     0,
        'pp_strokes_corrected': 0,
        'pp_corr_rate_pct':     0.0,
        'pp_delta_mean_mm':     0.0,
        'pp_delta_max_mm':      0.0,
        'pp_rot_mean_deg':      0.0,
        'pp_rot_max_deg':       0.0,
        'pp_scale_mean':        1.0,
        'pp_uwb_used_mean':     0.0,
        'pp_jerk_rate_pct':     0.0,
    }
    if not strokes:
        return empty

    total = len(strokes)
    corrected = 0
    deltas_mm, rots_deg, scales, uwb_used = [], [], [], []
    jerk_applied = 0

    for s in strokes:
        pp = s.get('postprocess', {})
        if not pp.get('applied'):
            continue
        corrected += 1
        ca = pp.get('centroid_align', {})
        if ca.get('applied'):
            deltas_mm.append(ca.get('delta_m', 0.0) * 1000.0)
            rots_deg.append(abs(ca.get('rotation_deg', 0.0)))
            scales.append(ca.get('scale', 1.0))
        uwb_used.append(pp.get('uwb_points_used', 0))
        if pp.get('min_jerk', {}).get('applied'):
            jerk_applied += 1

    return {
        'pp_strokes_total':     total,
        'pp_strokes_corrected': corrected,
        'pp_corr_rate_pct':     round(100.0 * corrected / total, 1),
        'pp_delta_mean_mm':     round(_mean(deltas_mm), 2),
        'pp_delta_max_mm':      round(max(deltas_mm), 2) if deltas_mm else 0.0,
        'pp_rot_mean_deg':      round(_mean(rots_deg), 2),
        'pp_rot_max_deg':       round(max(rots_deg), 2) if rots_deg else 0.0,
        'pp_scale_mean':        round(_mean(scales), 4),
        'pp_uwb_used_mean':     round(_mean(uwb_used), 1),
        'pp_jerk_rate_pct':     round(100.0 * jerk_applied / total, 1),
    }


# -----------------------------------------------------------------------------
# IMU Event Attachment
# -----------------------------------------------------------------------------

def attach_imu_events(results: list[dict], raw_path: str) -> None:
    """Attach _imu_ev dicts to IMU-source result rows by re-running preprocess."""

    imu_prep = IMUPreprocessor()
    contact  = ContactStateDetector()
    events   = parse_dataset(raw_path)
    imu_annotated = []
    for ev in events:
        if ev['sensor'] == 'IMU':
            p = imu_prep.process_one(ev)
            if p is not None:
                a = contact.process_one(p)
                imu_annotated.append(a)

    imu_results = [r for r in results if r.get('source') == 'IMU']
    for i, r in enumerate(imu_results):
        r['_imu_ev'] = imu_annotated[i] if i < len(imu_annotated) else {}


# -----------------------------------------------------------------------------
# Metrics Computation
# -----------------------------------------------------------------------------

_LINE_DATASETS  = {'hline', 'vline', 'dline'}
_CLOSED_DATASETS = {'circle', 'square', 'triangle'}

# Expected line angles (degrees) for direction correctness check
_LINE_EXPECTED_ANGLE = {
    'hline':   0.0,
    'vline':  90.0,
    'dline': -45.0,   # measured path runs ~-40.8°; this diagonal drawn top-right to bottom-left
}


def _lateral_thickness(pts) -> float:
    """Bounding-box minor-axis thickness in mm (perpendicular to stroke direction)."""

    if len(pts) < 2:
        return 0.0
    w, h = _bbox(pts)
    return min(w, h)   # thinner dimension = lateral spread


def compute_metrics(results: list[dict], dataset: str, tag: str = '',
                    strokes: list[dict] | None = None) -> dict:
    """Compute full summary metrics from a run's results list.

    Fix 1: UWB reject accounting — separates UWB_WAIT_IMU_INIT (startup) from
           runtime rejects so initialization noise does not flag runtime quality.
    Fix 2: Line datasets use straightness/angle/lateral thickness, not closure.
    Fix 3: ABC stroke count uses processed strokes, not raw contact segments.
    Fix 4: P_trace_max, innovation_max, and UWB reject pct computed
           both globally and after-initialization (active window).
    """

    per_stroke = _per_stroke(results)
    ink        = _ink_rows(results)

    eskf_last = results[-1].get('eskf', {}) if results else {}
    fc = eskf_last.get('frames_contact', 0)
    ff = eskf_last.get('frames_fast',    0)
    fa = eskf_last.get('frames_air',     0)
    fs = eskf_last.get('frames_static',  0)
    total = fc + ff + fa + fs or 1

    uwb_acc = eskf_last.get('uwb_accepted', 0)
    uwb_rej = eskf_last.get('uwb_rejected', 0)
    uwb_tot = uwb_acc + uwb_rej or 1

    # Fix 1 — count UWB_WAIT_IMU_INIT rejects by scanning state labels.
    # These only fire before IMU attitude is established (first few dozen frames).
    uwb_wait_init_count = sum(
        1 for r in results
        if r.get('source') == 'POSITION' and r.get('state') == 'UWB_WAIT_IMU_INIT'
    )
    uwb_runtime_rej = max(0, uwb_rej - uwb_wait_init_count)
    uwb_runtime_tot = uwb_acc + uwb_runtime_rej or 1
    uwb_runtime_rej_pct = round(100 * uwb_runtime_rej / uwb_runtime_tot, 1)

    # Fix 4 — find the initialization boundary: first frame after UWB bootstrap
    # (state == 'UWB_BOOTSTRAP' marks the first accepted UWB fix).
    init_done_idx = 0
    for i, r in enumerate(results):
        if r.get('source') == 'POSITION' and r.get('state') == 'UWB_BOOTSTRAP':
            init_done_idx = i + 1
            break

    hp_mags = []; tip_mags = []; om_mags = []; al_mags = []
    jerks = []; ba_norms = []; bp_mags = []; innovs = []
    p_traces = []; k_draw = []; k_fast = []; k_air = []
    uwb_res = []; r_scales = []
    age_mults = []
    # Fix 4 — active-window (post-init) versions
    p_traces_active = []; innovs_active = []

    for i, r in enumerate(results):
        e   = r.get('eskf', {})
        imu = r.get('_imu_ev', {})
        hp   = imu.get('acc_board_hp_tip', (0.0, 0.0))
        tip  = imu.get('acc_board_tip', (0.0, 0.0))
        om   = imu.get('omega_world', (0.0, 0.0, 0.0))
        al   = imu.get('alpha_world', (0.0, 0.0, 0.0))

        pt = float(e.get('P_pos_trace', 0))
        inn = float(e.get('innovation_norm', 0))
        p_traces.append(pt)
        r_scales.append(float(e.get('r_scale', 1)))
        age_mults.append(float(e.get('age_cap_mult', 1.0)))
        rr = float(e.get('uwb_residual_rms', 0))
        if rr > 0:
            uwb_res.append(rr)

        ki = float(e.get('avg_K_contact', 0))
        kf = float(e.get('avg_K_fast', 0))
        ka = float(e.get('avg_K_air', 0))
        if ki > 0: k_draw.append(ki)
        if kf > 0: k_fast.append(kf)
        if ka > 0: k_air.append(ka)

        if r.get('stroke_active'):
            hp_mags.append(math.hypot(float(hp[0]), float(hp[1])))
            tip_mags.append(math.hypot(float(tip[0]), float(tip[1])))
            om_mags.append(math.sqrt(sum(float(x)**2 for x in om)))
            al_mags.append(math.sqrt(sum(float(x)**2 for x in al)))
            jerks.append(float(imu.get('jerk', 0.0)))
            ba_norms.append(float(e.get('b_a_norm', 0.0)))
            bp_mags.append(float(e.get('b_p_mag', 0.0)))
            innovs.append(inn)

        # Fix 4 — active-window lists (post-init only)
        if i >= init_done_idx:
            p_traces_active.append(pt)
            if r.get('stroke_active'):
                innovs_active.append(inn)

    all_ink_pts = [(r['fused_x'], r['fused_y']) for r in results
                   if r.get('stroke_active') and r.get('contact_raw', True) and r.get('stroke_id', 0)]
    all_uwb_pts = [(r['uwb_x'], r['uwb_y']) for r in results if r.get('uwb_x') is not None]

    fused_w, fused_h = _bbox(all_ink_pts)
    uwb_w,   uwb_h   = _bbox(all_uwb_pts)
    fused_path = _path_length(all_ink_pts)
    uwb_path   = _path_length(all_uwb_pts)
    closure    = _closure(all_ink_pts)
    straight   = _straightness(all_ink_pts)
    angle      = _angle_deg(all_ink_pts)
    aspect     = (fused_w / fused_h) if fused_h > 0 else 0.0

    # Fix 2 — line-specific geometry metrics
    lateral_mm = _lateral_thickness(all_ink_pts)
    expected_angle = _LINE_EXPECTED_ANGLE.get(dataset, None)
    angle_error = 0.0
    if expected_angle is not None and all_ink_pts:
        raw_err = abs(angle - expected_angle)
        angle_error = round(min(raw_err, 180.0 - raw_err), 2)

    fused_vs_uwb = []
    for r in results:
        if r.get('stroke_active') and r.get('uwb_x') is not None:
            d = math.hypot(r['fused_x'] - r['uwb_x'], r['fused_y'] - r['uwb_y']) * 1000
            fused_vs_uwb.append(d)

    stroke_rows = []
    for sid, rows in sorted(per_stroke.items()):
        pts = [(r['fused_x'], r['fused_y']) for r in rows]
        w, h = _bbox(pts)
        stroke_rows.append({
            'sid':         sid,
            'n_pts':       len(pts),
            'width_mm':    round(w, 1),
            'height_mm':   round(h, 1),
            'path_mm':     round(_path_length(pts), 1),
            'straightness':round(_straightness(pts), 3),
            'closure_mm':  round(_closure(pts), 1),
            'angle_deg':   round(_angle_deg(pts), 1),
            'lateral_mm':  round(_lateral_thickness(pts), 1),
        })

    return {
        'tag':                    tag,
        'dataset':                dataset,
        'total_events':           len(results),
        'ink_events':             len(ink),
        'strokes':                len(per_stroke),
        # mode
        'mode_pct_fast':          round(100 * ff / total, 1),
        'mode_pct_draw':          round(100 * fc / total, 1),
        'mode_pct_air':           round(100 * fa / total, 1),
        'mode_pct_static':        round(100 * fs / total, 1),
        # Fix 1 — UWB accounting (total + runtime-only)
        'uwb_accepted':           uwb_acc,
        'uwb_rejected':           uwb_rej,
        'uwb_wait_imu_init_count':uwb_wait_init_count,
        'uwb_runtime_reject':     uwb_runtime_rej,
        'uwb_accept_pct':         round(100 * uwb_acc / uwb_tot, 1),
        'uwb_reject_pct':         round(100 * uwb_rej / uwb_tot, 1),
        'uwb_runtime_reject_pct': uwb_runtime_rej_pct,
        'uwb_residual_mean':      round(_mean(uwb_res), 5),
        'uwb_residual_p95':       round(_pct(uwb_res, 95), 5),
        # innovation — global + Fix 4 active-window
        'innov_mean':             round(_mean(innovs), 4),
        'innov_p95':              round(_pct(innovs, 95), 4),
        'innov_max':              round(max(innovs), 4) if innovs else 0,
        'innov_p95_active':       round(_pct(innovs_active, 95), 4),
        'innov_max_active':       round(max(innovs_active), 4) if innovs_active else 0,
        # Fix 4 — P trace global + active-window
        'P_trace_mean':           round(_mean(p_traces), 5),
        'P_trace_max':            round(max(p_traces), 5) if p_traces else 0,
        'P_trace_max_active':     round(max(p_traces_active), 5) if p_traces_active else 0,
        # Kalman gains
        'K_draw_mean':            round(_mean(k_draw), 5),
        'K_draw_p95':             round(_pct(k_draw, 95), 5),
        'K_fast_mean':            round(_mean(k_fast), 5),
        'K_fast_p95':             round(_pct(k_fast, 95), 5),
        'K_air_mean':             round(_mean(k_air), 5),
        'K_air_p95':              round(_pct(k_air, 95), 5),
        # age ramp
        'age_mult_mean':          round(_mean(age_mults), 4),
        'age_mult_max':           round(max(age_mults), 4) if age_mults else 1.0,
        # biases
        'b_a_norm_mean':          round(_mean(ba_norms), 5),
        'b_a_norm_p95':           round(_pct(ba_norms, 95), 5),
        'b_a_norm_max':           round(max(ba_norms), 5) if ba_norms else 0,
        'b_p_mag_mean':           round(_mean(bp_mags), 4),
        'b_p_mag_p95':            round(_pct(bp_mags, 95), 4),
        'b_p_mag_max':            round(max(bp_mags), 4) if bp_mags else 0,
        # IMU signal quality
        'acc_hp_tip_mean':        round(_mean(hp_mags), 3),
        'acc_hp_tip_p95':         round(_pct(hp_mags, 95), 3),
        'acc_hp_tip_max':         round(max(hp_mags), 3) if hp_mags else 0,
        'acc_tip_mean':           round(_mean(tip_mags), 3),
        'acc_tip_p95':            round(_pct(tip_mags, 95), 3),
        'omega_mean':             round(_mean(om_mags), 3),
        'omega_p95':              round(_pct(om_mags, 95), 3),
        'alpha_mean':             round(_mean(al_mags), 3),
        'alpha_p95':              round(_pct(al_mags, 95), 3),
        'jerk_mean':              round(_mean(jerks), 2),
        'jerk_p95':               round(_pct(jerks, 95), 2),
        # geometry
        'fused_bbox_w_mm':        round(fused_w, 1),
        'fused_bbox_h_mm':        round(fused_h, 1),
        'uwb_bbox_w_mm':          round(uwb_w, 1),
        'uwb_bbox_h_mm':          round(uwb_h, 1),
        'fused_path_mm':          round(fused_path, 1),
        'uwb_path_mm':            round(uwb_path, 1),
        'fused_vs_uwb_mean_mm':   round(_mean(fused_vs_uwb), 1),
        'fused_vs_uwb_p95_mm':    round(_pct(fused_vs_uwb, 95), 1),
        # Fix 2 — closure only meaningful for closed shapes; lines use straightness+angle
        'closure_mm':             round(closure, 1),
        'straightness':           round(straight, 3),
        'angle_deg':              round(angle, 1),
        'angle_error_deg':        angle_error,
        'lateral_mm':             round(lateral_mm, 1),
        'aspect_ratio':           round(aspect, 3),
        # Fix 4 — initialization boundary
        'init_done_frame_idx':    init_done_idx,
        # stroke detail (not written to flat CSV — used for strokes.csv)
        '_stroke_rows':   stroke_rows,
        # postprocess closed stroke objects (for plot_postprocess)
        '_stroke_objects': strokes or [],
        **_compute_pp_metrics(strokes or []),
    }


# -----------------------------------------------------------------------------
# Run One Combination
# -----------------------------------------------------------------------------

def run_one(tag: str, dataset: str, overrides: dict,
            raw_dir: str, out_dir: str) -> dict:
    raw_path = os.path.join(raw_dir, f'{dataset}.csv')
    if not os.path.exists(raw_path):
        print(f'  [SKIP] {raw_path} not found')
        return {}

    reset_cfg()
    if overrides:
        patch_cfg(overrides)

    results = run_dataset(raw_path)
    attach_imu_events(results, raw_path)
    strokes = collect_strokes(results)

    out_csv = os.path.join(out_dir, f'{tag}_{dataset}.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDNAMES, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(_row(r))

    metrics = compute_metrics(results, dataset, tag, strokes=strokes)
    print(f'  [{tag}/{dataset}] strokes={metrics["strokes"]}  '
          f'fast%={metrics["mode_pct_fast"]}  '
          f'innov_p95={metrics["innov_p95"]}  '
          f'closure={metrics["closure_mm"]} mm')
    return metrics


# -----------------------------------------------------------------------------
# Summary CSV Writer
# -----------------------------------------------------------------------------

_SUMMARY_FIELDS = [
    'tag', 'dataset', 'strokes', 'ink_events', 'total_events',
    'mode_pct_fast', 'mode_pct_draw', 'mode_pct_air', 'mode_pct_static',
    # Fix 1 — UWB accounting
    'uwb_accepted', 'uwb_rejected', 'uwb_wait_imu_init_count',
    'uwb_runtime_reject', 'uwb_accept_pct', 'uwb_reject_pct', 'uwb_runtime_reject_pct',
    'uwb_residual_mean', 'uwb_residual_p95',
    # innovation global + Fix 4 active-window
    'innov_mean', 'innov_p95', 'innov_max',
    'innov_p95_active', 'innov_max_active',
    # Fix 4 — P trace
    'P_trace_mean', 'P_trace_max', 'P_trace_max_active',
    'init_done_frame_idx',
    'K_draw_mean', 'K_draw_p95', 'K_fast_mean', 'K_fast_p95',
    'K_air_mean', 'K_air_p95',
    'age_mult_mean', 'age_mult_max',
    'b_a_norm_mean', 'b_a_norm_p95', 'b_a_norm_max',
    'b_p_mag_mean', 'b_p_mag_p95', 'b_p_mag_max',
    'acc_hp_tip_mean', 'acc_hp_tip_p95', 'acc_hp_tip_max',
    'acc_tip_mean', 'acc_tip_p95',
    'omega_mean', 'omega_p95',
    'alpha_mean', 'alpha_p95',
    'jerk_mean', 'jerk_p95',
    'fused_bbox_w_mm', 'fused_bbox_h_mm',
    'uwb_bbox_w_mm', 'uwb_bbox_h_mm',
    'fused_path_mm', 'uwb_path_mm',
    'fused_vs_uwb_mean_mm', 'fused_vs_uwb_p95_mm',
    # Fix 2 — geometry: closure for closed shapes, line metrics for lines
    'closure_mm', 'straightness', 'angle_deg', 'angle_error_deg',
    'lateral_mm', 'aspect_ratio',
    # Postprocess
    'pp_strokes_total', 'pp_strokes_corrected', 'pp_corr_rate_pct',
    'pp_delta_mean_mm', 'pp_delta_max_mm',
    'pp_rot_mean_deg', 'pp_rot_max_deg',
    'pp_scale_mean', 'pp_uwb_used_mean', 'pp_jerk_rate_pct',
]

_STROKE_FIELDS = [
    'tag', 'dataset', 'sid', 'n_pts',
    'width_mm', 'height_mm', 'path_mm', 'straightness',
    'closure_mm', 'angle_deg', 'lateral_mm',
]


def write_summary(all_metrics: list[dict], summary_path: str,
                  strokes_path: str | None = None) -> None:
    with open(summary_path, 'w', newline='', encoding='utf-8') as sf:
        sw = csv.DictWriter(sf, fieldnames=_SUMMARY_FIELDS, extrasaction='ignore')
        sw.writeheader()
        for m in all_metrics:
            if m:
                sw.writerow(m)

    if strokes_path:
        with open(strokes_path, 'w', newline='', encoding='utf-8') as stf:
            stw = csv.DictWriter(stf, fieldnames=_STROKE_FIELDS, extrasaction='ignore')
            stw.writeheader()
            for m in all_metrics:
                if not m:
                    continue
                for s in m.get('_stroke_rows', []):
                    stw.writerow(dict(s, tag=m['tag'], dataset=m['dataset']))

    print(f'Summary -> {summary_path}')
    if strokes_path:
        print(f'Strokes -> {strokes_path}')


# -----------------------------------------------------------------------------
# Plot Helpers
# -----------------------------------------------------------------------------

def _f(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def _i(row: dict, key: str, default: int = 0) -> int:
    try:
        return int(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def _extract_paths_csv(rows: list[dict]):
    fused_ink, fused_air, uwb_pts = [], [], []
    for r in rows:
        fx, fy = _f(r, 'fused_x'), _f(r, 'fused_y')
        is_ink = _i(r, 'ink_written') == 1
        if is_ink:
            fused_ink.append((fx, fy))
        else:
            fused_air.append((fx, fy))
        try:
            uwb_pts.append((_f(r, 'uwb_x'), _f(r, 'uwb_y')))
        except Exception:
            pass
    return fused_ink, fused_air, uwb_pts


def _ink_by_stroke_csv(rows: list[dict]) -> dict:
    strokes: dict[int, list] = {}
    for r in rows:
        if _i(r, 'ink_written') == 1:
            sid = _i(r, 'stroke_id')
            strokes.setdefault(sid, []).append((_f(r, 'fused_x'), _f(r, 'fused_y')))
    return strokes


def _uwb_ink_csv(rows: list[dict]) -> list:
    return [(_f(r, 'uwb_x'), _f(r, 'uwb_y'))
            for r in rows if _i(r, 'ink_written') == 1]


def _mode_pct_csv(rows: list[dict]) -> tuple:
    if not rows:
        return 0, 0, 0
    last = rows[-1]
    fc = _i(last, 'frames_contact')
    ff = _i(last, 'frames_fast')
    fa = _i(last, 'frames_air')
    fs = _i(last, 'frames_static')
    total = fc + ff + fa + fs or 1
    return 100*ff/total, 100*fc/total, 100*fa/total


def _auto_zoom(rows: list[dict], margin: float = 0.04):
    strokes = _ink_by_stroke_csv(rows)
    all_pts = [p for pts in strokes.values() for p in pts]
    if not all_pts:
        return (0, BOARD_W), (0, BOARD_H)
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    return (min(xs)-margin, max(xs)+margin), (min(ys)-margin, max(ys)+margin)


def _setup_ax(ax, title: str, board: bool):
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=9, pad=4)
    ax.tick_params(labelsize=7)
    ax.set_xlabel('X (m)', fontsize=7)
    ax.set_ylabel('Y (m)', fontsize=7)
    if board:
        ax.set_xlim(-0.02, BOARD_W + 0.02)
        ax.set_ylim(-0.02, BOARD_H + 0.02)
        rect = plt.Rectangle((0, 0), BOARD_W, BOARD_H,
                              fill=False, edgecolor='#cccccc', lw=1.2, ls='--')
        ax.add_patch(rect)


def _draw_paths_on_ax(ax, rows: list[dict], alpha_air: float = 0.25):
    strokes  = _ink_by_stroke_csv(rows)
    uwb_ink  = _uwb_ink_csv(rows)
    _, fused_air, _ = _extract_paths_csv(rows)

    if uwb_ink:
        ax.scatter([p[0] for p in uwb_ink], [p[1] for p in uwb_ink],
                   s=1.5, c=_COL_UWB, alpha=0.5, linewidths=0, zorder=2)
    if fused_air:
        ax.scatter([p[0] for p in fused_air], [p[1] for p in fused_air],
                   s=0.8, c=_COL_AIR, alpha=alpha_air, linewidths=0, zorder=3)
    for sid, pts in sorted(strokes.items()):
        if not pts:
            continue
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        ax.plot(xs, ys, color=_COL_FUSED, lw=1.0, zorder=5)
        ax.scatter([xs[0]], [ys[0]], s=18, c='#2266cc', zorder=6, marker='o')
        ax.scatter([xs[-1]], [ys[-1]], s=18, c='#cc2222', zorder=6, marker='s')


def _info_box(ax, rows: list[dict], dataset: str):
    strokes  = _ink_by_stroke_csv(rows)
    fast_pct, _, _ = _mode_pct_csv(rows)
    all_ink  = [p for pts in strokes.values() for p in pts]
    lines = [f'strokes: {len(strokes)}', f'fast%: {fast_pct:.0f}']
    if all_ink:
        w, h = _bbox(all_ink)
        lines.append(f'bbox: {w:.0f}x{h:.0f} mm')
        if dataset in ('circle', 'square', 'triangle'):
            lines.append(f'closure: {_closure(all_ink):.0f} mm')
        elif dataset in ('hline', 'vline', 'dline'):
            lines.append(f'straight: {_straightness(all_ink):.3f}')
            lines.append(f'angle: {_angle_deg(all_ink):.1f}°')
    ax.text(0.02, 0.98, '\n'.join(lines), transform=ax.transAxes, fontsize=6,
            va='top', ha='left', family='monospace',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.75, lw=0))


def _legend():
    return [
        Line2D([0], [0], color=_COL_FUSED, lw=1.2, label='Fused ink'),
        Line2D([0], [0], color=_COL_AIR, lw=0, marker='o', ms=3, alpha=0.5, label='Fused air'),
        Line2D([0], [0], color=_COL_UWB, lw=0, marker='o', ms=3, alpha=0.7, label='UWB'),
        Line2D([0], [0], color='#2266cc', lw=0, marker='o', ms=5, label='Start'),
        Line2D([0], [0], color='#cc2222', lw=0, marker='s', ms=5, label='End'),
    ]


def _load_csv(path: str) -> list[dict]:
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def plot_zoom(csv_path: str, title: str, out_path: str, dataset: str = '') -> None:
    if not os.path.exists(csv_path):
        return
    rows = _load_csv(csv_path)
    fig, ax = plt.subplots(figsize=(5, 5))
    _setup_ax(ax, f'{title} [zoom]', board=False)
    _draw_paths_on_ax(ax, rows)
    xlim, ylim = _auto_zoom(rows)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    _info_box(ax, rows, dataset)
    ax.legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_full(csv_path: str, title: str, out_path: str, dataset: str = '') -> None:
    if not os.path.exists(csv_path):
        return
    rows = _load_csv(csv_path)
    fig, ax = plt.subplots(figsize=(5, 5))
    _setup_ax(ax, f'{title} [full]', board=True)
    _draw_paths_on_ax(ax, rows, alpha_air=0.15)
    _info_box(ax, rows, dataset)
    ax.legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_imu_diag(csv_path: str, title: str, out_path: str) -> None:
    if not os.path.exists(csv_path):
        return
    rows = _load_csv(csv_path)
    imu_rows = [r for r in rows if r.get('source') == 'IMU']
    if not imu_rows:
        return

    ts  = [_f(r, 'ts_hw') / 1e6 for r in imu_rows]
    t0  = ts[0]
    ts  = [t - t0 for t in ts]
    ink = [_i(r, 'ink_written') for r in imu_rows]

    hp  = [_f(r, 'acc_hp_tip_mag') for r in imu_rows]
    tip = [_f(r, 'acc_tip_mag')    for r in imu_rows]
    om  = [_f(r, 'omega_mag')      for r in imu_rows]
    al  = [_f(r, 'alpha_mag')      for r in imu_rows]
    jk  = [_f(r, 'jerk')           for r in imu_rows]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f'IMU diagnostics | {title}', fontsize=10)

    def _shade(ax):
        in_ink, t_start = False, 0.0
        for t, i in zip(ts, ink):
            if i and not in_ink:
                t_start, in_ink = t, True
            elif not i and in_ink:
                ax.axvspan(t_start, t, alpha=0.08, color='black')
                in_ink = False
        if in_ink:
            ax.axvspan(t_start, ts[-1], alpha=0.08, color='black')

    axes[0].plot(ts, hp,  lw=0.7, color='#2266cc', label='acc_hp_tip')
    axes[0].plot(ts, tip, lw=0.7, color='#66aaff', alpha=0.5, label='acc_tip')
    axes[0].axhline(7.0, color='red', lw=0.8, ls='--', alpha=0.6, label='clamp 7')
    axes[0].set_ylabel('m/s²', fontsize=7)
    axes[0].set_title('Acceleration magnitude', fontsize=8)
    axes[0].legend(fontsize=6, loc='upper right')
    _shade(axes[0])

    axes[1].plot(ts, om, lw=0.7, color='#cc6600')
    axes[1].set_ylabel('rad/s', fontsize=7)
    axes[1].set_title('Angular velocity (omega)', fontsize=8)
    _shade(axes[1])

    axes[2].plot(ts, al, lw=0.7, color='#aa2200')
    axes[2].set_ylabel('rad/s²', fontsize=7)
    axes[2].set_title('Angular acceleration (alpha)', fontsize=8)
    _shade(axes[2])

    axes[3].plot(ts, jk, lw=0.7, color='#006622')
    axes[3].set_ylabel('m/s³', fontsize=7)
    axes[3].set_xlabel('time (s)', fontsize=7)
    axes[3].set_title('Jerk', fontsize=8)
    _shade(axes[3])

    for ax in axes:
        ax.tick_params(labelsize=7)
        ax.grid(True, lw=0.3, alpha=0.5)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_postprocess(strokes: list[dict], title: str, out_path: str,
                     dataset: str = '') -> None:
    """Raw vs postprocessed stroke overlay plot.

    Gray = raw polyline (before PP); Black = corrected polyline (after PP).
    Skipped if no strokes are provided.
    """

    if not strokes:
        return

    fig, ax = plt.subplots(figsize=(5, 5))
    _setup_ax(ax, f'{title} [postprocess]', board=False)

    any_corrected = False
    all_pts: list[tuple] = []

    for s in strokes:
        pp = s.get('postprocess', {})
        pts_corr = [(float(p[0]), float(p[1])) for p in s.get('points', [])]
        pts_raw  = [(float(p[0]), float(p[1])) for p in s.get('raw_points', [])]

        applied = pp.get('applied', False)

        if pts_raw and applied and pts_raw != pts_corr:
            ax.plot([p[0] for p in pts_raw], [p[1] for p in pts_raw],
                    color='#cccccc', lw=0.8, zorder=3, alpha=0.8)
            any_corrected = True

        if pts_corr:
            xs = [p[0] for p in pts_corr]
            ys = [p[1] for p in pts_corr]
            ax.plot(xs, ys, color=_COL_FUSED, lw=1.2, zorder=5)
            ax.scatter([xs[0]],  [ys[0]],  s=18, c='#2266cc', zorder=6, marker='o')
            ax.scatter([xs[-1]], [ys[-1]], s=18, c='#cc2222', zorder=6, marker='s')
            all_pts.extend(pts_corr)

    if all_pts:
        margin = 0.04
        xs_all = [p[0] for p in all_pts]
        ys_all = [p[1] for p in all_pts]
        ax.set_xlim(min(xs_all) - margin, max(xs_all) + margin)
        ax.set_ylim(min(ys_all) - margin, max(ys_all) + margin)

    pp_mets = _compute_pp_metrics(strokes)
    info = [
        f'strokes: {pp_mets["pp_strokes_total"]}  corr: {pp_mets["pp_strokes_corrected"]}',
        f'delta: {pp_mets["pp_delta_mean_mm"]:.1f} mm mean',
        f'rot: {pp_mets["pp_rot_mean_deg"]:.1f}° mean',
        f'scale: {pp_mets["pp_scale_mean"]:.3f} mean',
        f'uwb/stroke: {pp_mets["pp_uwb_used_mean"]:.0f}',
        f'jerk%: {pp_mets["pp_jerk_rate_pct"]:.0f}%',
    ]
    if dataset in _CLOSED_DATASETS:
        all_ink = [p for s in strokes for p in [(float(pt[0]), float(pt[1])) for pt in s.get('points', [])]]
        if all_ink:
            info.append(f'closure: {_closure(all_ink):.1f} mm')
    ax.text(0.02, 0.98, '\n'.join(info), transform=ax.transAxes, fontsize=6,
            va='top', ha='left', family='monospace',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.75, lw=0))

    handles = [Line2D([0], [0], color=_COL_FUSED, lw=1.2, label='Corrected')]
    if any_corrected:
        handles.append(Line2D([0], [0], color='#cccccc', lw=0.8, label='Raw (pre-PP)'))
    handles += [
        Line2D([0], [0], color='#2266cc', lw=0, marker='o', ms=5, label='Start'),
        Line2D([0], [0], color='#cc2222', lw=0, marker='s', ms=5, label='End'),
    ]
    ax.legend(handles=handles, fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_compare(csv_paths: list[tuple], dataset: str,
                 zoom: bool, out_path: str) -> None:
    """Side-by-side comparison. csv_paths: list of (tag, csv_path)."""

    n = len(csv_paths)
    if n == 0:
        return
    fig, axes = plt.subplots(1, n, figsize=(4*n, 4.5), squeeze=False)
    axes = axes[0]
    view = 'zoom' if zoom else 'full'
    fig.suptitle(f'Dataset: {dataset}  |  {view}', fontsize=10, y=1.01)

    for ax, (tag, csv_path) in zip(axes, csv_paths):
        if not os.path.exists(csv_path):
            ax.set_visible(False)
            continue
        rows = _load_csv(csv_path)
        _setup_ax(ax, tag, board=not zoom)
        _draw_paths_on_ax(ax, rows, alpha_air=0.2)
        if zoom:
            xlim, ylim = _auto_zoom(rows)
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
        _info_box(ax, rows, dataset)

    axes[-1].legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130, bbox_inches='tight')
    plt.close(fig)
