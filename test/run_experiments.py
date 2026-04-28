"""
test/run_experiments.py — Headless A/B experiment matrix runner.

Runs Experiments A–D from the debugging matrix against all raw datasets.
Outputs per-run CSVs and a summary table to test/process/.

Usage (from project root):
    python -m test.run_experiments

Rounds:
    0 = baseline  (default config)
    A = rigid_body_sign = -1
    B = uwb_vel_stroke_gate = True
    C = acc_spike_clamp_enabled = True, acc_spike_clamp_ms2 = 7.0
    C2= acc_spike_clamp_ms2 = 5.0
"""

import csv
import math
import os
import sys

# ── path setup ────────────────────────────────────────────────────────────────
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tuning.headless_runner import run_dataset
from tuning.config_patcher  import patch_cfg, reset_cfg

# ── directories ───────────────────────────────────────────────────────────────
RAW_DIR  = os.path.join(os.path.dirname(__file__), 'raw')
OUT_DIR  = os.path.join(os.path.dirname(__file__), 'process')
os.makedirs(OUT_DIR, exist_ok=True)

# ── experiment matrix ─────────────────────────────────────────────────────────
# Each entry: (round_tag, patch_overrides)
EXPERIMENTS = [
    ('baseline',    {}),
    ('rbsign_minus', {'imu.rigid_body_sign': -1}),
    ('uwbvel_gate',  {'fusion_eskf.uwb_vel_stroke_gate': True}),
    ('accclamp7',    {'fusion_eskf.acc_spike_clamp_enabled': True,
                      'fusion_eskf.acc_spike_clamp_ms2': 7.0}),
    ('accclamp5',    {'fusion_eskf.acc_spike_clamp_enabled': True,
                      'fusion_eskf.acc_spike_clamp_ms2': 5.0}),
]

# Datasets to run per experiment (ordered by plan priority)
DATASETS = ['circle', 'abc', 'hline', 'vline', 'diagonal', 'square', 'triangle']

# ── CSV column schema (mirrors visualizer_log.csv) ────────────────────────────
_FIELDNAMES = [
    'ts_hw', 'source',
    'fused_x', 'fused_y',
    'uwb_x', 'uwb_y',
    'state', 'fusion_mode', 'stroke_id', 'stroke_active',
    'contact_raw', 'ink_written',
    # Kalman / fusion health
    'P_pos_trace', 'innovation_norm', 'r_scale', 'K_pos_diag',
    'stroke_age_s', 'age_cap_mult', 'kcap_eff',
    'b_a_x', 'b_a_y', 'b_a_norm',
    'b_p_x', 'b_p_y', 'b_p_mag',
    # Mode counters
    'frames_contact', 'frames_fast', 'frames_air', 'frames_static',
    'avg_K_contact', 'avg_K_fast', 'avg_K_air',
    'fast_arm_count', 'fast_burst_count',
    # UWB quality
    'uwb_residual_rms', 'uwb_accepted', 'uwb_rejected',
    # IMU curve
    'acc_hp_tip_x', 'acc_hp_tip_y', 'acc_hp_tip_mag',
    'acc_tip_x', 'acc_tip_y', 'acc_tip_mag',
    'omega_mag', 'alpha_mag', 'jerk',
]


def _row(ev: dict) -> dict:
    """Extract a CSV row dict from a fused event dict."""
    e   = ev.get('eskf', {})
    ba  = e.get('b_a', (0.0, 0.0))
    bp  = e.get('b_p', (0.0, 0.0))
    hp  = e.get('acc_hp_tip', (0.0, 0.0))   # from imu passthrough in eskf emit
    tip = e.get('acc_tip',    (0.0, 0.0))
    om  = e.get('omega_world_mag', 0.0)
    al  = e.get('alpha_world_mag', 0.0)

    # acc_hp_tip / acc_tip come from the imu event stored on the fused dict
    imu_ref = ev.get('_imu_ev', {})
    hp_raw  = imu_ref.get('acc_board_hp_tip', (0.0, 0.0))
    tip_raw = imu_ref.get('acc_board_tip',    (0.0, 0.0))
    om_raw  = imu_ref.get('omega_world', (0.0, 0.0, 0.0))
    al_raw  = imu_ref.get('alpha_world', (0.0, 0.0, 0.0))
    jerk    = imu_ref.get('jerk', 0.0)
    hx, hy  = float(hp_raw[0]),  float(hp_raw[1])
    tx, ty  = float(tip_raw[0]), float(tip_raw[1])
    om_mag  = math.sqrt(sum(float(x)**2 for x in om_raw))
    al_mag  = math.sqrt(sum(float(x)**2 for x in al_raw))

    is_ink = (
        bool(ev.get('stroke_active', False)) and
        bool(ev.get('contact_raw', True)) and
        int(ev.get('stroke_id', 0)) != 0
    )

    from background.pipelines.config import cfg as _cfg
    mode_p_cap = (
        _cfg.fusion_eskf.modes.drawing_fast if e.get('drawing_fast') else _cfg.fusion_eskf.modes.drawing
    ).pos_gain_cap
    kcap_eff = min(e.get('age_cap_mult', 1.0) * mode_p_cap, 1.0)

    return {
        'ts_hw':          ev.get('ts_hw', 0),
        'source':         ev.get('source', ''),
        'fused_x':        f"{ev.get('fused_x', 0):.5f}",
        'fused_y':        f"{ev.get('fused_y', 0):.5f}",
        'uwb_x':          f"{ev.get('uwb_x', 0):.5f}",
        'uwb_y':          f"{ev.get('uwb_y', 0):.5f}",
        'state':          ev.get('state', ''),
        'fusion_mode':    ev.get('fusion_mode', ''),
        'stroke_id':      ev.get('stroke_id', 0),
        'stroke_active':  int(ev.get('stroke_active', False)),
        'contact_raw':    int(ev.get('contact_raw', True)),
        'ink_written':    int(is_ink),
        'P_pos_trace':    f"{e.get('P_pos_trace', 0):.5f}",
        'innovation_norm':f"{e.get('innovation_norm', 0):.4f}",
        'r_scale':        f"{e.get('r_scale', 1):.3f}",
        'K_pos_diag':     f"{e.get('K_pos_diag', 0):.5f}",
        'stroke_age_s':   f"{e.get('stroke_age_s', 0.0):.3f}",
        'age_cap_mult':   f"{e.get('age_cap_mult', 1.0):.4f}",
        'kcap_eff':       f"{kcap_eff:.5f}",
        'b_a_x':          f"{float(ba[0]):.5f}",
        'b_a_y':          f"{float(ba[1]):.5f}",
        'b_a_norm':       f"{e.get('b_a_norm', 0):.5f}",
        'b_p_x':          f"{float(bp[0]):.5f}",
        'b_p_y':          f"{float(bp[1]):.5f}",
        'b_p_mag':        f"{e.get('b_p_mag', 0):.5f}",
        'frames_contact': e.get('frames_contact', 0),
        'frames_fast':    e.get('frames_fast', 0),
        'frames_air':     e.get('frames_air', 0),
        'frames_static':  e.get('frames_static', 0),
        'avg_K_contact':  f"{e.get('avg_K_contact', 0):.5f}",
        'avg_K_fast':     f"{e.get('avg_K_fast', 0):.5f}",
        'avg_K_air':      f"{e.get('avg_K_air', 0):.5f}",
        'fast_arm_count': e.get('fast_arm_count', 0),
        'fast_burst_count':e.get('fast_burst_count', 0),
        'uwb_residual_rms':f"{e.get('uwb_residual_rms', 0):.5f}",
        'uwb_accepted':   e.get('uwb_accepted', 0),
        'uwb_rejected':   e.get('uwb_rejected', 0),
        'acc_hp_tip_x':   f"{hx:.5f}",
        'acc_hp_tip_y':   f"{hy:.5f}",
        'acc_hp_tip_mag': f"{math.hypot(hx, hy):.5f}",
        'acc_tip_x':      f"{tx:.5f}",
        'acc_tip_y':      f"{ty:.5f}",
        'acc_tip_mag':    f"{math.hypot(tx, ty):.5f}",
        'omega_mag':      f"{om_mag:.5f}",
        'alpha_mag':      f"{al_mag:.5f}",
        'jerk':           f"{float(jerk):.4f}",
    }


# ── stroke geometry metrics ───────────────────────────────────────────────────

def _ink_rows(results: list[dict]) -> list[dict]:
    return [r for r in results if r.get('stroke_active') and r.get('contact_raw', True)
            and r.get('stroke_id', 0) != 0]


def _per_stroke(results: list[dict]) -> dict[int, list[dict]]:
    strokes: dict[int, list[dict]] = {}
    for r in results:
        sid = r.get('stroke_id', 0)
        if sid and r.get('stroke_active') and r.get('contact_raw', True):
            strokes.setdefault(sid, []).append(r)
    return strokes


def _bbox(pts):
    if not pts:
        return 0.0, 0.0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (max(xs) - min(xs)) * 1000, (max(ys) - min(ys)) * 1000   # mm


def _closure(pts):
    if len(pts) < 2:
        return 0.0
    dx = pts[-1][0] - pts[0][0]
    dy = pts[-1][1] - pts[0][1]
    return math.hypot(dx, dy) * 1000   # mm


def _straightness(pts):
    if len(pts) < 2:
        return 1.0
    dx = pts[-1][0] - pts[0][0]
    dy = pts[-1][1] - pts[0][1]
    chord = math.hypot(dx, dy)
    if chord == 0:
        return 0.0
    arc = sum(
        math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
        for i in range(1, len(pts))
    )
    return min(chord / arc, 1.0) if arc > 0 else 1.0


def _path_length(pts):
    return sum(
        math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
        for i in range(1, len(pts))
    ) * 1000   # mm


def _pct(vals, p):
    if not vals:
        return 0.0
    vals = sorted(vals)
    idx = int(len(vals) * p / 100)
    return vals[min(idx, len(vals)-1)]


def _stroke_metrics(results: list[dict], label: str) -> dict:
    strokes = _per_stroke(results)
    all_fused = [(r['fused_x'], r['fused_y']) for r in results
                 if r.get('stroke_active') and r.get('contact_raw', True) and r.get('stroke_id', 0)]

    eskf_last = results[-1].get('eskf', {}) if results else {}
    fc = eskf_last.get('frames_contact', 0)
    ff = eskf_last.get('frames_fast',    0)
    fa = eskf_last.get('frames_air',     0)
    fs = eskf_last.get('frames_static',  0)
    total = fc + ff + fa + fs or 1

    ink = _ink_rows(results)
    hp_mags  = []
    tip_mags = []
    om_mags  = []
    al_mags  = []
    jerks    = []
    ba_norms = []
    bp_mags  = []
    innov    = []

    for r in results:
        e = r.get('eskf', {})
        imu = r.get('_imu_ev', {})
        hp  = imu.get('acc_board_hp_tip', (0.0, 0.0))
        tip = imu.get('acc_board_tip', (0.0, 0.0))
        om  = imu.get('omega_world', (0.0, 0.0, 0.0))
        al  = imu.get('alpha_world', (0.0, 0.0, 0.0))
        if r.get('stroke_active'):
            hp_mags.append(math.hypot(float(hp[0]), float(hp[1])))
            tip_mags.append(math.hypot(float(tip[0]), float(tip[1])))
            om_mags.append(math.sqrt(sum(float(x)**2 for x in om)))
            al_mags.append(math.sqrt(sum(float(x)**2 for x in al)))
            jerks.append(float(imu.get('jerk', 0.0)))
            ba_norms.append(e.get('b_a_norm', 0.0))
            bp_mags.append(e.get('b_p_mag', 0.0))
            innov.append(e.get('innovation_norm', 0.0))

    stroke_rows = []
    for sid, rows in sorted(strokes.items()):
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
        })

    return {
        'label':         label,
        'total_events':  len(results),
        'ink_events':    len(ink),
        'strokes':       len(strokes),
        'mode_pct_fast': round(100 * ff / total, 1),
        'mode_pct_draw': round(100 * fc / total, 1),
        'mode_pct_air':  round(100 * fa / total, 1),
        'uwb_accepted':  eskf_last.get('uwb_accepted', 0),
        'uwb_rejected':  eskf_last.get('uwb_rejected', 0),
        # IMU quality (ink frames only)
        'acc_hp_tip_mean': round(sum(hp_mags)/len(hp_mags), 3) if hp_mags else 0,
        'acc_hp_tip_p95':  round(_pct(hp_mags, 95), 3),
        'acc_hp_tip_max':  round(max(hp_mags), 3) if hp_mags else 0,
        'acc_tip_mean':    round(sum(tip_mags)/len(tip_mags), 3) if tip_mags else 0,
        'acc_tip_p95':     round(_pct(tip_mags, 95), 3),
        'acc_tip_max':     round(max(tip_mags), 3) if tip_mags else 0,
        'omega_mean':      round(sum(om_mags)/len(om_mags), 3) if om_mags else 0,
        'omega_p95':       round(_pct(om_mags, 95), 3),
        'omega_max':       round(max(om_mags), 3) if om_mags else 0,
        'alpha_mean':      round(sum(al_mags)/len(al_mags), 3) if al_mags else 0,
        'alpha_p95':       round(_pct(al_mags, 95), 3),
        'alpha_max':       round(max(al_mags), 3) if al_mags else 0,
        'jerk_mean':       round(sum(jerks)/len(jerks), 2) if jerks else 0,
        'jerk_p95':        round(_pct(jerks, 95), 2),
        'jerk_max':        round(max(jerks), 2) if jerks else 0,
        'b_a_norm_mean':   round(sum(ba_norms)/len(ba_norms), 5) if ba_norms else 0,
        'b_a_norm_p95':    round(_pct(ba_norms, 95), 5),
        'b_a_norm_max':    round(max(ba_norms), 5) if ba_norms else 0,
        'b_p_mag_mean':    round(sum(bp_mags)/len(bp_mags), 4) if bp_mags else 0,
        'b_p_mag_p95':     round(_pct(bp_mags, 95), 4),
        'b_p_mag_max':     round(max(bp_mags), 4) if bp_mags else 0,
        'innov_mean':      round(sum(innov)/len(innov), 4) if innov else 0,
        'innov_p95':       round(_pct(innov, 95), 4),
        'innov_max':       round(max(innov), 4) if innov else 0,
        'stroke_detail':   stroke_rows,
    }


# ── run one experiment+dataset, write CSV, return metrics ─────────────────────

def run_one(exp_tag: str, dataset: str, overrides: dict) -> dict:
    raw_path = os.path.join(RAW_DIR, f'{dataset}.csv')
    if not os.path.exists(raw_path):
        print(f'  [SKIP] {raw_path} not found')
        return {}

    reset_cfg()
    if overrides:
        patch_cfg(overrides)

    results = run_dataset(raw_path)

    # Attach _imu_ev to every result so metrics can read IMU fields.
    # headless_runner doesn't do this by default — rebuild from annotation.
    # (The fused dict from ESKF._emit already carries eskf sub-dict with
    # diagnostics; IMU fields are not forwarded. We patch here post-hoc.)
    # NOTE: imu fields on fused dict that ARE present: source, state, etc.
    # IMU-specific sensor fields (acc_board_hp_tip etc.) are NOT in fused.
    # We re-run parse+preprocess to re-attach them.
    from tuning.headless_runner import parse_dataset
    from background.pipelines.preprocess.imu import IMUPreprocessor
    from background.pipelines.preprocess.contact import ContactStateDetector

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

    # Match fused IMU events to annotated by index (both pipelines ran same events)
    imu_results = [r for r in results if r.get('source') == 'IMU']
    for i, r in enumerate(imu_results):
        if i < len(imu_annotated):
            r['_imu_ev'] = imu_annotated[i]
        else:
            r['_imu_ev'] = {}

    # Write per-run CSV
    out_csv = os.path.join(OUT_DIR, f'{exp_tag}_{dataset}.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=_FIELDNAMES, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(_row(r))

    label = f'{exp_tag}/{dataset}'
    metrics = _stroke_metrics(results, label)
    print(f'  [{label}] strokes={metrics["strokes"]}  '
          f'fast%={metrics["mode_pct_fast"]}  '
          f'ink={metrics["ink_events"]}  '
          f'acc_hp_p95={metrics["acc_hp_tip_p95"]} m/s²')
    return metrics


# ── summary writer ────────────────────────────────────────────────────────────

_SUMMARY_FIELDS = [
    'exp', 'dataset', 'strokes', 'ink_events',
    'mode_pct_fast', 'mode_pct_draw', 'mode_pct_air',
    'uwb_accepted', 'uwb_rejected',
    'acc_hp_tip_mean', 'acc_hp_tip_p95', 'acc_hp_tip_max',
    'acc_tip_mean', 'acc_tip_p95', 'acc_tip_max',
    'omega_mean', 'omega_p95', 'omega_max',
    'alpha_mean', 'alpha_p95', 'alpha_max',
    'jerk_mean', 'jerk_p95', 'jerk_max',
    'b_a_norm_mean', 'b_a_norm_p95', 'b_a_norm_max',
    'b_p_mag_mean', 'b_p_mag_p95', 'b_p_mag_max',
    'innov_mean', 'innov_p95', 'innov_max',
]

_STROKE_FIELDS = [
    'exp', 'dataset', 'sid', 'n_pts',
    'width_mm', 'height_mm', 'path_mm', 'straightness', 'closure_mm',
]


def write_summaries(all_metrics: list[dict]):
    summary_path = os.path.join(OUT_DIR, 'summary.csv')
    strokes_path = os.path.join(OUT_DIR, 'strokes.csv')

    with open(summary_path, 'w', newline='', encoding='utf-8') as sf, \
         open(strokes_path, 'w', newline='', encoding='utf-8') as stf:

        sw  = csv.DictWriter(sf,  fieldnames=_SUMMARY_FIELDS,  extrasaction='ignore')
        stw = csv.DictWriter(stf, fieldnames=_STROKE_FIELDS, extrasaction='ignore')
        sw.writeheader()
        stw.writeheader()

        for m in all_metrics:
            if not m:
                continue
            label = m['label']
            exp, dataset = label.split('/', 1)
            row = dict(m, exp=exp, dataset=dataset)
            sw.writerow(row)
            for s in m.get('stroke_detail', []):
                stw.writerow(dict(s, exp=exp, dataset=dataset))

    print(f'\nSummary -> {summary_path}')
    print(f'Strokes -> {strokes_path}')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    all_metrics = []

    for exp_tag, overrides in EXPERIMENTS:
        print(f'\n=== Experiment: {exp_tag} ===')
        if overrides:
            print(f'    Overrides: {overrides}')
        for dataset in DATASETS:
            m = run_one(exp_tag, dataset, overrides)
            if m:
                all_metrics.append(m)

    reset_cfg()
    write_summaries(all_metrics)
    print('\nDone.')


if __name__ == '__main__':
    main()
