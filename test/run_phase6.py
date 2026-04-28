"""
test/run_phase6.py — Phase 6 validation: rigid_body_sign=-1 + lever-arm sweep.

Experiments:
  p6_baseline         sign=+1, lever=0.110  (reference, matches existing baseline)
  p6_rbsign           sign=-1, lever=0.110  (Exp-A winner)
  p6_rbsign_110       sign=-1, lever=0.110  (same as above, explicit)
  p6_rbsign_080       sign=-1, lever=0.080
  p6_rbsign_060       sign=-1, lever=0.060
  p6_rbsign_000       sign=-1, lever=0.000  (pure no-rigid-body reference)

Datasets: abc, circle, hline, vline

Outputs -> test/process/  (CSVs) and test/process/plots/  (PNGs)

Usage (from project root):
  python test/run_phase6.py
"""

import csv
import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tuning.headless_runner import run_dataset, parse_dataset
from tuning.config_patcher  import patch_cfg, reset_cfg
from background.pipelines.preprocess.imu     import IMUPreprocessor
from background.pipelines.preprocess.contact import ContactStateDetector

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

RAW_DIR  = os.path.join(os.path.dirname(__file__), 'raw')
OUT_DIR  = os.path.join(os.path.dirname(__file__), 'process')
PLOT_DIR = os.path.join(OUT_DIR, 'plots')
os.makedirs(OUT_DIR,  exist_ok=True)
os.makedirs(PLOT_DIR, exist_ok=True)

BOARD_W = 1.25
BOARD_H = 1.24

DATASETS = ['abc', 'circle', 'hline', 'vline']

EXPERIMENTS = [
    ('p6_baseline',   {'imu.rigid_body_sign':  1, 'marker.r_imu_body_m': (0.110, 0.0, 0.0)}),
    ('p6_rbsign_110', {'imu.rigid_body_sign': -1, 'marker.r_imu_body_m': (0.110, 0.0, 0.0)}),
    ('p6_rbsign_080', {'imu.rigid_body_sign': -1, 'marker.r_imu_body_m': (0.080, 0.0, 0.0)}),
    ('p6_rbsign_060', {'imu.rigid_body_sign': -1, 'marker.r_imu_body_m': (0.060, 0.0, 0.0)}),
    ('p6_rbsign_000', {'imu.rigid_body_sign': -1, 'marker.r_imu_body_m': (0.000, 0.0, 0.0)}),
]

# ── CSV schema ────────────────────────────────────────────────────────────────
_FIELDNAMES = [
    'ts_hw', 'source',
    'fused_x', 'fused_y', 'uwb_x', 'uwb_y',
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


def _f(d, k, default=0.0):
    try:
        return float(d[k])
    except Exception:
        return default


def _i(d, k, default=0):
    try:
        return int(d[k])
    except Exception:
        return default


# ── attach imu_ev to fused results ───────────────────────────────────────────

def _attach_imu(results, raw_path):
    imu_prep = IMUPreprocessor()
    contact  = ContactStateDetector()
    events   = parse_dataset(raw_path)
    annotated = []
    for ev in events:
        if ev['sensor'] == 'IMU':
            p = imu_prep.process_one(ev)
            if p is not None:
                annotated.append(contact.process_one(p))
    imu_results = [r for r in results if r.get('source') == 'IMU']
    for i, r in enumerate(imu_results):
        r['_imu_ev'] = annotated[i] if i < len(annotated) else {}


def _row(ev):
    e   = ev.get('eskf', {})
    ba  = e.get('b_a', (0.0, 0.0))
    bp  = e.get('b_p', (0.0, 0.0))
    imu = ev.get('_imu_ev', {})
    hp  = imu.get('acc_board_hp_tip', (0.0, 0.0))
    tip = imu.get('acc_board_tip',    (0.0, 0.0))
    om  = imu.get('omega_world', (0.0, 0.0, 0.0))
    al  = imu.get('alpha_world', (0.0, 0.0, 0.0))
    jerk = imu.get('jerk', 0.0)
    hx, hy = float(hp[0]),  float(hp[1])
    tx, ty = float(tip[0]), float(tip[1])
    om_mag = math.sqrt(sum(float(x)**2 for x in om))
    al_mag = math.sqrt(sum(float(x)**2 for x in al))

    from background.pipelines.config import cfg as _cfg
    mode_p_cap = (
        _cfg.fusion_eskf.modes.drawing_fast if e.get('drawing_fast')
        else _cfg.fusion_eskf.modes.drawing
    ).pos_gain_cap
    kcap = min(e.get('age_cap_mult', 1.0) * mode_p_cap, 1.0)

    is_ink = (
        bool(ev.get('stroke_active')) and
        bool(ev.get('contact_raw', True)) and
        int(ev.get('stroke_id', 0)) != 0
    )
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
        'stroke_active':   int(bool(ev.get('stroke_active'))),
        'contact_raw':     int(bool(ev.get('contact_raw', True))),
        'ink_written':     int(is_ink),
        'P_pos_trace':     f"{e.get('P_pos_trace', 0):.5f}",
        'innovation_norm': f"{e.get('innovation_norm', 0):.4f}",
        'r_scale':         f"{e.get('r_scale', 1):.3f}",
        'K_pos_diag':      f"{e.get('K_pos_diag', 0):.5f}",
        'stroke_age_s':    f"{e.get('stroke_age_s', 0.0):.3f}",
        'age_cap_mult':    f"{e.get('age_cap_mult', 1.0):.4f}",
        'kcap_eff':        f"{kcap:.5f}",
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


# ── geometry helpers ──────────────────────────────────────────────────────────

def _ink_pts(results):
    return [
        (_f(r, 'fused_x') if isinstance(r, dict) and 'fused_x' in r
         else float(r.get('fused_x', 0)),
         _f(r, 'fused_y') if isinstance(r, dict) and 'fused_y' in r
         else float(r.get('fused_y', 0)))
        for r in results
        if (bool(r.get('stroke_active')) and bool(r.get('contact_raw', True))
            and int(r.get('stroke_id', 0)) != 0)
    ]


def _bbox_mm(pts):
    if not pts:
        return 0.0, 0.0
    xs, ys = zip(*pts)
    return (max(xs)-min(xs))*1000, (max(ys)-min(ys))*1000


def _path_mm(pts):
    return sum(
        math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
        for i in range(1, len(pts))
    ) * 1000


def _closure_mm(pts):
    if len(pts) < 2:
        return 0.0
    return math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1]) * 1000


def _straightness(pts):
    if len(pts) < 2:
        return 1.0
    chord = math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1])
    arc   = _path_mm(pts) / 1000
    return min(chord/arc, 1.0) if arc > 0 else 1.0


def _pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(int(len(s)*p/100), len(s)-1)]


def _mode_pct(results):
    if not results:
        return 0, 0
    last = results[-1]
    e    = last.get('eskf', {}) if isinstance(last, dict) else {}
    fc = e.get('frames_contact', 0)
    ff = e.get('frames_fast', 0)
    fa = e.get('frames_air', 0)
    fs = e.get('frames_static', 0)
    total = fc + ff + fa + fs or 1
    return 100*ff/total, 100*fc/total


def _metrics(results, exp_tag, dataset):
    ink    = _ink_pts(results)
    fp, dp = _mode_pct(results)
    innov  = [r.get('eskf', {}).get('innovation_norm', 0.0)
              for r in results if r.get('stroke_active')]
    bp_mag = [r.get('eskf', {}).get('b_p_mag', 0.0)
              for r in results if r.get('stroke_active')]
    last_e = results[-1].get('eskf', {}) if results else {}

    # per-stroke
    by_sid: dict[int, list] = {}
    for r in results:
        if r.get('stroke_active') and r.get('contact_raw', True) and r.get('stroke_id', 0):
            sid = r.get('stroke_id', 0)
            by_sid.setdefault(sid, []).append(
                (float(r.get('fused_x', 0)), float(r.get('fused_y', 0)))
            )

    strokes = []
    for sid, pts in sorted(by_sid.items()):
        w, h = _bbox_mm(pts)
        strokes.append({
            'sid': sid, 'n': len(pts),
            'w': round(w, 1), 'h': round(h, 1),
            'path': round(_path_mm(pts), 1),
            'straight': round(_straightness(pts), 3),
            'closure': round(_closure_mm(pts), 1),
        })

    return {
        'exp': exp_tag, 'dataset': dataset,
        'fast_pct':    round(fp, 1),
        'draw_pct':    round(dp, 1),
        'ink_pts':     len(ink),
        'innov_p95':   round(_pct(innov, 95)*1000, 1),
        'bp_p95_mm':   round(_pct(bp_mag, 95)*1000, 1),
        'uwb_acc':     last_e.get('uwb_accepted', 0),
        'uwb_rej':     last_e.get('uwb_rejected', 0),
        'strokes':     strokes,
    }


# ── run one ───────────────────────────────────────────────────────────────────

def run_one(exp_tag, dataset, overrides):
    raw_path = os.path.join(RAW_DIR, f'{dataset}.csv')
    if not os.path.exists(raw_path):
        print(f'  [SKIP] {raw_path}')
        return None, []

    reset_cfg()
    patch_cfg(overrides)

    results = run_dataset(raw_path)
    _attach_imu(results, raw_path)

    # write CSV
    out_csv = os.path.join(OUT_DIR, f'{exp_tag}_{dataset}.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=_FIELDNAMES, extrasaction='ignore')
        w.writeheader()
        for r in results:
            w.writerow(_row(r))

    m = _metrics(results, exp_tag, dataset)
    print(f'  [{exp_tag}/{dataset}]  fast%={m["fast_pct"]}  '
          f'innov_p95={m["innov_p95"]} mm  bp_p95={m["bp_p95_mm"]} mm  '
          f'ink={m["ink_pts"]}')
    for s in m['strokes']:
        print(f'    stroke {s["sid"]}: {s["w"]:.0f}x{s["h"]:.0f} mm  '
              f'path={s["path"]:.0f} mm  closure={s["closure"]:.0f} mm  '
              f'straight={s["straight"]:.3f}')

    return m, results


# ── plotting ──────────────────────────────────────────────────────────────────

_COL_FUSED = '#1a1a1a'
_COL_AIR   = '#888888'
_COL_UWB   = '#e07b00'

STROKE_COLORS = ['#2244cc', '#cc2244', '#22aa44', '#aa6600', '#6622cc']


def _extract(rows, key, default=0.0):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, default)))
        except Exception:
            out.append(default)
    return out


def _shade_ink(ax, ts, ink_flags):
    in_ink, t0 = False, 0.0
    for t, i in zip(ts, ink_flags):
        if i and not in_ink:
            t0, in_ink = t, True
        elif not i and in_ink:
            ax.axvspan(t0, t, alpha=0.09, color='black')
            in_ink = False
    if in_ink:
        ax.axvspan(t0, ts[-1], alpha=0.09, color='black')


def _legend():
    return [
        Line2D([0], [0], color=_COL_FUSED, lw=1.2, label='Fused ink'),
        Line2D([0], [0], color=_COL_AIR,   lw=0, marker='o', ms=2.5, alpha=0.4, label='Fused air'),
        Line2D([0], [0], color=_COL_UWB,   lw=0, marker='o', ms=2.5, alpha=0.7, label='UWB'),
        Line2D([0], [0], color='#2266cc', lw=0, marker='o', ms=5, label='Start'),
        Line2D([0], [0], color='#cc2222', lw=0, marker='s', ms=5, label='End'),
    ]


def _draw_board(ax):
    ax.set_aspect('equal')
    ax.set_xlim(-0.02, BOARD_W+0.02)
    ax.set_ylim(-0.02, BOARD_H+0.02)
    rect = plt.Rectangle((0, 0), BOARD_W, BOARD_H,
                          fill=False, edgecolor='#cccccc', lw=1.0, ls='--')
    ax.add_patch(rect)


def _draw_paths(ax, rows_csv, zoom=False):
    """rows_csv: list of dicts from DictReader (strings)."""
    by_sid: dict[int, list] = {}
    uwb_x, uwb_y = [], []
    air_x, air_y = [], []

    for r in rows_csv:
        fx, fy = _f(r, 'fused_x'), _f(r, 'fused_y')
        ux, uy = _f(r, 'uwb_x'),   _f(r, 'uwb_y')
        is_ink = _i(r, 'ink_written') == 1
        sid    = _i(r, 'stroke_id')

        if is_ink:
            by_sid.setdefault(sid, []).append((fx, fy))
        else:
            air_x.append(fx); air_y.append(fy)

        uwb_x.append(ux); uwb_y.append(uy)

    if uwb_x:
        ax.scatter(uwb_x, uwb_y, s=1.2, c=_COL_UWB, alpha=0.4, linewidths=0, zorder=2)
    if air_x:
        ax.scatter(air_x, air_y, s=0.7, c=_COL_AIR, alpha=0.2, linewidths=0, zorder=3)

    for (sid, pts), col in zip(sorted(by_sid.items()), STROKE_COLORS):
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=col, lw=1.0, zorder=5)
        ax.scatter([xs[0]],  [ys[0]],  s=20, c='#2266cc', zorder=6, marker='o')
        ax.scatter([xs[-1]], [ys[-1]], s=20, c='#cc2222', zorder=6, marker='s')

    if zoom and by_sid:
        all_pts = [(x, y) for pts in by_sid.values() for x, y in pts]
        xs2, ys2 = zip(*all_pts)
        m = 0.04
        ax.set_xlim(min(xs2)-m, max(xs2)+m)
        ax.set_ylim(min(ys2)-m, max(ys2)+m)


def _ann(ax, rows_csv, dataset, metrics):
    lines = [f'ink: {metrics["ink_pts"]}  fast%: {metrics["fast_pct"]}']
    for s in metrics['strokes']:
        lines.append(f'S{s["sid"]}: {s["w"]:.0f}x{s["h"]:.0f}mm  path={s["path"]:.0f}')
        if dataset in ('circle', 'square'):
            lines.append(f'  closure={s["closure"]:.0f}mm')
        elif dataset in ('hline', 'vline'):
            lines.append(f'  straight={s["straight"]:.3f}')
    lines.append(f'innov_p95={metrics["innov_p95"]}mm')
    lines.append(f'bp_p95={metrics["bp_p95_mm"]}mm')
    ax.text(0.02, 0.98, '\n'.join(lines), transform=ax.transAxes,
            fontsize=5.5, va='top', ha='left', family='monospace',
            bbox=dict(boxstyle='round,pad=0.25', fc='white', alpha=0.8, lw=0))


def plot_run(exp_tag, dataset, rows_csv, metrics):
    for zoom in (True, False):
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.set_aspect('equal')
        ax.tick_params(labelsize=7)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        view = 'zoom' if zoom else 'full'
        ax.set_title(f'{exp_tag} / {dataset}  [{view}]', fontsize=9, pad=4)
        if not zoom:
            _draw_board(ax)
        _draw_paths(ax, rows_csv, zoom=zoom)
        _ann(ax, rows_csv, dataset, metrics)
        ax.legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
        fig.tight_layout()
        out = os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_{view}.png')
        fig.savefig(out, dpi=130)
        plt.close(fig)


def plot_imu_diag(exp_tag, dataset, rows_csv):
    imu_rows = [r for r in rows_csv if r.get('source') == 'IMU']
    if not imu_rows:
        return
    ts   = [(_f(r, 'ts_hw')/1e6) for r in imu_rows]
    t0   = ts[0]; ts = [t-t0 for t in ts]
    ink  = [_i(r, 'ink_written') for r in imu_rows]
    hp   = [_f(r, 'acc_hp_tip_mag') for r in imu_rows]
    tip  = [_f(r, 'acc_tip_mag')    for r in imu_rows]
    om   = [_f(r, 'omega_mag')      for r in imu_rows]
    al   = [_f(r, 'alpha_mag')      for r in imu_rows]
    jk   = [_f(r, 'jerk')           for r in imu_rows]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f'IMU diagnostics  |  {exp_tag} / {dataset}', fontsize=10)

    axes[0].plot(ts, hp,  lw=0.7, color='#2244cc', label='acc_hp_tip')
    axes[0].plot(ts, tip, lw=0.7, color='#66aaff', alpha=0.5, label='acc_tip')
    axes[0].axhline(7.0, color='red',    lw=0.7, ls='--', alpha=0.5, label='7 m/s²')
    axes[0].axhline(5.0, color='orange', lw=0.7, ls='--', alpha=0.5, label='5 m/s²')
    axes[0].set_ylabel('m/s²', fontsize=7)
    axes[0].set_title('Accel magnitude (HPF tip vs raw tip)', fontsize=8)
    axes[0].legend(fontsize=6, loc='upper right')
    _shade_ink(axes[0], ts, ink)

    axes[1].plot(ts, om, lw=0.7, color='#cc6600')
    axes[1].set_ylabel('rad/s', fontsize=7)
    axes[1].set_title('Angular velocity |omega|', fontsize=8)
    _shade_ink(axes[1], ts, ink)

    axes[2].plot(ts, al, lw=0.7, color='#aa2200')
    axes[2].set_ylabel('rad/s²', fontsize=7)
    axes[2].set_title('Angular acceleration |alpha|', fontsize=8)
    _shade_ink(axes[2], ts, ink)

    axes[3].plot(ts, jk, lw=0.7, color='#006622')
    axes[3].set_ylabel('m/s³', fontsize=7)
    axes[3].set_xlabel('time (s)', fontsize=7)
    axes[3].set_title('Jerk', fontsize=8)
    _shade_ink(axes[3], ts, ink)

    for ax in axes:
        ax.tick_params(labelsize=7)
        ax.grid(True, lw=0.3, alpha=0.5)

    fig.tight_layout()
    fig.savefig(os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_imu_diag.png'), dpi=130)
    plt.close(fig)


def plot_fusion_health(exp_tag, dataset, rows_csv):
    imu_rows = [r for r in rows_csv if r.get('source') == 'IMU']
    if not imu_rows:
        return
    ts   = [(_f(r, 'ts_hw')/1e6) for r in imu_rows]
    t0   = ts[0]; ts = [t-t0 for t in ts]
    ink  = [_i(r, 'ink_written') for r in imu_rows]
    kp   = [_f(r, 'K_pos_diag')     for r in imu_rows]
    ke   = [_f(r, 'kcap_eff')       for r in imu_rows]
    inn  = [_f(r, 'innovation_norm') for r in imu_rows]
    bp   = [_f(r, 'b_p_mag')        for r in imu_rows]
    ba   = [_f(r, 'b_a_norm')       for r in imu_rows]
    age  = [_f(r, 'stroke_age_s')   for r in imu_rows]
    mult = [_f(r, 'age_cap_mult')   for r in imu_rows]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f'Fusion health  |  {exp_tag} / {dataset}', fontsize=10)

    axes[0].plot(ts, kp, lw=0.7, color='#2244aa', label='K_pos_diag')
    axes[0].plot(ts, ke, lw=0.9, color='#cc2222', ls='--', label='kcap_eff')
    axes[0].set_ylabel('K', fontsize=7); axes[0].set_title('Kalman gain vs cap', fontsize=8)
    axes[0].legend(fontsize=6); _shade_ink(axes[0], ts, ink)

    axes[1].plot(ts, inn, lw=0.7, color='#cc6600')
    axes[1].set_ylabel('m', fontsize=7); axes[1].set_title('Innovation norm', fontsize=8)
    _shade_ink(axes[1], ts, ink)

    axes[2].plot(ts, bp, lw=0.7, color='#006688', label='b_p_mag')
    axes[2].plot(ts, ba, lw=0.7, color='#aa4400', alpha=0.6, label='b_a_norm')
    axes[2].set_ylabel('m', fontsize=7); axes[2].set_title('b_p and b_a', fontsize=8)
    axes[2].legend(fontsize=6); _shade_ink(axes[2], ts, ink)

    axes[3].plot(ts, age, lw=0.8, color='#444444', label='stroke_age_s')
    ax3b = axes[3].twinx()
    ax3b.plot(ts, mult, lw=0.8, color='#cc2222', ls='--', label='age_cap_mult')
    ax3b.set_ylabel('mult', fontsize=7, color='#cc2222')
    ax3b.tick_params(labelsize=7, colors='#cc2222')
    axes[3].set_ylabel('s', fontsize=7); axes[3].set_title('Stroke age / cap mult', fontsize=8)
    axes[3].set_xlabel('time (s)', fontsize=7)
    axes[3].legend(fontsize=6, loc='upper left')
    ax3b.legend(fontsize=6, loc='upper right')
    _shade_ink(axes[3], ts, ink)

    for ax in axes:
        ax.tick_params(labelsize=7)
        ax.grid(True, lw=0.3, alpha=0.5)

    fig.tight_layout()
    fig.savefig(os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_fusion_health.png'), dpi=130)
    plt.close(fig)


def plot_compare(dataset):
    """Side-by-side comparison of all Phase-6 experiments for one dataset."""
    n = len(EXPERIMENTS)
    for zoom in (True, False):
        fig, axes = plt.subplots(1, n, figsize=(4*n, 4.8), squeeze=False)
        axes = axes[0]
        view = 'zoom' if zoom else 'full'
        fig.suptitle(f'Phase-6 comparison  |  {dataset}  [{view}]', fontsize=10, y=1.01)

        for ax, (exp_tag, _) in zip(axes, EXPERIMENTS):
            csv_path = os.path.join(OUT_DIR, f'{exp_tag}_{dataset}.csv')
            if not os.path.exists(csv_path):
                ax.set_visible(False); continue
            import csv as _csv
            rows_csv = list(_csv.DictReader(open(csv_path, encoding='utf-8')))
            ax.set_aspect('equal')
            ax.set_title(exp_tag.replace('p6_', ''), fontsize=8, pad=3)
            ax.tick_params(labelsize=6)
            if not zoom:
                _draw_board(ax)
            _draw_paths(ax, rows_csv, zoom=zoom)

        axes[-1].legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
        fig.tight_layout()
        fig.savefig(os.path.join(PLOT_DIR, f'p6_compare_{dataset}_{view}.png'),
                    dpi=130, bbox_inches='tight')
        plt.close(fig)


# ── lever-arm sweep summary plot ─────────────────────────────────────────────

def plot_lever_sweep(all_metrics):
    """Line chart: key metrics vs lever-arm magnitude for sign=-1 experiments."""
    levers = {'p6_rbsign_110': 0.110, 'p6_rbsign_080': 0.080,
               'p6_rbsign_060': 0.060, 'p6_rbsign_000': 0.000}

    for dataset in DATASETS:
        rows = [m for m in all_metrics if m['dataset'] == dataset
                and m['exp'] in levers]
        if not rows:
            continue
        rows.sort(key=lambda m: levers[m['exp']])
        lv = [levers[m['exp']] for m in rows]

        fig, axes = plt.subplots(2, 2, figsize=(9, 6))
        fig.suptitle(f'Lever-arm sweep (sign=-1)  |  {dataset}', fontsize=10)

        def _first_stroke(m, key):
            if m['strokes']:
                return m['strokes'][0].get(key, 0)
            return 0

        axes[0][0].plot(lv, [m['fast_pct'] for m in rows], 'o-', color='#cc2222')
        axes[0][0].plot(lv, [m['draw_pct'] for m in rows], 's-', color='#2244cc')
        axes[0][0].set_title('Mode % (red=fast, blue=draw)', fontsize=8)
        axes[0][0].set_ylabel('%'); axes[0][0].set_xlabel('lever (m)')

        axes[0][1].plot(lv, [m['innov_p95'] for m in rows], 'o-', color='#cc6600')
        axes[0][1].set_title('Innovation p95 (mm)', fontsize=8)
        axes[0][1].set_ylabel('mm'); axes[0][1].set_xlabel('lever (m)')

        axes[1][0].plot(lv, [m['bp_p95_mm'] for m in rows], 'o-', color='#006688')
        axes[1][0].set_title('b_p p95 (mm)', fontsize=8)
        axes[1][0].set_ylabel('mm'); axes[1][0].set_xlabel('lever (m)')

        if dataset in ('circle', 'square'):
            axes[1][1].plot(lv, [_first_stroke(m, 'closure') for m in rows], 'o-', color='#226622')
            axes[1][1].set_title('Stroke closure (mm) — lower better', fontsize=8)
        elif dataset in ('hline', 'vline'):
            axes[1][1].plot(lv, [_first_stroke(m, 'straight') for m in rows], 'o-', color='#226622')
            axes[1][1].set_title('Straightness — higher better', fontsize=8)
        else:
            axes[1][1].plot(lv, [_first_stroke(m, 'path') for m in rows], 'o-', color='#226622')
            axes[1][1].set_title('Stroke-1 path length (mm)', fontsize=8)
        axes[1][1].set_xlabel('lever (m)')

        for ax in axes.flat:
            ax.tick_params(labelsize=7)
            ax.grid(True, lw=0.3, alpha=0.5)

        fig.tight_layout()
        fig.savefig(os.path.join(PLOT_DIR, f'p6_lever_sweep_{dataset}.png'), dpi=130)
        plt.close(fig)


# ── summary CSV ───────────────────────────────────────────────────────────────

def write_summary(all_metrics):
    path = os.path.join(OUT_DIR, 'p6_summary.csv')
    fields = ['exp', 'dataset', 'fast_pct', 'draw_pct', 'ink_pts',
              'innov_p95', 'bp_p95_mm', 'uwb_acc', 'uwb_rej']
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for m in all_metrics:
            w.writerow(m)

    spath = os.path.join(OUT_DIR, 'p6_strokes.csv')
    sfields = ['exp', 'dataset', 'sid', 'n', 'w', 'h', 'path', 'straight', 'closure']
    with open(spath, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=sfields, extrasaction='ignore')
        w.writeheader()
        for m in all_metrics:
            for s in m['strokes']:
                w.writerow(dict(s, exp=m['exp'], dataset=m['dataset']))

    print(f'\nSummary -> {path}')
    print(f'Strokes -> {spath}')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    import csv as _csv
    all_metrics = []

    for exp_tag, overrides in EXPERIMENTS:
        print(f'\n=== {exp_tag} ===')
        print(f'    {overrides}')
        for dataset in DATASETS:
            m, results = run_one(exp_tag, dataset, overrides)
            if m is None:
                continue
            all_metrics.append(m)

            csv_path = os.path.join(OUT_DIR, f'{exp_tag}_{dataset}.csv')
            rows_csv = list(_csv.DictReader(open(csv_path, encoding='utf-8')))

            plot_run(exp_tag, dataset, rows_csv, m)
            plot_imu_diag(exp_tag, dataset, rows_csv)
            plot_fusion_health(exp_tag, dataset, rows_csv)

    # comparison grids
    for dataset in DATASETS:
        print(f'  Comparison grid: {dataset}...')
        plot_compare(dataset)

    # lever-arm sweep summary chart
    plot_lever_sweep(all_metrics)

    reset_cfg()
    write_summary(all_metrics)

    n_plots = len(os.listdir(PLOT_DIR))
    print(f'\nDone. {n_plots} files in {PLOT_DIR}')


if __name__ == '__main__':
    main()
