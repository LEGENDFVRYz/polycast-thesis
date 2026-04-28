"""
test/plot_results.py — Generate PNG plots from test/process/ CSVs.

For every per-run CSV (e.g. baseline_circle.csv) produces two PNGs:
  <tag>_<dataset>_zoom.png   — stroke path zoomed in with margins
  <tag>_<dataset>_full.png   — full board view (1.25 m x 1.24 m)

Also produces comparison grids per dataset across experiments:
  compare_<dataset>_zoom.png
  compare_<dataset>_full.png

Usage (from project root):
  python test/plot_results.py
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# ── paths ─────────────────────────────────────────────────────────────────────
_ROOT    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROC_DIR = os.path.join(os.path.dirname(__file__), 'process')
PLOT_DIR = os.path.join(PROC_DIR, 'plots')
os.makedirs(PLOT_DIR, exist_ok=True)

BOARD_W = 1.25   # metres
BOARD_H = 1.24

EXPERIMENTS = ['baseline', 'rbsign_minus', 'uwbvel_gate', 'accclamp7', 'accclamp5']
DATASETS    = ['circle', 'abc', 'hline', 'vline', 'diagonal', 'square', 'triangle']

# colour scheme matching the live visualizer
_COL_FUSED = '#1a1a1a'   # black — fused ink
_COL_AIR   = '#888888'   # gray  — fused air
_COL_UWB   = '#e07b00'   # orange — UWB clean


# ── CSV loader ────────────────────────────────────────────────────────────────

def load_csv(path: str) -> list[dict]:
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _f(row: dict, key: str, default=0.0):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return default


def _i(row: dict, key: str, default=0):
    try:
        return int(row[key])
    except (KeyError, ValueError):
        return default


# ── stroke extraction ─────────────────────────────────────────────────────────

def extract_paths(rows: list[dict]):
    """Return (fused_ink, fused_air, uwb) as lists of (x, y) in metres."""
    fused_ink = []
    fused_air = []
    uwb_pts   = []

    for r in rows:
        fx = _f(r, 'fused_x')
        fy = _f(r, 'fused_y')
        ux = _f(r, 'uwb_x', None)
        uy = _f(r, 'uwb_y', None)

        is_ink = _i(r, 'ink_written') == 1
        is_active = _i(r, 'stroke_active') == 1

        if is_ink:
            fused_ink.append((fx, fy))
        elif is_active:
            fused_air.append((fx, fy))
        else:
            fused_air.append((fx, fy))

        # UWB is available on every row (last known fix carried forward)
        if ux is not None and uy is not None:
            try:
                uwb_pts.append((_f(r, 'uwb_x'), _f(r, 'uwb_y')))
            except Exception:
                pass

    return fused_ink, fused_air, uwb_pts


def ink_by_stroke(rows: list[dict]) -> dict[int, list]:
    """Group ink points by stroke_id for per-stroke colouring."""
    strokes: dict[int, list] = {}
    for r in rows:
        if _i(r, 'ink_written') == 1:
            sid = _i(r, 'stroke_id')
            strokes.setdefault(sid, []).append((_f(r, 'fused_x'), _f(r, 'fused_y')))
    return strokes


def uwb_ink_pts(rows: list[dict]) -> list:
    """UWB positions only during ink frames."""
    pts = []
    for r in rows:
        if _i(r, 'ink_written') == 1:
            try:
                pts.append((_f(r, 'uwb_x'), _f(r, 'uwb_y')))
            except Exception:
                pass
    return pts


# ── annotation helpers ────────────────────────────────────────────────────────

def _closure(pts):
    if len(pts) < 2:
        return 0.0
    return math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1]) * 1000


def _bbox_mm(pts):
    if not pts:
        return 0.0, 0.0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (max(xs)-min(xs))*1000, (max(ys)-min(ys))*1000


def _straightness(pts):
    if len(pts) < 2:
        return 1.0
    chord = math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1])
    arc   = sum(math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
                for i in range(1, len(pts)))
    return min(chord/arc, 1.0) if arc > 0 else 1.0


def _angle_deg(pts):
    if len(pts) < 2:
        return 0.0
    dx = pts[-1][0] - pts[0][0]
    dy = pts[-1][1] - pts[0][1]
    return math.degrees(math.atan2(dy, dx))


def _last_eskf(rows: list[dict]) -> dict:
    for r in reversed(rows):
        e = r.get('eskf') if isinstance(r.get('eskf'), dict) else {}
        if e:
            return e
    # fallback: read directly from csv fields
    return {}


def _mode_pct(rows: list[dict]) -> tuple:
    """Return (fast%, draw%, air%) from last row counters."""
    if not rows:
        return 0, 0, 0
    last = rows[-1]
    fc = _i(last, 'frames_contact')
    ff = _i(last, 'frames_fast')
    fa = _i(last, 'frames_air')
    fs = _i(last, 'frames_static')
    total = fc + ff + fa + fs or 1
    return 100*ff/total, 100*fc/total, 100*fa/total


# ── single plot ───────────────────────────────────────────────────────────────

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
        ax.text(BOARD_W/2, -0.015, f'{BOARD_W*1000:.0f} mm', ha='center',
                va='top', fontsize=6, color='#aaaaaa')


def _draw_paths(ax, rows, alpha_air=0.25):
    strokes  = ink_by_stroke(rows)
    uwb_ink  = uwb_ink_pts(rows)
    fused_ink, fused_air, _ = extract_paths(rows)

    # UWB orange scatter (ink frames only)
    if uwb_ink:
        ux = [p[0] for p in uwb_ink]
        uy = [p[1] for p in uwb_ink]
        ax.scatter(ux, uy, s=1.5, c=_COL_UWB, alpha=0.5, linewidths=0, zorder=2)

    # Fused air gray
    if fused_air:
        ax2 = [p[0] for p in fused_air]
        ay2 = [p[1] for p in fused_air]
        ax.scatter(ax2, ay2, s=0.8, c=_COL_AIR, alpha=alpha_air, linewidths=0, zorder=3)

    # Fused ink black — per stroke so closure can be marked
    for sid, pts in sorted(strokes.items()):
        if not pts:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=_COL_FUSED, lw=1.0, zorder=5)
        # start dot
        ax.scatter([xs[0]], [ys[0]], s=18, c='#2266cc', zorder=6, marker='o')
        # end dot
        ax.scatter([xs[-1]], [ys[-1]], s=18, c='#cc2222', zorder=6, marker='s')


def _info_box(ax, rows, dataset):
    """Add a small annotation with key metrics."""
    fused_ink, _, _ = extract_paths(rows)
    strokes = ink_by_stroke(rows)
    fast_pct, draw_pct, air_pct = _mode_pct(rows)

    lines = [f'ink pts: {len(fused_ink)}']
    all_ink = [(p[0], p[1]) for pts in strokes.values() for p in pts]
    if all_ink:
        w, h = _bbox_mm(all_ink)
        lines.append(f'bbox: {w:.0f}x{h:.0f} mm')
        if dataset in ('circle', 'square', 'triangle'):
            cl = _closure(all_ink)
            lines.append(f'closure: {cl:.0f} mm')
        elif dataset in ('hline', 'vline', 'diagonal'):
            st = _straightness(all_ink)
            ang = _angle_deg(all_ink)
            lines.append(f'straight: {st:.3f}')
            lines.append(f'angle: {ang:.1f} deg')
    lines.append(f'fast%: {fast_pct:.0f}')

    ax.text(0.02, 0.98, '\n'.join(lines),
            transform=ax.transAxes, fontsize=6,
            va='top', ha='left', family='monospace',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.75, lw=0))


def _auto_zoom(rows, margin=0.04):
    """Return (xlim, ylim) tight around ink points."""
    strokes = ink_by_stroke(rows)
    all_pts = [(p[0], p[1]) for pts in strokes.values() for p in pts]
    if not all_pts:
        return (0, BOARD_W), (0, BOARD_H)
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    return (min(xs)-margin, max(xs)+margin), (min(ys)-margin, max(ys)+margin)


def _legend():
    return [
        Line2D([0], [0], color=_COL_FUSED, lw=1.2, label='Fused ink'),
        Line2D([0], [0], color=_COL_AIR,   lw=0, marker='o', ms=3,
               alpha=0.5, label='Fused air'),
        Line2D([0], [0], color=_COL_UWB,   lw=0, marker='o', ms=3,
               alpha=0.7, label='UWB'),
        Line2D([0], [0], color='#2266cc', lw=0, marker='o', ms=5, label='Start'),
        Line2D([0], [0], color='#cc2222', lw=0, marker='s', ms=5, label='End'),
    ]


# ── per-run individual plots ──────────────────────────────────────────────────

def plot_run(exp_tag: str, dataset: str):
    csv_path = os.path.join(PROC_DIR, f'{exp_tag}_{dataset}.csv')
    if not os.path.exists(csv_path):
        return

    rows = load_csv(csv_path)
    title_base = f'{exp_tag} / {dataset}'

    # ── zoom plot ──
    fig, ax = plt.subplots(figsize=(5, 5))
    _setup_ax(ax, f'{title_base}  [zoom]', board=False)
    _draw_paths(ax, rows)
    xlim, ylim = _auto_zoom(rows)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    _info_box(ax, rows, dataset)
    ax.legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    out = os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_zoom.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)

    # ── full board plot ──
    fig, ax = plt.subplots(figsize=(5, 5))
    _setup_ax(ax, f'{title_base}  [full board]', board=True)
    _draw_paths(ax, rows, alpha_air=0.15)
    _info_box(ax, rows, dataset)
    ax.legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)
    fig.tight_layout()
    out = os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_full.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ── comparison grid per dataset ───────────────────────────────────────────────

def plot_compare(dataset: str, zoom: bool):
    """One figure with one column per experiment — side-by-side comparison."""
    n = len(EXPERIMENTS)
    fig, axes = plt.subplots(1, n, figsize=(4*n, 4.5), squeeze=False)
    axes = axes[0]

    view = 'zoom' if zoom else 'full'
    fig.suptitle(f'Dataset: {dataset}  |  view: {view}', fontsize=10, y=1.01)

    for ax, exp_tag in zip(axes, EXPERIMENTS):
        csv_path = os.path.join(PROC_DIR, f'{exp_tag}_{dataset}.csv')
        if not os.path.exists(csv_path):
            ax.set_visible(False)
            continue
        rows = load_csv(csv_path)
        _setup_ax(ax, exp_tag, board=not zoom)
        _draw_paths(ax, rows, alpha_air=0.2)
        if zoom:
            xlim, ylim = _auto_zoom(rows)
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
        _info_box(ax, rows, dataset)

    # shared legend on last visible ax
    axes[-1].legend(handles=_legend(), fontsize=6, loc='lower right', framealpha=0.8)

    fig.tight_layout()
    out = os.path.join(PLOT_DIR, f'compare_{dataset}_{view}.png')
    fig.savefig(out, dpi=130, bbox_inches='tight')
    plt.close(fig)


# ── IMU diagnostics time-series plot ─────────────────────────────────────────

def plot_imu_diagnostics(exp_tag: str, dataset: str):
    """4-panel time-series: acc_hp_tip_mag, omega_mag, alpha_mag, jerk."""
    csv_path = os.path.join(PROC_DIR, f'{exp_tag}_{dataset}.csv')
    if not os.path.exists(csv_path):
        return

    rows = load_csv(csv_path)
    # Only IMU-source rows
    imu_rows = [r for r in rows if r.get('source') == 'IMU']
    if not imu_rows:
        return

    ts  = [_f(r, 'ts_hw') / 1e6 for r in imu_rows]   # seconds
    t0  = ts[0]
    ts  = [t - t0 for t in ts]

    hp  = [_f(r, 'acc_hp_tip_mag') for r in imu_rows]
    tip = [_f(r, 'acc_tip_mag')    for r in imu_rows]
    om  = [_f(r, 'omega_mag')      for r in imu_rows]
    al  = [_f(r, 'alpha_mag')      for r in imu_rows]
    jk  = [_f(r, 'jerk')           for r in imu_rows]
    ink = [_i(r, 'ink_written')    for r in imu_rows]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f'IMU diagnostics  |  {exp_tag} / {dataset}', fontsize=10)

    # shade ink periods
    def _shade(ax):
        in_ink = False
        t_start = 0.0
        for t, i in zip(ts, ink):
            if i and not in_ink:
                t_start = t
                in_ink = True
            elif not i and in_ink:
                ax.axvspan(t_start, t, alpha=0.08, color='black')
                in_ink = False
        if in_ink:
            ax.axvspan(t_start, ts[-1], alpha=0.08, color='black')

    axes[0].plot(ts, hp,  lw=0.7, color='#2266cc', label='acc_hp_tip')
    axes[0].plot(ts, tip, lw=0.7, color='#66aaff', alpha=0.5, label='acc_tip')
    axes[0].set_ylabel('m/s²', fontsize=7)
    axes[0].set_title('Acceleration magnitude (HPF tip vs raw tip)', fontsize=8)
    axes[0].axhline(7.0, color='red', lw=0.8, ls='--', alpha=0.6, label='clamp 7')
    axes[0].axhline(5.0, color='orange', lw=0.8, ls='--', alpha=0.6, label='clamp 5')
    axes[0].legend(fontsize=6, loc='upper right')
    _shade(axes[0])

    axes[1].plot(ts, om, lw=0.7, color='#cc6600')
    axes[1].set_ylabel('rad/s', fontsize=7)
    axes[1].set_title('Angular velocity magnitude (omega)', fontsize=8)
    _shade(axes[1])

    axes[2].plot(ts, al, lw=0.7, color='#aa2200')
    axes[2].set_ylabel('rad/s²', fontsize=7)
    axes[2].set_title('Angular acceleration magnitude (alpha)', fontsize=8)
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
    out = os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_imu_diag.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ── fusion health time-series ─────────────────────────────────────────────────

def plot_fusion_health(exp_tag: str, dataset: str):
    """3-panel: K_pos_diag, innovation_norm, b_p_mag + b_a_norm."""
    csv_path = os.path.join(PROC_DIR, f'{exp_tag}_{dataset}.csv')
    if not os.path.exists(csv_path):
        return

    rows = load_csv(csv_path)
    imu_rows = [r for r in rows if r.get('source') == 'IMU']
    if not imu_rows:
        return

    ts  = [_f(r, 'ts_hw') / 1e6 for r in imu_rows]
    t0  = ts[0]
    ts  = [t - t0 for t in ts]
    ink = [_i(r, 'ink_written') for r in imu_rows]

    kp   = [_f(r, 'K_pos_diag')    for r in imu_rows]
    ke   = [_f(r, 'kcap_eff')      for r in imu_rows]
    inn  = [_f(r, 'innovation_norm') for r in imu_rows]
    bp   = [_f(r, 'b_p_mag')       for r in imu_rows]
    ba   = [_f(r, 'b_a_norm')      for r in imu_rows]
    age  = [_f(r, 'stroke_age_s')  for r in imu_rows]
    mult = [_f(r, 'age_cap_mult')  for r in imu_rows]

    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f'Fusion health  |  {exp_tag} / {dataset}', fontsize=10)

    def _shade(ax):
        in_ink = False
        t_start = 0.0
        for t, i in zip(ts, ink):
            if i and not in_ink:
                t_start = t
                in_ink = True
            elif not i and in_ink:
                ax.axvspan(t_start, t, alpha=0.08, color='black')
                in_ink = False
        if in_ink:
            ax.axvspan(t_start, ts[-1], alpha=0.08, color='black')

    axes[0].plot(ts, kp,  lw=0.7, color='#2244aa', label='K_pos_diag (raw)')
    axes[0].plot(ts, ke,  lw=0.9, color='#cc2222', ls='--', label='kcap_eff (capped)')
    axes[0].set_ylabel('K', fontsize=7)
    axes[0].set_title('Kalman gain vs effective cap', fontsize=8)
    axes[0].legend(fontsize=6)
    _shade(axes[0])

    axes[1].plot(ts, inn, lw=0.7, color='#cc6600')
    axes[1].set_ylabel('m', fontsize=7)
    axes[1].set_title('Innovation norm (UWB-IMU disagreement)', fontsize=8)
    _shade(axes[1])

    axes[2].plot(ts, bp,  lw=0.7, color='#006688', label='b_p_mag')
    axes[2].plot(ts, ba,  lw=0.7, color='#aa4400', alpha=0.6, label='b_a_norm')
    axes[2].set_ylabel('m / m/s²', fontsize=7)
    axes[2].set_title('Position bias (b_p) and acceleration bias (b_a)', fontsize=8)
    axes[2].legend(fontsize=6)
    _shade(axes[2])

    axes[3].plot(ts, age,  lw=0.8, color='#444444', label='stroke_age_s')
    ax3b = axes[3].twinx()
    ax3b.plot(ts, mult, lw=0.8, color='#cc2222', ls='--', label='age_cap_mult')
    ax3b.set_ylabel('multiplier', fontsize=7, color='#cc2222')
    ax3b.tick_params(labelsize=7, colors='#cc2222')
    axes[3].set_ylabel('s', fontsize=7)
    axes[3].set_title('Stroke age and cap multiplier', fontsize=8)
    axes[3].set_xlabel('time (s)', fontsize=7)
    axes[3].legend(fontsize=6, loc='upper left')
    ax3b.legend(fontsize=6, loc='upper right')
    _shade(axes[3])

    for ax in axes:
        ax.tick_params(labelsize=7)
        ax.grid(True, lw=0.3, alpha=0.5)

    fig.tight_layout()
    out = os.path.join(PLOT_DIR, f'{exp_tag}_{dataset}_fusion_health.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    total = 0

    # Individual zoom + full plots for every run
    for exp_tag in EXPERIMENTS:
        for dataset in DATASETS:
            csv_path = os.path.join(PROC_DIR, f'{exp_tag}_{dataset}.csv')
            if not os.path.exists(csv_path):
                continue
            print(f'  Plotting {exp_tag}/{dataset}...')
            plot_run(exp_tag, dataset)
            plot_imu_diagnostics(exp_tag, dataset)
            plot_fusion_health(exp_tag, dataset)
            total += 3

    # Comparison grids per dataset
    for dataset in DATASETS:
        print(f'  Comparison grid: {dataset}...')
        plot_compare(dataset, zoom=True)
        plot_compare(dataset, zoom=False)
        total += 2

    print(f'\nDone. {total} PNGs -> {PLOT_DIR}')


if __name__ == '__main__':
    main()
