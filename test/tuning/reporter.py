"""
Report generation for tuning results.

Outputs:
    stage{N}_sensitivity.txt  — text table of sensitivity scan, sorted by |delta|
    stage{N}_top{K}.csv       — top-K configs with all scores + parameter values
    stage{N}_best_vs_worst.png — matplotlib overlay: best config left, worst right
"""

import csv as csv_mod
import os
import numpy as np


def save_sensitivity_table(sensitivity_rows: list[dict], out_path: str) -> None:
    """
    Write sensitivity scan results as a plain-text table.
    Columns: Parameter  |  Value  |  Score  |  Delta
    Sorted by |delta| descending (rows already sorted by sensitivity_scan).
    """

    with open(out_path, 'w', encoding='utf-8') as f:
        header = f"{'Parameter':<58} {'Value':>12} {'Score':>8} {'Delta':>8}"
        f.write(header + '\n')
        f.write('-' * len(header) + '\n')
        for r in sensitivity_rows:
            f.write(
                f"{r['param']:<58} {r['value']:>12.5f} "
                f"{r['score']:>8.2f} {r['delta']:>+8.2f}\n"
            )


def save_top_configs(
    trial_results: list[dict],
    out_path: str,
    top_n: int = 10,
) -> None:
    """
    Write top-N grid search configs to a CSV file.
    Columns: rank, total, filter_health, smoothness, stability,
             n_frames, uwb_accepted, uwb_rejected, <param1>, <param2>, ...
    """

    if not trial_results:
        return
    top = trial_results[:top_n]
    param_keys = sorted(top[0].get('overrides', {}).keys())
    score_fields = ['rank', 'total', 'filter_health', 'smoothness', 'stability',
                    'n_frames', 'uwb_accepted', 'uwb_rejected']
    fieldnames = score_fields + param_keys

    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv_mod.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for rank, r in enumerate(top, 1):
            row = {
                'rank':          rank,
                'total':         r.get('total', 0.0),
                'filter_health': r.get('filter_health', 0.0),
                'smoothness':    r.get('smoothness', 0.0),
                'stability':     r.get('stability', 0.0),
                'n_frames':      int(r.get('n_frames', 0)),
                'uwb_accepted':  int(r.get('uwb_accepted', 0)),
                'uwb_rejected':  int(r.get('uwb_rejected', 0)),
            }
            row.update(r.get('overrides', {}))
            writer.writerow(row)


def _plot_config_trajectory(
    ax, results: list[dict], label: str, color: str,
    board_w: float, board_h: float, anchor_xy,
    plt_module, mpatches_module,
) -> None:
    """Draw one config's trajectory on ax: board outline, strokes, anchors."""

    if not results:
        ax.set_title(f'{label} (no data)')
        return

    xs      = np.array([r['fused_x']  for r in results])
    ys      = np.array([r['fused_y']  for r in results])
    contact = np.array([r['_contact'] for r in results], dtype=bool)

    ax.add_patch(
        plt_module.Rectangle((0, 0), board_w, board_h,
                              fill=False, edgecolor='black', lw=1.5, linestyle='--')
    )

    # Contact strokes drawn solid (colored); air movement drawn thin gray.
    for i in range(1, len(xs)):
        if contact[i - 1] and contact[i]:
            ax.plot([xs[i-1], xs[i]], [ys[i-1], ys[i]],
                    color=color, lw=1.5, alpha=0.85)
        else:
            ax.plot([xs[i-1], xs[i]], [ys[i-1], ys[i]],
                    color='gray', lw=0.6, alpha=0.25)

    ax.scatter(anchor_xy[:, 0], anchor_xy[:, 1],
               marker='^', s=60, c='black', zorder=5, label='Anchors')

    ax.set_xlim(-0.08, board_w + 0.08)
    ax.set_ylim(-0.08, board_h + 0.08)
    ax.set_aspect('equal')
    ax.set_title(label, fontsize=11)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, linestyle=':', alpha=0.4)

    stroke_patch = mpatches_module.Patch(color=color, label='Stroke (contact)')
    air_patch    = mpatches_module.Patch(color='gray', alpha=0.4, label='Air movement')
    ax.legend(handles=[stroke_patch, air_patch], fontsize=8, loc='upper right')


def plot_trajectory_comparison(
    best_results: list[dict],
    worst_results: list[dict],
    out_path: str,
    title: str = 'Best vs Worst Config',
) -> None:
    """
    Overlay trajectory plot comparing best and worst configs.

    Contact strokes drawn solid (colored); air movement drawn thin gray.
    Saves PNG to out_path. Requires matplotlib.
    """

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("  [reporter] matplotlib not available — skipping plot")
        return

    from background.pipelines.config import cfg
    board_w = cfg.anchors.board_size_x
    board_h = cfg.anchors.board_size_y
    anchor_xy = np.array(cfg.anchors.positions)[:, :2]

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle(title, fontsize=13, fontweight='bold')

    for ax, results, label, color in [
        (axes[0], best_results,  'Best Config',  'steelblue'),
        (axes[1], worst_results, 'Worst Config', 'firebrick'),
    ]:
        _plot_config_trajectory(
            ax, results, label, color, board_w, board_h, anchor_xy,
            plt, mpatches,
        )

    plt.tight_layout()
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  [reporter] Plot saved: {out_path}")
