"""
test/tuning/run.py — Systematic ESKF pipeline tuner

Replaces manual config → replay → inspect cycles with an automated
Measure → Score → Search → Compare → Keep Best workflow.

Usage:
  python test/tuning/run.py --stage 1 --datasets v2/abc_s1.csv v2/ct_square1.csv
  python test/tuning/run.py --stage 1 --datasets v2/abc_s1.csv --trials 81 --top-params 4
  python test/tuning/run.py --stage 2 --datasets v3/abc_c4.csv v3/abc_c5.csv --trials 27
  python test/tuning/run.py --stage 4 --datasets v2/abc_s1.csv v3/abc_c4.csv --top-n 5

Stage order (sequential — tune earlier stages before later ones):
  1  drawing mode      (sigma, drag, pos_floor, acc_scale)
  2  fast mode         (burst, speed threshold, fast sigma/acc)
  3  air mode          (air sigma, air drag)
  4  gates             (trilat_max_residual, k_nlos, r_scale_max, hard_reject_mult)
  0  core ESKF noise   (sigma_a, sigma_b_a, sigma_zupt, sigma_uwb)

Output files (in --out-dir, default: tuning_results/):
  stage{N}_sensitivity.txt   — per-parameter delta ranking
  stage{N}_top{K}.csv        — top-K configs ranked by composite score
  stage{N}_best_vs_worst.png — trajectory overlay: best vs worst config
"""

import argparse
import math
import os
import sys

# ── Project import path ──────────────────────────────────────────────────────
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _resolve_datasets(raw_paths: list[str]) -> list[str]:
    base = os.path.join(_ROOT, 'logs', 'datasets')
    resolved = []
    for p in raw_paths:
        full = os.path.join(base, p)
        if not os.path.exists(full):
            print(f"[ERROR] Dataset not found: {full}")
            sys.exit(1)
        resolved.append(full)
    return resolved


def main():
    parser = argparse.ArgumentParser(
        description='Systematic ESKF pipeline tuner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--stage', type=int, choices=[0, 1, 2, 3, 4], default=1,
                        help='Tuning stage (1=drawing, 2=fast, 3=air, 4=gates, 0=core)')
    parser.add_argument('--datasets', nargs='+', required=True,
                        help='Dataset paths relative to logs/datasets/ '
                             '(e.g. v2/abc_s1.csv v3/abc_c4.csv)')
    parser.add_argument('--trials', type=int, default=81,
                        help='Max Phase B grid search trials (default: 81)')
    parser.add_argument('--top-params', type=int, default=4,
                        help='Top-N sensitive parameters to use in grid search (default: 4)')
    parser.add_argument('--top-n', type=int, default=10,
                        help='Top-N configs to save in CSV report (default: 10)')
    parser.add_argument('--out-dir', default='tuning_results',
                        help='Output directory (default: tuning_results/)')
    parser.add_argument('--phase-a-only', action='store_true',
                        help='Run sensitivity scan only, skip grid search')
    args = parser.parse_args()

    datasets = _resolve_datasets(args.datasets)
    os.makedirs(args.out_dir, exist_ok=True)

    from test.tuning.grid_search import (
        STAGE_PARAMS, sensitivity_scan, top_sensitive_params, grid_search,
    )
    from test.tuning.reporter import (
        save_sensitivity_table, save_top_configs, plot_trajectory_comparison,
    )
    from test.tuning.headless_runner import run_dataset
    from test.tuning.config_patcher import patch_cfg, reset_cfg

    stage_label = {0: 'core', 1: 'drawing', 2: 'fast', 3: 'air', 4: 'gates'}
    print(f"\n{'='*60}")
    print(f"  ESKF Tuner  |  Stage {args.stage} ({stage_label[args.stage]})")
    print(f"  Datasets  : {[os.path.basename(d) for d in datasets]}")
    print(f"  Trials    : {args.trials}  |  Top params: {args.top_params}")
    print(f"{'='*60}\n")

    # ── Phase A: Sensitivity scan ────────────────────────────────────────────
    print("[Phase A] Sensitivity scan ...")
    sens = sensitivity_scan(args.stage, datasets)

    sens_path = os.path.join(args.out_dir, f'stage{args.stage}_sensitivity.txt')
    save_sensitivity_table(sens, sens_path)
    print(f"  Sensitivity table: {sens_path}")

    top_paths = top_sensitive_params(sens, args.top_params)
    print(f"\n  Top {args.top_params} most sensitive parameters:")
    seen_printed = set()
    for row in sens:
        if row['param'] not in seen_printed and row['param'] in top_paths:
            seen_printed.add(row['param'])
            print(f"    {row['param']:<58}  |delta|={abs(row['delta']):.2f}")

    if args.phase_a_only:
        print("\n[Done — Phase A only]\n")
        return

    # ── Phase B: Grid search ─────────────────────────────────────────────────
    n_values = max(2, int(math.floor(args.trials ** (1.0 / args.top_params))))
    print(f"\n[Phase B] Grid search: {args.top_params} params × {n_values} values "
          f"= {n_values ** args.top_params} combinations ...")

    trial_results = grid_search(args.stage, datasets, top_paths, n_values=n_values)

    top_csv = os.path.join(args.out_dir, f'stage{args.stage}_top{args.top_n}.csv')
    save_top_configs(trial_results, top_csv, top_n=args.top_n)
    print(f"\n  Top-{args.top_n} configs: {top_csv}")

    print(f"\n  Top 3 configs:")
    for r in trial_results[:3]:
        print(f"    score={r['total']:.2f}  "
              f"fh={r['filter_health']:.2f}  "
              f"sm={r['smoothness']:.2f}  "
              f"st={r['stability']:.2f}")
        for k, v in r['overrides'].items():
            print(f"      {k} = {v}")

    # ── Phase C: Overlay comparison plot ─────────────────────────────────────
    print("\n[Phase C] Generating best-vs-worst overlay plot ...")
    reset_cfg()

    patch_cfg(trial_results[0]['overrides'])
    best_results = run_dataset(datasets[0])
    reset_cfg()

    patch_cfg(trial_results[-1]['overrides'])
    worst_results = run_dataset(datasets[0])
    reset_cfg()

    plot_path = os.path.join(
        args.out_dir,
        f'stage{args.stage}_best_vs_worst.png',
    )
    plot_trajectory_comparison(
        best_results, worst_results, plot_path,
        title=f'Stage {args.stage} ({stage_label[args.stage]}): Best vs Worst Config',
    )

    print(f"\n{'='*60}")
    print(f"  Done.  Results in: {os.path.abspath(args.out_dir)}/")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
