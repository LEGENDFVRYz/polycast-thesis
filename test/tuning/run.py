"""
test/tuning/run.py — Systematic ESKF pipeline tuner.

Purpose:
    Replaces manual config -> replay -> inspect cycles with an automated
    Measure -> Score -> Search -> Compare -> Keep Best workflow, then save
    the run outputs under:

        test/tuning/results/<name>/

Output:
    test/tuning/results/<name>/
        stage{N}_sensitivity.txt   — per-parameter delta ranking
        stage{N}_top{K}.csv        — top-K configs ranked by composite score
        stage{N}_best_vs_worst.png — trajectory overlay: best vs worst config

Usage:
    python test/tuning/run.py --stage 1 --datasets abc_s1.csv ct_square1.csv

    python test/tuning/run.py --stage 1 --datasets abc_s1.csv --name my_run --trials 81 --top-params 4

    python test/tuning/run.py --stage 2 --datasets abc_c4.csv abc_c5.csv --trials 27

    python test/tuning/run.py --stage 4 --datasets abc_s1.csv abc_c4.csv --top-n 5

Stage order (sequential — tune earlier stages before later ones):
    1  drawing mode      (sigma, drag, pos_floor, acc_scale)
    2  fast mode         (burst, speed threshold, fast sigma/acc)
    3  air mode          (air sigma, air drag)
    4  gates             (trilat_max_residual, k_nlos, r_scale_max, hard_reject_mult)
    0  core ESKF noise   (sigma_a, sigma_b_a, sigma_zupt, sigma_uwb)

Notes:
    - The script asks for a run name when --name is not supplied.
    - Dataset filenames are resolved relative to --dataset-dir (default
      test/_datasets/), matching test/batch/run.py's convention.
"""

import argparse
import math
import os
import re
import sys

from rich.console import Console
from rich.panel import Panel
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    SpinnerColumn,
    TextColumn,
    TimeElapsedColumn,
)
from rich.table import Table


# ── Project import path ──────────────────────────────────────────────────────
_TUNING_DIR = os.path.dirname(os.path.abspath(__file__))
_TEST_ROOT = os.path.dirname(_TUNING_DIR)
_ROOT = os.path.dirname(_TEST_ROOT)

if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


DEFAULT_DATASET_DIR = os.path.join(_TEST_ROOT, "_datasets")
DEFAULT_OUT_BASE = os.path.join(_TUNING_DIR, "results")

console = Console()


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _safe_name(name: str) -> str:
    """Make a filesystem-safe run name."""
    name = name.strip()
    name = re.sub(r"[^A-Za-z0-9._-]+", "_", name)
    name = name.strip("._-")
    return name or "untitled"


def _resolve_datasets(raw_paths: list[str], dataset_dir: str) -> list[str]:
    resolved = []
    for p in raw_paths:
        full = os.path.join(dataset_dir, p)
        if not os.path.exists(full):
            console.print(f"[bold red][ERROR][/] Dataset not found: {full}")
            sys.exit(1)
        resolved.append(full)
    return resolved


def _progress() -> Progress:
    """Standard spinner + bar + count + elapsed progress bar."""
    return Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TextColumn("•"),
        TimeElapsedColumn(),
        TextColumn("{task.fields[status]}"),
        console=console,
    )


def _config_table(rank_start: int, results: list[dict]) -> Table:
    table = Table(title="Top Configs", show_lines=True)
    table.add_column("#", justify="right", style="bold")
    table.add_column("Score", justify="right", style="bold green")
    table.add_column("FH", justify="right")
    table.add_column("SM", justify="right")
    table.add_column("ST", justify="right")
    table.add_column("Overrides")

    for i, r in enumerate(results, rank_start):
        overrides = "\n".join(f"{k} = {v}" for k, v in r["overrides"].items())
        table.add_row(
            str(i),
            f"{r['total']:.2f}",
            f"{r['filter_health']:.2f}",
            f"{r['smoothness']:.2f}",
            f"{r['stability']:.2f}",
            overrides,
        )
    return table


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Systematic ESKF pipeline tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--stage", type=int, choices=[0, 1, 2, 3, 4], default=1,
                        help="Tuning stage (1=drawing, 2=fast, 3=air, 4=gates, 0=core)")
    parser.add_argument("--datasets", nargs="+", required=True,
                        help="Dataset filenames relative to --dataset-dir "
                             "(e.g. abc_s1.csv abc_c4.csv)")
    parser.add_argument("--dataset-dir", default=DEFAULT_DATASET_DIR,
                        help="Dataset folder. Default: test/_datasets/")
    parser.add_argument("--name", default=None,
                        help="Name for output folder under test/tuning/results/<name>/")
    parser.add_argument("--out-base", default=DEFAULT_OUT_BASE,
                        help="Base output folder. Default: test/tuning/results/")
    parser.add_argument("--trials", type=int, default=81,
                        help="Max Phase B grid search trials (default: 81)")
    parser.add_argument("--top-params", type=int, default=4,
                        help="Top-N sensitive parameters to use in grid search (default: 4)")
    parser.add_argument("--top-n", type=int, default=10,
                        help="Top-N configs to save in CSV report (default: 10)")
    parser.add_argument("--phase-a-only", action="store_true",
                        help="Run sensitivity scan only, skip grid search")
    args = parser.parse_args()

    name = args.name
    if not name:
        name = input("Enter tuning run name: ").strip()
    name = _safe_name(name)

    datasets = _resolve_datasets(args.datasets, args.dataset_dir)

    out_base = os.path.abspath(args.out_base)
    out_dir = os.path.join(out_base, name)
    os.makedirs(out_dir, exist_ok=True)

    from test.tuning.grid_search import (
        sensitivity_scan, top_sensitive_params, grid_search,
    )
    from test.tuning.reporter import (
        save_sensitivity_table, save_top_configs, plot_trajectory_comparison,
    )
    from test.tuning.headless_runner import run_dataset
    from test.tuning.config_patcher import patch_cfg, reset_cfg

    stage_label = {0: "core", 1: "drawing", 2: "fast", 3: "air", 4: "gates"}

    header = (
        f"[bold]ESKF Tuner[/]  |  Stage {args.stage} ({stage_label[args.stage]})\n"
        f"Run name  : {name}\n"
        f"Datasets  : {[os.path.basename(d) for d in datasets]}\n"
        f"Trials    : {args.trials}  |  Top params: {args.top_params}"
    )
    console.print(Panel(header, expand=False))
    console.print(f"[dim]Output: {out_dir}[/]")

    # ── Phase A: Sensitivity scan ────────────────────────────────────────────
    console.print("\n[bold cyan]\\[Phase A][/] Sensitivity scan")
    with _progress() as progress:
        task = progress.add_task("Scanning parameters", total=None, status="")

        def _on_progress_a(done, total):
            progress.update(task, total=total, completed=done, status="")

        sens = sensitivity_scan(args.stage, datasets, on_progress=_on_progress_a)

    sens_path = os.path.join(out_dir, f"stage{args.stage}_sensitivity.txt")
    save_sensitivity_table(sens, sens_path)
    console.print(f"  Sensitivity table: [underline]{sens_path}[/]")

    top_paths = top_sensitive_params(sens, args.top_params)
    console.print(f"\n  Top {args.top_params} most sensitive parameters:")
    seen_printed = set()
    for row in sens:
        if row["param"] not in seen_printed and row["param"] in top_paths:
            seen_printed.add(row["param"])
            console.print(f"    {row['param']:<58}  |delta|={abs(row['delta']):.2f}")

    if args.phase_a_only:
        console.print("\n[bold green]Done — Phase A only[/]\n")
        return

    # ── Phase B: Grid search ─────────────────────────────────────────────────
    n_values = max(2, int(math.floor(args.trials ** (1.0 / args.top_params))))
    total_combos = n_values ** args.top_params
    console.print(
        f"\n[bold cyan]\\[Phase B][/] Grid search: "
        f"{args.top_params} params × {n_values} values = {total_combos} combinations"
    )

    with _progress() as progress:
        task = progress.add_task("Grid search", total=total_combos, status="")

        def _on_progress_b(done, _total, best_so_far):
            progress.update(task, completed=done, status=f"best={best_so_far:.2f}")

        trial_results = grid_search(
            args.stage, datasets, top_paths, n_values=n_values,
            on_progress=_on_progress_b,
        )

    top_csv = os.path.join(out_dir, f"stage{args.stage}_top{args.top_n}.csv")
    save_top_configs(trial_results, top_csv, top_n=args.top_n)
    console.print(f"\n  Top-{args.top_n} configs: [underline]{top_csv}[/]")

    console.print()
    console.print(_config_table(1, trial_results[:3]))

    # ── Phase C: Overlay comparison plot ─────────────────────────────────────
    console.print("\n[bold cyan]\\[Phase C][/] Generating best-vs-worst overlay plot")
    with console.status("Rendering trajectory overlay..."):
        reset_cfg()

        patch_cfg(trial_results[0]["overrides"])
        best_results = run_dataset(datasets[0])
        reset_cfg()

        patch_cfg(trial_results[-1]["overrides"])
        worst_results = run_dataset(datasets[0])
        reset_cfg()

        plot_path = os.path.join(out_dir, f"stage{args.stage}_best_vs_worst.png")
        plot_trajectory_comparison(
            best_results, worst_results, plot_path,
            title=f"Stage {args.stage} ({stage_label[args.stage]}): Best vs Worst Config",
        )

    console.print(Panel("[bold green]Done.[/]", expand=False))
    console.print(f"[dim]Results in: {out_dir}/[/]")


if __name__ == "__main__":
    main()
