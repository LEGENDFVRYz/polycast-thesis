"""
test/tuning/run.py — Systematic ESKF pipeline tuner.

Purpose:
    Replaces "Manual configuration -> Replay -> Checking" process 
    with an automated "Measure -> Score -> Search -> Compare -> Save"
    
    Keeping the best workflow, then save the run outputs under:

        test/tuning/results/<name>/

Output:
    test/tuning/results/<name>/
        stage{N}_sensitivity.txt   — per-parameter delta ranking
        stage{N}_top{K}.csv        — top-K configs ranked by composite score
        stage{N}_best_vs_worst.png — trajectory overlay: best vs worst config

Usage:
    python -m test.tuning.run --stage 1 --datasets abc_s1.csv ct_square1.csv

    python -m test.tuning.run --stage 1 --datasets abc_s1.csv --name my_run --trials 81 --top-params 4

    python -m test.tuning.run --stage 2 --datasets abc_c4.csv abc_c5.csv --trials 27

    python -m test.tuning.run --stage 4 --datasets abc_s1.csv abc_c4.csv --top-n 5

Stage order (sequential):
    1  drawing mode        - sigma, drag, pos_floor, acc_scale
    2  fast mode           - burst, speed threshold, fast sigma/acc
    3  air mode            - air sigma, air drag
    4  gates               - trilat_max_residual, k_nlos, r_scale_max, hard_reject_mult
    0  core ESKF noise     - sigma_a, sigma_b_a, sigma_zupt, sigma_uwb

Notes:
    - The script asks for a run name when --name is not supplied.
    - Dataset filenames are resolved relative to --dataset-dir (test/_datasets/)
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


# Project import path
_TUNING_DIR = os.path.dirname(os.path.abspath(__file__))
_TEST_ROOT = os.path.dirname(_TUNING_DIR)
_ROOT = os.path.dirname(_TEST_ROOT)

if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


DEFAULT_DATASET_DIR = os.path.join(_TEST_ROOT, "_datasets")
DEFAULT_OUT_BASE = os.path.join(_TUNING_DIR, "results")

console = Console()


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
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


# -----------------------------------------------------------------------------
# Phases
#
# Each phase mirrors one step of the Measure -> Score -> Search -> Compare
# workflow described in the module docstring.
# -----------------------------------------------------------------------------
def _run_phase_a(stage: int, datasets: list[str], out_dir: str, top_params: int):
    """
    Sensitivity scan: rank stage parameters by |delta| from baseline score.

    Returns (sensitivity_rows, top_param_paths).
    """
    
    from test.tuning.grid_search import sensitivity_scan, top_sensitive_params
    from test.tuning.reporter import save_sensitivity_table

    console.print("\n[bold cyan]\\[Phase A][/] Sensitivity scan")
    with _progress() as progress:
        task = progress.add_task("Scanning parameters", total=None, status="")

        def _on_progress(done, total):
            progress.update(task, total=total, completed=done, status="")

        sens = sensitivity_scan(stage, datasets, on_progress=_on_progress)

    sens_path = os.path.join(out_dir, f"stage{stage}_sensitivity.txt")
    save_sensitivity_table(sens, sens_path)
    console.print(f"  Sensitivity table: [underline]{sens_path}[/]")

    top_paths = top_sensitive_params(sens, top_params)
    console.print(f"\n  Top {top_params} most sensitive parameters:")
    seen_printed = set()
    for row in sens:
        if row["param"] not in seen_printed and row["param"] in top_paths:
            seen_printed.add(row["param"])
            console.print(f"    {row['param']:<58}  |delta|={abs(row['delta']):.2f}")

    return sens, top_paths


def _run_phase_b(
    stage: int, datasets: list[str], 
    top_paths: list[str], 
    top_params: int,
    trials: int, 
    top_n: int, 
    out_dir: str,
) -> list[dict]:
    """
    Run a grid search over the most sensitive parameters.

    Returns:
        A list of trial results sorted by total score in descending order.
    """
    
    from test.tuning.grid_search import grid_search
    from test.tuning.reporter import save_top_configs

    # n_values is derived from top_params (the requested count), not len(top_paths)
    n_values = max(2, int(math.floor(trials ** (1.0 / top_params))))
    total_combos = n_values ** top_params
    console.print(
        f"\n[bold cyan]\\[Phase B][/] Grid search: "
        f"{len(top_paths)} params × {n_values} values = {total_combos} combinations"
    )

    with _progress() as progress:
        task = progress.add_task("Grid search", total=total_combos, status="")

        def _on_progress(done, _total, best_so_far):
            progress.update(task, completed=done, status=f"best={best_so_far:.2f}")

        trial_results = grid_search(
            stage, datasets, top_paths, n_values=n_values,
            on_progress=_on_progress,
        )

    top_csv = os.path.join(out_dir, f"stage{stage}_top{top_n}.csv")
    save_top_configs(trial_results, top_csv, top_n=top_n)
    console.print(f"\n  Top-{top_n} configs: [underline]{top_csv}[/]")

    console.print()
    console.print(_config_table(1, trial_results[:3]))

    return trial_results


def _run_phase_c(
    stage: int, 
    datasets: list[str], 
    trial_results: list[dict],
    out_dir: str, 
    stage_label: str,
) -> None:
    """
    Render a best-vs-worst trajectory overlay using the extreme configs
    from the Phase B grid search.
    """
    
    from test.tuning.reporter import plot_trajectory_comparison
    from test.tuning.headless_runner import run_dataset
    from test.tuning.config_patcher import patch_cfg, reset_cfg

    console.print("\n[bold cyan]\\[Phase C][/] Generating best-vs-worst overlay plot")
    with console.status("Rendering trajectory overlay..."):
        reset_cfg()

        # trial_results is sorted by total score descending in array,
        # so [0] is the best config and [-1] is the worst.
        patch_cfg(trial_results[0]["overrides"])
        best_results = run_dataset(datasets[0])
        reset_cfg()

        patch_cfg(trial_results[-1]["overrides"])
        worst_results = run_dataset(datasets[0])
        reset_cfg()

        plot_path = os.path.join(out_dir, f"stage{stage}_best_vs_worst.png")
        plot_trajectory_comparison(
            best_results, worst_results, plot_path,
            title=f"Stage {stage} ({stage_label}): Best vs Worst Config",
        )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Systematic ESKF pipeline tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--stage",
        type=int,
        choices=[0, 1, 2, 3, 4],
        default=1,
        help="Tuning stage (0=core, 1=drawing, 2=fast, 3=air, 4=gates).",
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        required=True,
        help=(
            "Dataset filenames relative to --dataset-dir "
            "(e.g. abc_s1.csv abc_c4.csv)."
        ),
    )
    parser.add_argument(
        "--dataset-dir",
        default=DEFAULT_DATASET_DIR,
        help="Dataset folder. Default: test/_datasets/.",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Output folder name under test/tuning/results/.",
    )
    parser.add_argument(
        "--out-base",
        default=DEFAULT_OUT_BASE,
        help="Base output folder. Default: test/tuning/results/.",
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=81,
        help="Maximum Phase B grid search trials.",
    )
    parser.add_argument(
        "--top-params",
        type=int,
        default=4,
        help="Number of most sensitive parameters used in Phase B.",
    )
    parser.add_argument(
        "--top-n",
        type=int,
        default=10,
        help="Number of top configurations to save in the CSV report.",
    )
    parser.add_argument(
        "--phase-a-only",
        action="store_true",
        help="Run the sensitivity scan only; skip Phase B.",
    )
    args = parser.parse_args()


    name = args.name
    if not name:
        name = input("Enter tuning run name: ").strip()
    name = _safe_name(name)

    datasets = _resolve_datasets(args.datasets, args.dataset_dir)

    out_base = os.path.abspath(args.out_base)
    out_dir = os.path.join(out_base, name)
    os.makedirs(out_dir, exist_ok=True)

    stage_label = {0: "core", 1: "drawing", 2: "fast", 3: "air", 4: "gates"}[args.stage]

    header = (
        f"[bold]ESKF Tuner[/]  |  Stage {args.stage} ({stage_label})\n"
        f"Run name  : {name}\n"
        f"Datasets  : {[os.path.basename(d) for d in datasets]}\n"
        f"Trials    : {args.trials}  |  Top params: {args.top_params}"
    )
    console.print(Panel(header, expand=False))
    console.print(f"[dim]Output: {out_dir}[/]")

    # Phase A
    _sens, top_paths = _run_phase_a(args.stage, datasets, out_dir, args.top_params)
    if args.phase_a_only:
        console.print("\n[bold green]Done — Phase A only[/]\n")
        return

    # Phase B
    trial_results = _run_phase_b(
        args.stage, datasets, top_paths, args.top_params,
        args.trials, args.top_n, out_dir,
    )
    
    # Phase C
    _run_phase_c(args.stage, datasets, trial_results, out_dir, stage_label)

    console.print(Panel("[bold green]Done.[/]", expand=False))
    console.print(f"[dim]Results in: {out_dir}/[/]")


if __name__ == "__main__":
    main()
