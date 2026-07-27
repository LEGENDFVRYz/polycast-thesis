"""
test/batch/run.py — Quick dataset plotting / replay helper.

Purpose:
    Quickly replay and plot every raw CSV in test/_datasets/ using the
    current pipeline config, then save the run outputs under:

        test/batch/outputs/<testname>/

Output:
    test/batch/outputs/<testname>/
        config_used.txt
        config_used.json
        config_used.py
        batch_summary.csv
        batch_strokes.csv
        <tag>_<dataset>.csv
        plots/
            <dataset>_zoom.png
            <dataset>_full.png
            <dataset>_imu_diag.png

Usage:
    python test/batch/run.py

    python test/batch/run.py --testname my_live_test

    python test/batch/run.py --testname my_live_test --raw-dir test/_datasets

Notes:
    - The script asks for testname when --testname is not supplied.
    - It discovers every *.csv file directly inside test/_datasets/ (subfolders
      are not scanned unless --raw-dir points at one explicitly).
    - It snapshots the config used during the run.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import os
import re
import shutil
import sys
from datetime import datetime
from typing import Any


# Project import path
_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_TEST_ROOT = os.path.dirname(_TEST_DIR)
_ROOT = os.path.dirname(_TEST_ROOT)

if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)


# Local test utilities
from _batch_utils import (  # noqa: E402
    run_one,
    write_summary,
    plot_zoom,
    plot_full,
    plot_imu_diag,
    plot_postprocess,
)

from background.pipelines.config import cfg  # noqa: E402
import background.pipelines.config as config_module  # noqa: E402


DEFAULT_RAW_DIR = os.path.join(_TEST_ROOT, "_datasets")
DEFAULT_OUT_BASE = os.path.join(_TEST_DIR, "outputs")


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _safe_name(name: str) -> str:
    """Make a filesystem-safe test name."""

    name = name.strip()
    name = re.sub(r"[^A-Za-z0-9._-]+", "_", name)
    name = name.strip("._-")
    return name or "untitled"


def _jsonable(obj: Any) -> Any:
    """Convert dataclasses/tuples/objects into JSON-safe values."""

    if dataclasses.is_dataclass(obj):
        return _jsonable(dataclasses.asdict(obj))
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return repr(obj)


def _config_dict() -> dict[str, Any]:
    """Full config snapshot as a JSON-serializable dict."""

    return _jsonable(cfg)


def _important_knobs() -> list[str]:
    """Human-readable list of key tuning knobs."""

    modes = cfg.fusion_eskf.modes
    return [
        f"serial.port:                  {cfg.serial.port}",
        f"serial.baud:                  {cfg.serial.baud}",
        "",
        f"imu.sample_rate_hz:           {cfg.imu.sample_rate_hz}",
        f"imu.rigid_body_enabled:       {cfg.imu.rigid_body_enabled}",
        f"imu.rigid_body_sign:          {getattr(cfg.imu, 'rigid_body_sign', 'N/A')}",
        f"imu.hpf_enabled:              {cfg.imu.hpf_enabled}",
        f"imu.hpf_cutoff_hz:            {cfg.imu.hpf_cutoff_hz}",
        f"imu.force_contact_threshold:  {cfg.imu.force_contact_threshold}",
        "",
        f"marker.r_imu_body_m:          {cfg.marker.r_imu_body_m}",
        f"marker.r_uwb_body_m:          {cfg.marker.r_uwb_body_m}",
        "",
        f"uwb.range_offsets_m:          {cfg.uwb.range_offsets_m}",
        f"uwb.trilat_max_residual:      {cfg.uwb.trilat_max_residual}",
        f"uwb.pos_alpha:                {cfg.uwb.pos_alpha}",
        f"uwb.pos_beta:                 {cfg.uwb.pos_beta}",
        "",
        f"drawing.sigma_scale:          {modes.drawing.sigma_scale}",
        f"drawing.pos_gain_cap:         {modes.drawing.pos_gain_cap}",
        f"drawing.drag_inv_s:           {modes.drawing.drag_inv_s}",
        f"drawing.acc_scale:            {modes.drawing.acc_scale}",
        "",
        f"drawing_fast.sigma_scale:     {modes.drawing_fast.sigma_scale}",
        f"drawing_fast.pos_gain_cap:    {modes.drawing_fast.pos_gain_cap}",
        f"drawing_fast.drag_inv_s:      {modes.drawing_fast.drag_inv_s}",
        f"drawing_fast.acc_scale:       {modes.drawing_fast.acc_scale}",
        "",
        f"air.sigma_scale:              {modes.air.sigma_scale}",
        f"air.drag_inv_s:               {modes.air.drag_inv_s}",
        f"air.acc_scale:                {modes.air.acc_scale}",
        "",
        f"static.sigma_scale:           {modes.static.sigma_scale}",
        f"static.drag_inv_s:            {modes.static.drag_inv_s}",
        f"static.acc_scale:             {modes.static.acc_scale}",
        "",
        f"fusion.sigma_a:               {cfg.fusion_eskf.sigma_a}",
        f"fusion.sigma_uwb:             {cfg.fusion_eskf.sigma_uwb}",
        f"fusion.sigma_b_a:             {cfg.fusion_eskf.sigma_b_a}",
        "",
        f"drawing_fast_speed_thresh:    {cfg.fusion_eskf.drawing_fast_speed_thresh}",
        f"drawing_fast_min_frames:      {cfg.fusion_eskf.drawing_fast_min_frames}",
        f"drawing_fast_burst_frames:    {cfg.fusion_eskf.drawing_fast_burst_frames}",
        "",
        f"bias_uwb_alpha:               {cfg.fusion_eskf.bias_uwb_alpha}",
        f"bias_decay:                   {cfg.fusion_eskf.bias_decay}",
        f"bias_max_m:                   {cfg.fusion_eskf.bias_max_m}",
        "",
        f"age_ramp_start_s:             {getattr(cfg.fusion_eskf, 'age_ramp_start_s', 'N/A')}",
        f"age_ramp_end_s:               {getattr(cfg.fusion_eskf, 'age_ramp_end_s', 'N/A')}",
        f"age_ramp_mult_max:            {getattr(cfg.fusion_eskf, 'age_ramp_mult_max', 'N/A')}",
    ]


def _write_config_snapshot(out_dir: str, testname: str) -> str:
    """Write config_used.txt, config_used.json, and copy config.py."""

    cfg_dict = _config_dict()
    cfg_json = json.dumps(cfg_dict, indent=2, sort_keys=True)
    cfg_hash = hashlib.sha256(cfg_json.encode("utf-8")).hexdigest()[:12]

    txt_path = os.path.join(out_dir, "config_used.txt")
    json_path = os.path.join(out_dir, "config_used.json")
    py_path = os.path.join(out_dir, "config_used.py")

    lines = [
        f"batch_name:     {testname}",
        f"created_at:     {datetime.now().isoformat(timespec='seconds')}",
        f"config_hash:    {cfg_hash}",
        "",
        "# Key config knobs used for this batch test",
        *_important_knobs(),
        "",
    ]

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    with open(json_path, "w", encoding="utf-8") as f:
        f.write(cfg_json + "\n")

    src_config = getattr(config_module, "__file__", None)
    if src_config and os.path.exists(src_config):
        shutil.copyfile(src_config, py_path)

    return cfg_hash


def _discover_datasets(raw_dir: str) -> list[str]:
    """Return dataset names from every *.csv in raw_dir."""

    if not os.path.isdir(raw_dir):
        raise FileNotFoundError(f"Raw directory not found: {raw_dir}")

    datasets: list[str] = []
    for name in sorted(os.listdir(raw_dir)):
        if not name.lower().endswith(".csv"):
            continue
        if name.startswith(".") or name.startswith("_"):
            continue
        datasets.append(os.path.splitext(name)[0])

    return datasets


def _print_report(all_metrics: list[dict]) -> None:
    """Small terminal summary."""

    if not all_metrics:
        print("\nNo metrics generated.")
        return

    print("\n=== Batch Summary ===\n")
    hdr = (
        f"{'Dataset':<14} {'Strokes':>7} {'Fast%':>7} "
        f"{'InnovP95':>9} {'UWB_rtRej%':>11} "
        f"{'Straight':>9} {'Closure':>9} {'Path(mm)':>9}"
    )
    print(hdr)
    print("-" * len(hdr))

    for m in all_metrics:
        print(
            f"{m.get('dataset', '?'):<14} "
            f"{m.get('strokes', 0):>7} "
            f"{float(m.get('mode_pct_fast', 0.0)):>6.1f}% "
            f"{float(m.get('innov_p95_active', m.get('innov_p95', 0.0))):>9.4f} "
            f"{float(m.get('uwb_runtime_reject_pct', 0.0)):>10.1f}% "
            f"{float(m.get('straightness', 0.0)):>9.3f} "
            f"{float(m.get('closure_mm', 0.0)):>9.1f} "
            f"{float(m.get('fused_path_mm', 0.0)):>9.1f}"
        )
    print()


# -----------------------------------------------------------------------------
# Per-dataset run
# -----------------------------------------------------------------------------

def _run_one_dataset(
    tag: str, dataset: str, testname: str,
    raw_dir: str, out_dir: str, plot_dir: str, no_imu_diag: bool,
) -> dict | None:
    """Replay one dataset, write its plots, and return its metrics dict.

    Returns None if the run failed or produced no metrics — the caller skips
    plotting and the summary row for that dataset in either case.
    """

    print(f"Running {dataset}...")
    try:
        metrics = run_one(tag, dataset, {}, raw_dir, out_dir)
    except Exception as exc:
        print(f"  [ERROR] {dataset}: {exc}")
        return None

    if not metrics:
        print(f"  [WARN] {dataset}: no metrics returned")
        return None

    csv_path = os.path.join(out_dir, f"{tag}_{dataset}.csv")
    title = f"batch/{testname} / {dataset}"

    try:
        plot_zoom(
            csv_path,
            title,
            os.path.join(plot_dir, f"{dataset}_zoom.png"),
            dataset=dataset,
        )
        plot_full(
            csv_path,
            title,
            os.path.join(plot_dir, f"{dataset}_full.png"),
            dataset=dataset,
        )
        plot_postprocess(
            metrics.get("_stroke_objects", []),
            title,
            os.path.join(plot_dir, f"{dataset}_postprocess.png"),
            dataset=dataset,
        )
        if not no_imu_diag:
            plot_imu_diag(
                csv_path,
                title,
                os.path.join(plot_dir, f"{dataset}_imu_diag.png"),
            )
    except Exception as exc:
        print(f"  [WARN] plot failed for {dataset}: {exc}")

    return metrics


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Quick replay + plot every raw CSV in test/_datasets/."
    )
    parser.add_argument(
        "--testname",
        default=None,
        help="Name for output folder under test/batch/outputs/<testname>/",
    )
    parser.add_argument(
        "--raw-dir",
        default=DEFAULT_RAW_DIR,
        help="Raw CSV folder. Default: test/_datasets/",
    )
    parser.add_argument(
        "--out-base",
        default=DEFAULT_OUT_BASE,
        help="Base output folder. Default: test/batch/outputs/",
    )
    parser.add_argument(
        "--no-imu-diag",
        action="store_true",
        help="Skip IMU diagnostic PNGs.",
    )
    args = parser.parse_args()

    testname = args.testname
    if not testname:
        testname = input("Enter batch name: ").strip()

    testname = _safe_name(testname)
    tag = f"batch_{testname}"

    raw_dir = os.path.abspath(args.raw_dir)
    out_base = os.path.abspath(args.out_base)
    out_dir = os.path.join(out_base, testname)
    plot_dir = os.path.join(out_dir, "plots")

    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(plot_dir, exist_ok=True)

    datasets = _discover_datasets(raw_dir)
    if not datasets:
        raise RuntimeError(f"No raw CSV datasets found in: {raw_dir}")

    print("=== Batch: Replay + Plot All Raw Datasets ===")
    print(f"Test name : {testname}")
    print(f"Raw dir   : {raw_dir}")
    print(f"Output    : {out_dir}")
    print(f"Plots     : {plot_dir}")
    print(f"Datasets  : {', '.join(datasets)}")

    cfg_hash = _write_config_snapshot(out_dir, testname)
    print(f"Config    : {cfg_hash}")
    print()

    all_metrics: list[dict] = []

    for ds in datasets:
        metrics = _run_one_dataset(
            tag, ds, testname, raw_dir, out_dir, plot_dir, args.no_imu_diag,
        )
        if metrics:
            all_metrics.append(metrics)

    summary_path = os.path.join(out_dir, "batch_summary.csv")
    strokes_path = os.path.join(out_dir, "batch_strokes.csv")

    write_summary(
        all_metrics,
        summary_path=summary_path,
        strokes_path=strokes_path,
    )

    _print_report(all_metrics)

    print("Batch complete.")
    print(f"Summary : {summary_path}")
    print(f"Strokes : {strokes_path}")
    print(f"Config  : {os.path.join(out_dir, 'config_used.txt')}")
    print(f"Plots   : {plot_dir}")


if __name__ == "__main__":
    main()