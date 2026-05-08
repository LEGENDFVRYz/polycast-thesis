#!/usr/bin/env python3
"""
force_contact_analyzer.py — offline FSR/contact diagnostics for PolyCast CSVs.

Usage:
    python force_contact_analyzer.py path/to/live_YYYYMMDD_HHMMSS.csv

Outputs:
    - Console summary of force percentiles, detector duty, strokes.
    - PNG beside the CSV: <csv>_force_contact.png

This is intentionally non-invasive: it does not change EKF behavior.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Iterable

import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception as e:  # pragma: no cover
    plt = None

try:
    from force_detector import ForceContactDetector
except Exception:
    ForceContactDetector = None


@dataclass
class ImuForceSample:
    t_us: int | None
    force: float


def _parse_imu_force_row(row: list[str]) -> ImuForceSample | None:
    if not row or row[0].strip().upper() != "I":
        return None
    # Supported formats:
    # legacy: I,seq,qx,qy,qz,qw,ax,ay,az,force,ts              len=11
    # gyro:   I,seq,qx,qy,qz,qw,ax,ay,az,gx,gy,gz,force,ts     len=14
    try:
        if len(row) >= 14:
            force = float(row[12])
            ts = int(float(row[13]))
        elif len(row) >= 11:
            force = float(row[9])
            ts = int(float(row[10]))
        else:
            return None
    except Exception:
        return None
    return ImuForceSample(t_us=ts, force=force)


def load_force_samples(csv_path: Path) -> list[ImuForceSample]:
    samples: list[ImuForceSample] = []
    with csv_path.open("r", newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for row in reader:
            s = _parse_imu_force_row([c.strip() for c in row])
            if s is not None and np.isfinite(s.force):
                samples.append(s)
    return samples


def run_legacy_detector(samples: list[ImuForceSample]):
    if ForceContactDetector is None:
        return None
    det = ForceContactDetector()
    states, smooth = [], []
    for s in samples:
        st, sm = det.process(s.force, ts=s.t_us)
        states.append(int(st))
        smooth.append(float(sm))
    return np.asarray(states, dtype=int), np.asarray(smooth, dtype=float)


def stroke_stats(t_s: np.ndarray, states: np.ndarray):
    stats = []
    in_stroke = False
    start = 0
    for i, st in enumerate(states):
        if st and not in_stroke:
            start = i
            in_stroke = True
        if in_stroke and ((not st) or i == len(states) - 1):
            end = i if not st else i + 1
            if end > start:
                stats.append((start, end, float(t_s[end - 1] - t_s[start]), int(end - start)))
            in_stroke = False
    return stats


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("Usage: python force_contact_analyzer.py path/to/live.csv")
        return 2
    csv_path = Path(argv[1]).expanduser()
    if not csv_path.exists():
        print(f"ERROR: CSV not found: {csv_path}")
        return 1

    samples = load_force_samples(csv_path)
    if not samples:
        print("ERROR: no IMU force samples found. Expected rows starting with I,...")
        return 1

    forces = np.asarray([s.force for s in samples], dtype=float)
    t_us_raw = [s.t_us for s in samples]
    if all(t is not None for t in t_us_raw):
        t_us = np.asarray(t_us_raw, dtype=np.int64)
        t_s = (t_us - t_us[0]) / 1_000_000.0
    else:
        t_s = np.arange(len(samples), dtype=float) * 0.005

    pct_marks = [0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100]
    pct = np.percentile(forces, pct_marks)

    print("\nPolyCast FSR force diagnostic")
    print("=" * 44)
    print(f"CSV: {csv_path}")
    print(f"IMU force samples: {len(forces)}")
    print(f"Duration: {t_s[-1]:.2f} s")
    print("Force percentiles (ADC):")
    for p, v in zip(pct_marks, pct):
        print(f"  p{p:>3}: {v:8.1f}")

    det_result = run_legacy_detector(samples)
    states = None
    smooth = None
    if det_result is not None:
        states, smooth = det_result
        duty = 100.0 * float(np.mean(states))
        strokes = stroke_stats(t_s, states)
        print("\nLegacy ForceContactDetector:")
        print(f"  duty: {duty:.1f}%")
        print(f"  strokes: {len(strokes)}")
        if strokes:
            durations = np.asarray([s[2] for s in strokes], dtype=float)
            sizes = np.asarray([s[3] for s in strokes], dtype=float)
            print(f"  stroke duration median/p90/max: {np.median(durations):.2f}s / {np.percentile(durations,90):.2f}s / {np.max(durations):.2f}s")
            print(f"  stroke samples median/p90/max: {np.median(sizes):.0f} / {np.percentile(sizes,90):.0f} / {np.max(sizes):.0f}")
    else:
        print("\nLegacy ForceContactDetector import failed; skipped contact-state replay.")

    # Suggest threshold search range based on distribution only. This is not a final calibration.
    idle = float(np.percentile(forces, 10))
    high = float(np.percentile(forces, 90))
    print("\nInitial threshold search suggestion:")
    print(f"  Sweep ENTER roughly {max(80, idle + 100):.0f} to {max(200, high * 0.75):.0f} ADC")
    print(f"  Sweep EXIT  roughly {max(40, idle + 40):.0f} to {max(100, high * 0.30):.0f} ADC")
    print("  Use the plot to pick thresholds that separate pen-up travel from real strokes.")

    if plt is not None:
        out_png = csv_path.with_name(csv_path.stem + "_force_contact.png")
        fig, ax = plt.subplots(figsize=(14, 5))
        ax.plot(t_s, forces, linewidth=0.8, label="raw force")
        if smooth is not None:
            ax.plot(t_s, smooth, linewidth=1.0, alpha=0.8, label="EMA force")
        if states is not None:
            ymax = max(1.0, float(np.nanmax(forces)))
            ax.fill_between(t_s, 0, ymax, where=states.astype(bool), alpha=0.12,
                            label="legacy detector contact")
        for y, name in [(250, "legacy enter 250"), (120, "legacy exit 120")]:
            ax.axhline(y, linestyle="--", linewidth=0.8, alpha=0.7, label=name)
        ax.set_title("PolyCast FSR force/contact diagnostic")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("FSR ADC")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(out_png, dpi=150)
        print(f"\nSaved plot: {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
