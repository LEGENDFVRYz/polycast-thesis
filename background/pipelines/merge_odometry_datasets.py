"""
Merge Odometry Datasets
=======================
Combines multiple per-stroke-pair .npz files into one training_data.npz.
Run this after all collection sessions are complete and before training.

Usage:
  python -m background.pipelines.merge_odometry_datasets \\
      --input-dir data/odometry/ \\
      --out       data/odometry/training_data.npz

  --input-dir  folder containing all per-pair .npz files (default: data/odometry/)
  --out        output path  (default: data/odometry/training_data.npz)
  --exclude    filename(s) to skip, e.g. --exclude training_data.npz

The script prints a per-file breakdown so you can verify the label diversity
before training.
"""

from __future__ import annotations
import argparse
import glob
import os
import sys

import numpy as np


def merge(input_dir: str, out_path: str, exclude: list[str]) -> None:
    pattern = os.path.join(input_dir, '*.npz')
    files   = sorted(glob.glob(pattern))

    if not files:
        sys.exit(f"[ERROR] No .npz files found in '{input_dir}'")

    exclude_set = {os.path.basename(e) for e in exclude}
    files = [f for f in files if os.path.basename(f) not in exclude_set]

    if not files:
        sys.exit("[ERROR] All files were excluded — nothing to merge.")

    print("=" * 62)
    print("  [MERGE] Odometry datasets")
    print()

    all_windows: list[np.ndarray] = []
    all_deltas:  list[np.ndarray] = []

    for f in files:
        d       = np.load(f)
        windows = d['windows']   # (N, W, 7)
        deltas  = d['deltas']    # (N, 2)

        if windows.ndim != 3 or deltas.ndim != 2:
            print(f"  [SKIP] {os.path.basename(f)} — unexpected shape: "
                  f"windows={windows.shape} deltas={deltas.shape}")
            continue

        n = len(windows)
        # Show the label for this file (should be the same for all rows in a known-points file).
        label_mean = deltas.mean(axis=0)
        print(f"  {os.path.basename(f):<40}  "
              f"n={n:>4}  "
              f"label≈Δx={label_mean[0]:+.3f}m  Δy={label_mean[1]:+.3f}m")

        all_windows.append(windows)
        all_deltas.append(deltas)

    if not all_windows:
        sys.exit("[ERROR] No valid files to merge.")

    merged_windows = np.concatenate(all_windows, axis=0).astype(np.float32)
    merged_deltas  = np.concatenate(all_deltas,  axis=0).astype(np.float32)

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    np.savez(out_path, windows=merged_windows, deltas=merged_deltas)

    print()
    print(f"  Total samples : {len(merged_windows)}")
    print(f"  Window shape  : {merged_windows.shape}")
    print(f"  Saved to      : {out_path}")
    print()

    # Displacement coverage — useful sanity check.
    d_min = merged_deltas.min(axis=0)
    d_max = merged_deltas.max(axis=0)
    d_std = merged_deltas.std(axis=0)
    print(f"  Label coverage:")
    print(f"    Δx  min={d_min[0]:+.3f}m  max={d_max[0]:+.3f}m  std={d_std[0]:.3f}m")
    print(f"    Δy  min={d_min[1]:+.3f}m  max={d_max[1]:+.3f}m  std={d_std[1]:.3f}m")
    print()
    print("  Ready to train:")
    print(f"    python -m background.pipelines.fusion.learned_odometry.train \\")
    print(f"        {out_path} \\")
    print(f"        --save-dir background/pipelines/fusion/learned_odometry/assets/")
    print("=" * 62)


def main() -> None:
    p = argparse.ArgumentParser(
        description='Merge per-stroke-pair .npz files into one training dataset.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--input-dir', default=os.path.join('data', 'odometry'),
                   help='Folder containing per-pair .npz files '
                        '(default: data/odometry/)')
    p.add_argument('--out', default=None,
                   help='Output path (default: <input-dir>/training_data.npz)')
    p.add_argument('--exclude', nargs='*', default=['training_data.npz'],
                   help='Filenames to skip (default: training_data.npz)')
    args = p.parse_args()

    out = args.out or os.path.join(args.input_dir, 'training_data.npz')
    merge(args.input_dir, out, args.exclude)


if __name__ == '__main__':
    main()
