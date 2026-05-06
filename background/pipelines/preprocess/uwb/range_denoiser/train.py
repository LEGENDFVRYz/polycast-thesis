"""
Offline training script for RangeDenoiserNet — one model per anchor.

Labels are generated automatically from your existing uwb_range_report.csv
files by applying a zero-phase Butterworth low-pass filter to each anchor's
raw distance signal. No manual annotation needed.

Usage:
    python -m background.pipelines.preprocess.uwb.range_denoiser.train
    python -m background.pipelines.preprocess.uwb.range_denoiser.train --csv uwb_range_report.csv
    python -m background.pipelines.preprocess.uwb.range_denoiser.train --csv s1.csv s2.csv s3.csv

Multiple CSV files are merged before training — run as many collection
sessions as you want and pass them all at once.

Outputs (written to assets/ next to this file):
    range_denoiser_a{0..3}.pt    — trained weights per anchor
    range_mean_a{0..3}.npy       — normalisation mean
    range_std_a{0..3}.npy        — normalisation std
"""

import argparse
import time
from pathlib import Path

import numpy as np

_ASSETS = Path(__file__).parent / "assets"


def _load_csv_one(path: str) -> "dict[int, np.ndarray]":
    """Load one uwb_range_report.csv, return per-anchor raw arrays."""
    import csv
    raw: dict[int, list[float]] = {i: [] for i in range(4)}
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for i in range(4):
                try:
                    v = float(row[f'raw_{i}'])
                    if v > 0.05:
                        raw[i].append(v)
                except (KeyError, ValueError):
                    pass
    return {i: np.array(raw[i], dtype=np.float32) for i in range(4)}


def _load_csvs_split(paths: list[str]) -> "tuple[dict, dict]":
    """
    Load all CSVs and split by session — last file is held out as validation,
    all others are training. This gives a truly independent val set because
    UWB noise is temporally autocorrelated within a session.

    Returns (train_per_anchor, val_per_anchor) — both dict[int, np.ndarray].
    """
    if len(paths) == 1:
        # Single file — fall back to random split inside train_anchor
        data = _load_csv_one(paths[0])
        return data, {i: np.array([], dtype=np.float32) for i in range(4)}

    val_data = _load_csv_one(paths[-1])
    tr_raw: dict[int, list[float]] = {i: [] for i in range(4)}
    for p in paths[:-1]:
        session = _load_csv_one(p)
        for i in range(4):
            tr_raw[i].extend(session[i].tolist())

    tr_data = {i: np.array(tr_raw[i], dtype=np.float32) for i in range(4)}
    print(f"  Train sessions: {len(paths)-1} files  |  Val session: {paths[-1]}")
    return tr_data, val_data


def _butterworth_label(signal: np.ndarray, cutoff_hz: float,
                       order: int, fs: float) -> np.ndarray:
    """Zero-phase Butterworth low-pass filter — produces smooth ground-truth labels."""
    from scipy.signal import butter, filtfilt
    nyq = fs / 2.0
    b, a = butter(order, cutoff_hz / nyq, btype='low')
    return filtfilt(b, a, signal).astype(np.float32)


def _make_windows(signal: np.ndarray, labels: np.ndarray,
                  window_size: int) -> "tuple[np.ndarray, np.ndarray]":
    """Slide a window over signal → (N, window_size, 1) X and (N, 1) Y."""
    n = len(signal) - window_size + 1
    if n <= 0:
        return np.empty((0, window_size, 1), dtype=np.float32), np.empty((0, 1), dtype=np.float32)
    X = np.lib.stride_tricks.sliding_window_view(signal, window_size)  # (N, W)
    Y = labels[window_size - 1:].reshape(-1, 1)                         # (N, 1)
    return X.reshape(n, window_size, 1), Y


def train_anchor(anchor_idx: int, signal_tr: np.ndarray, signal_va: np.ndarray,
                 window_size: int, cutoff_hz: float, order: int,
                 fs: float, epochs: int, batch_size: int, lr: float,
                 val_frac: float, dropout: float = 0.2) -> None:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset
    from .model import RangeDenoiserNet

    n_tr = len(signal_tr)
    n_va = len(signal_va)
    print(f"\n── Anchor A{anchor_idx}  (train={n_tr}  val={n_va}  cutoff={cutoff_hz}Hz) ──")

    if n_tr < window_size + 10:
        print(f"  Skipping — not enough training data.")
        return

    # Compute normalisation from training data only
    mean = float(signal_tr.mean())
    std  = float(signal_tr.std()) + 1e-8

    # Generate labels via Butterworth on train signal
    labels_tr = _butterworth_label(signal_tr, cutoff_hz, order, fs)
    sig_tr_n  = (signal_tr - mean) / std
    lbl_tr_n  = (labels_tr - mean) / std
    X_tr_arr, Y_tr_arr = _make_windows(sig_tr_n, lbl_tr_n, window_size)

    # Val: use held-out session if available, else random split from train
    if n_va >= window_size + 10:
        labels_va = _butterworth_label(signal_va, cutoff_hz, order, fs)
        sig_va_n  = (signal_va - mean) / std
        lbl_va_n  = (labels_va - mean) / std
        X_va_arr, Y_va_arr = _make_windows(sig_va_n, lbl_va_n, window_size)
    else:
        # Single-file fallback: random split
        N   = len(X_tr_arr)
        idx = np.random.permutation(N)
        split = int(N * (1 - val_frac))
        X_va_arr = X_tr_arr[idx[split:]]
        Y_va_arr = Y_tr_arr[idx[split:]]
        X_tr_arr = X_tr_arr[idx[:split]]
        Y_tr_arr = Y_tr_arr[idx[:split]]

    print(f"  Windows — train: {len(X_tr_arr)}  val: {len(X_va_arr)}  "
          f"mean={mean:.3f}m  std={std:.4f}m")

    # Save normalisation stats
    _ASSETS.mkdir(exist_ok=True)
    np.save(_ASSETS / f"range_mean_a{anchor_idx}.npy", np.array(mean))
    np.save(_ASSETS / f"range_std_a{anchor_idx}.npy",  np.array(std))

    X_tr = torch.tensor(X_tr_arr);  Y_tr = torch.tensor(Y_tr_arr)
    X_va = torch.tensor(X_va_arr);  Y_va = torch.tensor(Y_va_arr)

    loader  = DataLoader(TensorDataset(X_tr, Y_tr),
                         batch_size=batch_size, shuffle=True)
    model   = RangeDenoiserNet(window_size=window_size, dropout=dropout)
    opt     = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    sched   = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr/10)
    loss_fn = nn.MSELoss()

    best_val  = float('inf')
    best_path = _ASSETS / f"range_denoiser_a{anchor_idx}.pt"
    t0 = time.time()

    print(f"  {'Epoch':>6}  {'Train RMSE (mm)':>16}  {'Val RMSE (mm)':>14}")
    print("  " + "─" * 40)

    for epoch in range(1, epochs + 1):
        model.train()
        tr_loss = 0.0
        for xb, yb in loader:
            pred = model(xb)
            loss = loss_fn(pred, yb)
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            tr_loss += loss.item() * len(xb)
        tr_rmse = (tr_loss / len(X_tr)) ** 0.5 * std * 1000   # → mm

        model.eval()
        with torch.no_grad():
            val_rmse = float(loss_fn(model(X_va), Y_va).item() ** 0.5) * std * 1000

        sched.step()
        if val_rmse < best_val:
            best_val = val_rmse
            torch.save(model.state_dict(), best_path)

        if epoch % 20 == 0 or epoch == 1:
            print(f"  {epoch:>6}  {tr_rmse:>16.1f}  {val_rmse:>14.1f}  "
                  f"({time.time()-t0:.0f}s)")

    print(f"  Best val RMSE: {best_val:.1f} mm  → {best_path}")


def train(csv_paths: list[str], window_size: int = 20,
          cutoff_hz_per_anchor: tuple = (6.0, 8.0, 3.0, 3.0),
          order: int = 4, fs: float = 100.0, epochs: int = 150,
          batch_size: int = 64, lr: float = 1e-3,
          val_frac: float = 0.15, dropout: float = 0.2,
          seed: int = 42) -> None:

    np.random.seed(seed)
    try:
        import torch; torch.manual_seed(seed)
    except ImportError:
        pass

    print(f"[RangeDenoiser] Loading {len(csv_paths)} CSV file(s)...")
    tr_per_anchor, va_per_anchor = _load_csvs_split(csv_paths)

    for i in range(4):
        train_anchor(
            anchor_idx=i,
            signal_tr=tr_per_anchor[i],
            signal_va=va_per_anchor[i],
            window_size=window_size,
            cutoff_hz=cutoff_hz_per_anchor[i],
            order=order,
            fs=fs,
            epochs=epochs,
            batch_size=batch_size,
            lr=lr,
            val_frac=val_frac,
            dropout=dropout,
        )

    print("\n[RangeDenoiser] All anchors trained.")
    print("Run the visualizer normally — denoised UWB ranges activate automatically.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train per-anchor UWB range denoiser")
    parser.add_argument("--csv",    nargs="+", default=["uwb_range_report.csv"],
                        help="One or more uwb_range_report.csv files")
    parser.add_argument("--epochs", type=int,   default=80)
    parser.add_argument("--lr",     type=float, default=5e-4)
    parser.add_argument("--cutoff", nargs=4, type=float, default=[3.0, 3.0, 3.0, 3.0],
                        metavar=("A0", "A1", "A2", "A3"),
                        help="Per-anchor Butterworth cutoff Hz (default: 3 3 3 3)")
    parser.add_argument("--fs",     type=float, default=100.0,
                        help="UWB sample rate Hz")
    parser.add_argument("--dropout",type=float, default=0.35)
    parser.add_argument("--window", type=int,   default=40,
                        help="Input window size in samples")
    parser.add_argument("--seed",   type=int,   default=42)
    args = parser.parse_args()

    train(
        csv_paths=args.csv,
        window_size=args.window,
        cutoff_hz_per_anchor=tuple(args.cutoff),
        fs=args.fs,
        epochs=args.epochs,
        lr=args.lr,
        dropout=args.dropout,
        seed=args.seed,
    )
