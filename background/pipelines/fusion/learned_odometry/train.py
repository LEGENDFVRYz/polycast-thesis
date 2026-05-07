"""Offline training script for OdometryNet.

Usage:
    python -m background.pipelines.fusion.learned_odometry.train \\
        training_data.npz --save-dir assets/ --epochs 100

The script:
  1. Loads windows + deltas from a .npz produced by OdometryDataCollector.
  2. Computes per-channel mean/std and saves them alongside the model.
  3. Trains with 2D rotation augmentation and cosine-annealing LR schedule.
  4. Saves the best-validation checkpoint to <save_dir>/odometry_net.pt.

Sanity check after training:
  Draw a straight line between two known board corners.  Accumulate the
  model's (Δx, Δy) outputs along the stroke.  If the reconstructed path
  is approximately straight, label quality is sufficient.  If it follows
  the UWB wobble, run OdometryDataCollector.smooth_uwb_labels() and retrain.
"""

from __future__ import annotations
import argparse
import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split

from .model import OdometryNet


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

class OdometryDataset(Dataset):
    def __init__(self, npz_path: str, augment: bool = True):
        data = np.load(npz_path)
        self.windows: np.ndarray = data['windows'].astype(np.float32)  # (N, W, 7)
        self.deltas:  np.ndarray = data['deltas'].astype(np.float32)   # (N, 2)
        self.augment = augment

        flat = self.windows.reshape(-1, 7)
        self.mean: np.ndarray = flat.mean(axis=0).astype(np.float32)
        self.std:  np.ndarray = (flat.std(axis=0) + 1e-8).astype(np.float32)

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, idx: int):
        w = (self.windows[idx] - self.mean) / self.std
        d = self.deltas[idx].copy()
        if self.augment:
            w, d = _rotate_augment(w, d)
        return torch.from_numpy(w), torch.from_numpy(d)


def _rotate_augment(
    window: np.ndarray,
    label:  np.ndarray,
    max_deg: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Random 2D in-plane rotation — physically valid because the board frame is fixed.

    Rotates both the ax_world/ay_world channels of the window and the (Δx,Δy) label
    by the same angle.  This doubles effective dataset diversity at zero hardware cost.
    """
    theta = np.radians(np.random.uniform(-max_deg, max_deg))
    c, s = np.cos(theta).astype(np.float32), np.sin(theta).astype(np.float32)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    w = window.copy()
    w[:, :2] = window[:, :2] @ R.T   # rotate ax_world, ay_world columns
    return w, (R @ label).astype(np.float32)


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------

def train(
    npz_path:   str,
    save_dir:   str  = 'assets',
    epochs:     int  = 100,
    batch_size: int  = 64,
    lr:         float = 1e-3,
    val_split:  float = 0.15,
) -> None:
    os.makedirs(save_dir, exist_ok=True)

    dataset = OdometryDataset(npz_path, augment=True)
    np.save(os.path.join(save_dir, 'imu_mean.npy'), dataset.mean)
    np.save(os.path.join(save_dir, 'imu_std.npy'),  dataset.std)
    print(f'Dataset: {len(dataset)} samples  |  mean/std saved to {save_dir}/')

    n_val   = max(1, int(len(dataset) * val_split))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42),
    )
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True,  num_workers=0, drop_last=True)
    val_dl   = DataLoader(val_ds,   batch_size=batch_size, shuffle=False, num_workers=0)

    model   = OdometryNet()
    opt     = torch.optim.Adam(model.parameters(), lr=lr)
    sched   = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr * 0.01)
    loss_fn = nn.MSELoss()

    best_val    = float('inf')
    model_path  = os.path.join(save_dir, 'odometry_net.pt')

    for epoch in range(1, epochs + 1):
        # Train
        model.train()
        train_loss = 0.0
        for x, y in train_dl:
            opt.zero_grad()
            loss = loss_fn(model(x), y)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            opt.step()
            train_loss += loss.item() * len(x)
        train_loss /= n_train
        sched.step()

        # Validate
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, y in val_dl:
                val_loss += loss_fn(model(x), y).item() * len(x)
        val_loss /= n_val

        if epoch % 10 == 0 or epoch == 1:
            # Report in cm² (MSE) and cm (RMSE) for interpretability
            print(
                f'Epoch {epoch:4d}/{epochs}'
                f'  train_rmse={_rmse_cm(train_loss):.1f}cm'
                f'  val_rmse={_rmse_cm(val_loss):.1f}cm'
                f'  lr={sched.get_last_lr()[0]:.2e}'
            )

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), model_path)

    print(f'\nDone. Best val RMSE = {_rmse_cm(best_val):.1f}cm  →  {model_path}')


def _rmse_cm(mse_m2: float) -> float:
    """MSE in m² → RMSE in cm."""
    return float(np.sqrt(mse_m2) * 100.0)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Train OdometryNet on collected whiteboard data.')
    p.add_argument('npz',             help='Path to training_data.npz from OdometryDataCollector')
    p.add_argument('--save-dir',      default='assets',  help='Directory for model + norm stats')
    p.add_argument('--epochs',        type=int,   default=100)
    p.add_argument('--batch-size',    type=int,   default=64)
    p.add_argument('--lr',            type=float, default=1e-3)
    p.add_argument('--val-split',     type=float, default=0.15)
    args = p.parse_args()
    train(args.npz, args.save_dir, args.epochs, args.batch_size, args.lr, args.val_split)
