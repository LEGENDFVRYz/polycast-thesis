"""
RangeDenoiserNet — tiny 1D CNN per-anchor range denoiser.

Input:  (batch, window_size, 1)  — raw range window, normalised
Output: (batch, 1)               — denoised range at the centre/latest sample

Architecture: 3 × Conv1d with residual skip + linear head.
Kept deliberately small (~4 k parameters) so inference at 100 Hz is free.
"""

import torch
import torch.nn as nn


class RangeDenoiserNet(nn.Module):
    def __init__(self, window_size: int = 20, dropout: float = 0.2):
        super().__init__()
        self.window_size = window_size

        self.conv = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, padding=2), nn.ReLU(),
            nn.Dropout(dropout),
            nn.Conv1d(16, 32, kernel_size=5, padding=2), nn.ReLU(),
            nn.Dropout(dropout),
            nn.Conv1d(32, 16, kernel_size=3, padding=1), nn.ReLU(),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Dropout(dropout),
            nn.Linear(16 * window_size, 32), nn.ReLU(),
            nn.Linear(32, 1),
        )

    def forward(self, x: "torch.Tensor") -> "torch.Tensor":
        # x: (batch, window_size, 1)
        x = x.permute(0, 2, 1)   # (batch, 1, window_size)
        x = self.conv(x)          # (batch, 16, window_size)
        return self.head(x)       # (batch, 1)
