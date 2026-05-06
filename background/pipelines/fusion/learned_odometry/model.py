"""
OdometryNet — CNN + BiLSTM displacement predictor.

Input:  (batch, window_size, 6)  — 6 features per IMU frame
Output: (batch, 2)               — (Δx, Δy) in meters
"""

import torch
import torch.nn as nn


class OdometryNet(nn.Module):
    def __init__(self, input_size: int = 6, hidden: int = 256):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv1d(input_size, 64,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(64,        128,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(128,       256,  kernel_size=11, padding=5), nn.ReLU(),
        )
        self.lstm = nn.LSTM(256, hidden, batch_first=True, bidirectional=True)
        self.head = nn.Sequential(
            nn.Linear(hidden * 2, 128), nn.ReLU(),
            nn.Linear(128, 2),
        )

    def forward(self, x: "torch.Tensor") -> "torch.Tensor":
        # x: (batch, N, 6)
        x = x.permute(0, 2, 1)           # (batch, 6, N)
        x = self.cnn(x)
        x = x.permute(0, 2, 1)           # (batch, N, 256)
        _, (h, _) = self.lstm(x)
        h = torch.cat([h[-2], h[-1]], dim=-1)  # fwd + bwd hidden → (batch, 512)
        return self.head(h)               # (batch, 2)
