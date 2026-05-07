import torch
import torch.nn as nn


class OdometryNet(nn.Module):
    """CNN + Bi-LSTM inertial odometry network.

    Input:  (batch, window, 7)  — z-scored [ax_world, ay_world, az_world, qx, qy, qz, qw]
    Output: (batch, 2)          — predicted (Δx, Δy) displacement in metres (world frame)

    Architecture rationale:
      - Three Conv1D layers extract local stroke features (peaks, start/stop patterns).
      - LayerNorm stabilises the heterogeneous accel/quaternion feature scales before LSTM.
      - Bi-LSTM captures temporal context across the full window (fwd + bwd final states).
      - Dropout(0.3) before the FC head guards against overfitting on small datasets.
    """

    def __init__(self, input_size: int = 7, hidden: int = 256, dropout: float = 0.3):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv1d(input_size, 64,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(64,        128,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(128,       256,  kernel_size=11, padding=5), nn.ReLU(),
        )
        # Normalise over the 256 feature channels before LSTM to avoid
        # scale mismatch between accel (m/s²) and quaternion ([-1,1]) channels.
        self.ln   = nn.LayerNorm(256)
        self.lstm = nn.LSTM(256, hidden, batch_first=True, bidirectional=True)
        self.head = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(hidden * 2, 128), nn.ReLU(),
            nn.Linear(128, 2),          # Δx, Δy
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, window, 7)
        x = x.permute(0, 2, 1)          # (batch, 7, window)  — Conv1d expects (N, C, L)
        x = self.cnn(x)                  # (batch, 256, window)
        x = x.permute(0, 2, 1)          # (batch, window, 256)
        x = self.ln(x)                   # normalise feature dim, preserve temporal dim
        _, (h, _) = self.lstm(x)
        h = torch.cat([h[-2], h[-1]], dim=-1)  # concat fwd + bwd final hidden: (batch, 512)
        return self.head(h)              # (batch, 2)
