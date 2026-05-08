"""OdometryNet model definitions.

Three architectures available via build_model():

  'lstm'     — CNN + Bi-LSTM (original).  Best when dataset is large (>2000 samples).
               Captures long-range temporal dependencies but heavy (~580K params).

  'tcn'      — Temporal Convolutional Network.  Best for small-to-medium datasets.
               Fully parallelizable, dilated convolutions cover the full 512-frame
               window, ~100K params with channels=32.  Usually matches or beats LSTM
               on datasets <2000 samples.

  'cnn_pool' — CNN + Global Average Pooling.  Simplest and fewest parameters (~50K).
               Good baseline for very small datasets (<500 samples).
               Loses temporal ordering — adequate when stroke shape matters less
               than magnitude.
"""

import torch
import torch.nn as nn


# ---------------------------------------------------------------------------
# Architecture 1 — CNN + Bi-LSTM  (original)
# ---------------------------------------------------------------------------

class OdometryNet(nn.Module):
    """CNN feature extractor + Bi-LSTM temporal model.

    Input:  (batch, window, 7)   z-scored [ax, ay, az, qx, qy, qz, qw]
    Output: (batch, 2)           predicted (Δx, Δy) in metres
    """

    def __init__(self, input_size: int = 7, hidden: int = 128, dropout: float = 0.3):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv1d(input_size, 64,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(64,        128,  kernel_size=11, padding=5), nn.ReLU(),
            nn.Conv1d(128,       256,  kernel_size=11, padding=5), nn.ReLU(),
        )
        self.ln   = nn.LayerNorm(256)
        self.lstm = nn.LSTM(256, hidden, batch_first=True, bidirectional=True)
        self.head = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(hidden * 2, 128), nn.ReLU(),
            nn.Linear(128, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.permute(0, 2, 1)
        x = self.cnn(x)
        x = x.permute(0, 2, 1)
        x = self.ln(x)
        _, (h, _) = self.lstm(x)
        h = torch.cat([h[-2], h[-1]], dim=-1)
        return self.head(h)


# ---------------------------------------------------------------------------
# Architecture 2 — TCN  (recommended for small datasets)
# ---------------------------------------------------------------------------

class _TCNBlock(nn.Module):
    """One residual dilated conv block."""

    def __init__(self, in_ch: int, out_ch: int, kernel_size: int,
                 dilation: int, dropout: float):
        super().__init__()
        pad = (kernel_size - 1) * dilation // 2
        self.block = nn.Sequential(
            nn.Conv1d(in_ch,  out_ch, kernel_size, dilation=dilation, padding=pad),
            nn.BatchNorm1d(out_ch), nn.ReLU(), nn.Dropout(dropout),
            nn.Conv1d(out_ch, out_ch, kernel_size, dilation=dilation, padding=pad),
            nn.BatchNorm1d(out_ch), nn.ReLU(), nn.Dropout(dropout),
        )
        self.skip = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.block(x) + self.skip(x)


class OdometryTCN(nn.Module):
    """Temporal Convolutional Network for inertial odometry.

    Dilations [1,2,4,8,16,32,64] with kernel=7 give receptive field ~889 frames,
    covering the full 512-frame window with margin.

    Parameter counts:
      channels=32  →  ~98K params   (good for 500–1500 samples)
      channels=64  →  ~380K params  (good for 1500+ samples)

    Input:  (batch, window, 7)
    Output: (batch, 2)
    """

    def __init__(self, input_size: int = 7, channels: int = 32, dropout: float = 0.2):
        super().__init__()
        dilations = [1, 2, 4, 8, 16, 32, 64]
        blocks: list[nn.Module] = []
        in_ch = input_size
        for d in dilations:
            blocks.append(_TCNBlock(in_ch, channels, kernel_size=7,
                                    dilation=d, dropout=dropout))
            in_ch = channels
        self.net  = nn.Sequential(*blocks)
        self.head = nn.Sequential(
            nn.AdaptiveAvgPool1d(1),
            nn.Flatten(),
            nn.Linear(channels, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.permute(0, 2, 1)   # (batch, features, time)
        x = self.net(x)           # (batch, channels, time)
        return self.head(x)       # (batch, 2)


# ---------------------------------------------------------------------------
# Architecture 3 — CNN + Global Average Pool  (simplest)
# ---------------------------------------------------------------------------

class OdometryCNNPool(nn.Module):
    """Three Conv1D layers followed by global average pooling.

    No temporal recurrence — fast, few parameters, less overfitting.
    Trades temporal ordering for simplicity.

    Parameter counts:
      channels=32  →  ~47K params   (good for <500 samples)
      channels=64  →  ~182K params  (good for 500–1000 samples)

    Input:  (batch, window, 7)
    Output: (batch, 2)
    """

    def __init__(self, input_size: int = 7, channels: int = 64, dropout: float = 0.2):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv1d(input_size,    channels,   kernel_size=11, padding=5),
            nn.BatchNorm1d(channels), nn.ReLU(),
            nn.Conv1d(channels,      channels*2, kernel_size=11, padding=5),
            nn.BatchNorm1d(channels*2), nn.ReLU(),
            nn.Conv1d(channels*2,    channels*4, kernel_size=11, padding=5),
            nn.BatchNorm1d(channels*4), nn.ReLU(),
        )
        self.head = nn.Sequential(
            nn.AdaptiveAvgPool1d(1),
            nn.Flatten(),
            nn.Dropout(dropout),
            nn.Linear(channels * 4, 64), nn.ReLU(),
            nn.Linear(64, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.permute(0, 2, 1)
        x = self.cnn(x)
        return self.head(x)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Denoising Autoencoder  (pre-processing layer for odometry model)
# ---------------------------------------------------------------------------

class IMUDenoiser(nn.Module):
    """Conv1D denoising autoencoder for z-scored IMU windows.

    Trained with synthetic noise corruption to suppress sensor noise before
    the odometry model sees the input.  No temporal downsampling — output
    shape matches input exactly.

    Input:  (batch, window, input_size)
    Output: (batch, window, input_size)
    """

    def __init__(self, input_size: int = 7, channels: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(input_size,  channels,   kernel_size=7, padding=3), nn.ReLU(),
            nn.Conv1d(channels,    channels*2, kernel_size=7, padding=3), nn.ReLU(),
            nn.Conv1d(channels*2,  channels,   kernel_size=7, padding=3), nn.ReLU(),
            nn.Conv1d(channels,    input_size, kernel_size=7, padding=3),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.permute(0, 2, 1)   # (batch, features, time)
        x = self.net(x)
        return x.permute(0, 2, 1)  # (batch, time, features)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def build_model(
    arch:       str   = 'tcn',
    hidden:     int   = 128,
    channels:   int   = 32,
    dropout:    float = 0.3,
    input_size: int   = 7,
) -> nn.Module:
    """Instantiate a model by architecture name.

    arch='lstm'     → OdometryNet(hidden=hidden, dropout=dropout)
    arch='tcn'      → OdometryTCN(channels=channels, dropout=dropout)
    arch='cnn_pool' → OdometryCNNPool(channels=channels, dropout=dropout)
    """
    if arch == 'lstm':
        return OdometryNet(input_size=input_size, hidden=hidden, dropout=dropout)
    if arch == 'tcn':
        return OdometryTCN(input_size=input_size, channels=channels, dropout=dropout)
    if arch == 'cnn_pool':
        return OdometryCNNPool(input_size=input_size, channels=channels, dropout=dropout)
    raise ValueError(f"Unknown arch '{arch}'. Choose: lstm | tcn | cnn_pool")


def build_denoiser(input_size: int = 7, channels: int = 32) -> IMUDenoiser:
    """Instantiate an IMUDenoiser."""
    return IMUDenoiser(input_size=input_size, channels=channels)
