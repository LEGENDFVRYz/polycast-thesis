"""
trail_smoother.py -- Causal weighted moving average for the EKF drawing trail.

Shared between main_ekf.py (live/CSV playback visualiser) and the layer 6
verification replay so both produce the same rendered strokes from identical
EKF state trajectories.
"""

from collections import deque

import numpy as np


class TrailSmoother:
    """Weighted moving average over the last N positions (causal, no lookahead)."""

    _WEIGHTS = np.array([0.05, 0.10, 0.20, 0.30, 0.35])

    def __init__(self):
        n = len(self._WEIGHTS)
        self._buf_x = deque(maxlen=n)
        self._buf_y = deque(maxlen=n)

    def push(self, x: float, y: float):
        self._buf_x.append(x)
        self._buf_y.append(y)

    def get(self):
        n = len(self._buf_x)
        if n == 0:
            return 0.0, 0.0
        if n == 1:
            return self._buf_x[0], self._buf_y[0]
        w = self._WEIGHTS[-n:]
        w = w / w.sum()
        sx = sum(w[i] * self._buf_x[i] for i in range(n))
        sy = sum(w[i] * self._buf_y[i] for i in range(n))
        return float(sx), float(sy)

    def reset(self):
        self._buf_x.clear()
        self._buf_y.clear()
