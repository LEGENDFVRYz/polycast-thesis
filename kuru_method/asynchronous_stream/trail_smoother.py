"""
trail_smoother.py -- ONLINE causal weighted moving average for the live trail.

Priority 6: this module is now strictly the *online* (display-time) smoother.
The previous class is renamed `OnlineTrailSmoother` so its role is unambiguous;
`TrailSmoother` remains as a backward-compatible alias for legacy call-sites.

For *offline* per-stroke smoothing of saved notes, see `note_smoother.py`,
which runs an RTS pass over the EKF state/covariance history of each stroke.
The two filters are deliberately separate so live latency and final note
quality can be tuned independently.
"""

from collections import deque

import numpy as np


class OnlineTrailSmoother:
    """5-point causal weighted moving average — live display only.

    The kernel weights newest samples most heavily so the rendered tip
    tracks the marker with minimal lag. There is no lookahead. Reset on
    every contact transition (pen-up ↔ pen-down) so hover positions never
    bleed into the next written stroke.
    """

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


# Backward-compatible alias for older call-sites (main_ekf.py,
# verification/layer6, batch_main_ekf_plot.py). New code should import
# OnlineTrailSmoother directly.
TrailSmoother = OnlineTrailSmoother
