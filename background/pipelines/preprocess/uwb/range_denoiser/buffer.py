"""
RangeDenoiserBuffer — rolling window accumulator for one anchor.

Feeds raw (offset-corrected, dropout-held) range samples into a fixed-length
window. Returns a (window_size, 1) numpy array on every call once seeded —
there is no stride delay; inference runs on every new sample.
"""

from collections import deque
import numpy as np


class RangeDenoiserBuffer:
    def __init__(self, window_size: int = 20):
        self.window_size = window_size
        self._buf: deque = deque(maxlen=window_size)

    def push(self, value: float) -> "np.ndarray | None":
        """
        Push one range sample. Returns (window_size, 1) float32 array once the
        buffer is full, else None.
        """
        self._buf.append(float(value))
        if len(self._buf) < self.window_size:
            return None
        return np.array(self._buf, dtype=np.float32).reshape(self.window_size, 1)

    @property
    def ready(self) -> bool:
        return len(self._buf) >= self.window_size

    def reset(self) -> None:
        self._buf.clear()
