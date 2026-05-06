"""
IMUOdometryBuffer — rolling window accumulator for learned odometry.

Accepts pre-processed IMU frames (post preprocess/imu.py, gravity already
removed). Each ESP32 packet delivers up to 3 samples; push_packet() handles
the multi-frame-per-call contract.

Input per frame: np.array([ax, ay, qw, qx, qy, qz], float32)
    ax, ay  — acc_board_tip (2D tip-corrected board-plane acceleration, m/s²)
    qw,qx,qy,qz — unit quaternion (body → world)

Returns a (window_size, 6) ndarray when the stride condition is met, else None.
"""

from collections import deque

import numpy as np


class IMUOdometryBuffer:
    def __init__(self, window_size: int = 200, stride: int = 40):
        self.window_size = window_size
        self.stride      = stride
        self._buf = deque(maxlen=window_size)
        self._frames_since_last = 0

    def push_packet(self, imu_frames: list) -> "np.ndarray | None":
        """
        imu_frames: list of np.ndarray, each shape (6,)
                    [ax, ay, qw, qx, qy, qz]
        Returns (window_size, 6) when a full window is ready, else None.
        """
        for frame in imu_frames:
            self._buf.append(frame)
            self._frames_since_last += 1

        if (len(self._buf) >= self.window_size
                and self._frames_since_last >= self.stride):
            self._frames_since_last = 0
            return np.stack(self._buf, axis=0)   # (window_size, 6)

        return None

    def fill_ratio(self) -> float:
        return len(self._buf) / self.window_size

    def reset(self):
        self._buf.clear()
        self._frames_since_last = 0
