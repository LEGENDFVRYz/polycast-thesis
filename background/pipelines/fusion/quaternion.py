"""
Quaternion and 2D vector helpers shared across the fusion package.

All quaternions use the [x, y, z, w] storage order emitted by the BNO085
rotation vector, matching preprocess/imu.py. Rotation matrices returned here
map body-frame vectors into the world frame.
"""

import math

import numpy as np


# -----------------------------------------------------------------------------
# Quaternion Math
# -----------------------------------------------------------------------------

def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert a unit quaternion [x, y, z, w] to a 3x3 body-to-world matrix."""

    x, y, z, w = quaternion
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product left (x) right, both stored as [x, y, z, w]."""

    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    """Conjugate of a quaternion, which equals its inverse when unit-norm."""

    return np.array([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]], dtype=float)


def slerp(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    """
    Spherical linear interpolation between two unit quaternions.

    Used to reconstruct pen attitude at a UWB timestamp that falls between two
    buffered IMU samples.
    """

    dot = float(np.clip(np.dot(start, end), -1.0, 1.0))

    # Negating one input picks the shorter arc; both quaternions represent the
    # same rotation, so this changes only the interpolation path.
    if dot < 0.0:
        end = -end
        dot = -dot

    # Near-parallel inputs make the sin(theta) denominator numerically unstable,
    # and plain lerp is indistinguishable from slerp at this angle.
    if dot > 0.9995:
        blended = start + fraction * (end - start)
        return blended / np.linalg.norm(blended)

    angle_between = math.acos(dot)
    angle_step = angle_between * fraction

    perpendicular = end - dot * start
    perpendicular_norm = float(np.linalg.norm(perpendicular))
    if perpendicular_norm < 1e-9:
        return start.copy()
    perpendicular /= perpendicular_norm

    return math.cos(angle_step) * start + math.sin(angle_step) * perpendicular


# -----------------------------------------------------------------------------
# Vector Helpers
# -----------------------------------------------------------------------------

def clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    """
    Scale a vector down to max_norm, preserving direction.

    Limiting by magnitude rather than per-axis keeps a diagonal correction from
    being clipped harder along one axis than the other, which would rotate it.
    """

    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= max_norm or norm < 1e-12:
        return vector
    return vector * (max_norm / norm)


def board_axis_indices() -> tuple[int, int]:
    """Return the world-frame component indices that form the 2D board plane."""

    from background.pipelines.config import cfg

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    return axis_map[cfg.imu.board_axes[0]], axis_map[cfg.imu.board_axes[1]]


def project_to_board(vector_world: np.ndarray) -> np.ndarray:
    """Project a world-frame 3-vector onto the 2D board plane."""

    first_axis, second_axis = board_axis_indices()
    return np.array([vector_world[first_axis], vector_world[second_axis]], dtype=float)
