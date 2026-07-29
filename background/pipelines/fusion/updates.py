"""
Kalman prediction and measurement-update primitives.

Every correction the filter applies goes through `apply_measurement_update`,
which computes the gain, clips the resulting error state, and folds the
covariance in Joseph form. Callers differ only in their observation matrix and
their clip limits, which are expressed as `ErrorStateClipLimits`.

Clip limits are the main safety mechanism. A mathematically optimal gain can
still produce a visible teleport when the covariance is briefly wrong, so each
update path caps how far one measurement is allowed to move the pen.
"""

from dataclasses import dataclass

import numpy as np

from background.pipelines.config import cfg
from background.pipelines.fusion.quaternion import clip_vector_norm
from background.pipelines.fusion.state import (
    BIAS_SLICE,
    POSITION_SLICE,
    STATE_DIM,
    VELOCITY_SLICE,
    ESKFState,
)

# Accelerometer bias must only ever creep. A large jump here would mean the
# filter is explaining a position error as sensor bias, which then biases every
# later prediction.
_BIAS_CLIP_MS2 = 0.03


@dataclass(frozen=True)
class ErrorStateClipLimits:
    """Per-update ceilings on how far one measurement may move the state."""

    position_m: float
    velocity_ms: float
    bias_ms2: float = _BIAS_CLIP_MS2

    # Norm clipping preserves correction direction; axis clipping is the legacy
    # behavior for the pseudo-measurement paths and is kept bit-for-bit.
    clip_position_by_norm: bool = False
    clip_velocity_by_norm: bool = False

    # Set where a correction must never move position at all, only velocity.
    freeze_position: bool = False


def position_observation_matrix() -> np.ndarray:
    """Observation matrix for a direct position measurement, H = [I 0 0]."""

    observation = np.zeros((2, STATE_DIM))
    observation[0, 0] = 1.0
    observation[1, 1] = 1.0
    return observation


def velocity_observation_matrix() -> np.ndarray:
    """Observation matrix for a direct velocity measurement, H = [0 I 0]."""

    observation = np.zeros((2, STATE_DIM))
    observation[0, 2] = 1.0
    observation[1, 3] = 1.0
    return observation


# -----------------------------------------------------------------------------
# Prediction
# -----------------------------------------------------------------------------

def build_transition_matrix(dt_s: float) -> np.ndarray:
    """
    Error-state transition matrix over dt.

    Because true acceleration is (measured - bias), a bias error feeds straight
    into both velocity and position error:
        d(dp)/d(dv)   =  dt
        d(dp)/d(db_a) = -0.5 * dt^2
        d(dv)/d(db_a) = -dt
    """

    transition = np.eye(STATE_DIM)
    identity_2d = np.eye(2)
    transition[0:2, 2:4] = dt_s * identity_2d
    transition[0:2, 4:6] = -0.5 * dt_s * dt_s * identity_2d
    transition[2:4, 4:6] = -dt_s * identity_2d
    return transition


def build_process_noise(dt_s: float, turn_detected: bool) -> np.ndarray:
    """
    Discrete-time process noise for the error state.

    Accelerometer white noise enters through the kinematic chain, so it couples
    position and velocity error rather than only inflating the diagonal. During
    a detected corner the acceleration noise is inflated so UWB can correct
    shape aggressively where IMU integration is least trustworthy.
    """

    eskf_cfg = cfg.fusion_eskf

    accel_sigma = eskf_cfg.sigma_a * (eskf_cfg.turn_k_q if turn_detected else 1.0)
    accel_variance = accel_sigma * accel_sigma
    bias_variance = eskf_cfg.sigma_b_a * eskf_cfg.sigma_b_a

    identity_2d = np.eye(2)
    process_noise = np.zeros((STATE_DIM, STATE_DIM))
    process_noise[0:2, 0:2] = 0.25 * accel_variance * dt_s ** 4 * identity_2d
    process_noise[0:2, 2:4] = 0.50 * accel_variance * dt_s ** 3 * identity_2d
    process_noise[2:4, 0:2] = 0.50 * accel_variance * dt_s ** 3 * identity_2d
    process_noise[2:4, 2:4] = accel_variance * dt_s * dt_s * identity_2d
    process_noise[4:6, 4:6] = bias_variance * dt_s * identity_2d
    return process_noise


# -----------------------------------------------------------------------------
# Measurement Update
# -----------------------------------------------------------------------------

def compute_kalman_gain(
    state: ESKFState,
    observation_matrix: np.ndarray,
    measurement_noise: np.ndarray,
) -> np.ndarray:
    """Standard gain K = P H' (H P H' + R)^-1."""

    innovation_covariance = (
        observation_matrix @ state.covariance @ observation_matrix.T + measurement_noise
    )
    return state.covariance @ observation_matrix.T @ np.linalg.inv(innovation_covariance)


def clip_error_state(error_state: np.ndarray, limits: ErrorStateClipLimits) -> np.ndarray:
    """Apply the caller's per-block ceilings to a raw error-state correction."""

    if limits.freeze_position:
        error_state[POSITION_SLICE] = 0.0
    elif limits.clip_position_by_norm:
        error_state[POSITION_SLICE] = clip_vector_norm(
            error_state[POSITION_SLICE], limits.position_m
        )
    else:
        error_state[POSITION_SLICE] = np.clip(
            error_state[POSITION_SLICE], -limits.position_m, limits.position_m
        )

    if limits.clip_velocity_by_norm:
        error_state[VELOCITY_SLICE] = clip_vector_norm(
            error_state[VELOCITY_SLICE], limits.velocity_ms
        )
    else:
        error_state[VELOCITY_SLICE] = np.clip(
            error_state[VELOCITY_SLICE], -limits.velocity_ms, limits.velocity_ms
        )

    error_state[BIAS_SLICE] = np.clip(error_state[BIAS_SLICE], -limits.bias_ms2, limits.bias_ms2)
    return error_state


def apply_measurement_update(
    state: ESKFState,
    observation_matrix: np.ndarray,
    innovation: np.ndarray,
    measurement_noise: np.ndarray,
    limits: ErrorStateClipLimits,
    gain_cap: float | None = None,
) -> tuple[bool, np.ndarray, float]:
    """
    Run one Kalman correction against the nominal state.

    `gain_cap` hard-limits the position rows of the gain, letting UWB stabilize
    the stroke without being able to sculpt its shape.

    Returns (applied, clipped_error_state, position_gain) where `applied` is
    False when the computed correction was non-finite and nothing was changed.
    """

    kalman_gain = compute_kalman_gain(state, observation_matrix, measurement_noise)

    if gain_cap is not None and gain_cap < 1.0:
        kalman_gain[POSITION_SLICE, :] = np.clip(kalman_gain[POSITION_SLICE, :], -gain_cap, gain_cap)

    position_gain = float(kalman_gain[0, 0])

    error_state = kalman_gain @ innovation
    if not np.all(np.isfinite(error_state)):
        return False, np.zeros(STATE_DIM), position_gain

    error_state = clip_error_state(error_state, limits)
    state.apply_error_state(error_state)
    state.apply_joseph_covariance_update(kalman_gain, observation_matrix, measurement_noise)
    return True, error_state, position_gain


def apply_zero_velocity_update(state: ESKFState, sigma: float) -> None:
    """
    Pseudo-measurement asserting the pen is not moving.

    The innovation is -velocity because the measurement is exactly zero. Used
    both for detected stillness and, with a looser sigma, at pen-down to stop
    air-move momentum leaking into the start of a stroke.
    """

    measurement_noise = (sigma ** 2) * np.eye(2)
    innovation = -state.velocity.copy()

    apply_measurement_update(
        state=state,
        observation_matrix=velocity_observation_matrix(),
        innovation=innovation,
        measurement_noise=measurement_noise,
        limits=ErrorStateClipLimits(position_m=0.08, velocity_ms=0.50),
    )
