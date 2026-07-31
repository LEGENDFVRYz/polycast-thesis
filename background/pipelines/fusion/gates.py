"""
UWB measurement admission gates.

A UWB fix must clear every gate here before it is allowed to correct the filter.
Each gate targets a distinct failure mode of the ranging hardware, and each has
its own rejection label so the reason survives into the emitted diagnostics.

Gates are evaluated in cost order - cheap geometric rejections first, so an
obviously bad fix never reaches the Kalman update.
"""

import numpy as np

from background.pipelines.config import cfg

REJECT_JUMP = 'UWB_JUMP_REJECT'
REJECT_LOW_CONFIDENCE = 'UWB_LOW_CONF_REJECT'
REJECT_BOARD_MARGIN = 'UWB_BOARD_MARGIN_REJECT'
REJECT_AWAITING_IMU = 'UWB_WAIT_IMU_INIT'
REJECT_NLOS = 'UWB_NLOS_REJECT'

# Tolerance beyond the physical board edge before a fix is called impossible.
# Absorbs normal ranging noise at the boundary without admitting solver blowups.
_BOARD_MARGIN_M = 0.03


def exceeds_jump_limit(
    tip_measurement: np.ndarray,
    ts_uwb: int,
    previous_tip: np.ndarray | None,
    previous_tip_ts: int | None,
    imu_speed_ms: float,
    jump_speed_max: float,
) -> bool:
    """
    True when UWB implies motion far faster than the pen can move.

    Requires IMU corroboration to reject: if the IMU also reports fast motion
    the move is probably genuine, so only an unconfirmed UWB jump - the NLOS
    spike signature - is thrown away.
    """

    if previous_tip is None or previous_tip_ts is None:
        return False

    elapsed_s = max((ts_uwb - previous_tip_ts) / 1_000_000.0, 1e-6)
    implied_speed = float(np.linalg.norm(tip_measurement - previous_tip)) / elapsed_s

    return (
        implied_speed > jump_speed_max
        and imu_speed_ms < cfg.fusion_eskf.uwb_jump_imu_speed_min
    )


def is_unusable_low_confidence(uwb_quality: dict, stroke_active: bool) -> bool:
    """
    True for warning-band fixes arriving while the pen is lifted.

    In air the gain is already near zero, so a marginal fix contributes almost
    no useful correction while still letting the alpha-beta tail nudge the state
    toward a bad measurement. During a stroke the same fix is kept, because
    placement matters more there than the small risk.
    """

    return bool(uwb_quality.get('low_confidence')) and not stroke_active


def is_outside_board(tip_measurement: np.ndarray, board_width: float, board_height: float) -> bool:
    """
    True when the lever-arm-corrected tip lands off the physical board.

    Catches bad trilateration solves that passed the residual threshold but
    were pushed off-board once the tag-to-tip offset was applied.
    """

    return (
        tip_measurement[0] < -_BOARD_MARGIN_M
        or tip_measurement[0] > board_width + _BOARD_MARGIN_M
        or tip_measurement[1] < -_BOARD_MARGIN_M
        or tip_measurement[1] > board_height + _BOARD_MARGIN_M
    )


def exceeds_nlos_residual(solve_error: float) -> bool:
    """True when the trilateration residual is far past nominal, implying NLOS."""

    hard_threshold = cfg.fusion_eskf.hard_reject_mult * cfg.uwb.trilat_max_residual
    return solve_error > hard_threshold


def nlos_noise_scale(solve_error: float) -> float:
    """
    Measurement-noise multiplier grown from trilateration residual severity.

    Scales quadratically so a mildly degraded fix is still used at reduced
    weight while a badly degraded one is effectively ignored, capped at
    r_scale_max to keep the fix from being silently dropped altogether.
    """

    eskf_cfg = cfg.fusion_eskf
    if eskf_cfg.k_nlos <= 0.0 or eskf_cfg.sigma_trilat <= 0.0:
        return 1.0

    severity = solve_error / eskf_cfg.sigma_trilat
    scale = 1.0 + eskf_cfg.k_nlos * severity * severity
    return max(1.0, min(eskf_cfg.r_scale_max, scale))


def quality_noise_multiplier(uwb_quality: dict) -> float:
    """
    Extra noise inflation from position-filter quality flags.

    A clamped fix was moved by the board clamp and no longer reflects a real
    measurement; a low-confidence fix had poor solver geometry. Both are
    down-weighted rather than rejected so coverage is not lost.
    """

    multiplier = 1.0
    if uwb_quality.get('was_clamped'):
        multiplier *= 4.0
    if uwb_quality.get('low_confidence'):
        multiplier *= 3.0
    return multiplier


def direction_disagreement_penalty(
    velocity: np.ndarray,
    innovation: np.ndarray,
    speed_ms: float,
    innovation_norm: float,
    mode_penalty: float,
) -> float:
    """
    Noise multiplier applied when UWB pulls against the IMU's direction of travel.

    A correction opposing established motion is more likely ranging error than
    real displacement. Both magnitudes must clear their minimums first, since
    the angle between two near-zero vectors is meaningless.
    """

    eskf_cfg = cfg.fusion_eskf
    if speed_ms <= eskf_cfg.dir_check_v_min or innovation_norm <= eskf_cfg.dir_check_y_min:
        return 1.0

    alignment = float(np.dot(velocity, innovation) / (speed_ms * innovation_norm))
    return mode_penalty if alignment < eskf_cfg.dir_check_cos_thresh else 1.0


def adaptive_trust_multiplier(nlos_scale: float, speed_ms: float) -> float:
    """
    Noise multiplier balancing UWB degradation against how fast the pen is moving.

    A degraded fix during fast motion is the worst case - the IMU is producing
    real shape that a bad correction would distort - so it is trusted least.
    """

    # Above 1.05 the NLOS model has meaningfully inflated R already, which is
    # the signal that this fix is degraded rather than merely noisy.
    uwb_is_degraded = nlos_scale > 1.05
    if not uwb_is_degraded:
        return 1.0

    pen_is_moving_fast = speed_ms > 0.08
    return 4.0 if pen_is_moving_fast else 1.8
