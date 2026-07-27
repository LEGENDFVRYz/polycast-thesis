"""
Proxy-metric scorer for ESKF tuning trials.

No ground truth is available, so we score using internal filter diagnostics
extracted from ESKF._emit() output dicts.  All scores are 0 - 100, "higher = better".

Three sub-scores:
    filter_health  (weight 0.45) — Kalman gain balance, UWB accept rate, NLOS, innovation
    smoothness     (weight 0.35) — jerk, post-stroke oscillation, static variance
    stability      (weight 0.20) — end-of-stroke speed, air drift, re-entry jump

Composite:
    total = 0.45 * filter_health + 0.35 * smoothness + 0.20 * stability
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _arrays(results: list[dict]) -> dict:
    """Unpack list of ESKF output dicts into numpy arrays for vectorized scoring."""

    ts            = np.array([r['ts_hw']         for r in results], dtype=np.float64)
    fused_x       = np.array([r['fused_x']       for r in results], dtype=np.float64)
    fused_y       = np.array([r['fused_y']       for r in results], dtype=np.float64)
    source        = np.array([r['source']        for r in results])
    stroke_active = np.array([r['stroke_active'] for r in results], dtype=bool)
    contact       = np.array([r['_contact']      for r in results], dtype=bool)
    is_static     = np.array([r['_is_static']    for r in results], dtype=bool)

    eskf_dicts    = [r['eskf']                      for r in results]
    K_pos         = np.array([e['K_pos_diag']       for e in eskf_dicts], dtype=np.float64)
    r_scale       = np.array([e['r_scale']          for e in eskf_dicts], dtype=np.float64)
    innov         = np.array([e['innovation_norm']  for e in eskf_dicts], dtype=np.float64)

    # uwb_accepted / uwb_rejected are cumulative counters; use the last frame's value
    last_eskf = eskf_dicts[-1] if eskf_dicts else {}
    uwb_accepted = int(last_eskf.get('uwb_accepted', 0))
    uwb_rejected = int(last_eskf.get('uwb_rejected', 0))

    imu_mask  = source == 'IMU'
    pos_mask  = source == 'POSITION'

    return dict(
        ts=ts, fused_x=fused_x, fused_y=fused_y,
        source=source, stroke_active=stroke_active,
        contact=contact, is_static=is_static,
        K_pos=K_pos, r_scale=r_scale, innov=innov,
        uwb_accepted=uwb_accepted, uwb_rejected=uwb_rejected,
        imu_mask=imu_mask, pos_mask=pos_mask,
    )


def _clamp(value: float, lo: float = 0.0, hi: float = 100.0) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Sub-scores
# ---------------------------------------------------------------------------

def _kalman_gain_balance_score(metrics: dict) -> float:
    """K_pos_diag mean during contact (ideal range 0.02-0.15, center 0.085)."""
    
    contact_imu = metrics['contact'] & metrics['imu_mask']
    gain_values = metrics['K_pos'][contact_imu]
    if len(gain_values) < 3:
        return 50.0  # not enough contact samples to trust the mean

    gain_mean = float(np.mean(gain_values))

    # Gaussian centered on 0.085 (empirical ideal) with sigma=0.065, so the
    # score decays smoothly instead of hard-cutting off outside the range.
    return 100.0 * math.exp(-0.5 * ((gain_mean - 0.085) / 0.065) ** 2)


def _uwb_rejection_rate_score(metrics: dict) -> float:
    """UWB rejection rate: flat 100 up to 15% rejected, then linear to 0 at 30%."""
    
    total_uwb = metrics['uwb_accepted'] + metrics['uwb_rejected']
    if total_uwb == 0:
        return 50.0  # no UWB frames observed; neutral score, not a penalty

    rejection_rate = metrics['uwb_rejected'] / total_uwb

    # Some rejection is expected/healthy (NLOS filtering working as intended),
    # so only rates above 15% start costing points.
    return _clamp(100.0 * (1.0 - max(0.0, rejection_rate - 0.15) / 0.15))


def _nlos_residual_score(metrics: dict) -> float:
    """Mean r_scale across POSITION frames (1.0 = clean, 5.0 = NLOS saturated)."""
    
    residual_values = metrics['r_scale'][metrics['pos_mask']]
    if len(residual_values) < 2:
        return 50.0

    residual_mean = float(np.mean(residual_values))

    # r_scale is the ESKF's own NLOS inflation factor: 1.0 means it trusted
    # UWB fully, up to ~5.0 at hard saturation. Linear falloff between them.
    return _clamp(100.0 * (1.0 - (residual_mean - 1.0) / 4.0))


def _innovation_norm_score(metrics: dict) -> float:
    """Innovation norm mean during POSITION frames (want < 0.05 m)."""
    
    innovation_values = metrics['innov'][metrics['pos_mask']]
    if len(innovation_values) < 2:
        return 50.0

    innovation_mean = float(np.mean(innovation_values))

    # 0.05 m is the target measurement-to-prediction gap; scores hit 0 once
    # the average innovation doubles that (0.10 m), signaling poor tracking.
    return _clamp(100.0 * (1.0 - max(0.0, innovation_mean - 0.05) / 0.05))


def filter_health_score(metrics: dict) -> float:
    """
    The score is a weighted combination of Kalman gain balance, UWB rejection
    rate, NLOS residuals, and innovation norm.
    """
    
    kalman_gain = _kalman_gain_balance_score(metrics)
    rejection_rate = _uwb_rejection_rate_score(metrics)
    nlos_residual = _nlos_residual_score(metrics)
    innovation_norm = _innovation_norm_score(metrics)

    weighted_score = (
        0.30 * kalman_gain
        + 0.30 * rejection_rate
        + 0.20 * nlos_residual
        + 0.20 * innovation_norm
    )

    return _clamp(weighted_score)


def _jerk_score(metrics: dict, contact_idx: np.ndarray, contact_ts_s: np.ndarray) -> float:
    """RMS jerk during contact (second derivative of position, m/s^3)."""
    
    contact_x = metrics['fused_x'][contact_idx]
    contact_y = metrics['fused_y'][contact_idx]

    dt = np.diff(contact_ts_s)
    dt = np.where(dt < 1e-6, 1e-6, dt)  # avoid div-by-zero on duplicate timestamps
    velocity_x = np.diff(contact_x) / dt
    velocity_y = np.diff(contact_y) / dt
    if len(velocity_x) < 2:
        return _clamp(100.0)  # too few samples to compute a second derivative

    dt2 = np.where(dt[:-1] < 1e-6, 1e-6, dt[:-1])
    accel_x = np.diff(velocity_x) / dt2
    accel_y = np.diff(velocity_y) / dt2
    jerk_rms = float(np.sqrt(np.mean(accel_x**2 + accel_y**2)))

    # 200 m/s^3 is the empirical "very jerky" ceiling; score hits 0 there.
    return _clamp(100.0 * (1.0 - jerk_rms / 200.0))


def _post_stroke_oscillation_score(metrics: dict) -> float:
    """Zero-crossings of x-velocity in the 50 frames after each stroke ends."""
    
    stroke_active_int = metrics['stroke_active'].astype(np.int8)
    stroke_end_idx = np.where(np.diff(stroke_active_int) == -1)[0]

    oscillation_counts = []
    for end_idx in stroke_end_idx:
        # 50-frame window: long enough to catch decaying oscillation, short
        # enough not to bleed into the next stroke.
        window_end = min(end_idx + 51, len(metrics['fused_x']))
        segment_x = metrics['fused_x'][end_idx + 1:window_end]
        segment_ts = metrics['ts'][end_idx + 1:window_end] / 1e6
        if len(segment_x) < 2:
            continue

        segment_dt = np.where(np.diff(segment_ts) < 1e-6, 1e-6, np.diff(segment_ts))
        segment_velocity_x = np.diff(segment_x) / segment_dt
        if len(segment_velocity_x) >= 2:
            # Sign flips in velocity = the pen overshooting and correcting
            # (ringing) instead of settling cleanly after lift-off.
            zero_crossings = int(np.sum(np.diff(np.sign(segment_velocity_x)) != 0))
            oscillation_counts.append(zero_crossings)

    mean_oscillations = float(np.mean(oscillation_counts)) if oscillation_counts else 0.0
    return _clamp(100.0 * (1.0 - mean_oscillations / 10.0))  # 10+ crossings = 0 score


def _static_variance_score(metrics: dict) -> float:
    """Position variance in ZUPT / static windows."""
    
    static_imu = metrics['is_static'] & metrics['imu_mask']
    if np.sum(static_imu) < 5:
        return 50.0  # not enough static samples to trust the variance

    position_variance = float(
        np.var(metrics['fused_x'][static_imu]) + np.var(metrics['fused_y'][static_imu])
    )

    # 0.001 m^2 combined variance is the noise floor we expect from a pen
    # held still; anything above that means the filter is drifting at rest.
    return _clamp(100.0 * (1.0 - position_variance / 0.001))


def smoothness_score(metrics: dict) -> float:
    """
    The score is a weighted combination of jerk, post-stroke oscillation,
    and static-window position variance.
    """
    
    contact_imu = metrics['contact'] & metrics['imu_mask']
    contact_idx = np.where(contact_imu)[0]
    if len(contact_idx) < 5:
        return 50.0

    contact_ts_s = metrics['ts'][contact_idx] / 1_000_000.0

    jerk = _jerk_score(metrics, contact_idx, contact_ts_s)
    post_stroke_oscillation = _post_stroke_oscillation_score(metrics)
    static_variance = _static_variance_score(metrics)

    weighted_score = (
        0.50 * jerk
        + 0.30 * post_stroke_oscillation
        + 0.20 * static_variance
    )

    return _clamp(weighted_score)


def _stroke_end_speed_score(metrics: dict, stroke_end_idx: np.ndarray) -> float:
    """Speed at stroke end (want approx. 0)."""
    
    end_speeds = []
    for end_idx in stroke_end_idx:
        next_idx = min(end_idx + 1, len(metrics['fused_x']) - 1)  # clamp: end_idx may be the last sample
        dx = metrics['fused_x'][next_idx] - metrics['fused_x'][end_idx]
        dy = metrics['fused_y'][next_idx] - metrics['fused_y'][end_idx]
        dt = max((metrics['ts'][next_idx] - metrics['ts'][end_idx]) / 1e6, 1e-6)
        end_speeds.append(math.hypot(dx, dy) / dt)

    mean_end_speed = float(np.mean(end_speeds)) if end_speeds else 0.0

    # A pen that's still "moving" the instant contact ends means the filter
    # didn't settle in time; 0.5 m/s is treated as fully unsettled (score 0).
    return _clamp(100.0 * (1.0 - mean_end_speed / 0.5))


def _air_drift_score(metrics: dict) -> float:
    """Per-frame position drift while the pen is in the air (not in contact)."""

    air_mask = (~metrics['contact']) & metrics['imu_mask']
    air_idx = np.where(air_mask)[0]
    if len(air_idx) < 2:
        return 100.0  # no air-time observed, so nothing to penalize

    delta_x = np.diff(metrics['fused_x'][air_idx])
    delta_y = np.diff(metrics['fused_y'][air_idx])
    total_drift = float(np.sum(np.sqrt(delta_x**2 + delta_y**2)))
    drift_per_frame = total_drift / len(air_idx)

    # In air there's no UWB correction gating the IMU, so any drift here is
    # pure integration error; 2mm/frame is the tolerated ceiling.
    return _clamp(100.0 * (1.0 - drift_per_frame / 0.002))


def _reentry_jump_score(metrics: dict, stroke_start_idx: np.ndarray) -> float:
    """Position jump at re-entry (first contact frame after an air gap)."""
    
    reentry_errors = []
    for start_idx in stroke_start_idx:
        next_idx = min(start_idx + 1, len(metrics['fused_x']) - 1)
        dx = metrics['fused_x'][next_idx] - metrics['fused_x'][start_idx]
        dy = metrics['fused_y'][next_idx] - metrics['fused_y'][start_idx]
        reentry_errors.append(math.hypot(dx, dy))

    mean_reentry_error = float(np.mean(reentry_errors)) if reentry_errors else 0.0

    # A visible "snap" when the pen touches back down means accumulated air
    # drift is being corrected too abruptly; 0.05 m is the max tolerated jump.
    return _clamp(100.0 * (1.0 - mean_reentry_error / 0.05))


def stability_score(metrics: dict) -> float:
    """
    The score is a weighted combination of end-of-stroke speed, air drift,
    and re-entry position jump.
    """
    
    stroke_active_int = metrics['stroke_active'].astype(np.int8)
    stroke_end_idx = np.where(np.diff(stroke_active_int) == -1)[0]
    stroke_start_idx = np.where(np.diff(stroke_active_int) == 1)[0]

    end_speed = _stroke_end_speed_score(metrics, stroke_end_idx)
    air_drift = _air_drift_score(metrics)
    reentry_jump = _reentry_jump_score(metrics, stroke_start_idx)

    weighted_score = (
        0.40 * end_speed
        + 0.35 * air_drift
        + 0.25 * reentry_jump
    )

    return _clamp(weighted_score)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def score_results(results: list[dict]) -> dict:
    """
    Score a single run's results (list of ESKF output dicts from run_dataset).
    Returns {'total', 'filter_health', 'smoothness', 'stability', 'n_frames',
             'uwb_accepted', 'uwb_rejected'}.
    """
    
    if not results:
        return {
            'total': 0.0, 'filter_health': 0.0,
            'smoothness': 0.0, 'stability': 0.0,
            'n_frames': 0, 'uwb_accepted': 0, 'uwb_rejected': 0,
        }

    metrics = _arrays(results)
    filter_health = filter_health_score(metrics)
    smoothness = smoothness_score(metrics)
    stability = stability_score(metrics)

    total_score = (
        0.45 * filter_health
        + 0.35 * smoothness
        + 0.20 * stability
    )

    return {
        'total':         round(total_score, 3),
        'filter_health': round(filter_health, 3),
        'smoothness':    round(smoothness, 3),
        'stability':     round(stability, 3),
        'n_frames':      len(results),
        'uwb_accepted':  metrics['uwb_accepted'],
        'uwb_rejected':  metrics['uwb_rejected'],
    }


def score_datasets(dataset_paths: list[str], overrides: dict | None = None) -> dict:
    """
    Run multiple datasets with the same config overrides and return averaged scores.
    Patches cfg before running, resets cfg in a finally block.
    """
    
    from test.tuning.config_patcher import patch_cfg, reset_cfg
    from test.tuning.headless_runner import run_dataset

    if overrides:
        patch_cfg(overrides)
    try:
        all_scores = [score_results(run_dataset(p)) for p in dataset_paths]
    finally:
        reset_cfg()

    if not all_scores:
        return {'total': 0.0, 'filter_health': 0.0, 'smoothness': 0.0, 'stability': 0.0,
                'n_frames': 0, 'uwb_accepted': 0, 'uwb_rejected': 0}

    keys = ['total', 'filter_health', 'smoothness', 'stability',
            'n_frames', 'uwb_accepted', 'uwb_rejected']
    return {k: float(np.mean([s[k] for s in all_scores])) for k in keys}
