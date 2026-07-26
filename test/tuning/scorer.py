"""
Proxy-metric scorer for ESKF tuning trials.

No ground truth is available, so we score using internal filter diagnostics
extracted from ESKF._emit() output dicts.  All scores are 0–100, higher = better.

Three sub-scores:
  filter_health  (weight 0.45) — Kalman gain balance, UWB accept rate, NLOS, innovation
  smoothness     (weight 0.35) — jerk, post-stroke oscillation, static variance
  stability      (weight 0.20) — end-of-stroke speed, air drift, re-entry jump

Composite: total = 0.45 * filter_health + 0.35 * smoothness + 0.20 * stability
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _arrays(results: list[dict]) -> dict:
    """Unpack list of ESKF output dicts into numpy arrays for vectorized scoring."""
    ts           = np.array([r['ts_hw']        for r in results], dtype=np.float64)
    fused_x      = np.array([r['fused_x']      for r in results], dtype=np.float64)
    fused_y      = np.array([r['fused_y']      for r in results], dtype=np.float64)
    source       = np.array([r['source']       for r in results])
    stroke_active= np.array([r['stroke_active'] for r in results], dtype=bool)
    contact      = np.array([r['_contact']     for r in results], dtype=bool)
    is_static    = np.array([r['_is_static']   for r in results], dtype=bool)

    eskf_dicts   = [r['eskf'] for r in results]
    K_pos        = np.array([e['K_pos_diag']      for e in eskf_dicts], dtype=np.float64)
    r_scale      = np.array([e['r_scale']          for e in eskf_dicts], dtype=np.float64)
    innov        = np.array([e['innovation_norm']  for e in eskf_dicts], dtype=np.float64)

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


def _clamp(x: float, lo: float = 0.0, hi: float = 100.0) -> float:
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------
# Sub-scores
# ---------------------------------------------------------------------------

def filter_health_score(d: dict) -> float:
    contact_imu = d['contact'] & d['imu_mask']

    # A: K_pos_diag mean during contact (ideal range 0.02–0.15, center 0.085)
    k_vals = d['K_pos'][contact_imu]
    if len(k_vals) >= 3:
        k_mean = float(np.mean(k_vals))
        k_score = 100.0 * math.exp(-0.5 * ((k_mean - 0.085) / 0.065) ** 2)
    else:
        k_score = 50.0

    # B: UWB rejection rate (flat 100 up to 15%, then linear to 0 at 30%)
    total_uwb = d['uwb_accepted'] + d['uwb_rejected']
    if total_uwb > 0:
        rate = d['uwb_rejected'] / total_uwb
        rej_score = _clamp(100.0 * (1.0 - max(0.0, rate - 0.15) / 0.15))
    else:
        rej_score = 50.0

    # C: Mean r_scale across all POSITION frames (1.0 = clean, 5.0 = NLOS saturated)
    r_vals = d['r_scale'][d['pos_mask']]
    if len(r_vals) >= 2:
        r_mean = float(np.mean(r_vals))
        r_score = _clamp(100.0 * (1.0 - (r_mean - 1.0) / 4.0))
    else:
        r_score = 50.0

    # D: Innovation norm mean during POSITION frames (want < 0.05 m)
    innov_vals = d['innov'][d['pos_mask']]
    if len(innov_vals) >= 2:
        innov_mean = float(np.mean(innov_vals))
        innov_score = _clamp(100.0 * (1.0 - max(0.0, innov_mean - 0.05) / 0.05))
    else:
        innov_score = 50.0

    return _clamp(0.30 * k_score + 0.30 * rej_score + 0.20 * r_score + 0.20 * innov_score)


def smoothness_score(d: dict) -> float:
    contact_imu = d['contact'] & d['imu_mask']
    idx = np.where(contact_imu)[0]

    if len(idx) < 5:
        return 50.0

    ts_s = d['ts'][idx] / 1_000_000.0
    fx   = d['fused_x'][idx]
    fy   = d['fused_y'][idx]

    # A: RMS jerk during contact (second derivative of position, m/s³)
    dt  = np.diff(ts_s)
    dt  = np.where(dt < 1e-6, 1e-6, dt)
    vx  = np.diff(fx) / dt
    vy  = np.diff(fy) / dt
    if len(vx) >= 2:
        dt2 = dt[:-1]
        dt2 = np.where(dt2 < 1e-6, 1e-6, dt2)
        ax  = np.diff(vx) / dt2
        ay  = np.diff(vy) / dt2
        jerk_rms = float(np.sqrt(np.mean(ax**2 + ay**2)))
    else:
        jerk_rms = 0.0
    jerk_score = _clamp(100.0 * (1.0 - jerk_rms / 200.0))

    # B: Post-stroke velocity oscillations (zero-crossings of vx in first 50 air frames)
    active_int = d['stroke_active'].astype(np.int8)
    end_edges  = np.where(np.diff(active_int) == -1)[0]
    osc_counts = []
    for ei in end_edges:
        win_end = min(ei + 51, len(d['fused_x']))
        seg_x = d['fused_x'][ei + 1:win_end]
        seg_ts = d['ts'][ei + 1:win_end] / 1e6
        if len(seg_x) >= 2:
            seg_dt = np.diff(seg_ts)
            seg_dt = np.where(seg_dt < 1e-6, 1e-6, seg_dt)
            seg_vx = np.diff(seg_x) / seg_dt
            if len(seg_vx) >= 2:
                zc = int(np.sum(np.diff(np.sign(seg_vx)) != 0))
                osc_counts.append(zc)
    mean_osc = float(np.mean(osc_counts)) if osc_counts else 0.0
    osc_score = _clamp(100.0 * (1.0 - mean_osc / 10.0))

    # C: Position variance in ZUPT / static windows
    static_imu = d['is_static'] & d['imu_mask']
    if np.sum(static_imu) >= 5:
        var_s = float(np.var(d['fused_x'][static_imu]) + np.var(d['fused_y'][static_imu]))
        static_score = _clamp(100.0 * (1.0 - var_s / 0.001))
    else:
        static_score = 50.0

    return _clamp(0.50 * jerk_score + 0.30 * osc_score + 0.20 * static_score)


def stability_score(d: dict) -> float:
    active_int  = d['stroke_active'].astype(np.int8)
    end_edges   = np.where(np.diff(active_int) == -1)[0]
    start_edges = np.where(np.diff(active_int) ==  1)[0]
    imu_only    = d['imu_mask']

    # A: Speed at stroke end (want ≈ 0)
    end_speeds = []
    for ei in end_edges:
        i1 = min(ei + 1, len(d['fused_x']) - 1)
        dx = d['fused_x'][i1] - d['fused_x'][ei]
        dy = d['fused_y'][i1] - d['fused_y'][ei]
        dt = max((d['ts'][i1] - d['ts'][ei]) / 1e6, 1e-6)
        end_speeds.append(math.hypot(dx, dy) / dt)
    mean_end_speed = float(np.mean(end_speeds)) if end_speeds else 0.0
    end_score = _clamp(100.0 * (1.0 - mean_end_speed / 0.5))

    # B: Per-frame drift while pen is in air
    air_mask = (~d['contact']) & imu_only
    air_idx  = np.where(air_mask)[0]
    if len(air_idx) >= 2:
        ddx = np.diff(d['fused_x'][air_idx])
        ddy = np.diff(d['fused_y'][air_idx])
        total_drift = float(np.sum(np.sqrt(ddx**2 + ddy**2)))
        drift_per_frame = total_drift / len(air_idx)
        drift_score = _clamp(100.0 * (1.0 - drift_per_frame / 0.002))
    else:
        drift_score = 100.0

    # C: Position jump at re-entry (first contact frame after air)
    reentry_errors = []
    for si in start_edges:
        i1 = min(si + 1, len(d['fused_x']) - 1)
        dx = d['fused_x'][i1] - d['fused_x'][si]
        dy = d['fused_y'][i1] - d['fused_y'][si]
        reentry_errors.append(math.hypot(dx, dy))
    mean_reentry = float(np.mean(reentry_errors)) if reentry_errors else 0.0
    reentry_score = _clamp(100.0 * (1.0 - mean_reentry / 0.05))

    return _clamp(0.40 * end_score + 0.35 * drift_score + 0.25 * reentry_score)


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
    d  = _arrays(results)
    fh = filter_health_score(d)
    sm = smoothness_score(d)
    st = stability_score(d)
    total = 0.45 * fh + 0.35 * sm + 0.20 * st
    return {
        'total':         round(total, 3),
        'filter_health': round(fh, 3),
        'smoothness':    round(sm, 3),
        'stability':     round(st, 3),
        'n_frames':      len(results),
        'uwb_accepted':  d['uwb_accepted'],
        'uwb_rejected':  d['uwb_rejected'],
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
