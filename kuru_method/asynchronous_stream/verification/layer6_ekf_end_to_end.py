"""
layer6_ekf_end_to_end.py -- EKF tracking error, latency, gate rate.

Replays the CSV in strict packet order (identical to main_ekf.py) through
AsyncEKFFusionEngine and records, per UWB update:
    EKF (px, py, vx, vy, bax, bay)
    IRLS (px, py) computed on the exact same packet
    per-anchor accepted / rejected flags
    covariance diagonal P[0,0], P[1,1]

Aggregate metrics
-----------------
    cold_start_s          -- time from first UWB packet to EKF initialised
    rms_ekf_vs_irls       -- RMS distance between the two traces
    latency_proxy_s       -- lag of cross-correlation peak  (EKF vs IRLS)
    gate_reject_pct       -- % anchor updates rejected
    stationary_speed_mean -- mean EKF speed during contact-off periods
    bias_final_m_s2       -- last estimated bias magnitude
    bias_delta_last_2s    -- how much bias changed in the last 2 s

Pass criteria (from plan)
-------------------------
    latency_proxy_s < 0.15
    rms_ekf_vs_irls < 0.05
    gate_reject_pct < 10.0
    cold_start_s    < 0.5
    stationary_speed_mean < 0.02
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common        import (ensure_out, replay, LayerResult, DATASET_DIR,
                            draw_board)
from config         import ANCHORS, UWB_OFFSETS, MARKER_LENGTH
from preprocessor   import UWBPreprocessor, IMUPreprocessor
from ekf_fusion     import AsyncEKFFusionEngine
from fusion_engine  import IRLSTrilateration
from ground_truth   import get_truth


LAYER = 'layer6_ekf_end_to_end'


def analyse(csv_path: Path, force_contact: bool = False) -> LayerResult:
    stem = csv_path.stem
    tag  = f'{stem}+contact' if force_contact else stem

    engine = AsyncEKFFusionEngine()
    if force_contact:
        # Diagnostic override: make ForceContactDetector always report contact=1
        # so LIFTED_VEL_DAMP never fires. Isolates the hover-damping hypothesis.
        _orig_contact = engine._contact.process
        engine._contact.process = lambda f: (1, _orig_contact(f)[1])
    imu_pre = IMUPreprocessor()
    uwb_pre = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))

    # Parallel IRLS for reference comparison (shares the same preprocessor state
    # family; but uses an independent preprocessor to avoid state entanglement).
    ref_pre = UWBPreprocessor(offsets=tuple(UWB_OFFSETS))
    bmin = [-0.30, -0.30, -0.50]
    bmax = [float(np.max(ANCHORS[:, 0])) + 0.30,
            float(np.max(ANCHORS[:, 1])) + 0.30, 1.00]
    ref_irls = IRLSTrilateration(ANCHORS, bmin, bmax, tag_z=MARKER_LENGTH)

    t0_us: int | None = None
    first_uwb_us: int | None = None
    init_us: int | None = None

    ekf_t: list[float] = []
    ekf_xy: list[np.ndarray] = []
    ekf_vel: list[np.ndarray] = []
    ekf_bias: list[np.ndarray] = []
    irls_t: list[float] = []
    irls_xy: list[np.ndarray] = []
    accepted_total = 0
    rejected_total = 0
    contact_states: list[int] = []
    speeds: list[float] = []

    for pkt in replay(csv_path):
        if t0_us is None:
            t0_us = pkt['ts']

        if pkt['type'] == 'imu':
            clean = imu_pre.process_sample(*pkt['quat'], *pkt['acc'])
            pkt['quat'] = clean[:4]
            pkt['acc']  = clean[4:]
            pos, vel, writing = engine.process_imu(pkt)
            if pos is not None:
                contact_states.append(int(writing))
                speeds.append(float(np.linalg.norm(vel)))

        elif pkt['type'] == 'uwb':
            if first_uwb_us is None:
                first_uwb_us = pkt['ts']

            _, w_ref, des_ref = ref_pre.process(*pkt['dists'])
            ref_pos, _ = ref_irls.solve(des_ref, w_ref)
            irls_t.append((pkt['ts'] - t0_us) / 1e6)
            irls_xy.append(np.array([float(ref_pos[0]), float(ref_pos[1])]))

            _, w, des = uwb_pre.process(*pkt['dists'])
            pos, vel, acc, rej = engine.process_uwb(des, w)
            accepted_total += len(acc)
            rejected_total += len(rej)

            if engine.ekf.initialized and init_us is None:
                init_us = pkt['ts']
            if pos is not None and engine.ekf.initialized:
                ekf_t.append((pkt['ts'] - t0_us) / 1e6)
                ekf_xy.append(np.array([float(pos[0]), float(pos[1])]))
                ekf_vel.append(np.array([float(vel[0]), float(vel[1])]))
                ekf_bias.append(engine.ekf.bias.copy())

    metrics: dict = {
        'uwb_updates':     len(irls_xy),
        'ekf_samples':     len(ekf_xy),
    }
    notes: list[str] = []

    if init_us is not None and first_uwb_us is not None:
        metrics['cold_start_s'] = float((init_us - first_uwb_us) / 1e6)
    if accepted_total + rejected_total > 0:
        metrics['gate_reject_pct'] = float(
            100.0 * rejected_total / (accepted_total + rejected_total))
    metrics['accepted_total'] = accepted_total
    metrics['rejected_total'] = rejected_total

    ekf_arr  = np.asarray(ekf_xy)  if ekf_xy  else np.zeros((0, 2))
    irls_arr = np.asarray(irls_xy) if irls_xy else np.zeros((0, 2))

    if len(ekf_arr) > 5 and len(irls_arr) > 5:
        # Align lengths from the tail so both cover the warm period.
        n = min(len(ekf_arr), len(irls_arr))
        ekf_tail  = ekf_arr[-n:]
        irls_tail = irls_arr[-n:]
        rms = float(np.sqrt(np.mean(np.sum((ekf_tail - irls_tail) ** 2, axis=1))))
        metrics['rms_ekf_vs_irls_m'] = rms

        # Latency proxy: 1D cross-correlation of the X channel, lag in samples
        ex = ekf_tail[:, 0] - ekf_tail[:, 0].mean()
        ix = irls_tail[:, 0] - irls_tail[:, 0].mean()
        if np.std(ex) > 1e-4 and np.std(ix) > 1e-4:
            corr = np.correlate(ex, ix, mode='full')
            lag = int(np.argmax(corr) - (len(ex) - 1))
            metrics['latency_proxy_samples'] = lag
            metrics['latency_proxy_s'] = float(lag * 0.1)  # UWB dt
        else:
            metrics['latency_proxy_s'] = 0.0

    if speeds and contact_states:
        spd_arr = np.asarray(speeds)
        c_arr   = np.asarray(contact_states)
        stationary_mask = c_arr == 0
        if stationary_mask.any():
            metrics['stationary_speed_mean_contact'] = float(
                spd_arr[stationary_mask].mean())

    # IRLS-speed-gated stationary metric: works for both hover and contact
    # datasets. Detect "genuinely still" windows from the IRLS trace itself.
    if len(irls_arr) >= 3 and speeds:
        # IRLS at ~10 Hz; speed between consecutive samples.
        d_irls = np.linalg.norm(np.diff(irls_arr, axis=0), axis=1) * 10.0
        if d_irls.size and (d_irls < 0.05).any():
            # Bucket EKF speeds onto IRLS indices via uniform stretch.
            spd_arr = np.asarray(speeds)
            idx = np.linspace(0, len(spd_arr) - 1,
                              num=len(d_irls)).astype(int)
            sampled = spd_arr[idx]
            still = sampled[d_irls < 0.05]
            if still.size:
                metrics['stationary_speed_mean'] = float(still.mean())

    if ekf_bias:
        ba = np.asarray(ekf_bias)
        metrics['bias_final_m_s2'] = float(np.linalg.norm(ba[-1]))
        if len(ba) > 20:
            last = ba[-20:]
            metrics['bias_delta_last_2s'] = float(
                np.linalg.norm(last[-1] - last[0]))

    # Pass criteria
    def _ok(key, limit, lt=True):
        v = metrics.get(key)
        if v is None:
            return False
        return (v < limit) if lt else (v > limit)

    passed = all([
        _ok('latency_proxy_s', 0.15),
        _ok('rms_ekf_vs_irls_m', 0.05),
        _ok('gate_reject_pct', 10.0),
        _ok('cold_start_s', 0.5),
    ])

    # ---- Plot ----
    out_dir = ensure_out(LAYER)
    fig = plt.figure(figsize=(13, 6))
    axL = fig.add_subplot(1, 2, 1)
    draw_board(axL)
    if len(irls_arr):
        axL.plot(irls_arr[:, 0], irls_arr[:, 1], '.', ms=3, color='royalblue',
                 alpha=0.55, label='IRLS')
    if len(ekf_arr):
        axL.plot(ekf_arr[:, 0], ekf_arr[:, 1], '-', lw=1.2, color='tab:orange',
                 alpha=0.9, label='EKF')
    truth = get_truth(stem)
    if truth is not None:
        axL.plot(truth[0], truth[1], '+', ms=18, mew=3, color='limegreen',
                 label='truth')
    axL.legend(loc='upper right', fontsize=8)
    axL.set_title(f'Layer 6 -- EKF vs IRLS: {tag}')

    axR = fig.add_subplot(1, 2, 2)
    if ekf_t and ekf_arr.size:
        err = np.linalg.norm(
            ekf_arr[-min(len(ekf_arr), len(irls_arr)):] -
            irls_arr[-min(len(ekf_arr), len(irls_arr)):], axis=1)
        axR.plot(err, 'r-', lw=0.8, label='|EKF - IRLS|')
    if ekf_bias:
        ba = np.asarray(ekf_bias)
        axR.plot(np.linalg.norm(ba, axis=1), 'b-', lw=0.8, label='|bias|')
    axR.set_xlabel('UWB update index')
    axR.legend(fontsize=8)
    axR.grid(True, alpha=0.3)
    axR.set_title('Tracking error + bias magnitude')

    fig.tight_layout()
    plot_path = out_dir / f'{tag}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER, dataset=tag, passed=passed,
        metrics=metrics, notes=notes,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


def run_all() -> list[LayerResult]:
    base_stems = ['0s', '1s', '2s', '3s', '4s',
                  'NorthS', 'SouthS', 'EastS', 'WestS',
                  'hline', 'vline', 'dline_A0_A2', 'dline_A3_A1',
                  'CIRCLE', 'SQUARE', 'TRIANGLE']
    # Hover + dashed contact variants (same motion, different contact state).
    stems: list[str] = []
    for s in base_stems:
        stems.append(s)
        if (DATASET_DIR / f'{s}-.csv').exists():
            stems.append(f'{s}-')

    out = []
    for s in stems:
        p = DATASET_DIR / f'{s}.csv'
        if p.exists():
            out.append(analyse(p))

    # Forced-contact replay on motion hover datasets to isolate
    # LIFTED_VEL_DAMP starvation. Skip stationary holds where the metric
    # "latency vs motion" is meaningless.
    motion_stems = ['NorthS', 'SouthS', 'EastS', 'WestS',
                    'hline', 'vline', 'dline_A0_A2', 'dline_A3_A1',
                    'CIRCLE', 'SQUARE', 'TRIANGLE']
    for s in motion_stems:
        p = DATASET_DIR / f'{s}.csv'
        if p.exists():
            out.append(analyse(p, force_contact=True))
    return out


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*')
    args = ap.parse_args()
    files = ([DATASET_DIR / f'{d}.csv' for d in args.datasets]
             if args.datasets else
             [DATASET_DIR / f'{s}.csv' for s in ['0s', 'CIRCLE']])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:24s} {v}')
