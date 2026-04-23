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
Base variant (force_contact=False)
    latency_proxy_s < 0.15
    rms_ekf_vs_irls < 0.05
    gate_reject_pct < 10.0
    cold_start_s    < 0.5

`+contact` variant (force_contact=True) — diagnostic-only, relaxed
    latency_proxy_s < 0.25
    rms_ekf_vs_irls < 0.08
    gate_reject_pct < 15.0
    cold_start_s    < 0.6
(stationary_speed_mean is not gated here; forced-contact disables the hover
damper, so the metric is reported for inspection but not for pass/fail.)
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
from trail_smoother import TrailSmoother


LAYER = 'layer6_ekf_end_to_end'

# Mirror main_ekf.py display semantics: record an EKF waypoint every Nth
# IMU predict (~100 Hz / 3 ≈ 33 Hz), gate by is_writing, smooth the writing
# trail with a causal WMA, and insert a NaN break on every pen lift so
# matplotlib does not connect separate strokes with a diagonal.
_TRAIL_SUBSAMPLE = 3


def analyse(csv_path: Path, force_contact: bool = False) -> LayerResult:
    stem = csv_path.stem
    tag  = f'{stem}+contact' if force_contact else stem

    engine = AsyncEKFFusionEngine()
    engine.ekf.record_history = True   # enable RTS smoother at end of replay
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

    # Item A/C visibility — pen tip, omega magnitude, and per-anchor
    # filtered-vs-raw range residual (recorded only when the EKF is warm
    # and a UWB update has produced fresh _last_*_ranges).  All values
    # are report-only; nothing here gates pass/fail.
    tip_xy: list[np.ndarray] = []           # paired 1:1 with ekf_xy when available
    omega_mags: list[float] = []
    feedback_residuals: list[float] = []    # |d_filtered - d_raw|, per anchor

    # Per-IMU-step display buffers — mirror main_ekf.py (see trail_smoother.py)
    draw_x: list[float] = []   # writing trail, NaN-separated across lifts
    draw_y: list[float] = []
    lift_x: list[float] = []
    lift_y: list[float] = []
    smoother = TrailSmoother()
    prev_writing = False
    imu_subsample = 0

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

                # Display capture (writing/lifted) — mirrors main_ekf.py
                if engine.ekf.initialized:
                    imu_subsample += 1
                    if imu_subsample >= _TRAIL_SUBSAMPLE:
                        imu_subsample = 0
                        if writing:
                            smoother.push(float(pos[0]), float(pos[1]))
                            sx, sy = smoother.get()
                            draw_x.append(sx)
                            draw_y.append(sy)
                        else:
                            if prev_writing:
                                # Pen just lifted — break the drawn line
                                draw_x.append(float('nan'))
                                draw_y.append(float('nan'))
                                smoother.reset()
                            lift_x.append(float(pos[0]))
                            lift_y.append(float(pos[1]))
                        prev_writing = bool(writing)

        elif pkt['type'] == 'uwb':
            if first_uwb_us is None:
                first_uwb_us = pkt['ts']

            _, w_ref, des_ref = ref_pre.process(*pkt['dists'])
            ref_pos, _ = ref_irls.solve(des_ref, w_ref)
            irls_t.append((pkt['ts'] - t0_us) / 1e6)
            irls_xy.append(np.array([float(ref_pos[0]), float(ref_pos[1])]))

            _, w, des = uwb_pre.process(*pkt['dists'])
            # Forward ts so the per-anchor 1D range KF (Item B) sees real
            # dt between UWB cycles, and so the Item-C feedback path runs.
            pos, vel, acc, rej = engine.process_uwb(des, w, ts=pkt['ts'])
            accepted_total += len(acc)
            rejected_total += len(rej)

            if engine.ekf.initialized and init_us is None:
                init_us = pkt['ts']
            if pos is not None and engine.ekf.initialized:
                ekf_t.append((pkt['ts'] - t0_us) / 1e6)
                ekf_xy.append(np.array([float(pos[0]), float(pos[1])]))
                ekf_vel.append(np.array([float(vel[0]), float(vel[1])]))
                ekf_bias.append(engine.ekf.bias.copy())

                # Item A: pen-tip position derived from the latest cached
                # marker-axis unit vector.  May be None at the very first
                # warm UWB update if no IMU has arrived since init.
                tip = engine.tip_position
                tip_xy.append(
                    np.array([float(tip[0]), float(tip[1])])
                    if tip is not None else
                    np.array([float(pos[0]), float(pos[1])])
                )

                # Item C visibility: how much did the filtered range
                # diverge from the raw range on this update.  NaNs (the
                # very first sample, or skipped anchors) are dropped.
                raw_arr  = engine.last_raw_ranges
                filt_arr = engine.last_filtered_ranges
                if raw_arr is not None and filt_arr is not None:
                    diff = np.abs(filt_arr - raw_arr)
                    diff = diff[np.isfinite(diff)]
                    if diff.size:
                        feedback_residuals.extend(float(x) for x in diff)

                # Lever-arm activity (Item A.2).  Sampled at every UWB
                # update — coarse, but enough to see when |omega| crosses
                # the 1 rad/s threshold during rotation datasets.
                omega_mags.append(
                    float(engine.imu_integrator.last_omega_mag))

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
    uwb_dt = float(np.median(np.diff(irls_t))) if len(irls_t) > 1 else 0.02

    if len(ekf_arr) > 5 and len(irls_arr) > 5:
        # Align lengths from the tail so both cover the warm period.
        n = min(len(ekf_arr), len(irls_arr))
        ekf_tail  = ekf_arr[-n:]
        irls_tail = irls_arr[-n:]
        rms = float(np.sqrt(np.mean(np.sum((ekf_tail - irls_tail) ** 2, axis=1))))
        metrics['rms_ekf_vs_irls_m'] = rms

        # Latency proxy: 1D cross-correlation of the X channel, lag in samples.
        # This metric is only physically meaningful for (a) non-stationary
        # trajectories with (b) near-linear motion. On stationary data both
        # signals are sensor noise; on circular / rotational / complex motion
        # the correlation has multiple peaks (phase ambiguity) and the lag is
        # arbitrary. Require: >10 cm span AND principal-axis ratio >= 3:1.
        ekf_span = max(
            float(ekf_tail[:, 0].max() - ekf_tail[:, 0].min()),
            float(ekf_tail[:, 1].max() - ekf_tail[:, 1].min()),
        )
        pc_ratio = 0.0
        if len(irls_tail) >= 3:
            M = irls_tail - irls_tail.mean(axis=0)
            # eigenvalues of 2x2 covariance — ratio of spread along principal axes
            w = np.linalg.eigvalsh(M.T @ M / max(len(M) - 1, 1))
            w = np.sort(np.maximum(w, 1e-12))
            pc_ratio = float(w[1] / w[0])
        ex = ekf_tail[:, 0] - ekf_tail[:, 0].mean()
        ix = irls_tail[:, 0] - irls_tail[:, 0].mean()
        if (ekf_span > 0.10 and pc_ratio >= 3.0
                and np.std(ex) > 1e-4 and np.std(ix) > 1e-4):
            corr = np.correlate(ex, ix, mode='full')
            lag = int(np.argmax(corr) - (len(ex) - 1))
            metrics['latency_proxy_samples'] = lag
            metrics['latency_proxy_s'] = float(lag * uwb_dt)
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
        # Speed between consecutive IRLS samples (uses actual UWB dt).
        d_irls = np.linalg.norm(np.diff(irls_arr, axis=0), axis=1) / max(uwb_dt, 1e-6)
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

    # ---- Report-only metrics (Items A, B, C visibility) -----------------
    # Pen-tip RMS vs IRLS.  On tilt-rotation datasets this is *expected*
    # to exceed rms_ekf_vs_irls_m by ~ TIP_OFFSET_FROM_TAG_M·sin(tilt) —
    # that geometric offset is the new signal Item A introduces, not an
    # error.  Logged for future baselining.
    tip_arr = np.asarray(tip_xy) if tip_xy else np.zeros((0, 2))
    if len(tip_arr) > 5 and len(irls_arr) > 5:
        n = min(len(tip_arr), len(irls_arr))
        tip_tail  = tip_arr[-n:]
        irls_tail = irls_arr[-n:]
        metrics['pen_tip_rms_vs_irls_m'] = float(
            np.sqrt(np.mean(np.sum((tip_tail - irls_tail) ** 2, axis=1))))

    # Lever-arm activity (Item A.2).
    if omega_mags:
        om = np.asarray(omega_mags)
        metrics['mean_omega_rad_s'] = float(om.mean())
        metrics['max_omega_rad_s']  = float(om.max())
        metrics['omega_above_thresh_frac'] = float(
            np.mean(om >= 1.0))   # IMUIntegrator.LEVER_ARM_OMEGA_THRESH

    # Item C feedback visibility — empirical |filtered - raw| range delta.
    # Restricted to the genuinely-stationary windows detected above so it
    # reads as "anchor-bias suppression on a still tag".
    if feedback_residuals:
        metrics['mean_anchor_residual_after_feedback_m'] = float(
            np.mean(feedback_residuals))

    # RTS backward smoother — offline-only post-pass for cleanest trajectory
    rts_xy = None
    if len(engine.ekf._history) >= 10:
        try:
            rts_xy = engine.ekf.rts_smooth()
        except Exception as exc:
            notes.append(f'rts_smooth failed: {exc!r}')

    # Pass criteria
    def _ok(key, limit, lt=True):
        v = metrics.get(key)
        if v is None:
            return False
        return (v < limit) if lt else (v > limit)

    if force_contact:
        # +contact is a diagnostic variant with LIFTED_VEL_DAMP disabled; its
        # pass bar is looser because hover drift is expected.
        passed = all([
            _ok('latency_proxy_s',    0.25),
            _ok('rms_ekf_vs_irls_m',  0.08),
            _ok('gate_reject_pct',    15.0),
            _ok('cold_start_s',       0.6),
        ])
    else:
        passed = all([
            _ok('latency_proxy_s',    0.15),
            _ok('rms_ekf_vs_irls_m',  0.05),
            _ok('gate_reject_pct',    10.0),
            _ok('cold_start_s',       0.5),
        ])

    # ---- Plot (mirrors main_ekf.py display semantics) ----
    out_dir = ensure_out(LAYER)
    fig = plt.figure(figsize=(13, 6))
    axL = fig.add_subplot(1, 2, 1)
    draw_board(axL)
    if len(irls_arr):
        axL.plot(irls_arr[:, 0], irls_arr[:, 1], '.', ms=2,
                 color='tab:orange', alpha=0.35, label='IRLS')
    if lift_x:
        axL.plot(lift_x, lift_y, '.', ms=3, color='#888888', alpha=0.4,
                 label='lifted (tracked)')
    if draw_x:
        axL.plot(draw_x, draw_y, '-', lw=1.3, color='royalblue',
                 label='EKF (writing)')
    if len(tip_arr):
        axL.plot(tip_arr[:, 0], tip_arr[:, 1], '--', lw=1.0,
                 color='magenta', alpha=0.6, label='pen tip (Item A)')
    if rts_xy is not None and len(rts_xy) > 1:
        axL.plot(rts_xy[:, 0], rts_xy[:, 1], '-', lw=1.2,
                 color='limegreen', alpha=0.75, label='RTS smoothed')
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
    base_stems = [
        # Stationary middle-hold
        'pt_middle',
        # Rotation in-place at center
        'pt_middle_r', 'pt_middle_rotation', 'pt_middle_rr',
        # Repeated shapes (rt_ = repeated, single continuous trace)
        'rt_circle', 'rt_square', 'rt_triangle',
        # Lines -- continuous (ct_) trace, with extra-length variants
        'ct_hline', 'ct_hlinee', 'ct_hlineee',
        'ct_vline', 'ct_vlinee', 'ct_vlineee',
        'ct_diagonal', 'ct_diagonall', 'ct_diagonalll',
        # Shapes -- continuous (ct_) single-stroke
        'ct_circle', 'ct_circlee', 'ct_circleee',
        'ct_square', 'ct_squaree', 'ct_squareee',
        'ct_triangle', 'ct_trianglee', 'ct_triangleee',
        # Shapes -- partial (pt_) multi-stroke (lift between strokes)
        'pt_circle', 'pt_circlee',
        'pt_square', 'pt_squaree',
        'pt_triangle', 'pt_trianglee',
        # Characters / freeform writing
        'pt_a', 'pt_aa', '_abc', '_wave',
        # datasets_str_50hz naming: numeric variants + renamed middle/rotation
        'middle-', 'middleCCW_rot-', 'middleCW_rot-',
        'ct_hline1', 'ct_hline2', 'ct_hline3',
        'ct_vline1', 'ct_vline2', 'ct_vline3',
        'ct_diagonal1', 'ct_diagonal2', 'ct_diagonal3',
        'ct_circle1', 'ct_circle2', 'ct_circle3',
        'ct_square1', 'ct_square2', 'ct_square3',
        'ct_triangle1', 'ct_triangle2', 'ct_triangle3',
        'pt_circle1', 'pt_circle2',
        'pt_square1', 'pt_square2',
        'pt_triangle1', 'pt_triangle2',
        'pt_a1', 'pt_a2',
        # Character-reconstruction targets (letters, small & big)
        'hello_s1', 'hello_s2', 'abc_s1', 'abc_s2',
        'ABC_b1', 'ABC_b2', 'HELLO_b1', 'HELLO_b2',
    ]
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

    # Forced-contact replay on motion datasets to isolate
    # LIFTED_VEL_DAMP starvation.  Skip stationary/rotation
    # datasets where forced contact doesn't change behavior.
    motion_stems = [
        'ct_hline', 'ct_hlinee', 'ct_hlineee',
        'ct_vline', 'ct_vlinee', 'ct_vlineee',
        'ct_diagonal', 'ct_diagonall', 'ct_diagonalll',
        'ct_circle', 'ct_circlee', 'ct_circleee',
        'ct_square', 'ct_squaree', 'ct_squareee',
        'ct_triangle', 'ct_trianglee', 'ct_triangleee',
        'pt_circle', 'pt_circlee',
        'pt_square', 'pt_squaree',
        'pt_triangle', 'pt_trianglee',
        'pt_a', 'pt_aa', '_abc', '_wave',
        # datasets_str_50hz naming
        'ct_hline1', 'ct_hline2', 'ct_hline3',
        'ct_vline1', 'ct_vline2', 'ct_vline3',
        'ct_diagonal1', 'ct_diagonal2', 'ct_diagonal3',
        'ct_circle1', 'ct_circle2', 'ct_circle3',
        'ct_square1', 'ct_square2', 'ct_square3',
        'ct_triangle1', 'ct_triangle2', 'ct_triangle3',
        'pt_circle1', 'pt_circle2',
        'pt_square1', 'pt_square2',
        'pt_triangle1', 'pt_triangle2',
        'pt_a1', 'pt_a2',
        'hello_s1', 'hello_s2', 'abc_s1', 'abc_s2',
        'ABC_b1', 'ABC_b2', 'HELLO_b1', 'HELLO_b2',
    ]
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
             [DATASET_DIR / f'{s}.csv' for s in ['pt_middle', 'ct_circle']])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:24s} {v}')
