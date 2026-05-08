"""
layer5_force_contact.py -- FSR contact detection health.

Uses the production ForceContactDetector.  Reports raw-force histogram,
number of transitions, glitch rate, and mean contact-episode duration.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from _common        import (ensure_out, collect, LayerResult, DATASET_DIR,
                             list_datasets)
from force_detector import ForceContactDetector


LAYER = 'layer5_force_contact'


def analyse(csv_path: Path) -> LayerResult:
    stem = csv_path.stem
    imu_pkts, _ = collect(csv_path)
    if not imu_pkts:
        return LayerResult(LAYER, stem, passed=False,
                           metrics={'imu_count': 0}, notes=['no IMU packets'])

    det = ForceContactDetector()
    raw_seq: list[float] = []
    smooth_seq: list[float] = []
    state_seq: list[int] = []
    for p in imu_pkts:
        state, smooth = det.process(p['force'], ts=p.get('ts'))
        raw_seq.append(p['force'])
        smooth_seq.append(smooth)
        state_seq.append(int(state))

    raw   = np.asarray(raw_seq, dtype=float)
    state = np.asarray(state_seq, dtype=int)

    # Transitions
    diff = np.diff(state)
    up   = int(np.sum(diff == 1))
    down = int(np.sum(diff == -1))

    # Contact episode durations (samples where state == 1)
    episodes = []
    i = 0
    n = len(state)
    while i < n:
        if state[i] == 1:
            j = i
            while j < n and state[j] == 1:
                j += 1
            episodes.append(j - i)
            i = j
        else:
            i += 1
    ep = np.asarray(episodes) if episodes else np.zeros(0)
    mean_ep_samples = float(ep.mean()) if len(ep) else 0.0
    min_ep_samples  = int(ep.min())   if len(ep) else 0

    # Glitch: episode shorter than the combined debounce envelope (should be
    # rare/zero). Convert the time-based debounce thresholds into a sample
    # count using the rate_profile FSR cadence.
    from config import FSR_DT_NOM_S as _FSR_DT_NOM_S
    debounce_envelope_samples = int(np.ceil(
        (ForceContactDetector.DEBOUNCE_ON_S
         + ForceContactDetector.DEBOUNCE_OFF_S) / max(_FSR_DT_NOM_S, 1e-6)))
    glitch_count = int(np.sum(ep < debounce_envelope_samples))

    metrics = {
        'imu_count':           len(imu_pkts),
        'raw_min':              float(raw.min()),
        'raw_max':              float(raw.max()),
        'raw_median':           float(np.median(raw)),
        'up_transitions':       up,
        'down_transitions':     down,
        'contact_episodes':     len(episodes),
        'mean_episode_samples': mean_ep_samples,
        'min_episode_samples':  min_ep_samples,
        'glitch_count':         glitch_count,
        'contact_duty_pct':     float(100.0 * state.mean()),
    }

    # Pass: no glitches and (if the user pressed at all) minimum episode > 20 samples
    passed = glitch_count == 0 and (len(episodes) == 0 or min_ep_samples >= 20)

    # Plot
    out_dir = ensure_out(LAYER)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    t = np.arange(len(raw)) * 0.010
    axes[0].plot(t, raw,        'grey',      lw=0.5, label='raw ADC')
    axes[0].plot(t, smooth_seq, 'tab:blue',  lw=1.0, label='EMA')
    axes[0].axhline(ForceContactDetector.THRESH_ENTER, color='green',
                    ls='--', lw=0.8, label='enter')
    axes[0].axhline(ForceContactDetector.THRESH_EXIT,  color='orange',
                    ls='--', lw=0.8, label='exit')
    axes[0].axhline(ForceContactDetector.NOISE_FLOOR,  color='red',
                    ls=':',  lw=0.8, label='noise floor')
    axes[0].set_ylabel('force (ADC)')
    axes[0].legend(fontsize=7, loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Layer 5 -- force detector: {stem}')

    axes[1].plot(t, state, 'k-', lw=1.0)
    axes[1].set_ylabel('contact state')
    axes[1].set_xlabel('time (s)')
    axes[1].set_yticks([0, 1])
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    plot_path = out_dir / f'{stem}.png'
    fig.savefig(plot_path, dpi=110)
    plt.close(fig)

    return LayerResult(
        layer=LAYER, dataset=stem, passed=passed,
        metrics=metrics,
        plot=str(plot_path.relative_to(Path(__file__).resolve().parent)),
    )


def run_all() -> list[LayerResult]:
    # Run contact detection against every CSV in the dataset folder.
    # Naming conventions have drifted across captures (legacy dashed,
    # numbered variants in datasets_str_50hz, char files) — safest is to
    # analyse everything and let the dataset contents drive the result.
    return [analyse(p) for p in list_datasets()]


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', nargs='*')
    args = ap.parse_args()
    files = ([DATASET_DIR / f'{d}.csv' for d in args.datasets]
             if args.datasets else [DATASET_DIR / '_wave.csv'])
    for p in files:
        r = analyse(p)
        print(r.summary_line())
        for k, v in r.metrics.items():
            print(f'    {k:20s} {v}')
