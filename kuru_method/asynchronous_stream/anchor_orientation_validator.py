"""
anchor_orientation_validator.py  —  PolyCast Anchor Orientation Optimizer
==========================================================================
Find the best physical orientation of the UWB anchors by running labeled
trials and comparing per-anchor quality metrics.

Workflow
--------
1.  Start the program.
2.  For each orientation you want to test:
      a. Physically set the anchors to that orientation.
      b. Enter a trial name (e.g. "flat", "tilted-up-15", "rotated-90").
      c. Visit each reference point on the board, hold the marker tip at
         the marked position, press ENTER to collect samples.
      d. The program prints per-anchor metrics and a composite score.
3.  After all trials, a side-by-side comparison declares the winner and
    a bar chart visualises the scores.

Per-anchor metrics (raw UWB quality)
------------------------------------
    Bias      : mean(raw + offset - expected)      — calibration residual
    Stddev    : RMS of per-point stddevs           — ranging jitter (NLOS sensitive)
    RMSE      : sqrt(mean(bias^2)) across points   — overall accuracy
    Outlier % : samples > 3σ from per-point mean   — multipath spikes
    Dropout % : missing UWB packets (seq gaps)     — NLOS blockage
    Score     : composite 0–100                    — higher = better

The composite score weights stddev and NLOS indicators (outliers,
dropouts) more heavily than bias since calibration can be re-run
but NLOS is a physical-layout problem.

Configuration
-------------
Uses ANCHORS, UWB_OFFSETS, MARKER_LENGTH from config.py.
Edit REFERENCE_POINTS and SAMPLES_PER_POINT below if needed.
"""

import sys
import time
import queue
import threading
import numpy as np
import matplotlib.pyplot as plt

from config import (
    SERIAL_PORT, BAUD_RATE,
    ANCHORS, MARKER_LENGTH, UWB_OFFSETS,
)
from data_parser import AsyncDataParser


# -------------------------------------------------------------------------
#  CONFIGURATION
# -------------------------------------------------------------------------

# Reference points — tip positions (x, y) in metres from A0's antenna.
# These are the physical locations you visit during each trial.
# More points = more robust orientation score. 3 is the minimum.
#
# Extents derived from ANCHORS so they stay in sync with config.py.
_X_MAX = float(ANCHORS[1, 0])   # A1 x
_Y_MAX = float(ANCHORS[2, 1])   # A2 y

REFERENCE_POINTS = [
    # label,         tip_x,            tip_y
    ("Centre",        _X_MAX * 0.50,   _Y_MAX * 0.50),
    ("Mid-left",      0.00,            _Y_MAX * 0.50),
    ("Mid-right",     _X_MAX,          _Y_MAX * 0.50),
    ("Mid-bottom",    _X_MAX * 0.50,   0.00),
    ("Mid-top",       _X_MAX * 0.50,   _Y_MAX),
]

# Number of UWB packets to collect per reference point
# ~5 seconds at 50 Hz UWB rate = 250 packets
SAMPLES_PER_POINT = 250

# Timeout per point (seconds) — abort if no data
POINT_TIMEOUT_S = 30.0


# -------------------------------------------------------------------------
#  METRICS
# -------------------------------------------------------------------------

def expected_dists(tip_x: float, tip_y: float) -> np.ndarray:
    """True 3D distances from tag (rear of marker) to each anchor."""
    tag = np.array([tip_x, tip_y, MARKER_LENGTH])
    return np.array([np.linalg.norm(tag - a) for a in ANCHORS])


def compute_point_metrics(samples: np.ndarray,
                          expected: np.ndarray,
                          offsets: tuple) -> dict:
    """
    Parameters
    ----------
    samples  : (N, 4) raw UWB distances
    expected : (4,)   true 3D distances to each anchor
    offsets  : (4,)   calibration offsets from config.py

    Returns
    -------
    dict with per-anchor (4,) arrays:
        mean, std, bias, outlier_rate
    """
    corrected = samples + np.array(offsets)                   # (N, 4)

    mean_dist = np.mean(corrected, axis=0)                    # (4,)
    std_dist  = np.std(corrected, axis=0)                     # (4,)
    bias      = mean_dist - expected                          # (4,)

    # Per-anchor outlier fraction (>3σ from per-point mean)
    outliers = np.zeros(4)
    for i in range(4):
        if std_dist[i] < 1e-6:
            continue
        deviations = np.abs(corrected[:, i] - mean_dist[i])
        outliers[i] = float(np.sum(deviations > 3.0 * std_dist[i])) / len(corrected)

    return {
        'mean'         : mean_dist,
        'std'          : std_dist,
        'bias'         : bias,
        'outlier_rate' : outliers,
    }


def aggregate_trial(point_metrics: list) -> dict:
    """
    Combine multiple point metrics into per-anchor trial totals.

    Returns dict with (4,) arrays: bias_avg, std_avg, rmse, outlier_avg
    """
    all_bias = np.array([m['bias']         for _, m in point_metrics])  # (P, 4)
    all_std  = np.array([m['std']          for _, m in point_metrics])
    all_out  = np.array([m['outlier_rate'] for _, m in point_metrics])

    # Signed mean bias across points (systematic offset)
    bias_avg = np.mean(all_bias, axis=0)
    # RMS of per-point stddevs (overall ranging noise)
    std_avg  = np.sqrt(np.mean(all_std ** 2, axis=0))
    # RMSE across points (overall accuracy)
    rmse     = np.sqrt(np.mean(all_bias ** 2, axis=0))
    # Mean outlier fraction
    out_avg  = np.mean(all_out, axis=0)

    return {
        'bias'    : bias_avg,
        'std'     : std_avg,
        'rmse'    : rmse,
        'outlier' : out_avg,
    }


def anchor_score(bias: float, std: float, rmse: float,
                 outlier: float, dropout: float) -> float:
    """
    Composite 0–100 quality score for a single anchor.
    Higher = better. Weights stddev and NLOS indicators most.
    """
    bias_t    = max(0.0, 1.0 - abs(bias) / 0.10)    # 0 at 10cm bias
    std_t     = max(0.0, 1.0 - std      / 0.05)    # 0 at 5cm stddev
    rmse_t    = max(0.0, 1.0 - rmse     / 0.10)    # 0 at 10cm RMSE
    out_t     = max(0.0, 1.0 - outlier  / 0.20)    # 0 at 20% outliers
    drop_t    = max(0.0, 1.0 - dropout  / 0.30)    # 0 at 30% dropout

    # Weights: stddev + NLOS indicators > bias (bias can be recalibrated)
    return (0.15 * bias_t +
            0.25 * std_t  +
            0.20 * rmse_t +
            0.20 * out_t  +
            0.20 * drop_t) * 100.0


# -------------------------------------------------------------------------
#  BACKGROUND SERIAL DRAIN
# -------------------------------------------------------------------------
# The main thread blocks on input() between points/trials. Without a
# continuous reader, Windows' COM buffer overflows and the port handle
# enters an error state that only a replug can clear. This drain thread
# keeps serial consumed at all times and buffers UWB packets for
# collect_samples() to pull from.

_uwb_q     = queue.Queue(maxsize=5000)
_stop_evt  = threading.Event()
_drain_thr = None


def _drain_loop(parser: AsyncDataParser):
    while not _stop_evt.is_set():
        pkt = parser.get_packet()
        if pkt == 'EOF':
            break
        if pkt is None:
            time.sleep(0.002)
            continue
        if isinstance(pkt, dict) and pkt.get('type') == 'uwb':
            try:
                _uwb_q.put_nowait(pkt)
            except queue.Full:
                try: _uwb_q.get_nowait()
                except queue.Empty: pass
                try: _uwb_q.put_nowait(pkt)
                except queue.Full: pass


def start_drain(parser: AsyncDataParser):
    global _drain_thr
    _stop_evt.clear()
    _drain_thr = threading.Thread(target=_drain_loop, args=(parser,), daemon=True)
    _drain_thr.start()


def stop_drain():
    _stop_evt.set()
    if _drain_thr is not None:
        _drain_thr.join(timeout=1.0)


# -------------------------------------------------------------------------
#  SAMPLE COLLECTION
# -------------------------------------------------------------------------

def collect_samples(parser: AsyncDataParser, n: int):
    """
    Collect n valid UWB packets and compute dropout rate from seq gaps.

    Returns
    -------
    samples      : (n, 4) ndarray of raw UWB distances
    dropout_rate : float  fraction of packets lost during this window
    """
    # Drop any packets buffered during the input() wait — we only want
    # samples taken strictly after the user said "ready".
    while True:
        try: _uwb_q.get_nowait()
        except queue.Empty: break

    samples        = []
    uwb_recv_start = parser.uwb_received
    uwb_lost_start = parser.uwb_lost

    t_start = time.time()
    print(f"  Collecting {n} UWB samples", end="", flush=True)

    while len(samples) < n:
        if time.time() - t_start > POINT_TIMEOUT_S:
            print(f"\n  WARNING: Timeout — only got {len(samples)}/{n} samples")
            break

        try:
            pkt = _uwb_q.get(timeout=0.1)
        except queue.Empty:
            continue

        d = pkt['dists']
        if all(v > 0.05 for v in d):
            samples.append(list(d))
            if len(samples) % 5 == 0:
                print(".", end="", flush=True)

    recv_total = parser.uwb_received - uwb_recv_start
    lost_total = parser.uwb_lost     - uwb_lost_start
    total = recv_total + lost_total
    dropout = (lost_total / total) if total > 0 else 0.0

    print(f" done. (dropout {dropout*100:.1f}%)")
    return np.array(samples), dropout


# -------------------------------------------------------------------------
#  TRIAL EXECUTION
# -------------------------------------------------------------------------

def run_trial(parser: AsyncDataParser, trial_name: str) -> dict:
    """Walk through all reference points for one orientation trial."""
    print()
    print("=" * 60)
    print(f"  TRIAL: {trial_name}")
    print("=" * 60)

    point_metrics = []
    dropouts      = []

    for label, tx, ty in REFERENCE_POINTS:
        print(f"\n--- Point: {label}  ({tx:.3f}, {ty:.3f}) ---")
        print("  Hold marker tip at this position, completely still.")
        input("  Press ENTER when ready to sample... ")

        samples, dropout = collect_samples(parser, SAMPLES_PER_POINT)
        if len(samples) < SAMPLES_PER_POINT // 2:
            print(f"  Too few samples, skipping this point.")
            continue

        expected = expected_dists(tx, ty)
        m = compute_point_metrics(samples, expected, UWB_OFFSETS)
        point_metrics.append((label, m))
        dropouts.append(dropout)

        print(f"  Per-anchor stddev:  [{', '.join(f'{v*100:.2f}cm' for v in m['std'])}]")
        print(f"  Per-anchor bias:    [{', '.join(f'{v*100:+.2f}cm' for v in m['bias'])}]")

    if not point_metrics:
        print("\n  ERROR: No valid points collected for this trial.")
        return None

    # Aggregate per-anchor
    agg = aggregate_trial(point_metrics)
    dropout_avg = float(np.mean(dropouts))

    # Per-anchor scores
    scores = np.array([
        anchor_score(agg['bias'][i], agg['std'][i], agg['rmse'][i],
                     agg['outlier'][i], dropout_avg)
        for i in range(4)
    ])
    overall = float(np.mean(scores))

    # Print trial summary
    print()
    print("-" * 72)
    print(f"  Trial '{trial_name}' — per-anchor metrics")
    print("-" * 72)
    print(f"  {'Anchor':<7}{'Bias':>10}{'Std':>10}{'RMSE':>10}{'Out%':>8}{'Drop%':>8}{'Score':>9}")
    print("  " + "-" * 60)
    for i in range(4):
        print(f"  A{i:<6}"
              f"{agg['bias'][i]*100:>+8.2f}cm"
              f"{agg['std'][i]*100:>8.2f}cm"
              f"{agg['rmse'][i]*100:>8.2f}cm"
              f"{agg['outlier'][i]*100:>7.1f}%"
              f"{dropout_avg*100:>7.1f}%"
              f"{scores[i]:>8.1f}")
    print("  " + "-" * 60)
    print(f"  OVERALL SCORE: {overall:.1f} / 100")
    print()

    # Per-point bias breakdown — used to localise position-dependent multipath.
    print("-" * 72)
    print(f"  Trial '{trial_name}' — per-point bias (cm from expected)")
    print("-" * 72)
    print(f"  {'Point':<17}{'A0 bias':>10}{'A1 bias':>10}{'A2 bias':>10}{'A3 bias':>10}")
    print("  " + "-" * 55)
    worst_val  = 0.0
    worst_info = ""
    for label, m in point_metrics:
        row = f"  {label:<17}"
        for i in range(4):
            b = m['bias'][i] * 100
            row += f"{b:>+9.2f} "
            if abs(b) > abs(worst_val):
                worst_val  = b
                worst_info = f"A{i} @ {label}"
        print(row)
    print("  " + "-" * 55)
    print(f"  Worst anchor-point: {worst_info} ({worst_val:+.2f} cm)")
    print()

    return {
        'name'    : trial_name,
        'bias'    : agg['bias'],
        'std'     : agg['std'],
        'rmse'    : agg['rmse'],
        'outlier' : agg['outlier'],
        'dropout' : dropout_avg,
        'scores'  : scores,
        'overall' : overall,
    }


# -------------------------------------------------------------------------
#  COMPARISON & VISUALISATION
# -------------------------------------------------------------------------

def print_comparison(trials: list):
    """Side-by-side comparison of all trials."""
    print()
    print("=" * 72)
    print("  TRIAL COMPARISON")
    print("=" * 72)
    print(f"  {'Trial':<22}{'Overall':>10}{'A0':>9}{'A1':>9}{'A2':>9}{'A3':>9}")
    print("  " + "-" * 68)

    best_idx = int(np.argmax([t['overall'] for t in trials]))

    for idx, t in enumerate(trials):
        marker = "  <-- WINNER" if idx == best_idx else ""
        print(f"  {t['name']:<22}"
              f"{t['overall']:>9.1f} "
              f"{t['scores'][0]:>8.1f} "
              f"{t['scores'][1]:>8.1f} "
              f"{t['scores'][2]:>8.1f} "
              f"{t['scores'][3]:>8.1f}{marker}")

    print()
    print(f"  RECOMMENDED ORIENTATION: {trials[best_idx]['name']}")
    print()


def plot_comparison(trials: list):
    """Grouped bar chart: per-anchor + overall scores for each trial."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.canvas.manager.set_window_title('Anchor Orientation — Trial Comparison')

    n_trials = len(trials)
    labels   = ['A0', 'A1', 'A2', 'A3', 'Overall']
    x        = np.arange(len(labels))
    width    = 0.8 / n_trials

    colors = plt.cm.viridis(np.linspace(0.2, 0.85, n_trials))

    for i, t in enumerate(trials):
        values = list(t['scores']) + [t['overall']]
        offset = (i - (n_trials - 1) / 2) * width
        bars = ax.bar(x + offset, values, width, label=t['name'],
                      color=colors[i], edgecolor='white', linewidth=0.5)
        for b, v in zip(bars, values):
            ax.text(b.get_x() + b.get_width() / 2, v + 1, f'{v:.0f}',
                    ha='center', va='bottom', fontsize=8, color='white')

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylim(0, 110)
    ax.set_ylabel('Quality Score (0–100, higher is better)', fontsize=11)
    ax.set_title('Anchor Orientation Quality — Trial Comparison',
                 fontsize=13, weight='bold')
    ax.axhline(50, color='#888888', linestyle='--', lw=1, alpha=0.5,
               label='50 (baseline)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.2, axis='y')

    plt.tight_layout()
    plt.show()


# -------------------------------------------------------------------------
#  MAIN
# -------------------------------------------------------------------------

def main():
    print()
    print("=" * 60)
    print("  PolyCast Anchor Orientation Validator")
    print("=" * 60)
    print(f"\n  Anchor positions (from config.py):")
    for i, a in enumerate(ANCHORS):
        print(f"    A{i}: ({a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f})")
    print(f"\n  UWB offsets: {UWB_OFFSETS}")
    print(f"  Marker length: {MARKER_LENGTH} m")
    print(f"  Reference points: {len(REFERENCE_POINTS)}")
    print(f"  Samples per point: {SAMPLES_PER_POINT}")
    print()
    print("  For each orientation you want to test, run a trial.")
    print("  Enter empty trial name to finish and see comparison.")
    print()

    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path='')
    if not parser.connect():
        sys.exit(1)

    start_drain(parser)

    trials = []

    try:
        while True:
            name = input("Trial name (empty to finish): ").strip()
            if not name:
                break
            if any(t['name'] == name for t in trials):
                print(f"  Name '{name}' already used. Pick a different one.")
                continue

            result = run_trial(parser, name)
            if result is not None:
                trials.append(result)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        stop_drain()
        parser.close()

    if len(trials) == 0:
        print("\nNo trials collected.")
        return
    if len(trials) == 1:
        print(f"\nOnly one trial '{trials[0]['name']}' collected — "
              f"overall score {trials[0]['overall']:.1f}/100.")
        print("Run more trials with different orientations to compare.")
        return

    print_comparison(trials)
    plot_comparison(trials)


if __name__ == "__main__":
    main()
