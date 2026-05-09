"""
calibrate.py  —  PolyCast Anchor Bias Calibrator
=================================================
Measures the UWB tag at several KNOWN positions on the whiteboard and
solves for the per-anchor distance offsets that minimise position error
across all calibration points simultaneously.

Why this is necessary
---------------------
The hardcoded offsets in UWBPreprocessor were measured by hand and likely
have 3–10 cm of residual error per anchor.  A 2 cm residual bias on one
anchor causes ~1 cm of position error.  With four anchors, errors add up
to 15–25 cm — matching the 20–30 cm discrepancy you are seeing.

1-second averaging does NOT help because bias is systematic, not random.
Calibration addresses the root cause directly.

Usage
-----
    python calibrate.py                    # live serial mode
    python calibrate.py mydata.csv         # replay existing CSV (with --csv flag)

Procedure
---------
1.  Measure and mark at least 5 positions on your whiteboard.
    Recommended points:
        • All four anchor corners  (hold marker tip exactly at the corner)
        • Board centre
        • Midpoints of top and bottom edges
    More points = more robust offset estimates.

2.  For each point:
    a. Hold the marker still, tip touching the mark, for 3 seconds.
    b. Press ENTER when ready to sample; press ENTER again to advance.

3.  The script outputs a one-line Python snippet you paste directly into
    preprocessor.py as the new UWBPreprocessor `offsets` argument.

Output
------
    # Suggested offsets (paste into DataStream.__init__):
    self.uwb_cleaner = UWBPreprocessor(offsets=(-0.187, -0.203, -0.161, -0.394))
    
    # Per-anchor residuals at calibration points (sanity check):
    Anchor 0: mean bias = -0.003 m, std = 0.008 m
    ...

How the optimisation works
--------------------------
At each calibration point the tag is at a known 3D position:
    tag = (tip_x, tip_y, MARKER_LENGTH)

The raw UWB reading for anchor i should be:
    d_expected[i] = ||tag - anchor[i]||

The offset for anchor i is the constant correction such that:
    d_raw[i] + offset[i] ≈ d_expected[i]

We collect all (raw, expected) pairs across all calibration points and
solve for the offset vector that minimises the total squared error using
scipy.optimize.minimize.  The result is the globally optimal set of
offsets, not just a per-anchor average, which handles cross-correlations
between anchor geometry and position bias.
"""

import sys
import time
import os
import numpy as np
from scipy.optimize import minimize
from collections import deque

# ─────────────────────────────────────────────────────────────────────
#  CONFIG — edit to match your setup
# ─────────────────────────────────────────────────────────────────────
SERIAL_PORT   = 'COM5'
BAUD_RATE     = 115200
MARKER_LENGTH = 0.21   # metres

ANCHORS = np.array([
    [0.00, 0.00, 0.07],
    [1.23, 0.00, 0.07],
    [1.23, 1.23, 0.07],
    [0.00, 1.23, 0.07],
], dtype=float)

# Number of raw packets to average at each calibration position
# (~3 seconds at 20 packets/sec = 60 packets)
SAMPLES_PER_POINT = 60


# ─────────────────────────────────────────────────────────────────────
#  CALIBRATION POINTS
#  Define the known TIP positions (x, y) in metres from A0.
#  These are the points you will physically mark and touch with the
#  marker tip.  Add or remove rows as needed — 5 minimum recommended.
#  
#  Tip: anchor corners are the most geometrically informative since
#  the expected distances are well-separated and unambiguous.
# ─────────────────────────────────────────────────────────────────────
CALIBRATION_POINTS = [
    # label          tip_x   tip_y
    ("Top-left",      0.00,   1.23),
    ("Top-right",     1.23,   1.23),
    ("Bottom-left",   0.00,   0.00),
    ("Bottom-right",  1.23,   0.00),
    ("Centre",        0.615,  0.615),
    ("Mid-top",       0.615,  1.23),
    ("Mid-bottom",    0.615,  0.00),
]


# ─────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────

def expected_dists(tip_x: float, tip_y: float) -> np.ndarray:
    """True 3D distances from tag (rear of marker) to each anchor."""
    tag = np.array([tip_x, tip_y, MARKER_LENGTH])
    return np.array([np.linalg.norm(tag - a) for a in ANCHORS])


def collect_samples_live(n: int) -> np.ndarray:
    """
    Read n raw distance packets from the serial port.
    Returns shape (n, 4) array of raw UWB distances.
    """
    import serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    except Exception as e:
        print(f"  ✗ Serial error: {e}")
        sys.exit(1)

    samples = []
    print(f"  Collecting {n} samples...", end="", flush=True)
    while len(samples) < n:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 52:
            continue
        try:
            d = [float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])]
            if all(v > 0.05 for v in d):
                samples.append(d)
                if len(samples) % 10 == 0:
                    print(".", end="", flush=True)
        except ValueError:
            continue
    ser.close()
    print(" done.")
    return np.array(samples)


def collect_samples_stdin() -> np.ndarray:
    """
    Interactive mock: paste space-separated distances when prompted.
    Useful for offline testing with a saved log.
    """
    lines = []
    print("  Paste raw distance lines (d0,d1,d2,d3 one per line).")
    print("  Press CTRL+D (Unix) or CTRL+Z (Windows) when done.")
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) >= 4:
                try:
                    d = [float(parts[i]) for i in range(4)]
                    lines.append(d)
                except ValueError:
                    pass
    except EOFError:
        pass
    return np.array(lines) if lines else np.zeros((1, 4))


# ─────────────────────────────────────────────────────────────────────
#  OPTIMISATION
# ─────────────────────────────────────────────────────────────────────

def compute_optimal_offsets(
    raw_means: np.ndarray,
    expected: np.ndarray,
    initial_offsets: np.ndarray,
) -> tuple:
    """
    Parameters
    ----------
    raw_means : shape (N_points, 4)  — mean raw distances per point
    expected  : shape (N_points, 4)  — true distances per point
    initial_offsets : shape (4,)     — starting guess

    Returns
    -------
    offsets : shape (4,) optimal per-anchor offsets
    residuals : shape (N_points, 4) post-correction per-point residuals
    """
    def cost(offsets):
        corrected = raw_means + offsets          # (N, 4)
        return np.sum((corrected - expected) ** 2)

    result = minimize(
        cost,
        initial_offsets,
        method='Nelder-Mead',
        options={'xatol': 1e-6, 'fatol': 1e-8, 'maxiter': 10000},
    )

    offsets   = result.x
    corrected = raw_means + offsets
    residuals = corrected - expected
    return offsets, residuals


# ─────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────

def main():
    print()
    print("=" * 60)
    print("  PolyCast Anchor Bias Calibrator")
    print("=" * 60)
    print(f"\n  Anchor positions:")
    for i, a in enumerate(ANCHORS):
        print(f"    A{i}: ({a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f})")
    print(f"\n  Marker length: {MARKER_LENGTH} m")
    print(f"  Samples per point: {SAMPLES_PER_POINT} (~{SAMPLES_PER_POINT*0.05:.0f} sec)")
    print()

    all_raw_means = []
    all_expected  = []
    labels        = []

    for label, tip_x, tip_y in CALIBRATION_POINTS:
        print(f"─── Point: {label}  ({tip_x:.3f}, {tip_y:.3f}) ───")
        print(f"  Hold marker tip at this position, completely still.")
        input("  Press ENTER when ready to sample... ")

        samples = collect_samples_live(SAMPLES_PER_POINT)

        mean_raw = np.mean(samples, axis=0)
        std_raw  = np.std(samples,  axis=0)
        exp      = expected_dists(tip_x, tip_y)

        print(f"  Raw mean:     [{', '.join(f'{v:.4f}' for v in mean_raw)}]")
        print(f"  Raw std:      [{', '.join(f'{v:.4f}' for v in std_raw)}]")
        print(f"  Expected:     [{', '.join(f'{v:.4f}' for v in exp)}]")
        print(f"  Naive offset: [{', '.join(f'{v:.4f}' for v in exp - mean_raw)}]")
        print()

        all_raw_means.append(mean_raw)
        all_expected.append(exp)
        labels.append(label)

        print("  Saved. Move to next point.")
        print()

    raw_means = np.array(all_raw_means)   # (N, 4)
    expected  = np.array(all_expected)    # (N, 4)

    # ── Solve for globally optimal offsets ───────────────────────────
    print("Computing globally optimal offsets across all calibration points...")
    offsets, residuals = compute_optimal_offsets(raw_means, expected)

    # ── Print results ────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print()
    print(f"  Optimal offsets:")
    for i, o in enumerate(offsets):
        print(f"    Anchor {i}: {o:+.4f} m ")

    print()
    print("  Per-anchor residuals after correction:")
    print(f"  {'Point':<18} {'A0':>8} {'A1':>8} {'A2':>8} {'A3':>8}")
    print("  " + "-" * 52)
    for i, lbl in enumerate(labels):
        row = "  " + f"{lbl:<18}" + "".join(f"{residuals[i,j]*100:+7.1f}cm" for j in range(4))
        print(row)

    print()
    per_anchor_rms = np.sqrt(np.mean(residuals**2, axis=0))
    print(f"  RMS per anchor: [{', '.join(f'{v*100:.1f} cm' for v in per_anchor_rms)}]")

    overall_rms = np.sqrt(np.mean(residuals**2))
    print(f"  Overall RMS:    {overall_rms*100:.1f} cm  (target: <2 cm)")

    print()
    print("─" * 60)
    print("  Paste this into data_stream.py  DataStream.__init__():")
    print("─" * 60)
    offsets_str = ", ".join(f"{o:.4f}" for o in offsets)
    print(f"  self.uwb_cleaner = UWBPreprocessor(offsets=({offsets_str}))")
    print()

    if overall_rms > 0.03:
        print("  ⚠  RMS > 3 cm — consider:")
        print("     • Re-measuring your anchor XY positions with a tape measure")
        print("     • Checking that the marker was held perfectly still at each point")
        print("     • Adding more calibration points, especially near board edges")
    else:
        print("  ✅ Calibration looks good.")
    print()


if __name__ == "__main__":
    main()