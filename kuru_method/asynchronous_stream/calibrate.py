"""
calibrate.py  —  PolyCast Anchor Bias Calibrator (Async Stream)
===============================================================
Measures the UWB tag at several KNOWN positions on the whiteboard and
solves for the per-anchor distance offsets that minimise position error
across all calibration points simultaneously.

Why this is necessary
---------------------
UWB ranging has a systematic per-anchor bias (antenna delay mismatch,
cable length, multipath from mounting).  A 2 cm residual bias on one
anchor causes ~1 cm of position error.  With four anchors, errors add
up to 5-15 cm — enough to distort shapes.  Calibration addresses the
root cause directly.  Averaging does NOT help because bias is constant.

How to measure anchor positions
-------------------------------
    1.  Measure ANTENNA to ANTENNA, not PCB center to PCB center.
        UWB ranging is time-of-flight between antenna phase centers.
        On the AI Thinker BU03, the antenna is the PCB trace antenna
        near the DW3000 chip (one end of the board).

    2.  Place A0's antenna at your chosen origin (0, 0).
        Measure from A0's antenna to each other anchor's antenna.

    3.  The Z coordinate (0.07 m) is how far the anchor sits in front
        of the whiteboard surface.

    4.  Record the (X, Y, Z) of each anchor in the ANCHORS array below.
        These MUST match the values in ekf_fusion.py, main_ekf.py, etc.

Usage
-----
    python calibrate.py              # live serial from receiver

Procedure
---------
1.  Measure and mark at least 5 positions on your whiteboard.
    Recommended points:
        - All four anchor corners (hold marker tip exactly at the corner)
        - Board centre
        - Midpoints of top and bottom edges
    More points = more robust offset estimates.

2.  For each point:
    a. Hold the marker still, tip touching the mark, for ~3 seconds.
    b. Press ENTER when ready to sample; press ENTER again to advance.

3.  The script outputs a one-line Python snippet you paste directly into
    main_ekf.py (and uwb_validator.py, imu_validator.py) as the new
    UWBPreprocessor offsets.

Output
------
    # Suggested offsets:
    uwb_cleaner = UWBPreprocessor(offsets=(-0.187, -0.203, -0.161, -0.394))

    # Per-anchor residuals at calibration points (sanity check)

How the optimisation works
--------------------------
At each calibration point the tag is at a known 3D position:
    tag = (tip_x, tip_y, MARKER_LENGTH)

The raw UWB reading for anchor i should be:
    d_expected[i] = ||tag - anchor[i]||

The offset for anchor i is the constant correction such that:
    d_raw[i] + offset[i] ~ d_expected[i]

We collect all (raw, expected) pairs across all calibration points and
solve for the offset vector that minimises the total squared error using
scipy.optimize.minimize (Nelder-Mead).
"""

import sys
import time
import numpy as np
from scipy.optimize import minimize
import serial

# -------------------------------------------------------------------------
#  CONFIG — shared constants from config.py
# -------------------------------------------------------------------------
from config import SERIAL_PORT, BAUD_RATE, MARKER_LENGTH, ANCHORS

# Number of UWB packets to average at each calibration position
# ~3 seconds at 10 Hz UWB rate = ~30 packets
SAMPLES_PER_POINT = 50


# -------------------------------------------------------------------------
#  CALIBRATION POINTS
#  Define the known TIP positions (x, y) in metres from A0's antenna.
#  These are the points you will physically mark and touch with the
#  marker tip.  Add or remove rows as needed — 5 minimum recommended.
#
#  Tip: anchor corners are the most geometrically informative since
#  the expected distances are well-separated and unambiguous.
# -------------------------------------------------------------------------
# Board extents derived from ANCHORS — keeps calibration points in sync
# with config.py if anchors are ever remeasured.
_X_MAX = float(ANCHORS[1, 0])   # A1 x
_Y_MAX = float(ANCHORS[2, 1])   # A2 y

CALIBRATION_POINTS = [
    # Interior grid — all points >0.30m from every anchor to avoid
    # near-field UWB effects (DW3000 minimum reliable range ~0.15-0.30m).
    # Corner points REMOVED: at 0.14m from nearest anchor they produce
    # unreliable ranging data that corrupts the constant-offset model.
    #
    # label          tip_x            tip_y
    ("Centre",        _X_MAX * 0.50,  _Y_MAX * 0.50),
    ("Mid-left",      0.00,           _Y_MAX * 0.50),
    ("Mid-right",     _X_MAX,         _Y_MAX * 0.50),
    ("Mid-bottom",    _X_MAX * 0.50,  0.00),
    ("Mid-top",       _X_MAX * 0.50,  _Y_MAX),
    ("Quarter-BL",    _X_MAX * 0.25,  _Y_MAX * 0.25),
    ("Quarter-BR",    _X_MAX * 0.75,  _Y_MAX * 0.25),
    ("Quarter-TL",    _X_MAX * 0.25,  _Y_MAX * 0.75),
    ("Quarter-TR",    _X_MAX * 0.75,  _Y_MAX * 0.75),
]


# -------------------------------------------------------------------------
#  HELPERS
# -------------------------------------------------------------------------

def expected_dists(tip_x: float, tip_y: float) -> np.ndarray:
    """True 3D distances from tag (rear of marker) to each anchor."""
    tag = np.array([tip_x, tip_y, MARKER_LENGTH])
    return np.array([np.linalg.norm(tag - a) for a in ANCHORS])


def collect_samples_live(n: int) -> np.ndarray:
    """
    Read n raw UWB distance packets from the async serial stream.
    Filters for 'U' lines only, skips IMU lines.
    Returns shape (n, 4) array of raw UWB distances.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    except Exception as e:
        print(f"  Serial error: {e}")
        sys.exit(1)

    time.sleep(2)  # wait for Arduino reset
    samples = []
    print(f"  Collecting {n} UWB samples...", end="", flush=True)

    while len(samples) < n:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split(',')

        # Async UWB format: U,seq,d0,d1,d2,d3,ts
        if parts[0] != 'U' or len(parts) != 7:
            continue

        try:
            d = [float(parts[2]), float(parts[3]),
                 float(parts[4]), float(parts[5])]
            if all(v > 0.05 for v in d):
                samples.append(d)
                if len(samples) % 5 == 0:
                    print(".", end="", flush=True)
        except ValueError:
            continue

    ser.close()
    print(" done.")
    return np.array(samples)


# -------------------------------------------------------------------------
#  OPTIMISATION
# -------------------------------------------------------------------------

def compute_optimal_offsets(
    raw_means: np.ndarray,
    expected: np.ndarray,
    initial_offsets: np.ndarray = None,
) -> tuple:
    """
    Parameters
    ----------
    raw_means       : shape (N_points, 4)  mean raw distances per point
    expected        : shape (N_points, 4)  true distances per point
    initial_offsets : shape (4,)           starting guess (or None)

    Returns
    -------
    offsets   : shape (4,) optimal per-anchor offsets
    residuals : shape (N_points, 4) post-correction per-point residuals
    """
    if initial_offsets is None:
        # Naive starting guess: mean of (expected - raw) across all points
        initial_offsets = np.mean(expected - raw_means, axis=0)

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


# -------------------------------------------------------------------------
#  MAIN
# -------------------------------------------------------------------------

def main():
    print()
    print("=" * 60)
    print("  PolyCast Anchor Bias Calibrator (Async Stream)")
    print("=" * 60)
    print(f"\n  Anchor positions (antenna locations):")
    for i, a in enumerate(ANCHORS):
        print(f"    A{i}: ({a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f})")
    print(f"\n  Marker length (tip to tag): {MARKER_LENGTH} m")
    print(f"  UWB samples per point: {SAMPLES_PER_POINT} "
          f"(~{SAMPLES_PER_POINT / 10:.0f} sec at 10 Hz)")
    print()
    print("  IMPORTANT: Measure anchor positions antenna-to-antenna.")
    print("  On the AI Thinker BU03, the antenna is the PCB trace")
    print("  antenna near the DW3000 chip (one end of the board).")
    print()

    all_raw_means = []
    all_expected  = []
    labels        = []

    for label, tip_x, tip_y in CALIBRATION_POINTS:
        print(f"--- Point: {label}  ({tip_x:.3f}, {tip_y:.3f}) ---")
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
    expected  = np.array(all_expected)     # (N, 4)

    # -- Solve for globally optimal offsets --------------------------------
    print("Computing globally optimal offsets across all calibration points...")
    offsets, residuals = compute_optimal_offsets(raw_means, expected)

    # -- Print results -----------------------------------------------------
    print()
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print()
    print(f"  Optimal offsets:")
    for i, o in enumerate(offsets):
        print(f"    Anchor {i}: {o:+.4f} m")

    print()
    print("  Per-anchor residuals after correction:")
    print(f"  {'Point':<18} {'A0':>8} {'A1':>8} {'A2':>8} {'A3':>8}")
    print("  " + "-" * 52)
    for i, lbl in enumerate(labels):
        row = ("  " + f"{lbl:<18}"
               + "".join(f"{residuals[i,j]*100:+7.1f}cm" for j in range(4)))
        print(row)

    print()
    per_anchor_rms = np.sqrt(np.mean(residuals**2, axis=0))
    print(f"  RMS per anchor: "
          f"[{', '.join(f'{v*100:.1f} cm' for v in per_anchor_rms)}]")

    overall_rms = np.sqrt(np.mean(residuals**2))
    print(f"  Overall RMS:    {overall_rms*100:.1f} cm  (target: <2 cm)")

    print()
    print("-" * 60)
    print("  Update UWB_OFFSETS in config.py:")
    print("-" * 60)
    offsets_str = ", ".join(f"{o:.4f}" for o in offsets)
    print(f"  UWB_OFFSETS = ({offsets_str})")
    print()

    if overall_rms > 0.03:
        print("  WARNING: RMS > 3 cm — consider:")
        print("    - Re-measuring anchor positions antenna-to-antenna")
        print("    - Ensuring the marker was held perfectly still")
        print("    - Adding more calibration points near board edges")
        print("    - Checking for NLOS obstructions (hand blocking anchor)")
    else:
        print("  Calibration looks good.")
    print()


if __name__ == "__main__":
    main()
