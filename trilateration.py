import numpy as np
from collections import deque
from scipy.optimize import least_squares


# ===============================================================
# CONFIGURATION (EASY TO MODIFY)
# ===============================================================
TRILAT_CONFIG = {

    # Anchor positions in (x, y)
    "anchors": np.array([
        [1.75, 0.00],   # Anchor 0
        [1.75, 1.61],   # Anchor 1
        [0.00, 0.00]    # Anchor 2
    ]),

    # Sliding window length for median filtering
    "window_size": 5,

    # Maximum allowed instantaneous jump (meters)
    "max_range_jump": 0.40,

    # Base assumed measurement noise
    "base_variance": 0.015,

    # Physical movement bounds on the wall plane (2D)
    "bounds": {
        "x_min": -0.5,
        "x_max": 2.5,
        "y_min": -0.5,
        "y_max": 2.5,
    }
}



# ===============================================================
# INTERNAL STATE
# ===============================================================
# Sliding windows for distance smoothing
dist_windows = [
    deque(maxlen=TRILAT_CONFIG["window_size"]),
    deque(maxlen=TRILAT_CONFIG["window_size"]),
    deque(maxlen=TRILAT_CONFIG["window_size"])
]

# Previous distances (for spike rejection)
prev_dists = None



# ===============================================================
# PREPROCESSING: Reject spikes + median smoothing
# ===============================================================
def preprocess_distances(raw_dists):
    """
    - Replaces sudden absurd jumps (> threshold)
    - Stores data in sliding window
    - Applies median filtering
    """
    global prev_dists

    clean_dists = []

    # First sample — initialize baseline
    if prev_dists is None:
        prev_dists = raw_dists.copy()

    for i, dist in enumerate(raw_dists):

        # --- Outlier rejection ---
        if abs(dist - prev_dists[i]) > TRILAT_CONFIG["max_range_jump"]:
            dist = prev_dists[i]   # replace with previous

        # Update previous value
        prev_dists[i] = dist

        # Add to sliding window
        dist_windows[i].append(dist)

        # Median filter output
        clean_dists.append(np.median(dist_windows[i]))

    return np.array(clean_dists)



# ===============================================================
# WLS NON-LINEAR LEAST SQUARES TRILATERATION (2D)
# ===============================================================
def trilaterate_2D_WLS(distances):
    """
    Performs trilateration in 2D (z=0 plane)
    using Weighted Non-Linear Least Squares.
    """

    anchors = TRILAT_CONFIG["anchors"]

    # --- Estimate per-anchor variance from sliding windows ---
    variances = np.zeros_like(distances)

    for i in range(len(distances)):
        if len(dist_windows[i]) > 1:
            window = np.array(dist_windows[i])
            variances[i] = np.var(window)
        else:
            variances[i] = TRILAT_CONFIG["base_variance"]

    # Avoid zero division
    variances = np.maximum(variances, 1e-6)

    # Weight is inverse variance
    weights = 1.0 / variances

    # --- Optimization target ---
    def residuals(x, anchors, d, w):
        predicted = np.linalg.norm(anchors - x, axis=1)
        return (predicted - d) * np.sqrt(w)

    # Start at center of anchors
    x0 = np.mean(anchors, axis=0)

    result = least_squares(
        residuals,
        x0,
        args=(anchors, distances, weights),
        bounds=([
            TRILAT_CONFIG["bounds"]["x_min"],
            TRILAT_CONFIG["bounds"]["y_min"]
        ], [
            TRILAT_CONFIG["bounds"]["x_max"],
            TRILAT_CONFIG["bounds"]["y_max"]
        ])
    )

    return result.x   # (x, y)



# ===============================================================
# PUBLIC API ENTRY POINT
# ===============================================================
def solve_position_from_distances(raw_dists):
    """
    Call this function from main.py
    - Receives [dist0, dist1, dist2]
    - Cleans the measurements
    - Performs robust trilateration
    Returns (x, y)
    """

    cleaned = preprocess_distances(raw_dists)
    position = trilaterate_2D_WLS(cleaned)
    return position
