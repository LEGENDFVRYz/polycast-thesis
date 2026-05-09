import csv
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

GROUND_TRUTH_CSV = BASE_DIR / "abcdefgh_ground_truth_points.csv"
POLYCAST_CSV = BASE_DIR / "abcdefgh.csv"
CORRECTED_IMAGE_PATH = BASE_DIR / "abcdefgh_corrected_truth.png"

ERROR_CSV = BASE_DIR / "abcdefgh_polycast_error_distances.csv"
REPORT_JSON = BASE_DIR / "abcdefgh_comparison_report.json"
OVERLAY_OUTPUT = BASE_DIR / "abcdefgh_comparison_overlay.png"
ERROR_HISTOGRAM = BASE_DIR / "abcdefgh_error_histogram.png"

# Same values you used in make_ground_truth.py
BOARD_WIDTH_MM = 1900
BOARD_HEIGHT_MM =1200

OUTPUT_WIDTH_PX = 1900
OUTPUT_HEIGHT_PX = int(OUTPUT_WIDTH_PX * BOARD_HEIGHT_MM / BOARD_WIDTH_MM)

# ------------------------------------------------------------
# IMPORTANT: Set these based on your PolyCast output
# ------------------------------------------------------------

# Column names in your PolyCast CSV
POLYCAST_X_COL = "x"
POLYCAST_Y_COL = "y"

# Choose one:
# "mm"         -> x,y are already in millimeters
# "m"          -> x,y are in meters
# "normalized" -> x,y are 0 to 1 values across the board
POLYCAST_UNITS = "m"

# Choose one:
# "top_left"    -> origin is top-left, y increases downward
# "bottom_left" -> origin is bottom-left, y increases upward
POLYCAST_ORIGIN = "bottom_left"

# If your PolyCast CSV has pen_down column, only pen_down == 1 is used.
PEN_DOWN_COL = "pen_down"


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def require_file(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")


def convert_polycast_to_mm(poly_df: pd.DataFrame) -> pd.DataFrame:
    if POLYCAST_X_COL not in poly_df.columns or POLYCAST_Y_COL not in poly_df.columns:
        raise ValueError(
            f"PolyCast CSV must contain columns '{POLYCAST_X_COL}' and '{POLYCAST_Y_COL}'. "
            f"Found columns: {list(poly_df.columns)}"
        )

    df = poly_df.copy()

    # Keep only pen-down points if available
    if PEN_DOWN_COL in df.columns:
        df = df[df[PEN_DOWN_COL].astype(float) == 1].copy()

    x = df[POLYCAST_X_COL].astype(float).to_numpy()
    y = df[POLYCAST_Y_COL].astype(float).to_numpy()

    if POLYCAST_UNITS == "mm":
        x_mm = x
        y_mm = y

    elif POLYCAST_UNITS == "m":
        x_mm = x * 1000.0
        y_mm = y * 1000.0

    elif POLYCAST_UNITS == "normalized":
        x_mm = x * BOARD_WIDTH_MM
        y_mm = y * BOARD_HEIGHT_MM

    else:
        raise ValueError("POLYCAST_UNITS must be 'mm', 'm', or 'normalized'.")

    # Convert to same coordinate system as ground_truth_points.csv:
    # top-left origin, y increases downward.
    if POLYCAST_ORIGIN == "top_left":
        y_mm_converted = y_mm

    elif POLYCAST_ORIGIN == "bottom_left":
        y_mm_converted = y_mm 

    else:
        raise ValueError("POLYCAST_ORIGIN must be 'top_left' or 'bottom_left'.")

    out = pd.DataFrame({
        "x_mm": x_mm,
        "y_mm": y_mm_converted
    })

    if "timestamp_ms" in df.columns:
        out["timestamp_ms"] = df["timestamp_ms"].values

    return out


def mm_to_px(x_mm, y_mm):
    x_px = x_mm * OUTPUT_WIDTH_PX / BOARD_WIDTH_MM
    y_px = (BOARD_HEIGHT_MM - y_mm) * OUTPUT_HEIGHT_PX / BOARD_HEIGHT_MM
    return x_px, y_px


def summarize_distances(name, distances):
    return {
        f"{name}_count": int(len(distances)),
        f"{name}_mean_mm": float(np.mean(distances)),
        f"{name}_median_mm": float(np.median(distances)),
        f"{name}_p90_mm": float(np.percentile(distances, 90)),
        f"{name}_p95_mm": float(np.percentile(distances, 95)),
        f"{name}_max_mm": float(np.max(distances)),
    }


# ============================================================
# 1. LOAD FILES
# ============================================================

require_file(GROUND_TRUTH_CSV)
require_file(POLYCAST_CSV)
require_file(CORRECTED_IMAGE_PATH)

truth_df = pd.read_csv(GROUND_TRUTH_CSV)
poly_raw_df = pd.read_csv(POLYCAST_CSV)

if "x_mm" not in truth_df.columns or "y_mm" not in truth_df.columns:
    raise ValueError("ground_truth_points.csv must contain x_mm and y_mm columns.")

poly_df = convert_polycast_to_mm(poly_raw_df)

if len(poly_df) == 0:
    raise ValueError("No PolyCast points found. Check pen_down column or CSV contents.")

print(f"Ground-truth points: {len(truth_df)}")
print(f"PolyCast points: {len(poly_df)}")


# ============================================================
# 2. BUILD NEAREST-NEIGHBOR TREES
# ============================================================

truth_points = truth_df[["x_mm", "y_mm"]].to_numpy()
poly_points = poly_df[["x_mm", "y_mm"]].to_numpy()

truth_tree = cKDTree(truth_points)
poly_tree = cKDTree(poly_points)


# ============================================================
# 3. POLYCAST -> TRUTH ERROR
# ============================================================
# This answers:
# How far is each PolyCast point from the real writing?

poly_to_truth_dist, nearest_truth_idx = truth_tree.query(poly_points, k=1)

poly_error_df = poly_df.copy()
poly_error_df["nearest_truth_x_mm"] = truth_points[nearest_truth_idx, 0]
poly_error_df["nearest_truth_y_mm"] = truth_points[nearest_truth_idx, 1]
poly_error_df["error_mm"] = poly_to_truth_dist

poly_error_df.to_csv(ERROR_CSV, index=False)


# ============================================================
# 4. TRUTH -> POLYCAST ERROR
# ============================================================
# This catches missing strokes.
# If the real writing has areas with no nearby PolyCast point,
# this error will be high.

truth_to_poly_dist, _ = poly_tree.query(truth_points, k=1)


# ============================================================
# 5. REPORT METRICS
# ============================================================

report = {}
report.update(summarize_distances("polycast_to_truth", poly_to_truth_dist))
report.update(summarize_distances("truth_to_polycast", truth_to_poly_dist))

with open(REPORT_JSON, "w") as f:
    json.dump(report, f, indent=2)

print("\n=== PolyCast -> Truth Error ===")
print(f"Median error: {report['polycast_to_truth_median_mm']:.2f} mm")
print(f"90th percentile error: {report['polycast_to_truth_p90_mm']:.2f} mm")
print(f"95th percentile error: {report['polycast_to_truth_p95_mm']:.2f} mm")
print(f"Max error: {report['polycast_to_truth_max_mm']:.2f} mm")

print("\n=== Truth -> PolyCast Error ===")
print(f"Median missing-stroke error: {report['truth_to_polycast_median_mm']:.2f} mm")
print(f"90th percentile missing-stroke error: {report['truth_to_polycast_p90_mm']:.2f} mm")
print(f"Max missing-stroke error: {report['truth_to_polycast_max_mm']:.2f} mm")


# ============================================================
# 6. CREATE OVERLAY IMAGE
# ============================================================

img = cv2.imread(str(CORRECTED_IMAGE_PATH))
if img is None:
    raise FileNotFoundError(f"Could not read: {CORRECTED_IMAGE_PATH}")

overlay = img.copy()

# Draw ground-truth points lightly in red
truth_x_px, truth_y_px = mm_to_px(truth_points[:, 0], truth_points[:, 1])
for x_px, y_px in zip(truth_x_px.astype(int), truth_y_px.astype(int)):
    if 0 <= x_px < OUTPUT_WIDTH_PX and 0 <= y_px < OUTPUT_HEIGHT_PX:
        overlay[y_px, x_px] = (0, 0, 255)

# Draw PolyCast points in blue
poly_x_px, poly_y_px = mm_to_px(poly_points[:, 0], poly_points[:, 1])
for x_px, y_px in zip(poly_x_px.astype(int), poly_y_px.astype(int)):
    if 0 <= x_px < OUTPUT_WIDTH_PX and 0 <= y_px < OUTPUT_HEIGHT_PX:
        cv2.circle(overlay, (x_px, y_px), 2, (255, 0, 0), -1)

cv2.imwrite(str(OVERLAY_OUTPUT), overlay)
print(f"\nSaved overlay: {OVERLAY_OUTPUT}")


# ============================================================
# 7. CREATE ERROR HISTOGRAM
# ============================================================

plt.figure(figsize=(8, 5))
plt.hist(poly_to_truth_dist, bins=30)
plt.xlabel("Error distance in mm")
plt.ylabel("Number of PolyCast points")
plt.title("PolyCast to Ground Truth Error Distribution")
plt.grid(True)
plt.savefig(ERROR_HISTOGRAM, dpi=150)
plt.close()

print(f"Saved error histogram: {ERROR_HISTOGRAM}")
print(f"Saved error CSV: {ERROR_CSV}")
print(f"Saved report JSON: {REPORT_JSON}")

print("\nDone.")