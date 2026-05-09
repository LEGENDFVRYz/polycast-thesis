import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
from pathlib import Path

# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

# Put your truth image inside the same "groundtruth" folder
IMAGE_PATH = BASE_DIR / "abcdefgh.jpg"

# Real physical size of your board / drawing area in millimeters
BOARD_WIDTH_MM = 1900
BOARD_HEIGHT_MM = 1200

# Output image size after perspective correction
OUTPUT_WIDTH_PX = 1900
OUTPUT_HEIGHT_PX = int(OUTPUT_WIDTH_PX * BOARD_HEIGHT_MM / BOARD_WIDTH_MM)

# Output files
CORRECTED_IMAGE_PATH = BASE_DIR / "abcdefgh_corrected_truth.png"
MASK_IMAGE_PATH = BASE_DIR / "abcdefgh_truth_mask.png"
OVERLAY_IMAGE_PATH = BASE_DIR / "abcdefgh_truth_overlay.png"
CSV_PATH = BASE_DIR / "abcdefgh_ground_truth_points.csv"

# Ink detection threshold.
# Lower = stricter, detects only darker writing.
# Higher = detects more, but may include noise.
INK_THRESHOLD = 110

# Minimum connected component area to keep.
# Increase this if small noise remains.
# Decrease this if parts of the letters disappear.
MIN_COMPONENT_AREA = 30


# ============================================================
# 1. LOAD IMAGE
# ============================================================

print("Script folder:", BASE_DIR)
print("Image path:", IMAGE_PATH)
print("Image exists:", IMAGE_PATH.exists())

img = cv2.imread(str(IMAGE_PATH))

if img is None:
    raise FileNotFoundError(
        f"Could not read image: {IMAGE_PATH}\n"
        "Make sure abc_truth.jpg is inside the same groundtruth folder."
    )

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


# ============================================================
# 2. CLICK 4 CORNERS OF BOARD / PAPER
# ============================================================

plt.figure(figsize=(10, 7))
plt.imshow(img_rgb)
plt.title(
    "Click 4 board/paper corners in order:\n"
    "top-left, top-right, bottom-right, bottom-left"
)
plt.axis("on")

corner_pts = plt.ginput(4, timeout=0)
plt.close()

if len(corner_pts) != 4:
    raise ValueError("You must click exactly 4 corners.")

src = np.array(corner_pts, dtype=np.float32)

dst = np.array(
    [
        [0, 0],
        [OUTPUT_WIDTH_PX - 1, 0],
        [OUTPUT_WIDTH_PX - 1, OUTPUT_HEIGHT_PX - 1],
        [0, OUTPUT_HEIGHT_PX - 1],
    ],
    dtype=np.float32,
)


# ============================================================
# 3. PERSPECTIVE CORRECTION / FLATTEN IMAGE
# ============================================================

H = cv2.getPerspectiveTransform(src, dst)
warped = cv2.warpPerspective(img, H, (OUTPUT_WIDTH_PX, OUTPUT_HEIGHT_PX))

cv2.imwrite(str(CORRECTED_IMAGE_PATH), warped)
print(f"Saved: {CORRECTED_IMAGE_PATH}")


# ============================================================
# 4. SELECT WRITING REGION ONLY
# ============================================================

warped_rgb = cv2.cvtColor(warped, cv2.COLOR_BGR2RGB)

plt.figure(figsize=(10, 7))
plt.imshow(warped_rgb)
plt.title(
    "Click 2 points around the writing only:\n"
    "top-left of writing area, then bottom-right of writing area"
)
plt.axis("on")

roi_pts = plt.ginput(2, timeout=0)
plt.close()

if len(roi_pts) != 2:
    raise ValueError("You must click exactly 2 points for the writing area.")

(x1, y1), (x2, y2) = roi_pts

x1, x2 = sorted([int(x1), int(x2)])
y1, y2 = sorted([int(y1), int(y2)])

# Clamp to image bounds
x1 = max(0, min(x1, OUTPUT_WIDTH_PX - 1))
x2 = max(0, min(x2, OUTPUT_WIDTH_PX - 1))
y1 = max(0, min(y1, OUTPUT_HEIGHT_PX - 1))
y2 = max(0, min(y2, OUTPUT_HEIGHT_PX - 1))

if x2 <= x1 or y2 <= y1:
    raise ValueError("Invalid writing area selection.")

roi = warped[y1:y2, x1:x2]


# ============================================================
# 5. DETECT DARK WRITING INSIDE ROI
# ============================================================

gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

# Slight blur helps reduce tiny noise
gray = cv2.GaussianBlur(gray, (3, 3), 0)

# Detect dark ink
_, roi_mask = cv2.threshold(
    gray,
    INK_THRESHOLD,
    255,
    cv2.THRESH_BINARY_INV
)


# ============================================================
# 6. CLEAN MASK
# ============================================================

# Remove tiny isolated noise
kernel = np.ones((2, 2), np.uint8)
roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

# Keep only components large enough to be handwriting
num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
    roi_mask,
    connectivity=8
)

clean_roi_mask = np.zeros_like(roi_mask)

for label in range(1, num_labels):
    area = stats[label, cv2.CC_STAT_AREA]

    if area >= MIN_COMPONENT_AREA:
        clean_roi_mask[labels == label] = 255


# ============================================================
# 7. PLACE ROI MASK BACK INTO FULL BOARD MASK
# ============================================================

mask = np.zeros((OUTPUT_HEIGHT_PX, OUTPUT_WIDTH_PX), dtype=np.uint8)
mask[y1:y2, x1:x2] = clean_roi_mask

cv2.imwrite(str(MASK_IMAGE_PATH), mask)
print(f"Saved: {MASK_IMAGE_PATH}")


# ============================================================
# 8. CREATE OVERLAY IMAGE FOR CHECKING
# ============================================================

overlay = warped.copy()

# Mark detected ink in red
overlay[mask > 0] = [0, 0, 255]

# Blend original corrected image with detected ink
blended = cv2.addWeighted(warped, 0.75, overlay, 0.25, 0)

cv2.imwrite(str(OVERLAY_IMAGE_PATH), blended)
print(f"Saved: {OVERLAY_IMAGE_PATH}")


# ============================================================
# 9. CONVERT MASK PIXELS TO GROUND-TRUTH COORDINATES
# ============================================================

ys, xs = np.where(mask > 0)

coords = []

for x_px, y_px in zip(xs, ys):
    x_mm = x_px * BOARD_WIDTH_MM / OUTPUT_WIDTH_PX
    y_mm = BOARD_HEIGHT_MM - (y_px * BOARD_HEIGHT_MM / OUTPUT_HEIGHT_PX)

    coords.append([
        int(x_px),
        int(y_px),
        float(x_mm),
        float(y_mm)
    ])

with open(CSV_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x_px", "y_px", "x_mm", "y_mm"])
    writer.writerows(coords)

print(f"Saved: {CSV_PATH}")
print(f"Ground-truth points detected: {len(coords)}")


# ============================================================
# 10. SHOW RESULT
# ============================================================

plt.figure(figsize=(10, 6))
plt.imshow(mask, cmap="gray")
plt.title("Detected Ground-Truth Ink Mask")
plt.axis("off")
plt.show()

plt.figure(figsize=(10, 6))
plt.imshow(cv2.cvtColor(blended, cv2.COLOR_BGR2RGB))
plt.title("Overlay: Detected Ink on Corrected Image")
plt.axis("off")
plt.show()

print("\nDone.")
print("All outputs were saved in:")
print(BASE_DIR)