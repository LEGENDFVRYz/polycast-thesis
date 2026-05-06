"""
Camera homography calibration tool.

Opens the webcam and lets you click the 4 anchor corners on the board
(which you can see in the camera frame). It then computes the homography
matrix that maps camera pixels → board coordinates in metres.

Run once after mounting the camera. The homography is saved to
background/pipelines/camera/homography.npy and loaded automatically
by MarkerCapDetector.

Usage:
    python -m background.pipelines.camera.calibrate_camera

Instructions:
    1. Mount the webcam so the full board is visible
    2. Run this script — a live camera window opens
    3. Click the 4 anchor corners IN ORDER: A0, A1, A2, A3
       A0 = bottom-left, A1 = bottom-right,
       A2 = top-right,   A3 = top-left
    4. Press ENTER to confirm and save
    5. Press R to reset and re-click if you made a mistake
"""

import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV not installed. Run: pip install opencv-python")

from background.pipelines.config import cfg

# Output path
OUT_PATH = Path(cfg.camera.homography_file)
OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

# Calibration reference points in metres (board coordinate frame).
# These do NOT have to be the anchor corners — any 4 known points work.
# Mark these exact positions on the board with tape before calibrating.
# Current points form an interior rectangle visible from below the board.
BOARD_CORNERS_M = np.array([
    [0.20, 0.20],   # P0 — bottom-left  reference mark
    [1.05, 0.20],   # P1 — bottom-right reference mark
    [0.20, 1.00],   # P2 — top-left     reference mark
    [1.15, 1.00],   # P3 — top-right    reference mark
], dtype=np.float32)

CORNER_NAMES = [
    'P0 bottom-left  (0.20m, 0.20m)',
    'P1 bottom-right (1.05m, 0.20m)',
    'P2 top-left     (0.20m, 1.00m)',
    'P3 top-right    (1.15m, 1.00m)',
]

clicked_points: list = []


def _on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"  Clicked point {len(clicked_points)}: pixel ({x}, {y})  "
              f"→ {CORNER_NAMES[len(clicked_points)-1]}")


def main():
    import time as _time
    cap = cv2.VideoCapture(cfg.camera.camera_index)
    _time.sleep(2.0)   # Windows MSMF needs time to initialise the camera device
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg.camera.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.camera.frame_height)

    if not cap.isOpened():
        sys.exit(f"[Error] Cannot open camera index {cfg.camera.camera_index}. "
                 f"Try changing camera_index in config.py (0, 1, 2...)")

    print("=" * 60)
    print("  Camera Homography Calibration")
    print(f"  Board: {cfg.anchors.board_size_x}m × {cfg.anchors.board_size_y}m")
    print("=" * 60)
    print()
    print("Click the 4 anchor corners IN ORDER:")
    for i, name in enumerate(CORNER_NAMES):
        m = BOARD_CORNERS_M[i]
        print(f"  {i+1}. {name}  →  ({m[0]:.2f}m, {m[1]:.2f}m)")
    print()
    print("Controls:  ENTER = confirm & save   R = reset   Q = quit")
    print()

    # Warm up camera — discard first several frames
    print("Warming up camera...", end='', flush=True)
    for _ in range(30):
        cap.read()
        _time.sleep(0.05)
    print(" ready.")

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", _on_mouse)

    last_good_frame = None
    fail_count = 0

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            fail_count += 1
            if fail_count > 60 and last_good_frame is None:
                print("[Error] Camera not producing frames after 30 attempts.")
                print("  Check that the webcam is plugged in and not in use by another app.")
                break
            if last_good_frame is not None:
                frame = last_good_frame   # show last good frame while waiting
            else:
                import time; time.sleep(0.05)
                continue
        else:
            last_good_frame = frame.copy()
            fail_count = 0

        display = frame.copy()

        # Draw clicked points
        colors = [(0, 255, 0), (0, 200, 255), (255, 100, 0), (200, 0, 255)]
        for idx, (px, py) in enumerate(clicked_points):
            cv2.circle(display, (px, py), 6, colors[idx], -1)
            cv2.putText(display, f"A{idx}", (px + 8, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[idx], 2)

        # Draw lines between clicked points
        if len(clicked_points) >= 2:
            for i in range(len(clicked_points) - 1):
                cv2.line(display, clicked_points[i], clicked_points[i+1],
                         (255, 255, 0), 1)
        if len(clicked_points) == 4:
            cv2.line(display, clicked_points[3], clicked_points[0],
                     (255, 255, 0), 1)

        # Status text
        n = len(clicked_points)
        if n < 4:
            msg = f"Click {CORNER_NAMES[n]} ({n+1}/4)"
        else:
            msg = "All 4 corners set.  ENTER=save  R=reset"
        cv2.putText(display, msg, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Cancelled.")
            break

        elif key == ord('r'):
            clicked_points.clear()
            print("Reset — click again from A0.")

        elif key == 13 and len(clicked_points) == 4:  # ENTER
            src = np.array(clicked_points, dtype=np.float32)
            dst = BOARD_CORNERS_M.copy()

            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            if H is None:
                print("[Error] Homography failed — try clicking more carefully.")
                clicked_points.clear()
                continue

            np.save(OUT_PATH, H)
            print()
            print(f"[Saved] Homography → {OUT_PATH}")
            print()

            # Verify by projecting clicked points back
            print("=== Verification ===")
            for i, (px, py) in enumerate(clicked_points):
                pt  = np.array([[[float(px), float(py)]]], dtype=np.float32)
                res = cv2.perspectiveTransform(pt, H)
                rx, ry = res[0, 0]
                ex, ey = BOARD_CORNERS_M[i]
                err = ((rx - ex)**2 + (ry - ey)**2) ** 0.5 * 1000
                print(f"  A{i}: pixel({px},{py}) → ({rx:.4f},{ry:.4f})m  "
                      f"expected ({ex:.3f},{ey:.3f})m  err={err:.1f}mm")
            print()
            print("Calibration complete. Close the window.")
            cv2.waitKey(0)
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
