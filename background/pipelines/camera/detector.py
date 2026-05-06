"""
MarkerCapDetector — detects the red marker cap in the camera frame and
converts its pixel position to board coordinates using a pre-computed
homography matrix.

Pipeline:
    frame → HSV → red mask → largest contour centroid → pixel (u,v)
          → homography → board cap position (x,y) in metres
          → IMU quaternion → forward direction
          → tip = cap - pen_length * forward_vector

Output per frame:
    {
        'cap_board':  (x, y)      — board coords of red cap (metres)
        'tip_board':  (x, y)      — estimated tip position (metres)
        'confidence': float       — 0.0–1.0 (based on contour area/quality)
        'pixel':      (u, v)      — raw pixel centroid for debug overlay
        'ts':         float       — time.time() of detection
    }
    Returns None if no red marker detected this frame.
"""

import time
import math
from pathlib import Path

import numpy as np

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

from background.pipelines.config import cfg


def _quat_to_forward(q: tuple) -> np.ndarray:
    """
    Convert quaternion [qx, qy, qz, qw] to the forward unit vector
    pointing from the back end (cap) toward the tip.

    In the marker's body frame, the tip direction is -Y (since the
    marker is held perpendicular to the board with sensors at the top).
    We rotate body -Y into world frame using the IMU quaternion.
    """
    qx, qy, qz, qw = q
    # Rotate body vector (0, -1, 0) by quaternion
    # Using rotation matrix column for -Y axis
    # R * [0,-1,0]^T = [-2(qxqy - qwqz), -(1-2(qx²+qz²)), -2(qyqz+qwqx)]
    fx = -2.0 * (qx*qy - qw*qz)
    fy = -(1.0 - 2.0*(qx*qx + qz*qz))
    fz = -2.0 * (qy*qz + qw*qx)
    # Board plane is X-Z in world, so tip offset uses X and Z
    n = math.sqrt(fx*fx + fz*fz)
    if n < 1e-6:
        return np.array([0.0, 0.0])
    return np.array([fx / n, fz / n])   # 2D board projection


class MarkerCapDetector:
    """
    Detects the red marker cap each frame and returns board-frame tip position.

    Usage:
        detector = MarkerCapDetector()
        result = detector.process_frame(frame, imu_quat)
    """

    def __init__(self):
        if not _CV2_AVAILABLE:
            raise ImportError("OpenCV not installed. Run: pip install opencv-python")

        self._H: "np.ndarray | None" = None  # homography matrix
        self._load_homography()

        ccfg = cfg.camera
        self._lower1 = np.array(ccfg.red_lower1, dtype=np.uint8)
        self._upper1 = np.array(ccfg.red_upper1, dtype=np.uint8)
        self._lower2 = np.array(ccfg.red_lower2, dtype=np.uint8)
        self._upper2 = np.array(ccfg.red_upper2, dtype=np.uint8)
        self._min_area   = ccfg.min_contour_area
        self._pen_length = ccfg.pen_length_m

    def _load_homography(self) -> None:
        path = Path(cfg.camera.homography_file)
        if path.exists():
            self._H = np.load(path)
            print(f"[Camera] Homography loaded from {path}")
        else:
            print(f"[Camera] No homography found at {path}. "
                  "Run calibrate_camera.py first.")

    @property
    def calibrated(self) -> bool:
        return self._H is not None

    def process_frame(self, frame: np.ndarray,
                      imu_quat: "tuple | None" = None) -> "dict | None":
        """
        frame:    BGR frame from cv2.VideoCapture
        imu_quat: (qx, qy, qz, qw) from latest IMU event, or None

        Returns detection dict or None if not detected / not calibrated.
        """
        if self._H is None:
            return None

        # ── Red mask ──────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self._lower1, self._upper1)
        mask2 = cv2.inRange(hsv, self._lower2, self._upper2)
        mask  = cv2.bitwise_or(mask1, mask2)

        # Small morphological close to fill gaps in the marker cap
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ── Find largest red contour ──────────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self._min_area:
            return None

        # Centroid
        M  = cv2.moments(best)
        if M['m00'] == 0:
            return None
        u = M['m10'] / M['m00']
        v = M['m01'] / M['m00']

        # Confidence: based on contour circularity (cap should be roughly round)
        perimeter = cv2.arcLength(best, True)
        circularity = (4 * math.pi * area / (perimeter * perimeter + 1e-6))
        confidence  = float(min(circularity, 1.0))

        # ── Homography: pixel → board coordinates ─────────────────────────
        pt  = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, self._H)
        cap_x = float(dst[0, 0, 0])
        cap_y = float(dst[0, 0, 1])

        # Clamp to board bounds
        cap_x = float(np.clip(cap_x, -0.05, cfg.anchors.board_size_x + 0.05))
        cap_y = float(np.clip(cap_y, -0.05, cfg.anchors.board_size_y + 0.05))

        # ── Tip position from cap + IMU orientation ───────────────────────
        if imu_quat is not None:
            fwd = _quat_to_forward(imu_quat)
            tip_x = cap_x + self._pen_length * fwd[0]
            tip_y = cap_y + self._pen_length * fwd[1]
        else:
            # No IMU — assume pen perpendicular, tip directly below cap
            tip_x = cap_x
            tip_y = cap_y

        # Clamp tip to board
        tip_x = float(np.clip(tip_x, 0.0, cfg.anchors.board_size_x))
        tip_y = float(np.clip(tip_y, 0.0, cfg.anchors.board_size_y))

        return {
            'cap_board':  (cap_x, cap_y),
            'tip_board':  (tip_x, tip_y),
            'confidence': confidence,
            'pixel':      (u, v),
            'area':       area,
            'ts':         time.time(),
        }

    def draw_debug(self, frame: np.ndarray,
                   result: "dict | None") -> np.ndarray:
        """Draw detection overlay on frame for debugging."""
        out = frame.copy()
        if result is None:
            cv2.putText(out, "NO MARKER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return out

        u, v = int(result['pixel'][0]), int(result['pixel'][1])
        cv2.circle(out, (u, v), 8, (0, 255, 0), 2)
        cv2.circle(out, (u, v), 2, (0, 255, 0), -1)

        cap_x, cap_y = result['cap_board']
        tip_x, tip_y = result['tip_board']
        conf = result['confidence']
        cv2.putText(
            out,
            f"cap=({cap_x:.3f},{cap_y:.3f}) "
            f"tip=({tip_x:.3f},{tip_y:.3f}) "
            f"conf={conf:.2f}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1,
        )
        return out
