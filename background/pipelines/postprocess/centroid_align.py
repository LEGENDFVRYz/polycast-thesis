"""
centroid_align.py
=================
Method 1 — Rigid-Body Centroid Alignment (translation-only).

After pen-up, the finished IMU polyline is translated as a single rigid object
so its centroid matches the centroid of the UWB point cloud buffered during
the same stroke. This fixes global placement drift without touching the shape.

Math: delta = mean(uwb_xy) - mean(imu_xy)
      corrected = imu_xy + delta

No rotation is applied — translation alone is sufficient for whiteboard use
and cannot distort letter shapes.

Self-test (run as __main__): synthetic square offset by a known delta,
assert the recovered translation is within 1 mm.
"""

import math
import numpy as np


def align_centroid(
    points_xy: np.ndarray,
    uwb_xy: np.ndarray,
    cfg,
) -> tuple[np.ndarray, dict]:
    """
    Translate *points_xy* so its centroid matches *uwb_xy*'s centroid.

    Parameters
    ----------
    points_xy : np.ndarray, shape (N, 2)
        IMU polyline in board coordinates (metres).
    uwb_xy : np.ndarray, shape (M, 2)
        UWB point cloud buffered during the stroke.
    cfg : CentroidAlignConfig
        Tuning parameters from cfg.postprocess.centroid.

    Returns
    -------
    corrected : np.ndarray, shape (N, 2)
        Translated polyline (or original if correction was skipped).
    meta : dict
        Diagnostic info stored under stroke['postprocess']['centroid_align'].
    """
    if len(uwb_xy) < cfg.min_uwb_points:
        return points_xy, {
            'applied': False,
            'reason': 'few_uwb',
            'uwb_count': int(len(uwb_xy)),
        }

    c_imu = points_xy.mean(axis=0)
    c_uwb = uwb_xy.mean(axis=0)
    delta = c_uwb - c_imu
    delta_norm = float(np.linalg.norm(delta))

    if delta_norm > cfg.max_translation_m:
        return points_xy, {
            'applied': False,
            'reason': 'translation_too_large',
            'delta_m': round(delta_norm, 4),
            'uwb_count': int(len(uwb_xy)),
        }

    corrected = points_xy + delta
    return corrected, {
        'applied': True,
        'delta': [round(float(delta[0]), 4), round(float(delta[1]), 4)],
        'delta_m': round(delta_norm, 4),
        'centroid_imu': [round(float(c_imu[0]), 4), round(float(c_imu[1]), 4)],
        'centroid_uwb': [round(float(c_uwb[0]), 4), round(float(c_uwb[1]), 4)],
        'uwb_count': int(len(uwb_xy)),
    }


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    TOLERANCE_M = 0.001  # 1 mm

    # Known offset to recover.
    TRUE_DELTA = np.array([0.05, 0.03])

    # Synthetic square IMU polyline centred exactly at (0.3, 0.4).
    # Use 4 distinct corners (no repeated endpoint) so the mean IS the centre.
    centre = np.array([0.3, 0.4])
    half = 0.08
    imu_pts = np.array([
        centre + [-half, -half],
        centre + [ half, -half],
        centre + [ half,  half],
        centre + [-half,  half],
    ])

    # UWB cloud: symmetric scatter around centre + TRUE_DELTA so the mean is exact.
    uwb_pts = np.array([
        centre + TRUE_DELTA + np.array([-0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01, -0.01]),
        centre + TRUE_DELTA + np.array([-0.01, -0.01]),
        centre + TRUE_DELTA + np.array([ 0.00,  0.00]),
    ])

    class _FakeCfg:
        min_uwb_points = 3
        max_translation_m = 0.20

    corrected, meta = align_centroid(imu_pts, uwb_pts, _FakeCfg())

    if not meta['applied']:
        print(f'FAIL: correction not applied — {meta}')
        sys.exit(1)

    recovered = np.array(meta['delta'])
    err = float(np.linalg.norm(recovered - TRUE_DELTA))
    if err > TOLERANCE_M:
        print(f'FAIL: delta error {err*1000:.2f} mm > {TOLERANCE_M*1000:.0f} mm tolerance')
        sys.exit(1)

    print(f'PASS  delta={meta["delta"]}  error={err*1000:.2f} mm  uwb_used={meta["uwb_count"]}')
