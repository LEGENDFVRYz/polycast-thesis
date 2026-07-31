"""
Post-stroke rigid/similarity alignment.

After pen-up, the finished fused/IMU polyline is corrected as one global
object using the UWB point cloud buffered during the same stroke:

    1. robust UWB centroid translation
    2. optional uniform scale correction
    3. optional bounded 2D Procrustes rotation

The important property is that no pointwise rubber-band correction is
applied. The relative stroke topology is preserved; only one global
transform is applied.

Usage (Import as helper or run directly to test rotation and scale recover):
    python -m background.pipelines.postprocess.centroid_align
"""

import math

import numpy as np


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------

def _bbox_radius(points_xy: np.ndarray, percentile: float) -> float:
    """Robust radius around the centroid, used for mild uniform scale."""

    if len(points_xy) < 2:
        return 0.0
    
    centre = points_xy.mean(axis=0)
    distances = np.linalg.norm(points_xy - centre, axis=1)
    return float(np.percentile(distances, max(0.0, min(1.0, percentile)) * 100.0))


def _pca_direction(points_xy: np.ndarray) -> tuple:
    """
    First principal component of points_xy and its eigenvalue ratio.

    Returns (unit_vector, ratio) where ratio = lambda_max / lambda_min.
    Returns (None, 0.0) for degenerate inputs.
    """

    if len(points_xy) < 2:
        return None, 0.0
    
    centre = points_xy.mean(axis=0)
    centered = points_xy - centre
    
    if np.allclose(centered, 0.0):
        return None, 0.0
    
    covariance = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)  # ascending order
    ratio = float(eigenvalues[-1]) / (float(eigenvalues[0]) + 1e-9)
    return eigenvectors[:, -1], ratio


def _rotation_2d(theta_rad: float) -> np.ndarray:
    """2x2 counter-clockwise rotation matrix for column-vector convention."""

    cos_theta, sin_theta = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]], dtype=float)


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    """Return the CCW angle in degrees for a 2D rotation matrix."""

    return math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))


def _trim_uwb_cloud(uwb_xy: np.ndarray, trim_quantile: float) -> np.ndarray:
    """Drop the farthest UWB points before centroid/transform calculation."""

    if len(uwb_xy) < 4:
        return uwb_xy
    centre = np.median(uwb_xy, axis=0)
    distances = np.linalg.norm(uwb_xy - centre, axis=1)
    quantile = float(np.quantile(distances, max(0.05, min(1.0, trim_quantile))))
    keep = distances <= quantile
    return uwb_xy[keep] if int(np.sum(keep)) >= 3 else uwb_xy


def _trim_endpoints(points_xy: np.ndarray, fraction: float) -> np.ndarray:
    """Trim a small chronological fraction at each end for transform fitting."""

    if len(points_xy) < 6 or fraction <= 0.0:
        return points_xy
    trim_count = int(round(len(points_xy) * max(0.0, min(0.20, fraction))))
    if trim_count <= 0 or (2 * trim_count) >= len(points_xy) - 2:
        return points_xy
    return points_xy[trim_count:-trim_count]


def _resample_polyline(points_xy: np.ndarray, count: int) -> np.ndarray:
    """Arc-length resample a 2D polyline to exactly `count` points."""

    points_xy = np.asarray(points_xy, dtype=float)
    if len(points_xy) == 0:
        return np.empty((0, 2), dtype=float)
    if len(points_xy) == 1 or count <= 1:
        return np.repeat(points_xy[:1], max(1, count), axis=0)

    segment_lengths = np.linalg.norm(np.diff(points_xy, axis=0), axis=1)
    keep = np.concatenate([[True], segment_lengths > 1e-9])
    points_xy = points_xy[keep]
    if len(points_xy) == 1:
        return np.repeat(points_xy[:1], count, axis=0)

    segment_lengths = np.linalg.norm(np.diff(points_xy, axis=0), axis=1)
    arc_length = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    total_length = float(arc_length[-1])
    if total_length < 1e-9:
        return np.repeat(points_xy[:1], count, axis=0)

    targets = np.linspace(0.0, total_length, int(count))
    resampled = np.empty((int(count), 2), dtype=float)
    resampled[:, 0] = np.interp(targets, arc_length, points_xy[:, 0])
    resampled[:, 1] = np.interp(targets, arc_length, points_xy[:, 1])
    return resampled


# -----------------------------------------------------------------------------
# Transform estimation
# -----------------------------------------------------------------------------

def _similarity_procrustes(
    source: np.ndarray,
    destination: np.ndarray,
    *,
    scale_enabled: bool,
    max_rotation_deg: float,
    scale_min: float,
    scale_max: float,
) -> tuple[np.ndarray, float, float, dict]:
    """
    Best bounded similarity transform from source to destination.

    Uses paired, resampled correspondences. Returns (rotation, scale,
    rotation_deg, meta), where transformed row-vector points are:
    dest_centre + scale * (source_centered @ rotation.T).
    """

    if len(source) < 3 or len(destination) < 3 or len(source) != len(destination):
        return np.eye(2), 1.0, 0.0, {'applied': False, 'reason': 'bad_correspondence'}

    source_centre = source.mean(axis=0)
    dest_centre = destination.mean(axis=0)
    source_centered = source - source_centre
    dest_centered = destination - dest_centre
    denominator = float(np.sum(source_centered * source_centered))
    
    if denominator < 1e-12:
        return np.eye(2), 1.0, 0.0, {'applied': False, 'reason': 'degenerate_src'}

    cross_covariance = source_centered.T @ dest_centered
    u, singular_values, vt = np.linalg.svd(cross_covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    raw_rotation_deg = _rotation_angle_deg(rotation)
    rotation_deg = max(-float(max_rotation_deg), min(float(max_rotation_deg), raw_rotation_deg))
    if abs(rotation_deg - raw_rotation_deg) > 1e-6:
        rotation = _rotation_2d(math.radians(rotation_deg))

    raw_scale = float(np.sum(singular_values) / denominator) if scale_enabled else 1.0
    scale = max(float(scale_min), min(float(scale_max), raw_scale)) if scale_enabled else 1.0

    rmse_before = float(np.sqrt(np.mean(np.sum((source_centered - dest_centered) ** 2, axis=1))))
    mapped = scale * (source_centered @ rotation.T)
    rmse_after = float(np.sqrt(np.mean(np.sum((mapped - dest_centered) ** 2, axis=1))))
    return rotation, scale, rotation_deg, {
        'applied': True,
        'raw_rotation_deg': round(raw_rotation_deg, 2),
        'rotation_clamped': bool(abs(rotation_deg - raw_rotation_deg) > 1e-6),
        'raw_scale': round(raw_scale, 4),
        'rmse_before_m': round(rmse_before, 4),
        'rmse_after_m': round(rmse_after, 4),
    }


def _pca_rotation(points_xy: np.ndarray, uwb_xy: np.ndarray, cfg) -> tuple[np.ndarray, float, dict]:
    """Fallback rotation based on principal axes."""

    imu_direction, imu_ratio = _pca_direction(points_xy)
    uwb_direction, uwb_ratio = _pca_direction(uwb_xy)
    min_ratio = float(getattr(cfg, 'pca_min_eigenratio', 2.0))
    
    if not (
        imu_direction is not None
        and uwb_direction is not None
        and imu_ratio >= min_ratio
        and uwb_ratio >= min_ratio
    ):
        return np.eye(2), 0.0, {
            'applied': False,
            'reason': 'weak_pca_axis',
            'ratio_imu': round(float(imu_ratio), 2),
            'ratio_uwb': round(float(uwb_ratio), 2),
        }

    if float(np.dot(imu_direction, uwb_direction)) < 0:
        uwb_direction = -uwb_direction
    
    cross = float(imu_direction[0] * uwb_direction[1] - imu_direction[1] * uwb_direction[0])
    dot = float(np.dot(imu_direction, uwb_direction))
    theta = math.atan2(cross, dot)
    max_rotation_rad = math.radians(float(getattr(cfg, 'rotation_max_deg', 20.0)))
    theta = max(-max_rotation_rad, min(max_rotation_rad, theta))
    
    return _rotation_2d(theta), math.degrees(theta), {
        'applied': True,
        'ratio_imu': round(float(imu_ratio), 2),
        'ratio_uwb': round(float(uwb_ratio), 2),
    }


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

def align_centroid(points_xy: np.ndarray, uwb_xy: np.ndarray, cfg) -> tuple[np.ndarray, dict]:
    """
    Translate, scale, and optionally rotate points_xy to match UWB.

    Performs one global similarity transform only. It does not pull
    individual stroke samples toward individual UWB samples.
    """

    points_xy = np.asarray(points_xy, dtype=float)
    uwb_xy = np.asarray(uwb_xy, dtype=float)

    if len(points_xy) < 2:
        return points_xy, {'applied': False, 'reason': 'few_points', 'n': int(len(points_xy))}

    if len(uwb_xy) < cfg.min_uwb_points:
        return points_xy, {
            'applied': False,
            'reason': 'few_uwb',
            'uwb_count': int(len(uwb_xy)),
        }

    uwb_trimmed = _trim_uwb_cloud(uwb_xy, getattr(cfg, 'trim_quantile', 1.0))

    endpoint_trim = float(getattr(cfg, 'procrustes_endpoint_trim', 0.0))
    fit_points = _trim_endpoints(points_xy, endpoint_trim)
    fit_uwb = _trim_endpoints(uwb_trimmed, endpoint_trim)
    
    if len(fit_points) < 3:
        fit_points = points_xy
    if len(fit_uwb) < 3:
        fit_uwb = uwb_trimmed

    points_centre = fit_points.mean(axis=0)
    uwb_centre = fit_uwb.mean(axis=0)
    translation = uwb_centre - points_centre
    translation_norm = float(np.linalg.norm(translation))

    if translation_norm > cfg.max_translation_m:
        return points_xy, {
            'applied': False,
            'reason': 'translation_too_large',
            'delta_m': round(translation_norm, 4),
            'uwb_count': int(len(uwb_xy)),
            'uwb_used': int(len(uwb_trimmed)),
        }

    # Radius ratio remains the default scale estimate for round/weak shapes.
    scale = 1.0
    points_radius = 0.0
    uwb_radius = 0.0
    
    if getattr(cfg, 'scale_enabled', False):
        percentile = getattr(cfg, 'scale_percentile', 0.80)
        points_radius = _bbox_radius(fit_points, percentile)
        uwb_radius = _bbox_radius(fit_uwb, percentile)
        
        if points_radius > 1e-6 and uwb_radius > 1e-6:
            raw_scale = uwb_radius / points_radius
            scale = max(float(cfg.scale_min), min(float(cfg.scale_max), float(raw_scale)))

    rotation_deg = 0.0
    rotation = np.eye(2)
    rotation_meta = {'applied': False, 'method': 'none'}

    if getattr(cfg, 'rotation_enabled', False):
        if getattr(cfg, 'procrustes_enabled', True) and len(fit_points) >= 3 and len(fit_uwb) >= 3:
            sample_count = int(max(
                8, min(int(getattr(cfg, 'procrustes_samples', 48)), len(fit_points), len(fit_uwb))
            ))
            
            resampled_points = _resample_polyline(fit_points, sample_count)
            resampled_uwb = _resample_polyline(fit_uwb, sample_count)
            procrustes_rotation, procrustes_scale, procrustes_rotation_deg, procrustes_meta = (
                _similarity_procrustes(
                    resampled_points, resampled_uwb,
                    scale_enabled=bool(getattr(cfg, 'scale_enabled', False)),
                    max_rotation_deg=float(getattr(cfg, 'rotation_max_deg', 20.0)),
                    scale_min=float(getattr(cfg, 'scale_min', 0.70)),
                    scale_max=float(getattr(cfg, 'scale_max', 1.10)),
                )
            )
            
            if procrustes_meta.get('applied'):
                rotation = procrustes_rotation
                rotation_deg = procrustes_rotation_deg
                # Prefer Procrustes scale when correspondences are available.
                if getattr(cfg, 'scale_enabled', False):
                    scale = procrustes_scale
                rotation_meta = {
                    'applied': True, 'method': 'procrustes', 'samples': sample_count, **procrustes_meta
                }
            else:
                rotation_meta = {'applied': False, 'method': 'procrustes', **procrustes_meta}

        if not rotation_meta.get('applied'):
            pca_rotation, pca_rotation_deg, pca_meta = _pca_rotation(fit_points, fit_uwb, cfg)
            if pca_meta.get('applied'):
                rotation = pca_rotation
                rotation_deg = pca_rotation_deg
                rotation_meta = {'method': 'pca', **pca_meta}
            else:
                rotation_meta = {'method': 'pca', **pca_meta}

    centered = points_xy - points_centre
    corrected = uwb_centre + scale * (centered @ rotation.T)
    
    return corrected, {
        'applied': True,
        'delta': [round(float(translation[0]), 4), round(float(translation[1]), 4)],
        'delta_m': round(translation_norm, 4),
        'scale': round(float(scale), 4),
        'rotation_deg': round(rotation_deg, 2),
        'rotation': rotation_meta,
        'r_imu_m': round(float(points_radius), 4),
        'r_uwb_m': round(float(uwb_radius), 4),
        'centroid_imu': [round(float(points_centre[0]), 4), round(float(points_centre[1]), 4)],
        'centroid_uwb': [round(float(uwb_centre[0]), 4), round(float(uwb_centre[1]), 4)],
        'uwb_count': int(len(uwb_xy)),
        'uwb_used': int(len(uwb_trimmed)),
        'fit_points': int(len(fit_points)),
        'fit_uwb': int(len(fit_uwb)),
    }


# -----------------------------------------------------------------------------
# Module Testing
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    import sys

    ok = True

    TOLERANCE_M = 0.001  # 1 mm
    TRUE_DELTA = np.array([0.05, 0.03])
    centre = np.array([0.3, 0.4])
    half_width = 0.08
    imu_points = np.array([
        centre + [-half_width, -half_width],
        centre + [ half_width, -half_width],
        centre + [ half_width,  half_width],
        centre + [-half_width,  half_width],
    ])
    uwb_points = np.array([
        centre + TRUE_DELTA + np.array([-0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01, -0.01]),
        centre + TRUE_DELTA + np.array([-0.01, -0.01]),
        centre + TRUE_DELTA + np.array([ 0.00,  0.00]),
    ])

    class _ConfigNoRotation:
        min_uwb_points = 3
        max_translation_m = 0.20
        trim_quantile = 1.0
        scale_enabled = False
        rotation_enabled = False
        procrustes_endpoint_trim = 0.0

    corrected, meta = align_centroid(imu_points, uwb_points, _ConfigNoRotation())
    if not meta['applied']:
        print(f'T1 FAIL: not applied - {meta}'); ok = False
    else:
        error_m = float(np.linalg.norm(np.array(meta['delta']) - TRUE_DELTA))
        if error_m > TOLERANCE_M:
            print(f'T1 FAIL: delta err {error_m*1000:.2f} mm'); ok = False
        else:
            print(f'T1 PASS  delta={meta["delta"]}  err={error_m*1000:.2f} mm')

    # Procrustes rotation recovery on a line segment.
    TRUE_ROTATION_DEG = -15.0
    TRUE_ROTATION_RAD = math.radians(TRUE_ROTATION_DEG)
    points_per_segment = 30
    imu_line = np.column_stack([
        np.linspace(-0.10, 0.10, points_per_segment), np.zeros(points_per_segment)
    ])
    imu_line += centre
    true_rotation = _rotation_2d(TRUE_ROTATION_RAD)
    uwb_line = (true_rotation @ (imu_line - centre).T).T + centre + TRUE_DELTA
    rng = np.random.default_rng(7)
    uwb_line += rng.normal(0, 0.0015, uwb_line.shape)

    class _ConfigRotation:
        min_uwb_points = 3
        max_translation_m = 0.20
        trim_quantile = 1.0
        scale_enabled = False
        rotation_enabled = True
        rotation_max_deg = 30.0
        procrustes_enabled = True
        procrustes_samples = 24
        procrustes_endpoint_trim = 0.0
        pca_min_eigenratio = 2.0

    _, meta_rotation = align_centroid(imu_line, uwb_line, _ConfigRotation())
    if not meta_rotation['applied']:
        print(f'T2 FAIL: not applied - {meta_rotation}'); ok = False
    else:
        rotation_error_deg = abs(meta_rotation['rotation_deg'] - TRUE_ROTATION_DEG)
        if rotation_error_deg > 3.0:
            print(
                f'T2 FAIL: rot err {rotation_error_deg:.2f} deg '
                f'(got {meta_rotation["rotation_deg"]} deg, expected {TRUE_ROTATION_DEG} deg)'
            )
            ok = False
        else:
            print(
                f'T2 PASS  rot={meta_rotation["rotation_deg"]} deg  '
                f'err={rotation_error_deg:.2f} deg  method={meta_rotation["rotation"].get("method")}'
            )

    # Procrustes scale recovery.
    TRUE_SCALE = 0.80
    uwb_scaled = TRUE_SCALE * (true_rotation @ (imu_line - centre).T).T + centre + TRUE_DELTA

    class _ConfigScale(_ConfigRotation):
        scale_enabled = True
        scale_min = 0.70
        scale_max = 1.10
        scale_percentile = 0.80

    _, meta_scale = align_centroid(imu_line, uwb_scaled, _ConfigScale())
    scale_error = abs(meta_scale.get('scale', 1.0) - TRUE_SCALE)
    if scale_error > 0.05:
        print(f'T3 FAIL: scale err {scale_error:.3f} (got {meta_scale.get("scale")}, expected {TRUE_SCALE})')
        ok = False
    else:
        print(f'T3 PASS  scale={meta_scale.get("scale")}  err={scale_error:.3f}')

    sys.exit(0 if ok else 1)
