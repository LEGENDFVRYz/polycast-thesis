"""
centroid_align.py
=================
Post-stroke rigid/similarity alignment.

After pen-up, the finished fused/IMU polyline is corrected as one global
object using the UWB point cloud buffered during the same stroke:

    1. robust UWB centroid translation
    2. optional uniform scale correction
    3. optional bounded 2D Procrustes rotation

The important property is that no pointwise rubber-band correction is applied.
The relative stroke topology is preserved; only one global transform is applied.
"""

import math
import numpy as np


def _bbox_radius(pts: np.ndarray, percentile: float) -> float:
    """Robust radius around the centroid, used for mild uniform scale."""
    if len(pts) < 2:
        return 0.0
    centre = pts.mean(axis=0)
    d = np.linalg.norm(pts - centre, axis=1)
    return float(np.percentile(d, max(0.0, min(1.0, percentile)) * 100.0))


def _pca_direction(pts: np.ndarray) -> tuple:
    """First principal component of *pts* and its eigenvalue ratio.

    Returns (unit_vec, ratio) where ratio = λ_max / λ_min.
    Returns (None, 0.0) for degenerate inputs.
    """
    if len(pts) < 2:
        return None, 0.0
    c = pts.mean(axis=0)
    centered = pts - c
    if np.allclose(centered, 0.0):
        return None, 0.0
    cov = np.cov(centered.T)
    vals, vecs = np.linalg.eigh(cov)   # ascending order
    ratio = float(vals[-1]) / (float(vals[0]) + 1e-9)
    return vecs[:, -1], ratio


def _rotation_2d(theta_rad: float) -> np.ndarray:
    """2×2 counter-clockwise rotation matrix for column-vector convention."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def _rotation_angle_deg(R: np.ndarray) -> float:
    """Return CCW angle in degrees for a 2D rotation matrix."""
    return math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0])))


def _trim_uwb_cloud(uwb_xy: np.ndarray, trim_quantile: float) -> np.ndarray:
    """Drop farthest UWB points before centroid/transform calculation."""
    if len(uwb_xy) < 4:
        return uwb_xy
    centre = np.median(uwb_xy, axis=0)
    d = np.linalg.norm(uwb_xy - centre, axis=1)
    q = float(np.quantile(d, max(0.05, min(1.0, trim_quantile))))
    keep = d <= q
    return uwb_xy[keep] if int(np.sum(keep)) >= 3 else uwb_xy


def _trim_endpoints(pts: np.ndarray, frac: float) -> np.ndarray:
    """Trim a small chronological fraction at each end for transform fitting."""
    if len(pts) < 6 or frac <= 0.0:
        return pts
    n_trim = int(round(len(pts) * max(0.0, min(0.20, frac))))
    if n_trim <= 0 or (2 * n_trim) >= len(pts) - 2:
        return pts
    return pts[n_trim:-n_trim]


def _resample_polyline(pts: np.ndarray, n: int) -> np.ndarray:
    """Arc-length resample a 2D polyline to exactly n points."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.empty((0, 2), dtype=float)
    if len(pts) == 1 or n <= 1:
        return np.repeat(pts[:1], max(1, n), axis=0)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-9])
    pts = pts[keep]
    if len(pts) == 1:
        return np.repeat(pts[:1], n, axis=0)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < 1e-9:
        return np.repeat(pts[:1], n, axis=0)

    targets = np.linspace(0.0, total, int(n))
    out = np.empty((int(n), 2), dtype=float)
    out[:, 0] = np.interp(targets, s, pts[:, 0])
    out[:, 1] = np.interp(targets, s, pts[:, 1])
    return out


def _similarity_procrustes(src: np.ndarray, dst: np.ndarray, *, scale_enabled: bool,
                           max_rotation_deg: float, scale_min: float,
                           scale_max: float) -> tuple[np.ndarray, float, float, dict]:
    """Best bounded similarity transform from src to dst.

    Uses paired, resampled correspondences.  Returns (R, scale, rot_deg, meta),
    where transformed row-vector points are:  dst_c + scale * (src_c @ R.T).
    """
    if len(src) < 3 or len(dst) < 3 or len(src) != len(dst):
        return np.eye(2), 1.0, 0.0, {'applied': False, 'reason': 'bad_correspondence'}

    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    X = src - cs
    Y = dst - cd
    denom = float(np.sum(X * X))
    if denom < 1e-12:
        return np.eye(2), 1.0, 0.0, {'applied': False, 'reason': 'degenerate_src'}

    H = X.T @ Y
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    raw_rot_deg = _rotation_angle_deg(R)
    rot_deg = max(-float(max_rotation_deg), min(float(max_rotation_deg), raw_rot_deg))
    if abs(rot_deg - raw_rot_deg) > 1e-6:
        R = _rotation_2d(math.radians(rot_deg))

    raw_scale = float(np.sum(S) / denom) if scale_enabled else 1.0
    scale = max(float(scale_min), min(float(scale_max), raw_scale)) if scale_enabled else 1.0

    before = float(np.sqrt(np.mean(np.sum((X - Y) ** 2, axis=1))))
    mapped = scale * (X @ R.T)
    after = float(np.sqrt(np.mean(np.sum((mapped - Y) ** 2, axis=1))))
    return R, scale, rot_deg, {
        'applied': True,
        'raw_rotation_deg': round(raw_rot_deg, 2),
        'rotation_clamped': bool(abs(rot_deg - raw_rot_deg) > 1e-6),
        'raw_scale': round(raw_scale, 4),
        'rmse_before_m': round(before, 4),
        'rmse_after_m': round(after, 4),
    }


def _pca_rotation(points_xy: np.ndarray, uwb_xy: np.ndarray, cfg) -> tuple[np.ndarray, float, dict]:
    """Fallback rotation based on principal axes."""
    v_imu, ratio_imu = _pca_direction(points_xy)
    v_uwb, ratio_uwb = _pca_direction(uwb_xy)
    min_ratio = float(getattr(cfg, 'pca_min_eigenratio', 2.0))
    if not (v_imu is not None and v_uwb is not None and ratio_imu >= min_ratio and ratio_uwb >= min_ratio):
        return np.eye(2), 0.0, {
            'applied': False,
            'reason': 'weak_pca_axis',
            'ratio_imu': round(float(ratio_imu), 2),
            'ratio_uwb': round(float(ratio_uwb), 2),
        }

    if float(np.dot(v_imu, v_uwb)) < 0:
        v_uwb = -v_uwb
    cross = float(v_imu[0] * v_uwb[1] - v_imu[1] * v_uwb[0])
    dot = float(np.dot(v_imu, v_uwb))
    theta = math.atan2(cross, dot)
    max_rad = math.radians(float(getattr(cfg, 'rotation_max_deg', 20.0)))
    theta = max(-max_rad, min(max_rad, theta))
    return _rotation_2d(theta), math.degrees(theta), {
        'applied': True,
        'ratio_imu': round(float(ratio_imu), 2),
        'ratio_uwb': round(float(ratio_uwb), 2),
    }


def align_centroid(points_xy: np.ndarray, uwb_xy: np.ndarray, cfg) -> tuple[np.ndarray, dict]:
    """Translate, scale, and optionally rotate *points_xy* to match UWB.

    This performs one global similarity transform only.  It does not pull
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

    uwb_trim = _trim_uwb_cloud(uwb_xy, getattr(cfg, 'trim_quantile', 1.0))

    fit_src = _trim_endpoints(points_xy, float(getattr(cfg, 'procrustes_endpoint_trim', 0.0)))
    fit_dst = _trim_endpoints(uwb_trim, float(getattr(cfg, 'procrustes_endpoint_trim', 0.0)))
    if len(fit_src) < 3:
        fit_src = points_xy
    if len(fit_dst) < 3:
        fit_dst = uwb_trim

    c_imu = fit_src.mean(axis=0)
    c_uwb = fit_dst.mean(axis=0)
    delta = c_uwb - c_imu
    delta_norm = float(np.linalg.norm(delta))

    if delta_norm > cfg.max_translation_m:
        return points_xy, {
            'applied': False,
            'reason': 'translation_too_large',
            'delta_m': round(delta_norm, 4),
            'uwb_count': int(len(uwb_xy)),
            'uwb_used': int(len(uwb_trim)),
        }

    # Radius ratio remains the default scale estimate for round/weak shapes.
    scale = 1.0
    r_imu = 0.0
    r_uwb = 0.0
    if getattr(cfg, 'scale_enabled', False):
        pct = getattr(cfg, 'scale_percentile', 0.80)
        r_imu = _bbox_radius(fit_src, pct)
        r_uwb = _bbox_radius(fit_dst, pct)
        if r_imu > 1e-6 and r_uwb > 1e-6:
            raw_scale = r_uwb / r_imu
            scale = max(float(cfg.scale_min), min(float(cfg.scale_max), float(raw_scale)))

    rotation_deg = 0.0
    R = np.eye(2)
    rotation_meta = {'applied': False, 'method': 'none'}

    if getattr(cfg, 'rotation_enabled', False):
        if getattr(cfg, 'procrustes_enabled', True) and len(fit_src) >= 3 and len(fit_dst) >= 3:
            n = int(max(8, min(int(getattr(cfg, 'procrustes_samples', 48)), len(fit_src), len(fit_dst))))
            src_rs = _resample_polyline(fit_src, n)
            dst_rs = _resample_polyline(fit_dst, n)
            R_p, sc_p, rot_p, meta_p = _similarity_procrustes(
                src_rs, dst_rs,
                scale_enabled=bool(getattr(cfg, 'scale_enabled', False)),
                max_rotation_deg=float(getattr(cfg, 'rotation_max_deg', 20.0)),
                scale_min=float(getattr(cfg, 'scale_min', 0.70)),
                scale_max=float(getattr(cfg, 'scale_max', 1.10)),
            )
            if meta_p.get('applied'):
                R = R_p
                rotation_deg = rot_p
                # Prefer Procrustes scale when correspondences are available.
                if getattr(cfg, 'scale_enabled', False):
                    scale = sc_p
                rotation_meta = {'applied': True, 'method': 'procrustes', 'samples': n, **meta_p}
            else:
                rotation_meta = {'applied': False, 'method': 'procrustes', **meta_p}

        if not rotation_meta.get('applied'):
            R_pca, rot_pca, meta_pca = _pca_rotation(fit_src, fit_dst, cfg)
            if meta_pca.get('applied'):
                R = R_pca
                rotation_deg = rot_pca
                rotation_meta = {'method': 'pca', **meta_pca}
            else:
                rotation_meta = {'method': 'pca', **meta_pca}

    centered = points_xy - c_imu
    corrected = c_uwb + scale * (centered @ R.T)
    return corrected, {
        'applied': True,
        'delta': [round(float(delta[0]), 4), round(float(delta[1]), 4)],
        'delta_m': round(delta_norm, 4),
        'scale': round(float(scale), 4),
        'rotation_deg': round(rotation_deg, 2),
        'rotation': rotation_meta,
        'r_imu_m': round(float(r_imu), 4),
        'r_uwb_m': round(float(r_uwb), 4),
        'centroid_imu': [round(float(c_imu[0]), 4), round(float(c_imu[1]), 4)],
        'centroid_uwb': [round(float(c_uwb[0]), 4), round(float(c_uwb[1]), 4)],
        'uwb_count': int(len(uwb_xy)),
        'uwb_used': int(len(uwb_trim)),
        'fit_points': int(len(fit_src)),
        'fit_uwb': int(len(fit_dst)),
    }


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    ok = True

    TOLERANCE_M = 0.001  # 1 mm
    TRUE_DELTA = np.array([0.05, 0.03])
    centre = np.array([0.3, 0.4])
    half = 0.08
    imu_pts = np.array([
        centre + [-half, -half],
        centre + [ half, -half],
        centre + [ half,  half],
        centre + [-half,  half],
    ])
    uwb_pts = np.array([
        centre + TRUE_DELTA + np.array([-0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01,  0.01]),
        centre + TRUE_DELTA + np.array([ 0.01, -0.01]),
        centre + TRUE_DELTA + np.array([-0.01, -0.01]),
        centre + TRUE_DELTA + np.array([ 0.00,  0.00]),
    ])

    class _CfgNoRot:
        min_uwb_points = 3
        max_translation_m = 0.20
        trim_quantile = 1.0
        scale_enabled = False
        rotation_enabled = False
        procrustes_endpoint_trim = 0.0

    corrected, meta = align_centroid(imu_pts, uwb_pts, _CfgNoRot())
    if not meta['applied']:
        print(f'T1 FAIL: not applied — {meta}'); ok = False
    else:
        err = float(np.linalg.norm(np.array(meta['delta']) - TRUE_DELTA))
        if err > TOLERANCE_M:
            print(f'T1 FAIL: delta err {err*1000:.2f} mm'); ok = False
        else:
            print(f'T1 PASS  delta={meta["delta"]}  err={err*1000:.2f} mm')

    # Procrustes rotation recovery on a line segment.
    TRUE_ROT_DEG = -15.0
    TRUE_ROT_RAD = math.radians(TRUE_ROT_DEG)
    n_seg = 30
    imu_line = np.column_stack([np.linspace(-0.10, 0.10, n_seg), np.zeros(n_seg)])
    imu_line += centre
    R_true = _rotation_2d(TRUE_ROT_RAD)
    uwb_line = (R_true @ (imu_line - centre).T).T + centre + TRUE_DELTA
    rng = np.random.default_rng(7)
    uwb_line += rng.normal(0, 0.0015, uwb_line.shape)

    class _CfgRot:
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

    _, meta2 = align_centroid(imu_line, uwb_line, _CfgRot())
    if not meta2['applied']:
        print(f'T2 FAIL: not applied — {meta2}'); ok = False
    else:
        rot_err = abs(meta2['rotation_deg'] - TRUE_ROT_DEG)
        if rot_err > 3.0:
            print(f'T2 FAIL: rot err {rot_err:.2f}° (got {meta2["rotation_deg"]}°, expected {TRUE_ROT_DEG}°)')
            ok = False
        else:
            print(f'T2 PASS  rot={meta2["rotation_deg"]}°  err={rot_err:.2f}°  method={meta2["rotation"].get("method")}')

    # Procrustes scale recovery.
    TRUE_SCALE = 0.80
    uwb_scaled = TRUE_SCALE * (R_true @ (imu_line - centre).T).T + centre + TRUE_DELTA

    class _CfgScale(_CfgRot):
        scale_enabled = True
        scale_min = 0.70
        scale_max = 1.10
        scale_percentile = 0.80

    _, meta3 = align_centroid(imu_line, uwb_scaled, _CfgScale())
    scale_err = abs(meta3.get('scale', 1.0) - TRUE_SCALE)
    if scale_err > 0.05:
        print(f'T3 FAIL: scale err {scale_err:.3f} (got {meta3.get("scale")}, expected {TRUE_SCALE})')
        ok = False
    else:
        print(f'T3 PASS  scale={meta3.get("scale")}  err={scale_err:.3f}')

    sys.exit(0 if ok else 1)
