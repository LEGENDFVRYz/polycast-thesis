"""
shape_classifier.py
===================
Pure geometric analysis pass — no side effects, no resampling.

classify_shape(pts_xy, cfg) -> ShapeFit

Tries primitives in specificity order:
  line -> circle -> arc -> polygon (rectangle / triangle)
  -> freeform fallback

All helper functions reuse existing utilities from centroid_align.py and
minimum_jerk.py where they exist (imported below).

Self-test: run as __main__ for five synthetic strokes (line, circle,
arc, rectangle, squiggle) and assert expected kinds / confidence bands.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from background.pipelines.postprocess.centroid_align import (
    _pca_direction,
    _resample_polyline,
)
from background.pipelines.postprocess.minimum_jerk import _bbox_diag, _curvature


# ---------------------------------------------------------------------------
# Result type
# ---------------------------------------------------------------------------

@dataclass
class ShapeFit:
    kind:       str          # 'line'|'circle'|'arc'|'rectangle'|'triangle'|'freeform'
    confidence: float        # [0, 1]
    params:     dict = field(default_factory=dict)   # primitive-specific geometry
    notes:      dict = field(default_factory=dict)   # diagnostics


_FREEFORM = ShapeFit(kind='freeform', confidence=0.0)


# ---------------------------------------------------------------------------
# Arc-length helpers (avoid re-importing private cumsum every time)
# ---------------------------------------------------------------------------

def _arc_lengths(pts: np.ndarray) -> np.ndarray:
    """Cumulative arc-length array, same length as pts. First element = 0."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _stroke_length(pts: np.ndarray) -> float:
    s = _arc_lengths(pts)
    return float(s[-1]) if len(s) else 0.0


# ---------------------------------------------------------------------------
# Line fit
# ---------------------------------------------------------------------------

def _fit_line(pts: np.ndarray, cfg) -> ShapeFit:
    """Total-least-squares line via SVD. Reuses _pca_direction."""
    n = len(pts)
    if n < 2:
        return _FREEFORM

    length = _stroke_length(pts)
    if length < cfg.min_line_length_m:
        return ShapeFit('freeform', 0.0, {}, {'reason': 'too_short', 'length_m': round(length, 4)})

    direction, eigen_ratio = _pca_direction(pts)
    if direction is None:
        return _FREEFORM

    centre = pts.mean(axis=0)
    # Perpendicular residuals.
    perp = np.array([-direction[1], direction[0]])
    residuals = np.abs((pts - centre) @ perp)
    rms_perp = float(np.sqrt(np.mean(residuals ** 2)))

    # Project endpoints onto the line for the snap params.
    t0 = float((pts[0]  - centre) @ direction)
    t1 = float((pts[-1] - centre) @ direction)
    p0 = centre + t0 * direction
    p1 = centre + t1 * direction

    conf = max(0.0, 1.0 - rms_perp / (length * cfg.line_residual_scale + 1e-9))
    return ShapeFit(
        kind='line',
        confidence=conf,
        params={'p0': p0.tolist(), 'p1': p1.tolist(), 'direction': direction.tolist()},
        notes={'rms_perp_m': round(rms_perp, 5), 'length_m': round(length, 4),
               'eigen_ratio': round(float(eigen_ratio), 2)},
    )


# ---------------------------------------------------------------------------
# Circle / arc fit (Kåsa algebraic method)
# ---------------------------------------------------------------------------

def _kasa_circle(pts: np.ndarray):
    """
    Kåsa algebraic circle fit via lstsq.
    Solves [2x, 2y, 1] @ [cx, cy, c] = x²+y², returns (cx, cy, r) or None.
    """
    A = np.column_stack([2.0 * pts[:, 0], 2.0 * pts[:, 1], np.ones(len(pts))])
    b = pts[:, 0] ** 2 + pts[:, 1] ** 2
    result = np.linalg.lstsq(A, b, rcond=None)
    sol = result[0]
    cx, cy = sol[0], sol[1]
    r_sq = sol[2] + cx ** 2 + cy ** 2
    if r_sq <= 0:
        return None
    return float(cx), float(cy), float(math.sqrt(r_sq))


def _angular_coverage(pts: np.ndarray, cx: float, cy: float):
    """
    Returns (min_angle, max_angle, total_sweep, sign) in radians.
    sign = +1 CCW, -1 CW (from first crossing of the direction of travel).
    Uses unwrapped angles to handle strokes that cross the ±π boundary.
    """
    angles = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    unwrapped = np.unwrap(angles)
    sweep = float(unwrapped[-1] - unwrapped[0])
    return float(unwrapped.min()), float(unwrapped.max()), abs(sweep), np.sign(sweep)


def _fit_circle_or_arc(pts: np.ndarray, cfg) -> ShapeFit:
    """Attempt circle fit; return 'circle', 'arc', or 'freeform'."""
    if len(pts) < 6:
        return _FREEFORM

    result = _kasa_circle(pts)
    if result is None:
        return _FREEFORM
    cx, cy, r = result

    if r < cfg.min_circle_radius_m:
        return ShapeFit('freeform', 0.0, {}, {'reason': 'radius_too_small', 'r_m': round(r, 4)})

    centre = np.array([cx, cy])
    radial = np.linalg.norm(pts - centre, axis=1)
    rms_radial = float(np.sqrt(np.mean((radial - r) ** 2)))
    conf = max(0.0, 1.0 - rms_radial / (r * cfg.circle_residual_scale + 1e-9))

    # Angular coverage.
    _, _, sweep, direction_sign = _angular_coverage(pts, cx, cy)
    circumference = 2.0 * math.pi * r
    closure_ratio = float(np.linalg.norm(pts[0] - pts[-1])) / (circumference + 1e-9)

    is_circle = (closure_ratio < cfg.circle_closure_max) and (sweep >= cfg.circle_min_arc_rad)
    arc_ok    = (cfg.arc_min_arc_rad <= sweep <= cfg.arc_max_arc_rad)

    params = {
        'center': [round(cx, 5), round(cy, 5)],
        'radius': round(r, 5),
        'direction_sign': int(direction_sign),
        'theta0': round(float(math.atan2(pts[0, 1] - cy, pts[0, 0] - cx)), 5),
        'sweep': round(sweep, 5),
        'closure_ratio': round(closure_ratio, 4),
    }
    notes = {
        'rms_radial_m': round(rms_radial, 5),
        'arc_deg': round(math.degrees(sweep), 1),
        'closure_ratio': round(closure_ratio, 4),
    }

    if is_circle:
        return ShapeFit(kind='circle', confidence=conf, params=params, notes=notes)
    if arc_ok:
        return ShapeFit(kind='arc', confidence=conf, params=params, notes=notes)
    return _FREEFORM


# ---------------------------------------------------------------------------
# Douglas-Peucker simplification
# ---------------------------------------------------------------------------

def _douglas_peucker(pts: np.ndarray, eps: float) -> list:
    """Returns list of kept indices from pts (always includes 0 and len-1)."""
    def _dp_recurse(indices: list) -> list:
        if len(indices) <= 2:
            return indices
        start, end = indices[0], indices[-1]
        p0, p1 = pts[start], pts[end]
        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9:
            dists = np.linalg.norm(pts[indices] - p0, axis=1)
        else:
            # Perpendicular distance to the line start→end.
            perp = np.array([-seg[1], seg[0]]) / seg_len
            dists = np.abs((pts[indices] - p0) @ perp)

        max_idx = int(np.argmax(dists))
        max_dist = float(dists[max_idx])
        if max_dist <= eps:
            return [start, end]
        split = indices[max_idx]
        left  = _dp_recurse(indices[:max_idx + 1])
        right = _dp_recurse(indices[max_idx:])
        return left[:-1] + right

    all_idx = list(range(len(pts)))
    return _dp_recurse(all_idx)


# ---------------------------------------------------------------------------
# Polygon fit (rectangle / triangle)
# ---------------------------------------------------------------------------

def _interior_angle_deg(v_prev: np.ndarray, v_next: np.ndarray) -> float:
    """Interior angle in degrees at a vertex between two edge vectors."""
    cos_a = float(np.dot(v_prev, v_next) / (np.linalg.norm(v_prev) * np.linalg.norm(v_next) + 1e-9))
    cos_a = max(-1.0, min(1.0, cos_a))
    return math.degrees(math.acos(cos_a))


def _fit_polygon(pts: np.ndarray, cfg) -> ShapeFit:
    """Douglas-Peucker → vertex count check → angle / edge quality scoring."""
    n = len(pts)
    if n < 8:
        return _FREEFORM

    kept = _douglas_peucker(pts, cfg.dp_epsilon_m)
    n_verts = len(kept)

    # Accept 3–6 unique vertices (DP always keeps start + end).
    # A closed 4-sided shape can DP to 5 or 6 points depending on whether
    # the closing segment gets its own vertex retained.
    if n_verts < 3 or n_verts > 6:
        return ShapeFit('freeform', 0.0, {}, {'reason': 'dp_vertex_count', 'n_verts': n_verts})

    verts = pts[kept]

    # Closure check.
    perimeter = _stroke_length(verts)
    closure_ratio = float(np.linalg.norm(verts[0] - verts[-1])) / (perimeter + 1e-9)
    is_closed = closure_ratio < cfg.polygon_closure_max

    # For a closed polygon, only trim the closing vertex if it is genuinely
    # near the first vertex (DP closure artefact, not a real corner).
    if is_closed and n_verts >= 4:
        # Check if the last DP vertex is close to the first — true duplicate.
        span = float(np.linalg.norm(verts[0] - verts[-1]))
        typical_edge = perimeter / max(n_verts - 1, 1)
        if span < typical_edge * 0.5:
            corners = verts[:-1]
        else:
            # Last vertex is not a duplicate; the shape has one extra real corner.
            # Reject as too complex (> 4 sides for a supposed quad).
            return ShapeFit('freeform', 0.0, {}, {
                'reason': 'extra_dp_corner',
                'n_verts': n_verts,
                'closure_ratio': round(closure_ratio, 4),
            })
    else:
        corners = verts

    n_corners = len(corners)

    # Edge residuals: for each edge segment, compute RMS of original stroke
    # points that fall within that index span against the ideal line.
    edge_residuals = []
    edge_lengths = []
    for ei in range(len(kept) - 1):
        i0, i1 = kept[ei], kept[ei + 1]
        seg_pts = pts[i0: i1 + 1]
        p0, p1 = pts[i0], pts[i1]
        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        edge_lengths.append(seg_len)
        if seg_len < 1e-9 or len(seg_pts) < 2:
            edge_residuals.append(0.0)
            continue
        perp = np.array([-seg[1], seg[0]]) / seg_len
        res = np.abs((seg_pts - p0) @ perp)
        edge_residuals.append(float(np.sqrt(np.mean(res ** 2))))

    mean_edge_len = float(np.mean(edge_lengths)) if edge_lengths else 1e-9
    mean_edge_res = float(np.mean(edge_residuals)) if edge_residuals else 0.0
    straightness  = max(0.0, 1.0 - mean_edge_res / (mean_edge_len + 1e-9))

    # Interior angles.
    angles = []
    for i in range(n_corners):
        prev_v = corners[i] - corners[(i - 1) % n_corners]
        next_v = corners[(i + 1) % n_corners] - corners[i]
        angles.append(_interior_angle_deg(prev_v, next_v))

    # For a closed 5-corner shape try to recover a rectangle by dropping the
    # near-collinear "kink" vertex that DP preserved due to slight noise on
    # an otherwise straight edge.  _interior_angle_deg returns 0° for a
    # perfectly straight pass-through; the smallest angle is the kink.
    if n_corners == 5 and is_closed:
        kink_idx = int(np.argmin(angles))
        if angles[kink_idx] < 30.0:    # < 30° turn → near-collinear, safe to drop
            corners = np.delete(corners, kink_idx, axis=0)
            n_corners = 4
            angles = []
            for i in range(n_corners):
                prev_v = corners[i] - corners[(i - 1) % n_corners]
                next_v = corners[(i + 1) % n_corners] - corners[i]
                angles.append(_interior_angle_deg(prev_v, next_v))

    # Classify rectangle vs triangle.
    kind = 'freeform'
    angle_quality = 0.0

    if n_corners == 4 and is_closed:
        # Rectangle: all interior angles ~90°.
        deviations = [abs(a - 90.0) for a in angles]
        max_dev = max(deviations)
        if max_dev <= cfg.rect_angle_tol_deg:
            angle_quality = max(0.0, 1.0 - max_dev / cfg.rect_angle_tol_deg)
            kind = 'rectangle'
    elif n_corners == 3 or (n_corners == 3 and not is_closed):
        # Triangle: no angle constraint, only edge straightness.
        angle_quality = 1.0
        kind = 'triangle'
    elif n_corners == 3 and is_closed:
        angle_quality = 1.0
        kind = 'triangle'

    if kind == 'freeform':
        return ShapeFit('freeform', 0.0, {}, {
            'reason': 'shape_not_recognised',
            'n_corners': n_corners,
            'is_closed': is_closed,
            'angles_deg': [round(a, 1) for a in angles],
        })

    conf = straightness * angle_quality

    # Axis-snap check for rectangle.
    axis_snapped = False
    if kind == 'rectangle':
        direction, _ = _pca_direction(corners)
        if direction is not None:
            angle_to_h = abs(math.degrees(math.atan2(direction[1], direction[0])))
            angle_to_h = min(angle_to_h, 180.0 - angle_to_h)
            if angle_to_h <= cfg.axis_snap_deg or (90.0 - angle_to_h) <= cfg.axis_snap_deg:
                axis_snapped = True

    return ShapeFit(
        kind=kind,
        confidence=conf,
        params={
            'corners': [c.tolist() for c in corners],
            'is_closed': is_closed,
            'axis_snapped': axis_snapped,
        },
        notes={
            'n_verts': n_verts,
            'n_corners': n_corners,
            'angles_deg': [round(a, 1) for a in angles],
            'mean_edge_res_m': round(mean_edge_res, 5),
            'straightness': round(straightness, 4),
            'angle_quality': round(angle_quality, 4),
            'closure_ratio': round(closure_ratio, 4),
        },
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def classify_shape(pts_xy: np.ndarray, cfg) -> ShapeFit:
    """
    Classify the closed stroke polyline into a geometric primitive.

    Tries primitives in specificity order.  The highest-confidence fit that
    clears its threshold wins.  Freeform is returned when nothing qualifies.

    Parameters
    ----------
    pts_xy : np.ndarray, shape (N, 2)
        Board-frame polyline in metres, already centroid-aligned.
    cfg : ShapeCorrectorConfig
        Tuning parameters from cfg.postprocess.shape.
    """
    pts_xy = np.asarray(pts_xy, dtype=float)
    if len(pts_xy) < 4:
        return _FREEFORM

    candidates: list[ShapeFit] = []

    # --- Line ---
    if cfg.line_enabled:
        fit = _fit_line(pts_xy, cfg)
        if fit.kind == 'line' and fit.confidence >= cfg.line_confidence_min:
            candidates.append(fit)

    # --- Circle / Arc ---
    fit_ca = _fit_circle_or_arc(pts_xy, cfg)
    if fit_ca.kind == 'circle' and cfg.circle_enabled and fit_ca.confidence >= cfg.circle_confidence_min:
        candidates.append(fit_ca)
    elif fit_ca.kind == 'arc' and cfg.arc_enabled and fit_ca.confidence >= cfg.arc_confidence_min:
        candidates.append(fit_ca)

    # --- Polygon ---
    if cfg.rectangle_enabled or cfg.triangle_enabled:
        fit_p = _fit_polygon(pts_xy, cfg)
        if fit_p.kind == 'rectangle' and cfg.rectangle_enabled and fit_p.confidence >= cfg.rectangle_confidence_min:
            candidates.append(fit_p)
        elif fit_p.kind == 'triangle' and cfg.triangle_enabled and fit_p.confidence >= cfg.triangle_confidence_min:
            candidates.append(fit_p)

    if not candidates:
        return _FREEFORM

    # Pick the highest-confidence recognised primitive.
    return max(candidates, key=lambda f: f.confidence)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    rng = np.random.default_rng(42)
    NOISE = 0.002  # 2 mm

    class _Cfg:
        line_enabled = True
        circle_enabled = True
        arc_enabled = True
        rectangle_enabled = True
        triangle_enabled = True
        line_confidence_min = 0.80
        circle_confidence_min = 0.75
        arc_confidence_min = 0.80
        rectangle_confidence_min = 0.60
        triangle_confidence_min = 0.60
        min_line_length_m = 0.03
        min_circle_radius_m = 0.015
        circle_closure_max = 0.20
        circle_min_arc_rad = 5.65
        arc_min_arc_rad = 1.05
        arc_max_arc_rad = 5.59
        dp_epsilon_m = 0.008
        rect_angle_tol_deg = 12.0
        axis_snap_deg = 8.0
        polygon_closure_max = 0.15
        line_residual_scale = 0.10
        circle_residual_scale = 0.20

    cfg = _Cfg()
    ok = True

    # T1 — noisy line
    xs = np.linspace(0.0, 0.20, 50)
    line_pts = np.column_stack([xs, np.zeros(50)]) + rng.normal(0, NOISE, (50, 2))
    f = classify_shape(line_pts, cfg)
    passed = f.kind == 'line' and f.confidence >= 0.80
    print(f"T1 {'PASS' if passed else 'FAIL'}  line  conf={f.confidence:.3f}  kind={f.kind}")
    ok = ok and passed

    # T2 — noisy full circle
    angles_full = np.linspace(0, 2 * math.pi - 0.01, 80)
    cx, cy, r = 0.3, 0.3, 0.05
    circle_pts = np.column_stack([cx + r * np.cos(angles_full), cy + r * np.sin(angles_full)])
    circle_pts += rng.normal(0, NOISE, circle_pts.shape)
    f = classify_shape(circle_pts, cfg)
    passed = f.kind == 'circle' and f.confidence >= 0.70
    print(f"T2 {'PASS' if passed else 'FAIL'}  circle  conf={f.confidence:.3f}  kind={f.kind}")
    ok = ok and passed

    # T3 — noisy arc (upper half)
    angles_half = np.linspace(0.1, math.pi - 0.1, 50)
    arc_pts = np.column_stack([cx + r * np.cos(angles_half), cy + r * np.sin(angles_half)])
    arc_pts += rng.normal(0, NOISE, arc_pts.shape)
    f = classify_shape(arc_pts, cfg)
    passed = f.kind == 'arc' and f.confidence >= 0.70
    print(f"T3 {'PASS' if passed else 'FAIL'}  arc  conf={f.confidence:.3f}  kind={f.kind}")
    ok = ok and passed

    # T4 — noisy closed rectangle (4×30 points around a 0.1×0.1 box)
    box_w, box_h = 0.10, 0.10
    sides = [
        np.column_stack([np.linspace(0.0, box_w, 30), np.zeros(30)]),
        np.column_stack([np.full(30, box_w), np.linspace(0.0, box_h, 30)]),
        np.column_stack([np.linspace(box_w, 0.0, 30), np.full(30, box_h)]),
        np.column_stack([np.zeros(30), np.linspace(box_h, 0.0, 30)]),
    ]
    rect_pts = np.vstack(sides)
    rect_pts += rng.normal(0, NOISE, rect_pts.shape)
    f = classify_shape(rect_pts, cfg)
    passed = f.kind == 'rectangle' and f.confidence >= 0.60
    print(f"T4 {'PASS' if passed else 'FAIL'}  rectangle  conf={f.confidence:.3f}  kind={f.kind}")
    ok = ok and passed

    # T5 — squiggle that should not match any primitive
    xs5 = np.linspace(0.0, 0.20, 60)
    squiggle_pts = np.column_stack([xs5, np.sin(8 * math.pi * xs5 / 0.20) * 0.03])
    squiggle_pts += rng.normal(0, NOISE, squiggle_pts.shape)
    f = classify_shape(squiggle_pts, cfg)
    passed = f.kind == 'freeform'
    print(f"T5 {'PASS' if passed else 'FAIL'}  freeform  conf={f.confidence:.3f}  kind={f.kind}")
    ok = ok and passed

    sys.exit(0 if ok else 1)
