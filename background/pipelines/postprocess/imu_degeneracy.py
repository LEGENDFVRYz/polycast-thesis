"""
(Post-Process) Per-stroke IMU reliability gate with a UWB shape fallback.

Why this exists
---------------
For most strokes the IMU carries the letter shape and UWB only places it, which
is what `shape_mode` and `ink_from_dead_reckoner` are built around. For some
strokes it does not, and no amount of filtering recovers what was never
measured.

Measured on abcde_1, the three strokes forming the 'h' integrate to a nearly
one-dimensional path while UWB observes real two-dimensional motion over the same
interval. That is not a filtering artefact: undamped double integration of the
raw board acceleration, with no filter anywhere in the path, is equally flat.
The weak axis is simply absent from the inertial signal.

Detection
---------
Anisotropy is the ratio of the minor to the major principal axis of the stroke
(PCA), so 0 is a straight line and 1 is round. Across abcde_1 and
abc_extralarge the separation is unambiguous:

    good strokes    IMU anisotropy 0.227 - 0.739,  minor axis 2.4 - 5.5 cm
    'h' strokes     IMU anisotropy 0.020 - 0.141,  minor axis 0.10 - 0.25 cm

An order of magnitude apart with nothing between them. A stroke is treated as
degenerate when its IMU anisotropy is below threshold *and* UWB saw meaningfully
more structure over the same samples - both conditions, because a genuine
straight line (the stem of a 'T', an underline) is legitimately anisotropic and
must not be replaced by UWB noise.

Fallback
--------
A degenerate stroke is rebuilt from the UWB points captured during it, resampled
onto the stroke's own timestamps so downstream stages see the same point count
and timing. The result is noisier than a good IMU stroke - UWB carries ~2.4 cm of
noise - but a noisy 'h' is legible where a flat line is not.

Run directly for the self-test:
    python -m background.pipelines.postprocess.imu_degeneracy
"""

import math

from background.pipelines.config import cfg


def _principal_axes(points) -> tuple[float, float]:
    """
    Return (major, minor) singular values of the centred point cloud.

    Normalised by sqrt(n) so the values are comparable across strokes of
    different lengths - otherwise a long stroke looks structurally larger than a
    short one purely because it has more samples.
    """

    count = len(points)
    if count < 3:
        return 0.0, 0.0

    mean_x = sum(p[0] for p in points) / count
    mean_y = sum(p[1] for p in points) / count

    # 2x2 covariance, then its eigenvalues in closed form - avoids a numpy
    # dependency in a module that otherwise needs none.
    sxx = sum((p[0] - mean_x) ** 2 for p in points) / count
    syy = sum((p[1] - mean_y) ** 2 for p in points) / count
    sxy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points) / count

    trace = sxx + syy
    diff = math.sqrt(max(0.0, (sxx - syy) ** 2 + 4.0 * sxy * sxy))
    major = math.sqrt(max(0.0, 0.5 * (trace + diff)))
    minor = math.sqrt(max(0.0, 0.5 * (trace - diff)))
    return major, minor


def _anisotropy(points) -> float:
    major, minor = _principal_axes(points)
    if major <= 1e-12:
        return 1.0
    return minor / major


def _resample_to_timestamps(uwb_points, timestamps):
    """
    Interpolate the UWB track onto the stroke's own sample times.

    Keeping the original timestamps means the rebuilt stroke stays a drop-in
    replacement: point count, timing and every downstream assumption hold.
    """

    if len(uwb_points) < 2:
        return None

    ordered = sorted(uwb_points, key=lambda p: p[2])
    out = []
    index = 0

    for ts in timestamps:
        while index < len(ordered) - 2 and ordered[index + 1][2] < ts:
            index += 1

        earlier = ordered[index]
        later = ordered[index + 1]
        span = later[2] - earlier[2]

        if span <= 0:
            out.append((earlier[0], earlier[1], ts))
            continue

        # Clamped rather than extrapolated: beyond the UWB window the nearest
        # fix is a better guess than a projection off the end of the data.
        fraction = min(1.0, max(0.0, (ts - earlier[2]) / span))
        out.append((
            earlier[0] + (later[0] - earlier[0]) * fraction,
            earlier[1] + (later[1] - earlier[1]) * fraction,
            ts,
        ))

    return out


def _smooth(points, window: int):
    """Centred moving average, to take the edge off UWB noise in a rebuilt stroke."""

    if window <= 1 or len(points) < window:
        return points

    half = window // 2
    out = []
    for i in range(len(points)):
        low = max(0, i - half)
        high = min(len(points), i + half + 1)
        span = high - low
        out.append((
            sum(p[0] for p in points[low:high]) / span,
            sum(p[1] for p in points[low:high]) / span,
            points[i][2],
        ))
    return out


class IMUDegeneracyFallback:
    """Rebuilds strokes from UWB where the inertial signal carried no shape."""

    def __init__(self):
        self.strokes_seen = 0
        self.strokes_replaced = 0
        self.last_imu_anisotropy = 1.0

    def apply(self, stroke: dict, uwb_points) -> dict:
        """Return the stroke, rebuilt from UWB when the IMU shape is degenerate."""

        config = cfg.imu_degeneracy
        self.strokes_seen += 1

        if not config.enabled:
            return stroke

        points = stroke.get('points') or []
        if len(points) < config.min_points or len(uwb_points) < config.min_uwb_points:
            return stroke

        imu_anisotropy = _anisotropy(points)
        _major, imu_minor = _principal_axes(points)
        self.last_imu_anisotropy = imu_anisotropy

        if imu_anisotropy > config.max_anisotropy:
            return stroke
        if imu_minor > config.max_minor_axis_m:
            return stroke

        uwb_anisotropy = _anisotropy(uwb_points)
        _uwb_major, uwb_minor = _principal_axes(uwb_points)

        # Both conditions required. A genuinely straight stroke - the stem of a
        # 'T', an underline - is legitimately anisotropic, and replacing it with
        # UWB would only add noise. Only when UWB saw structure the IMU missed is
        # there anything to recover.
        if uwb_anisotropy < imu_anisotropy * config.uwb_structure_ratio:
            return stroke
        if uwb_minor < config.min_uwb_minor_axis_m:
            return stroke

        rebuilt = _resample_to_timestamps(uwb_points, [p[2] for p in points])
        if rebuilt is None:
            return stroke

        rebuilt = _smooth(rebuilt, config.smoothing_window)

        stroke['points'] = rebuilt
        stroke['imu_degeneracy'] = {
            'replaced': True,
            'imu_anisotropy': imu_anisotropy,
            'uwb_anisotropy': uwb_anisotropy,
            'imu_minor_m': imu_minor,
            'uwb_minor_m': uwb_minor,
        }
        self.strokes_replaced += 1
        return stroke

    def stats(self) -> dict:
        return {
            'strokes_seen': self.strokes_seen,
            'strokes_replaced': self.strokes_replaced,
            'last_imu_anisotropy': self.last_imu_anisotropy,
        }

    def reset(self) -> None:
        self.strokes_seen = 0
        self.strokes_replaced = 0
        self.last_imu_anisotropy = 1.0


# -----------------------------------------------------------------------------
# Self-test
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    passed = failed = 0

    def check(name, condition, detail=''):
        global passed, failed
        if condition:
            passed += 1
            print(f'  PASS  {name}')
        else:
            failed += 1
            print(f'  FAIL  {name}  {detail}')

    def stroke_of(points):
        return {
            'stroke_id': 1,
            'points': points,
            'start_ts': points[0][2],
            'end_ts': points[-1][2],
        }

    print('IMU degeneracy fallback self-test')
    print('-' * 60)

    # A collapsed 'h' arch: IMU sees a line, UWB sees the real arch.
    n = 60
    flat, arch = [], []
    for i in range(n):
        f = i / (n - 1)
        ts = 1_000_000 + i * 5000
        flat.append((0.4 + 0.10 * f, 0.5 + 0.0008 * f, ts))
        arch.append((0.4 + 0.10 * f, 0.5 + 0.05 * math.sin(math.pi * f), ts))
    uwb_arch = [(x, y, ts) for x, y, ts in arch[::3]]

    gate = IMUDegeneracyFallback()
    result = gate.apply(stroke_of(list(flat)), uwb_arch)

    check('degenerate stroke replaced', gate.strokes_replaced == 1)
    check('rebuilt stroke has real 2D extent',
          _anisotropy(result['points']) > 0.15,
          f'anisotropy {_anisotropy(result["points"]):.3f}')
    check('point count preserved', len(result['points']) == len(flat))
    check('timestamps preserved',
          [p[2] for p in result['points']] == [p[2] for p in flat])

    height = max(p[1] for p in result['points']) - min(p[1] for p in result['points'])
    check('arch height recovered', height > 0.03, f'height {height * 100:.1f} cm')

    # A genuinely straight stroke must survive: UWB has no structure to add.
    gate.reset()
    uwb_line = [(x, y, ts) for x, y, ts in flat[::3]]
    kept = gate.apply(stroke_of(list(flat)), uwb_line)
    check('true straight line not replaced',
          gate.strokes_replaced == 0 and kept['points'] == flat)

    # A healthy round stroke must never be touched.
    gate.reset()
    circle = [
        (0.5 + 0.03 * math.cos(2 * math.pi * i / (n - 1)),
         0.5 + 0.03 * math.sin(2 * math.pi * i / (n - 1)),
         1_000_000 + i * 5000)
        for i in range(n)
    ]
    untouched = gate.apply(stroke_of(list(circle)), uwb_arch)
    check('healthy stroke untouched',
          gate.strokes_replaced == 0 and untouched['points'] == circle)

    # Degenerate inputs pass through rather than raising.
    gate.reset()
    check('too-short stroke passes through',
          gate.apply(stroke_of(list(flat[:4])), uwb_arch)['points'] == flat[:4])
    check('too-few UWB points passes through',
          gate.apply(stroke_of(list(flat)), uwb_arch[:1])['points'] == flat)

    print('-' * 60)
    print(f'{passed} passed, {failed} failed')
    raise SystemExit(1 if failed else 0)
