"""
Module 4 - IMU Preprocessor

Turns raw BNO085 readings into fusion-ready motion events on the 2D board plane.

The marker's sensors sit at the back of the pen, but the pipeline needs motion
at the tip, so the main job here is a rigid-body correction that removes the
rotational contribution of the lever arm between them.

Processing order matters and is deliberate:

    1.  Filter in the body frame BEFORE differentiating. Angular acceleration and
        jerk are derivatives, so taking them from raw frame-to-frame deltas would
        amplify sensor noise into the very signals that gate turn detection.
    2.  Apply the rigid-body tip correction in the body frame, where omega and the
        lever arm share coordinates.
    3.  Only then rotate into the world frame and project onto the board plane.

Three filter paths run in parallel on the same acceleration, because different
consumers need different trade-offs:

    Path A  light EMA     retains slow real motion, and its bias with it
    Path B  heavy EMA     stillness detection only, never exported
    Path C  high-pass     strips slow bias, keeps fast stroke detail

The ESKF blends A and C; Path B exists purely so micro-tremor cannot block ZUPT.

Input:  normalized IMU events from cleaner/normalizer.py
Output: one motion event per sample, carrying both sensor-point (diagnostic) and
        tip-corrected (canonical) acceleration, angular kinematics, and the
        contact/stillness flags the downstream stages gate on.

Usage (Import as stage or run directly to trace normalized events):
    python -m background.pipelines.preprocess.imu
"""

import math

from background.pipelines.config import cfg

MICROSECONDS_PER_SECOND = 1_000_000

# Body-frame LPF divergence above which the raw sample is flagged as clipped.
_ACC_CLIP_THRESHOLD_MS2 = 0.5

# Path B smoothing weight. Deliberately heavy: 
# this signal only decides whether the pen is still
_ZUPT_EMA_PREVIOUS_WEIGHT = 0.95

_AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


# -----------------------------------------------------------------------------
# Vector and Quaternion Math
# -----------------------------------------------------------------------------

def _quat_norm(quaternion):
    return math.sqrt(sum(component * component for component in quaternion))


def _quat_normalize(quaternion):
    """Scale a quaternion to unit length, falling back to identity if degenerate."""

    norm = _quat_norm(quaternion)
    if norm < cfg.imu.quat_norm_epsilon:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quaternion)


def _quat_rotate(quaternion, vector):
    """Rotate a 3-vector by a unit quaternion [x, y, z, w]."""

    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _quat_multiply(left, right):
    """Hamilton product left (x) right, both stored as [x, y, z, w]."""

    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def _quat_conjugate(quaternion):
    """Conjugate, which equals the inverse for a unit quaternion."""

    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def _quat_to_rotation_matrix(quaternion):
    """Convert a unit quaternion to a 3x3 body-to-world rotation matrix."""

    x, y, z, w = quaternion
    return (
        (1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)),
        (    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)),
        (    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)),
    )


def _matrix_times_vector(matrix, vector):
    return (
        matrix[0][0]*vector[0] + matrix[0][1]*vector[1] + matrix[0][2]*vector[2],
        matrix[1][0]*vector[0] + matrix[1][1]*vector[1] + matrix[1][2]*vector[2],
        matrix[2][0]*vector[0] + matrix[2][1]*vector[1] + matrix[2][2]*vector[2],
    )


def _vec_magnitude(vector):
    return math.sqrt(sum(component * component for component in vector))


def _vec_subtract(left, right):
    return tuple(a - b for a, b in zip(left, right))


def _vec_add(left, right):
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _vec_scale(vector, scalar):
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def _vec_cross(left, right):
    return (
        left[1]*right[2] - left[2]*right[1],
        left[2]*right[0] - left[0]*right[2],
        left[0]*right[1] - left[1]*right[0],
    )


def _ema_vector(previous, current, current_weight: float):
    """Exponential moving average where current_weight applies to the new sample."""

    if previous is None:
        return current
    weight = max(0.0, min(1.0, float(current_weight)))
    return (
        weight * current[0] + (1.0 - weight) * previous[0],
        weight * current[1] + (1.0 - weight) * previous[1],
        weight * current[2] + (1.0 - weight) * previous[2],
    )


def _deadband_vector(vector, threshold: float):
    """Snap a vector to exact zero when its magnitude is below the noise floor."""

    if threshold <= 0.0:
        return vector
    return (0.0, 0.0, 0.0) if _vec_magnitude(vector) < threshold else vector


def _project_board_axes(vector_world):
    """Project a world-frame 3-vector onto the configured 2D board plane."""

    first_axis, second_axis = cfg.imu.board_axes
    return (vector_world[_AXIS_INDEX[first_axis]], vector_world[_AXIS_INDEX[second_axis]])


# -----------------------------------------------------------------------------
# IMU Preprocessor
# -----------------------------------------------------------------------------

class IMUPreprocessor:
    """Converts raw IMU samples into tip-corrected, board-projected motion events."""

    def __init__(self):
        self._prev_ts = None

        # Pre-derivative body-frame filter state.
        self._acc_body_lpf = None
        self._omega_body_lpf = None
        self._prev_omega_body_lpf = None
        self._ema_alpha_body = None
        self._prev_quat_rb = None

        # Time since the last quaternion update. The BNO085 often repeats the same
        # quaternion, so quat_delta uses this to compute rotation over the true interval.
        self._quat_hold_s = 0.0
        self._prev_omega_quat_delta = (0.0, 0.0, 0.0)

        # Path A: light EMA on world-frame acceleration, sensor point and tip.
        self._prev_acc_world_clean = None
        self._prev_acc_tip_clean = None

        # Path B: heavy EMA feeding stillness detection only.
        self._prev_acc_world_zupt = None
        self._prev_acc_world_zupt_old = None

        # Path C: high-pass filter state, sensor point and tip.
        self._acc_world_hp_prev = None
        self._acc_world_in_prev = None
        self._acc_tip_hp_prev = None
        self._acc_tip_in_prev = None

        # Jerk and stillness tracking.
        self._prev_acc_body = None
        self._prev_acc_body_lpf_for_jerk = None
        self._still_streak = 0
        self._zupt_active = False

        # Retained for external diagnostics that read these names.
        self._prev_omega_world = None
        self._ema_alpha_world = None

        nominal_dt_s = 1.0 / cfg.imu.sample_rate_hz
        self._zupt_min_samples = max(1, int(cfg.imu.zupt_min_duration_s / nominal_dt_s))

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def feed(self, events: list[dict]) -> list[dict]:
        """Process a batch, skipping non-IMU events and unusable samples."""

        processed = []
        for event in events:
            if event.get('sensor') != 'IMU':
                continue
            result = self.process_one(event)
            if result:
                processed.append(result)
        return processed

    def process_one(self, event: dict) -> dict | None:
        """
        Convert one raw IMU sample into a motion event.

        Returns None when the sample lacks required fields or its timestamp does
        not advance, since every derivative below divides by dt.
        """

        ts = event.get('ts_hw')
        quaternion = event.get('quat')
        acceleration = event.get('acc')

        if ts is None or quaternion is None or acceleration is None:
            return None

        dt_s = self._advance_clock(ts)
        if dt_s is None:
            return None

        # Hardware reports linear acceleration in m/s^2, already in the body frame.
        acc_body_raw = tuple(float(component) for component in acceleration)

        quat_norm_magnitude = _quat_norm(quaternion)
        quat_unit = _quat_normalize(quaternion)
        rotation_body_to_world = _quat_to_rotation_matrix(quat_unit)

        angular = self._compute_angular_kinematics(event, quat_unit, dt_s, rotation_body_to_world)
        acc_body_filtered, acc_clipped = self._filter_body_acceleration(acc_body_raw)

        acc_tip_body = self._apply_rigid_body_tip_correction(
            acc_body_filtered, angular['omega_body'], angular['alpha_body']
        )

        acc_world_raw = _quat_rotate(quat_unit, acc_body_filtered)
        acc_tip_world_raw = _quat_rotate(quat_unit, acc_tip_body)

        self._store_derivative_state(quat_unit, acc_body_filtered, angular)

        acc_world, acc_tip_world = self._apply_light_smoothing(acc_world_raw, acc_tip_world_raw)
        acc_world_zupt = self._apply_stillness_smoothing(acc_world_raw)

        high_pass = self._apply_high_pass(acc_world, acc_tip_world, dt_s)

        jerk = self._compute_jerk(acc_body_filtered, dt_s)
        zupt_jerk = self._compute_zupt_jerk(acc_world_zupt, dt_s)

        omega_magnitude = _vec_magnitude(angular['omega_world'])
        self._update_stillness(acc_world_zupt, zupt_jerk, omega_magnitude)

        self._prev_acc_body = acc_body_raw

        force = event.get('force', 0.0)

        return {
            'sensor': 'IMU',
            'ts_hw': ts,
            'packet_id': event.get('packet_id'),
            'sample_idx': event.get('sample_idx'),
            'quat': quat_unit,
            'acc_sensor': acc_body_raw,
            'acc_body_lpf': acc_body_filtered,

            # Sensor-point outputs, kept for diagnostics and legacy comparisons.
            'acc_world': acc_world,
            'acc_board': _project_board_axes(acc_world),
            'acc_board_hp': high_pass['acc_board_hp'],

            # Tip-corrected outputs, the canonical ESKF inputs.
            'acc_tip_world': acc_tip_world,
            'acc_board_tip': _project_board_axes(acc_tip_world),
            'acc_board_hp_tip': high_pass['acc_board_hp_tip'],

            'omega_world': angular['omega_world'],
            'omega_body': angular['omega_body'],
            'omega_body_raw': angular['omega_body_raw'],
            'alpha_world': angular['alpha_world'],
            'alpha_body': angular['alpha_body'],

            'jerk': round(jerk, 6),
            'omega_mag_world': round(omega_magnitude, 6),
            'is_static': self._zupt_active,
            'contact': force >= cfg.imu.force_contact_threshold,
            'force': force,

            'dt_s': dt_s,
            'hpf_alpha': high_pass['alpha'],
            'lever_arm_m': cfg.marker.r_imu_body_m,
            'gyro_source': angular['gyro_source'],
            'imu_quality': {
                'quat_norm_mag': quat_norm_magnitude,
                'is_static': self._zupt_active,
                'gyro_clipped': angular['gyro_clipped'],
                'acc_clipped': acc_clipped,
                'hpf_active': cfg.imu.hpf_enabled,
            },
        }

    def reset(self):
        """Clear all filter and integration history."""

        self.__init__()

    # -------------------------------------------------------------------------
    # Timing
    # -------------------------------------------------------------------------

    def _advance_clock(self, ts: int) -> float | None:
        """
        Seconds since the previous sample, or None if the timestamp did not advance.

        A long hardware gap means the filter histories describe motion that is no
        longer continuous with the current sample, so they are dropped rather
        than allowed to contaminate the first sample after the gap.
        """

        if self._prev_ts is None:
            self._prev_ts = ts
            return 1.0 / cfg.imu.sample_rate_hz

        dt_us = ts - self._prev_ts
        if dt_us <= 0:
            return None

        if dt_us > (cfg.pipeline.imu_max_dt_ms * 1000):
            self.reset()

        self._prev_ts = ts
        return dt_us / MICROSECONDS_PER_SECOND

    # -------------------------------------------------------------------------
    # Angular kinematics
    # -------------------------------------------------------------------------

    def _read_body_angular_velocity(self, event: dict, quat_unit, dt_s: float):
        """
        Obtain body-frame angular velocity in rad/s, preferring the hardware gyro.

        Falls back to consecutive-quaternion differentiation when gyro data is
        unavailable. Uses the exact rotation angle to preserve accuracy on fast
        turns, where the small-angle approximation breaks down.
        """

        # Presence, not truthiness: a genuine all-zero reading from a perfectly
        # still marker must not fall through to the quaternion path.
        gyro = event.get('gyro')
        if gyro is None:
            gyro = event.get('gyr')
        if gyro is None:
            gyro = event.get('omega_body_raw')

        if gyro is not None:
            try:
                components = tuple(float(component) for component in gyro)
            except (TypeError, ValueError):
                components = None

            if (components is not None
                    and len(components) == 3
                    and event.get('gyro_valid', True)
                    and all(math.isfinite(component) for component in components)):
                return components, 'hardware'

        if self._prev_quat_rb is not None and dt_s > 1e-6:

            self._quat_hold_s += dt_s

            delta = _quat_multiply(quat_unit, _quat_conjugate(self._prev_quat_rb))
            
            # Negating gives the shorter arc; both represent the same rotation.
            if delta[3] < 0:
                delta = (-delta[0], -delta[1], -delta[2], -delta[3])

            vector_norm = _vec_magnitude(delta[:3])
            if vector_norm <= 1e-12:
                
                # Quaternion unchanged: the sensor has not published a new
                # orientation yet. Holding the previous estimate is right
                return self._prev_omega_quat_delta, 'quat_delta'

            # omega = angle/elapsed about the unit rotation axis, with
            # angle = 2*atan2(|vec|, w). atan2 stays accurate across the whole
            # range, where 2*acos(w) loses precision as w approaches 1.
            angle = 2.0 * math.atan2(vector_norm, delta[3])
            omega = _vec_scale(delta[:3], angle / (vector_norm * self._quat_hold_s))

            self._quat_hold_s = 0.0
            self._prev_omega_quat_delta = omega
            return omega, 'quat_delta'

        return (0.0, 0.0, 0.0), 'quat_delta'

    def _compute_angular_kinematics(self, event, quat_unit, dt_s, rotation_body_to_world) -> dict:
        """
        Derive filtered angular velocity and acceleration in body and world frames.

        Omega is low-passed before alpha is taken from it, because differentiating
        raw gyro noise produces an alpha large enough to corrupt the centripetal
        term in the tip correction.
        """

        omega_body_raw, gyro_source = self._read_body_angular_velocity(event, quat_unit, dt_s)

        if cfg.imu.pre_derivative_lpf_enabled:
            omega_body = _ema_vector(self._omega_body_lpf, omega_body_raw, cfg.imu.gyro_lpf_alpha)
        else:
            omega_body = omega_body_raw

        # Silence near-zero gyro noise before it reaches the centripetal term,
        # turn detection, or ZUPT, so a held-still marker stays truly silent.
        omega_before_deadband = omega_body
        omega_body = _deadband_vector(omega_body, cfg.imu.omega_deadband_rads)
        gyro_clipped = (
            omega_body == (0.0, 0.0, 0.0) and omega_before_deadband != (0.0, 0.0, 0.0)
        )

        if self._prev_omega_body_lpf is not None and dt_s > 1e-6:
            alpha_body_raw = _vec_scale(
                _vec_subtract(omega_body, self._prev_omega_body_lpf), 1.0 / dt_s
            )
        else:
            alpha_body_raw = (0.0, 0.0, 0.0)

        # Second guard against derivative amplification. Uses previous-sample
        # weight, unlike the pre-derivative filters above.
        previous_weight = cfg.imu.alpha_ema_alpha
        if self._ema_alpha_body is None:
            alpha_body = alpha_body_raw
        else:
            alpha_body = tuple(
                previous_weight * self._ema_alpha_body[i]
                + (1.0 - previous_weight) * alpha_body_raw[i]
                for i in range(3)
            )
        alpha_body = _deadband_vector(alpha_body, cfg.imu.alpha_deadband_rads2)

        return {
            'omega_body_raw': omega_body_raw,
            'omega_body': omega_body,
            'omega_world': _matrix_times_vector(rotation_body_to_world, omega_body),
            'alpha_body': alpha_body,
            'alpha_world': _matrix_times_vector(rotation_body_to_world, alpha_body),
            'gyro_source': gyro_source,
            'gyro_clipped': gyro_clipped,
        }

    # -------------------------------------------------------------------------
    # Acceleration paths
    # -------------------------------------------------------------------------

    def _filter_body_acceleration(self, acc_body_raw):
        """Low-pass body acceleration and flag samples the filter had to pull hard."""

        if cfg.imu.pre_derivative_lpf_enabled:
            acc_body_filtered = _ema_vector(
                self._acc_body_lpf, acc_body_raw, cfg.imu.acc_lpf_alpha
            )
        else:
            acc_body_filtered = acc_body_raw

        divergence = _vec_magnitude(_vec_subtract(acc_body_filtered, acc_body_raw))
        return acc_body_filtered, divergence > _ACC_CLIP_THRESHOLD_MS2

    @staticmethod
    def _apply_rigid_body_tip_correction(acc_body, omega_body, alpha_body):
        """
        Convert sensor-point acceleration to marker-tip acceleration.

            a_tip = a_sensor - alpha x r - omega x (omega x r)

        Both correction terms are computed in the body frame, where omega and the
        lever arm share coordinates; rotating to world happens afterwards.
        """

        if not cfg.imu.rigid_body_enabled:
            return acc_body

        lever_arm = cfg.marker.r_imu_body_m
        sign = float(cfg.imu.rigid_body_sign)

        # Preserves the validated sign convention: with rigid_body_sign = -1 this
        # resolves back to +r_imu_body_m.
        tip_to_sensor = (
            -lever_arm[0] * sign, -lever_arm[1] * sign, -lever_arm[2] * sign
        )

        tangential = _vec_cross(alpha_body, tip_to_sensor)
        centripetal = _vec_cross(omega_body, _vec_cross(omega_body, tip_to_sensor))

        return (
            acc_body[0] - tangential[0] - centripetal[0],
            acc_body[1] - tangential[1] - centripetal[1],
            acc_body[2] - tangential[2] - centripetal[2],
        )

    def _apply_light_smoothing(self, acc_world_raw, acc_tip_world_raw):
        """Path A: light EMA on both world-frame acceleration signals."""

        # Config value is the previous-sample weight, so it is passed inverted.
        current_weight = 1.0 - cfg.imu.smooth_alpha_eskf

        acc_world = _ema_vector(self._prev_acc_world_clean, acc_world_raw, current_weight)
        self._prev_acc_world_clean = acc_world

        acc_tip_world = _ema_vector(self._prev_acc_tip_clean, acc_tip_world_raw, current_weight)
        self._prev_acc_tip_clean = acc_tip_world

        return acc_world, acc_tip_world

    def _apply_stillness_smoothing(self, acc_world_raw):
        """Path B: heavy EMA used only to decide whether the pen is still."""

        if self._prev_acc_world_zupt is None:
            self._prev_acc_world_zupt = acc_world_raw
            self._prev_acc_world_zupt_old = acc_world_raw

        acc_world_zupt = _ema_vector(
            self._prev_acc_world_zupt, acc_world_raw, 1.0 - _ZUPT_EMA_PREVIOUS_WEIGHT
        )
        self._prev_acc_world_zupt = acc_world_zupt
        return acc_world_zupt

    def _apply_high_pass(self, acc_world, acc_tip_world, dt_s) -> dict:
        """
        Path C: first-order high-pass that strips slow bias, keeping stroke detail.

            y[n] = alpha * (y[n-1] + x[n] - x[n-1]),  alpha = RC / (RC + dt)

        Runs on the tip-corrected signal so both rotational whip and sensor bias
        are removed in one pass.
        """

        if not cfg.imu.hpf_enabled:
            return {
                'alpha': 0.0,
                'acc_board_hp': _project_board_axes(acc_world),
                'acc_board_hp_tip': _project_board_axes(acc_tip_world),
            }

        rc = 1.0 / (2.0 * math.pi * cfg.imu.hpf_cutoff_hz)
        alpha = rc / (rc + dt_s)

        if self._acc_world_in_prev is None:
            acc_world_hp = (0.0, 0.0, 0.0)
            acc_tip_world_hp = (0.0, 0.0, 0.0)
        else:
            acc_world_hp = tuple(
                alpha * (self._acc_world_hp_prev[i] + acc_world[i] - self._acc_world_in_prev[i])
                for i in range(3)
            )
            acc_tip_world_hp = tuple(
                alpha * (self._acc_tip_hp_prev[i] + acc_tip_world[i] - self._acc_tip_in_prev[i])
                for i in range(3)
            )

        self._acc_world_hp_prev = acc_world_hp
        self._acc_tip_hp_prev = acc_tip_world_hp
        self._acc_world_in_prev = acc_world
        self._acc_tip_in_prev = acc_tip_world

        return {
            'alpha': alpha,
            'acc_board_hp': _project_board_axes(acc_world_hp),
            'acc_board_hp_tip': _project_board_axes(acc_tip_world_hp),
        }

    # -------------------------------------------------------------------------
    # Jerk and stillness
    # -------------------------------------------------------------------------

    def _compute_jerk(self, acc_body_filtered, dt_s) -> float:
        """
        Body-frame jerk magnitude, taken from the low-passed acceleration.

        Jerk arms turn detection, so deriving it from raw deltas would let
        micro-tremor and electrical noise trigger corner handling constantly.
        """

        if self._prev_acc_body_lpf_for_jerk is None or dt_s <= 0:
            jerk = 0.0
        else:
            delta = _vec_subtract(acc_body_filtered, self._prev_acc_body_lpf_for_jerk)
            jerk = _vec_magnitude(delta) / dt_s

        self._prev_acc_body_lpf_for_jerk = acc_body_filtered
        return jerk

    def _compute_zupt_jerk(self, acc_world_zupt, dt_s) -> float:
        """Jerk on the heavily-smoothed Path B signal, so tremor cannot block ZUPT."""

        if dt_s <= 0:
            zupt_jerk = 0.0
        else:
            delta = _vec_subtract(acc_world_zupt, self._prev_acc_world_zupt_old)
            zupt_jerk = _vec_magnitude(delta) / dt_s

        self._prev_acc_world_zupt_old = acc_world_zupt
        return zupt_jerk

    def _update_stillness(self, acc_world_zupt, zupt_jerk, omega_magnitude):
        """
        Update the ZUPT flag from sustained low acceleration, jerk, and rotation.

        All three must agree: a pen being rotated in place still reads low linear
        acceleration, and calling that "still" would zero a real velocity.
        A minimum duration prevents a single quiet sample from firing ZUPT.
        """

        still_now = (
            _vec_magnitude(acc_world_zupt) < cfg.imu.zupt_acc_threshold
            and zupt_jerk < cfg.imu.zupt_jerk_threshold
            and omega_magnitude < cfg.imu.zupt_omega_threshold
        )

        if still_now:
            self._still_streak += 1
        else:
            self._still_streak = 0
            self._zupt_active = False

        if self._still_streak >= self._zupt_min_samples:
            self._zupt_active = True

    def _store_derivative_state(self, quat_unit, acc_body_filtered, angular):
        """Carry the state the next sample's derivatives depend on."""

        self._prev_quat_rb = quat_unit
        self._acc_body_lpf = acc_body_filtered
        self._omega_body_lpf = angular['omega_body']
        self._prev_omega_body_lpf = angular['omega_body']
        self._ema_alpha_body = angular['alpha_body']

        # Legacy names, retained for external diagnostic consumers.
        self._prev_omega_world = angular['omega_world']
        self._ema_alpha_world = angular['alpha_world']


# =============================================================================
# MODULE TESTING
#   Live IMU dashboard: world acceleration, body jerk, force, and a 2D stroke
#   built by naive double integration.
#
#   The 2D plot is intentionally unfused - it shows what the IMU alone produces,
#   which is how you see raw drift before UWB corrects it. Exports a CSV and a
#   full-session plot on Ctrl+C.
#
#   Run:  python -m background.pipelines.preprocess.imu
# =============================================================================
if __name__ == '__main__':
    import csv
    import time

    import matplotlib.gridspec as gridspec
    import matplotlib.pyplot as plt

    from background.pipelines.cleaner.normalizer import StreamNormalizer
    from background.pipelines.cleaner.unpacker import SerialStreamer
    from background.pipelines.module_output import ModuleRunOutput

    REPORT_NAME = 'imu_stationary'
    WINDOW_SIZE = 500
    REFRESH_RATE_S = 0.1

    # Guards the naive integrator against a hardware timestamp gap.
    MAX_INTEGRATION_DT_S = 0.1

    streamer = SerialStreamer(port=cfg.serial.port, baud=cfg.serial.baud)
    normalizer = StreamNormalizer()
    preprocessor = IMUPreprocessor()

    print("=" * 60)
    print(f"  Live IMU Dashboard: {cfg.serial.port}")
    print("  Collecting data and plotting live... Press Ctrl+C to save and exit.")
    print("=" * 60)

    event_log = []
    velocity_board = [0.0, 0.0]
    position_board = [0.0, 0.0]
    last_integration_ts = None

    plt.ion()
    figure = plt.figure(figsize=(14, 8))
    figure.canvas.manager.set_window_title('Live IMU Kinematics & Drawing')
    figure.suptitle('Live IMU Kinematics & 2D Stroke Reconstruction',
                    fontsize=14, fontweight='bold')

    grid = gridspec.GridSpec(3, 2, width_ratios=[1.5, 1])
    axis_acc = figure.add_subplot(grid[0, 0])
    axis_jerk = figure.add_subplot(grid[1, 0])
    axis_force = figure.add_subplot(grid[2, 0])
    axis_stroke = figure.add_subplot(grid[:, 1])

    line_acc_x, = axis_acc.plot([], [], label='World X', alpha=0.8)
    line_acc_y, = axis_acc.plot([], [], label='World Y', alpha=0.8)
    line_acc_z, = axis_acc.plot([], [], label='World Z', alpha=0.8)
    axis_acc.set_title('World Acceleration')
    axis_acc.set_ylabel('Accel (m/s^2)')
    axis_acc.legend(loc='upper right')
    axis_acc.grid(True, linestyle='--', alpha=0.6)

    line_jerk, = axis_jerk.plot([], [], label='Jerk (m/s^3)', color='purple')
    axis_jerk.set_title('Body-Frame Jerk')
    axis_jerk.set_ylabel('Jerk')
    axis_jerk.legend(loc='upper right')
    axis_jerk.grid(True, linestyle='--', alpha=0.6)

    text_zupt = axis_jerk.text(
        0.02, 0.85, 'STATE: WAITING', transform=axis_jerk.transAxes,
        fontsize=12, fontweight='bold', bbox=dict(facecolor='white', alpha=0.8),
    )

    line_force, = axis_force.plot([], [], label='Raw Force', color='orange')
    axis_force.set_title('Force Sensor')
    axis_force.set_ylabel('Force')
    axis_force.set_xlabel('Time (Seconds)')
    axis_force.legend(loc='upper right')
    axis_force.grid(True, linestyle='--', alpha=0.6)

    text_contact = axis_force.text(
        0.02, 0.85, 'PEN: WAITING', transform=axis_force.transAxes,
        fontsize=12, fontweight='bold', bbox=dict(facecolor='white', alpha=0.8),
    )

    line_stroke, = axis_stroke.plot([], [], color='black', linewidth=2)
    axis_stroke.set_title('2D Board Strokes (Position)')
    axis_stroke.set_xlabel('Board X (m)')
    axis_stroke.set_ylabel('Board Z (m)')
    # Equal aspect keeps 1 cm on X visually equal to 1 cm on Z.
    axis_stroke.axis('equal')
    axis_stroke.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)

    start_ts = None
    last_plot_time = time.time()

    try:
        while True:
            raw_packets = streamer.read_new_packets()
            if raw_packets:
                processed_events = preprocessor.feed(normalizer.normalize(raw_packets))

                if processed_events and start_ts is None:
                    start_ts = processed_events[0]['ts_hw']

                for motion in processed_events:
                    if last_integration_ts is not None:
                        step_s = (motion['ts_hw'] - last_integration_ts) / MICROSECONDS_PER_SECOND
                        if 0 < step_s < MAX_INTEGRATION_DT_S:
                            if motion['is_static']:
                                velocity_board = [0.0, 0.0]
                            else:
                                velocity_board[0] += motion['acc_board'][0] * step_s
                                velocity_board[1] += motion['acc_board'][1] * step_s

                            position_board[0] += velocity_board[0] * step_s
                            position_board[1] += velocity_board[1] * step_s

                    last_integration_ts = motion['ts_hw']
                    motion['pos_x'] = position_board[0]
                    motion['pos_z'] = position_board[1]
                    event_log.append(motion)

            now = time.time()
            if event_log and (now - last_plot_time >= REFRESH_RATE_S):
                window = event_log[-WINDOW_SIZE:]
                seconds = [
                    (row['ts_hw'] - start_ts) / MICROSECONDS_PER_SECOND for row in window
                ]

                line_acc_x.set_data(seconds, [row['acc_world'][0] for row in window])
                line_acc_y.set_data(seconds, [row['acc_world'][1] for row in window])
                line_acc_z.set_data(seconds, [row['acc_world'][2] for row in window])
                line_jerk.set_data(seconds, [row['jerk'] for row in window])
                line_force.set_data(seconds, [row['force'] for row in window])

                # NaN between non-contact samples breaks the polyline so lifted
                # moves are not drawn as ink.
                line_stroke.set_data(
                    [row['pos_x'] if row['contact'] else float('nan') for row in event_log],
                    [row['pos_z'] if row['contact'] else float('nan') for row in event_log],
                )

                for axis in (axis_acc, axis_jerk, axis_force):
                    axis.set_xlim(seconds[0], max(seconds[-1], seconds[0] + 0.1))

                axis_acc.set_ylim(-3, 3)
                axis_force.set_ylim(0, 5000.0)

                if len(window) % 5 == 0:
                    axis_jerk.relim()
                    axis_jerk.autoscale_view(scalex=False, scaley=True)
                    axis_stroke.relim()
                    axis_stroke.autoscale_view()

                latest = window[-1]
                text_zupt.set_text('STATE: STATIC' if latest['is_static'] else 'STATE: MOVING')
                text_zupt.set_color('green' if latest['is_static'] else 'red')
                text_contact.set_text('PEN: DRAWING' if latest['contact'] else 'PEN: LIFTED')
                text_contact.set_color('blue' if latest['contact'] else 'gray')

                figure.canvas.draw_idle()
                plt.pause(0.001)
                last_plot_time = now

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n\n[STOP] Data collection halted.")
        streamer.close()

        if not event_log:
            print("No data collected. Exiting.")
            raise SystemExit(0)

        print(f"Captured {len(event_log)} IMU events. Exporting CSV...")

        output = ModuleRunOutput('preprocess/imu')
        output.save_csv(
            f"{REPORT_NAME}.csv",
            [[row['ts_hw'], row['packet_id'], row['sample_idx'],
              row['acc_world'][0], row['acc_world'][1], row['acc_world'][2],
              row['acc_board'][0], row['acc_board'][1],
              row['pos_x'], row['pos_z'],
              row['jerk'], int(row['is_static']), int(row['contact']), row['force']]
             for row in event_log],
            header=['ts_hw', 'packet_id', 'sample_idx',
                    'acc_world_x', 'acc_world_y', 'acc_world_z',
                    'acc_board_x', 'acc_board_z',
                    'pos_board_x', 'pos_board_z',
                    'jerk', 'is_static', 'contact', 'force'],
        )

        print("[EXPORT] Generating full-session plot...")
        report_figure = plt.figure(figsize=(16, 10))
        report_figure.suptitle('IMU Full Session Report', fontsize=16, fontweight='bold')
        report_grid = gridspec.GridSpec(3, 2, width_ratios=[1.5, 1])

        report_acc = report_figure.add_subplot(report_grid[0, 0])
        report_jerk = report_figure.add_subplot(report_grid[1, 0])
        report_force = report_figure.add_subplot(report_grid[2, 0])
        report_stroke = report_figure.add_subplot(report_grid[:, 1])

        all_seconds = [
            (row['ts_hw'] - start_ts) / MICROSECONDS_PER_SECOND for row in event_log
        ]

        for index, label in enumerate(('World X', 'World Y', 'World Z')):
            report_acc.plot(all_seconds, [row['acc_world'][index] for row in event_log], label=label)
        report_acc.set_title('World Acceleration')
        report_acc.set_ylabel('m/s^2')
        report_acc.legend()
        report_acc.grid(True, linestyle='--', alpha=0.6)

        report_jerk.plot(all_seconds, [row['jerk'] for row in event_log],
                         label='Jerk', color='purple')
        report_jerk.set_title('Body-Frame Jerk')
        report_jerk.set_ylabel('m/s^3')
        report_jerk.legend()
        report_jerk.grid(True, linestyle='--', alpha=0.6)

        report_force.plot(all_seconds, [row['force'] for row in event_log],
                          label='Force', color='orange')
        report_force.set_title('Force Sensor')
        report_force.set_ylabel('Force')
        report_force.set_xlabel('Time (Seconds)')
        report_force.legend()
        report_force.grid(True, linestyle='--', alpha=0.6)

        report_stroke.plot(
            [row['pos_x'] if row['contact'] else float('nan') for row in event_log],
            [row['pos_z'] if row['contact'] else float('nan') for row in event_log],
            color='black', linewidth=1.5,
        )
        report_stroke.set_title('Final 2D Drawing Path')
        report_stroke.set_xlabel('Board X (m)')
        report_stroke.set_ylabel('Board Z (m)')
        report_stroke.axis('equal')
        report_stroke.grid(True, linestyle='--', alpha=0.6)

        # Shade detected still periods so drift can be read against them.
        for axis in (report_acc, report_jerk, report_force):
            in_static = False
            static_start = 0.0
            for index, row in enumerate(event_log):
                if row['is_static'] and not in_static:
                    static_start = all_seconds[index]
                    in_static = True
                elif not row['is_static'] and in_static:
                    axis.axvspan(static_start, all_seconds[index], color='gray', alpha=0.15)
                    in_static = False
            if in_static:
                axis.axvspan(static_start, all_seconds[-1], color='gray', alpha=0.15)

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)

        plot_filename = f"{REPORT_NAME}.png"
        report_figure.savefig(output.path(plot_filename), dpi=150)
        output.record(plot_filename)
        output.finish()
        print("Done.")
