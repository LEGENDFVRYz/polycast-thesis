import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from collections import deque
import sys

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = 'COM2'
BAUD_RATE = 115200

ANCHORS = np.array([
    [1.75, 0.00, 0.00],
    [1.75, 1.61, 0.00],
    [0.00, 0.00, 0.00]
])

LP_ALPHA = 0.25
BIAS_ALPHA = 0.008
VELOCITY_DECAY = 0.97
TRAP_DT_MAX = 0.15
IMU_SAMPLES_PER_PACKET = 10

UWB_ALPHA_MOVING = 0.015
UWB_ALPHA_STATIC = 0.03

ACCEL_NOISE_FLOOR = 0.01
STATIC_ACCEL_THRESH = 0.06

SLOW_MOTION_THRESH = 0.15
NORMAL_MOTION_THRESH = 0.35
FAST_MOTION_THRESH = 0.8
EXTREME_MOTION_THRESH = 2.0

VELOCITY_STATIC_THRESH = 0.005
VELOCITY_SLOW_THRESH = 0.04
VELOCITY_NORMAL_THRESH = 0.25
VELOCITY_FAST_THRESH = 0.8

DRAW_VEL_THRESH = 0.005
DRAW_ACCEL_THRESH = 0.015

STATIC_TIME_BRIEF = 0.15
STATIC_TIME_NORMAL = 0.4
STATIC_TIME_LONG = 1.0

JITTER_WINDOW = 5
JITTER_VARIANCE_THRESH = 0.001
JITTER_MAX_VEL = 0.03

CURVATURE_WINDOW = 8
CURVATURE_SHARP_THRESH = 0.8
CURVATURE_SMOOTH_THRESH = 0.15

CONTACT_VEL_AVG_WINDOW = 4

# -------------------------
# STARTUP CALIBRATION (heading only)
# -------------------------
is_calibrating = True
calib_count = 0
CALIB_SAMPLES = 30  # number of quaternion samples to average yaw over
imu_heading_offset = 0.0  # radians

# uwb_pos = np.zeros(3)
# uwb_center_offset = np.zeros(3)
# fused_pos = np.zeros(3)
# first_uwb_fix = True

# -------------------------
# SERIAL
# -------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Serial Error: {e}")
    sys.exit(1)

_serial_buf = ""

# -------------------------
# MOTION STATE MACHINE
# -------------------------
class MotionState:
    STATIC = 0
    JITTER = 1
    SLOW_DRAW = 2
    NORMAL_DRAW = 3
    FAST_DRAW = 4
    TRAJECTORY = 5
    HOVER = 6

class MotionAnalyzer:
    def __init__(self, window_size=20):
        self.vel_history = deque(maxlen=window_size)
        self.accel_history = deque(maxlen=window_size)
        self.accel_mag_history = deque(maxlen=window_size)
        self.pos_history = deque(maxlen=CURVATURE_WINDOW)
        self.current_state = MotionState.STATIC
        self.time_in_state = 0.0
        self.state_transition_time = time.time()
        self.contact_vel_avg = deque(maxlen=CONTACT_VEL_AVG_WINDOW)
        self.is_contact_detected = False
        self.state_history = deque(maxlen=200)

    def update(self, vel, accel, accel_mag, pos, dt):
        self.vel_history.append(vel.copy())
        self.accel_history.append(accel.copy())
        self.accel_mag_history.append(accel_mag)
        self.pos_history.append(pos.copy())
        vel_mag = np.linalg.norm(vel)
        self.contact_vel_avg.append(vel_mag)

        features = self._compute_features(vel_mag, accel_mag)
        new_state = self._classify_motion(features)

        if new_state != self.current_state:
            self.current_state = new_state
            self.state_transition_time = time.time()
            self.time_in_state = 0.0
        else:
            self.time_in_state += dt

        self.state_history.append((self.current_state, time.time()))
        avg_contact_vel = np.mean(list(self.contact_vel_avg)) if self.contact_vel_avg else 0
        self.is_contact_detected = avg_contact_vel > DRAW_VEL_THRESH * 0.8
        return self.current_state, features

    def _compute_features(self, vel_mag, accel_mag):
        return {
            'vel_mag': vel_mag,
            'accel_mag': accel_mag,
            'vel_variance': np.var(list(self.vel_history)) if len(self.vel_history) > 2 else 0.0,
            'accel_variance': np.var(list(self.accel_mag_history)) if len(self.accel_mag_history) > 2 else 0.0,
            'is_jittering': self._detect_jitter(),
            'curvature': self._compute_curvature(),
            'acceleration_direction_change': self._detect_accel_direction_change(),
        }

    def _detect_jitter(self):
        if len(self.vel_history) < 3:
            return False
        vels = list(self.vel_history)
        vel_mags = [np.linalg.norm(v) for v in vels[-JITTER_WINDOW:]]
        if not vel_mags:
            return False
        variance = np.var(vel_mags)
        max_vel = max(vel_mags)
        return variance > JITTER_VARIANCE_THRESH and max_vel < JITTER_MAX_VEL

    def _compute_curvature(self):
        if len(self.pos_history) < 3:
            return 0.0
        positions = list(self.pos_history)
        v1 = positions[-2] - positions[-3]
        v2 = positions[-1] - positions[-2]
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        if v1_norm < 1e-6 or v2_norm < 1e-6:
            return 0.0
        v1u = v1 / v1_norm
        v2u = v2 / v2_norm
        dot_product = np.clip(np.dot(v1u, v2u), -1.0, 1.0)
        angle = np.arccos(dot_product)
        return angle / np.pi

    def _detect_accel_direction_change(self):
        if len(self.accel_history) < 2:
            return 0.0
        accels = list(self.accel_history)[-3:]
        if len(accels) < 2:
            return 0.0
        a1 = accels[-2]; a2 = accels[-1]
        a1n = np.linalg.norm(a1); a2n = np.linalg.norm(a2)
        if a1n < 1e-6 or a2n < 1e-6:
            return 0.0
        a1u = a1 / a1n; a2u = a2 / a2n
        dot_product = np.clip(np.dot(a1u, a2u), -1.0, 1.0)
        angle = np.arccos(dot_product)
        return angle / np.pi

    def _classify_motion(self, features):
        vel_mag = features['vel_mag']
        accel_mag = features['accel_mag']
        is_jittering = features['is_jittering']
        curvature = features['curvature']

        if is_jittering and vel_mag < JITTER_MAX_VEL:
            return MotionState.JITTER
        if vel_mag < VELOCITY_STATIC_THRESH and accel_mag < STATIC_ACCEL_THRESH:
            return MotionState.STATIC
        if vel_mag < VELOCITY_SLOW_THRESH and accel_mag < ACCEL_NOISE_FLOOR:
            return MotionState.HOVER
        if accel_mag < SLOW_MOTION_THRESH and vel_mag < VELOCITY_NORMAL_THRESH:
            if vel_mag > DRAW_VEL_THRESH:
                return MotionState.SLOW_DRAW
            if accel_mag > DRAW_ACCEL_THRESH:
                return MotionState.SLOW_DRAW
        if accel_mag < NORMAL_MOTION_THRESH and vel_mag < VELOCITY_FAST_THRESH:
            return MotionState.NORMAL_DRAW
        if accel_mag < FAST_MOTION_THRESH and vel_mag > VELOCITY_NORMAL_THRESH:
            return MotionState.FAST_DRAW
        if vel_mag > VELOCITY_FAST_THRESH and accel_mag < ACCEL_NOISE_FLOOR:
            return MotionState.TRAJECTORY
        if vel_mag > DRAW_VEL_THRESH:
            return MotionState.NORMAL_DRAW
        return MotionState.STATIC

# -------------------------
# STATE
# -------------------------
pos = np.zeros(3)
vel = np.zeros(3)
accel_filtered = np.zeros(3)
accel_bias = np.zeros(3)
prev_accel = None
prev_vel = None
last_timestamp_us = None

uwb_pos = np.zeros(3)
uwb_center_offset = np.zeros(3)
fused_pos = np.zeros(3)

motion_analyzer = MotionAnalyzer()

yz_strokes = []
current_stroke = []
is_drawing = False
stroke_items = []

# -------------------------
# HELPERS
# -------------------------
def micros_dt(last, now):
    if last is None:
        return None
    diff = now - last
    if diff < 0:
        diff += 4294967296
    return diff / 1e6

def rotate_vec2d(x, y, theta):
    cs = np.cos(theta); sn = np.sin(theta)
    return x * cs - y * sn, x * sn + y * cs

def trilaterate(distances, anchors):
    def residuals(x, anchors, dists):
        return np.linalg.norm(anchors - x, axis=1) - dists
    x0 = np.mean(anchors, axis=0)
    res = least_squares(residuals, x0, args=(anchors, distances),
                        bounds=([-10, -10, -1], [10, 10, 3]))
    return res.x

def apply_uwb_fusion(is_moving_state):
    global pos, fused_pos, uwb_pos, uwb_center_offset
    imu_only_pos = pos
    target_offset = uwb_pos - imu_only_pos
    alpha = UWB_ALPHA_MOVING if is_moving_state else UWB_ALPHA_STATIC
    uwb_center_offset = (1 - alpha) * uwb_center_offset + alpha * target_offset
    fused_pos[:] = imu_only_pos + uwb_center_offset
    return True

def get_motion_state_name(state):
    names = {
        MotionState.STATIC: "STATIC",
        MotionState.JITTER: "JITTER",
        MotionState.SLOW_DRAW: "SLOW_DRAW",
        MotionState.NORMAL_DRAW: "NORMAL_DRAW",
        MotionState.FAST_DRAW: "FAST_DRAW",
        MotionState.TRAJECTORY: "TRAJECTORY",
        MotionState.HOVER: "HOVER",
    }
    return names.get(state, "UNKNOWN")

def calibrate_alignment_with_quat(r: R):
    global imu_heading_offset
    # extract yaw from rotation matrix (assuming 'xyz' ordering)
    yaw = r.as_euler('xyz', degrees=False)[2]
    imu_heading_offset = -yaw
    print(f"Heading Calibrated: offset {np.degrees(imu_heading_offset):.2f}°")

# -------------------------
# GUI
# -------------------------
app = QtWidgets.QApplication([])

yz_win = pg.plot(title="Advanced Motion Tracking - XY Plane")
yz_win.setLabel('bottom', 'X (m)')
yz_win.setLabel('left', 'Y (m)')
yz_win.setAspectLocked(True)
yz_win.showGrid(x=True, y=True)
yz_dot = yz_win.plot(symbol='o', symbolSize=8, symbolBrush='r')
uwb_ghost = yz_win.plot(symbol='+', symbolSize=10, symbolBrush='g')
yz_win.show()

info_win = QtWidgets.QGroupBox("System State & Motion Analysis")
info_layout = QtWidgets.QVBoxLayout(info_win)
info_label = QtWidgets.QLabel("Status: Waiting...")
info_layout.addWidget(info_label)
info_win.show()

def reset_position():
    global pos, vel, yz_strokes, current_stroke, is_drawing, stroke_items, last_timestamp_us
    global uwb_pos, fused_pos, uwb_center_offset, motion_analyzer
    global is_calibrating, calib_count

    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    fused_pos[:] = 0
    uwb_center_offset[:] = 0
    last_timestamp_us = None

    is_calibrating = True
    calib_count = 0

    motion_analyzer = MotionAnalyzer()

    yz_strokes.clear()
    current_stroke.clear()
    is_drawing = False
    for item in stroke_items:
        try:
            yz_win.removeItem(item)
        except Exception:
            pass
    stroke_items.clear()

    yz_dot.setData([0], [0])
    print("--- Reset ---")

reset_btn = QtWidgets.QPushButton("Zero Position", parent=yz_win)
reset_btn.clicked.connect(reset_position)
reset_btn.show()

# -------------------------
# IMU PROCESSING
# -------------------------
def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us
    global yz_strokes, current_stroke, is_drawing, stroke_items, fused_pos, imu_heading_offset
    global is_calibrating, calib_count, motion_analyzer

    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)

    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us
    if dt is None or dt <= 0 or dt > TRAP_DT_MAX:
        return False

    # rotate linear acceleration from sensor frame -> world frame
    r = R.from_quat([qx, qy, qz, qw])
    accel_world = r.apply(accel_local)

    # axis sign correction (keep as in your previous code)
    accel_world[1] *= -1
    # yaw_rot = R.from_euler('z', imu_heading_offset, degrees=False)
    # accel_world = yaw_rot.apply(accel_world)
    

    # heading alignment calibration (first few samples only)
    if is_calibrating:
        calib_count += 1
        if calib_count >= CALIB_SAMPLES:
            calibrate_alignment_with_quat(r)
            is_calibrating = False
            print("--- Calibration done ---")
        return True

    # low-pass filter on (already linear) acceleration
    accel_filtered[:] = LP_ALPHA * accel_world + (1 - LP_ALPHA) * accel_filtered
    accel_mag = np.linalg.norm(accel_filtered)

    # motion analysis
    motion_state, features = motion_analyzer.update(vel, accel_filtered, accel_mag, fused_pos, dt)

    # state-handling & integration
    if motion_state == MotionState.STATIC:
        accel_bias[:] = (1 - BIAS_ALPHA) * accel_bias + BIAS_ALPHA * accel_filtered
        vel[:] = 0.0
        is_drawing = False

    elif motion_state == MotionState.JITTER:
        vel[:] *= VELOCITY_DECAY

    elif motion_state == MotionState.HOVER:
        is_drawing = False
        vel[:] *= VELOCITY_DECAY * 0.95

    elif motion_state in [MotionState.SLOW_DRAW, MotionState.NORMAL_DRAW, MotionState.FAST_DRAW]:
        accel_corrected = accel_filtered - accel_bias
        if prev_accel is None:
            prev_accel = accel_corrected.copy()
            prev_vel = vel.copy()
        vel += 0.5 * (prev_accel + accel_corrected) * dt
        vel *= VELOCITY_DECAY
        pos += 0.5 * (prev_vel + vel) * dt
        prev_accel = accel_corrected.copy()
        prev_vel = vel.copy()

    elif motion_state == MotionState.TRAJECTORY:
        accel_corrected = accel_filtered - accel_bias
        vel += accel_corrected * dt * 0.3
        vel *= VELOCITY_DECAY * 0.98
        pos += vel * dt
        is_drawing = False

    # fused position with UWB offset
    fused_pos[:] = pos[:] + uwb_center_offset

    # drawing decisions
    vel_mag = np.linalg.norm(vel)
    should_draw = (
        motion_state in [MotionState.SLOW_DRAW, MotionState.NORMAL_DRAW, MotionState.FAST_DRAW]
        and vel_mag > DRAW_VEL_THRESH and not features['is_jittering']
    )

    if should_draw:
        if not is_drawing:
            current_stroke = []
            yz_strokes.append(current_stroke)
            new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
            yz_win.addItem(new_curve)
            stroke_items.append(new_curve)
            is_drawing = True
        current_stroke.append((fused_pos[0], fused_pos[2]))
        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
    else:
        is_drawing = False

    yz_dot.setData([fused_pos[0]], [fused_pos[2]])
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[1]])

    state_name = get_motion_state_name(motion_state)
    info_label.setText(
        f"Status: {'CALIBRATING...' if is_calibrating else 'READY'}\n"
        f"Motion State: {state_name}\n"
        f"Pos X: {fused_pos[0]:.3f} | Y: {fused_pos[1]:.3f}\n"
        f"Vel: {vel_mag:.4f} m/s | Accel: {accel_mag:.4f} m/s²\n"
        f"Curvature: {features['curvature']:.3f} | Contact: {'YES' if motion_analyzer.is_contact_detected else 'NO'}\n"
        f"Vel Var: {features['vel_variance']:.6f} | Accel Var: {features['accel_variance']:.6f}"
    )

    if len(yz_strokes) > 100:
        yz_strokes.pop(0)
        old_item = stroke_items.pop(0)
        yz_win.removeItem(old_item)

    return True

# -------------------------
# UPDATE LOOP
# -------------------------
def update():
    global _serial_buf, uwb_pos
    try:
        n = ser.in_waiting
        if n:
            _serial_buf += ser.read(n).decode('utf-8', errors='ignore')
        if '\n' not in _serial_buf:
            return

        parts = _serial_buf.split('\n')
        lines_to_process = parts[:-1]
        _serial_buf = parts[-1]

        for raw in lines_to_process:
            raw = raw.strip()
            if not raw:
                continue
            toks = raw.split(',')
            if len(toks) < 86:
                continue

            try:
                d0 = float(toks[2]); d1 = float(toks[3]); d2 = float(toks[4])
                if d0 > 0 and d1 > 0 and d2 > 0:
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    uwb_pos[:] = raw_uwb_3d[:]
                        
                    is_moving = motion_analyzer.current_state not in [MotionState.STATIC, MotionState.JITTER]
                    apply_uwb_fusion(is_moving_state=is_moving)

                for i in range(IMU_SAMPLES_PER_PACKET):
                    s = 6 + i * 8
                    qx, qy, qz, qw = map(float, toks[s:s+4])
                    ax, ay, az = map(float, toks[s+4:s+7])
                    t_us = int(toks[s+7])
                    process_imu_sample(qx, qy, qz, qw, ax, ay, az, t_us)
            except ValueError:
                continue
    except Exception as ex:
        print(f"Update loop error: {ex}")

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == "__main__":
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), yz_win, reset_position)
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), yz_win, calibrate_alignment_with_quat)
    sys.exit(pg.exec())