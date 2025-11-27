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
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

ANCHORS = np.array([
    [1.41, 0.00, 0.00],
    [1.41, 1.35, 0.00],
    [0.00, 1.35, 0.00]
])

PEN_TIP_OFFSET = np.array([0.0, 0.0, 0.0])

# System Constants
TRAP_DT_MAX = 0.15
IMU_SAMPLES_PER_PACKET = 10
FORCE_ACTIVATION_THRESHOLD = 3.04 # Values are 0-31. >5 is considered contact.

# Filter & Bias Constants
BIAS_ALPHA = 0.01
VELOCITY_DECAY = 0.98

# Motion Thresholds (Used for ZVUP - Zero Velocity Update)
ACCEL_NOISE_FLOOR = 0.01
STATIC_ACCEL_THRESH = 0.06

SLOW_MOTION_THRESH = 0.15
NORMAL_MOTION_THRESH = 0.35
FAST_MOTION_THRESH = 0.8

VELOCITY_STATIC_THRESH = 0.005
VELOCITY_SLOW_THRESH = 0.04
VELOCITY_NORMAL_THRESH = 0.25
VELOCITY_FAST_THRESH = 0.8

JITTER_WINDOW = 5
JITTER_VARIANCE_THRESH = 0.001
JITTER_MAX_VEL = 0.03

CURVATURE_WINDOW = 8
CONTACT_VEL_AVG_WINDOW = 4

MAX_HANDWRITING_VELOCITY = 1.0

# -------------------------
# STARTUP CALIBRATION
# -------------------------
is_calibrating = True
calib_count = 0
CALIB_SAMPLES = 30  
imu_heading_offset = 0.0 

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
        self.contact_vel_avg = deque(maxlen=CONTACT_VEL_AVG_WINDOW)
        self.is_contact_detected = False

    def update(self, vel, accel, accel_mag, pos, dt):
        self.vel_history.append(vel.copy())
        self.accel_history.append(accel.copy())
        self.accel_mag_history.append(accel_mag)
        self.pos_history.append(pos.copy())
        self.contact_vel_avg.append(np.linalg.norm(vel))

        features = self._compute_features(np.linalg.norm(vel), accel_mag)
        new_state = self._classify_motion(features)

        if new_state != self.current_state:
            self.current_state = new_state
        
        avg_contact_vel = np.mean(list(self.contact_vel_avg)) if self.contact_vel_avg else 0
        self.is_contact_detected = avg_contact_vel > 0.01 
        
        return self.current_state, features

    def _compute_features(self, vel_mag, accel_mag):
        return {
            'vel_mag': vel_mag,
            'accel_mag': accel_mag,
            'vel_variance': np.var(list(self.vel_history)) if len(self.vel_history) > 2 else 0.0,
            'accel_variance': np.var(list(self.accel_mag_history)) if len(self.accel_mag_history) > 2 else 0.0,
            'is_jittering': self._detect_jitter(),
            'curvature': self._compute_curvature(),
        }

    def _detect_jitter(self):
        if len(self.vel_history) < 3: return False
        vels = list(self.vel_history)
        vel_mags = [np.linalg.norm(v) for v in vels[-JITTER_WINDOW:]]
        if not vel_mags: return False
        return np.var(vel_mags) > JITTER_VARIANCE_THRESH and max(vel_mags) < JITTER_MAX_VEL

    def _compute_curvature(self):
        if len(self.pos_history) < 3: return 0.0
        positions = list(self.pos_history)
        v1 = positions[-2] - positions[-3]
        v2 = positions[-1] - positions[-2]
        v1_norm = np.linalg.norm(v1); v2_norm = np.linalg.norm(v2)
        if v1_norm < 1e-6 or v2_norm < 1e-6: return 0.0
        dot_product = np.clip(np.dot(v1/v1_norm, v2/v2_norm), -1.0, 1.0)
        return np.arccos(dot_product) / np.pi

    def _classify_motion(self, features):
        vel_mag = features['vel_mag']
        accel_mag = features['accel_mag']
        is_jittering = features['is_jittering']

        if is_jittering and vel_mag < JITTER_MAX_VEL: return MotionState.JITTER
        if vel_mag < VELOCITY_STATIC_THRESH and accel_mag < STATIC_ACCEL_THRESH: return MotionState.STATIC
        if vel_mag < VELOCITY_SLOW_THRESH and accel_mag < ACCEL_NOISE_FLOOR: return MotionState.HOVER
        
        if accel_mag < SLOW_MOTION_THRESH: return MotionState.SLOW_DRAW
        if accel_mag < NORMAL_MOTION_THRESH: return MotionState.NORMAL_DRAW
        if accel_mag < FAST_MOTION_THRESH: return MotionState.FAST_DRAW
        
        if vel_mag > VELOCITY_FAST_THRESH: return MotionState.TRAJECTORY
        return MotionState.NORMAL_DRAW

class UWBIMUFusionFilter:
    """
    Complementary filter fusing UWB position (noisy, low-freq) 
    with IMU velocity (clean, high-freq).
    """
    
    def __init__(self):
        self.filtered_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        self.uwb_position_variance = np.zeros(3)
        self.position_history = deque(maxlen=10)
        self.measurement_count = 0
        
    def update_with_uwb(self, raw_uwb_pos, imu_vel, current_time):
        """Fuse UWB measurement with IMU velocity estimate."""
        
        # 1. Predict using IMU velocity since last UWB update
        if self.measurement_count > 0 and self.last_uwb_time > 0:
            dt = current_time - self.last_uwb_time
            if dt > 0 and dt < 1.0:
                predicted_pos = self.filtered_pos + imu_vel * dt
            else:
                predicted_pos = self.filtered_pos.copy()
        else:
            predicted_pos = raw_uwb_pos.copy()
        
        # 2. Measure position variance from recent UWB readings
        self.position_history.append(raw_uwb_pos.copy())
        if len(self.position_history) >= 3:
            history_array = np.array(list(self.position_history))
            self.uwb_position_variance = np.var(history_array, axis=0)
        
        # 3. Adaptive Kalman gain based on UWB noise
        base_variance = 0.02
        kalman_gain = np.clip(
            self.uwb_position_variance / (self.uwb_position_variance + base_variance),
            0.001, 0.02
        )
        
        # 4. Fuse: blend prediction with measurement
        innovation = raw_uwb_pos - predicted_pos
        self.filtered_pos = predicted_pos + kalman_gain * innovation
        
        # 5. Store for next iteration
        self.last_uwb_pos = raw_uwb_pos.copy()
        self.last_uwb_time = current_time
        self.measurement_count += 1
        
        return self.filtered_pos.copy(), kalman_gain
    
    def get_filtered_position(self):
        return self.filtered_pos.copy()
    
    def reset(self):
        self.filtered_pos[:] = 0
        self.last_uwb_pos[:] = 0
        self.last_uwb_time = 0
        self.uwb_position_variance[:] = 0
        self.position_history.clear()
        self.measurement_count = 0

# -------------------------
# STATE
# -------------------------
pos = np.zeros(3)
vel = np.zeros(3)
accel_filtered = np.zeros(3)
accel_bias = np.zeros(3)
gravity_component = np.zeros(3)

prev_accel = None
prev_vel = None
last_timestamp_us = None

uwb_pos = np.zeros(3)
last_uwb_pos = np.zeros(3)
last_uwb_time = 0

has_first_fix = False
is_writing = False # Replaces is_simulated_contact

motion_analyzer = MotionAnalyzer()
uwb_imu_fusion = UWBIMUFusionFilter()

# Visualization History
yz_strokes = []
current_stroke = []
stroke_items = []

# -------------------------
# HELPERS
# -------------------------
def micros_dt(last, now):
    if last is None: return None
    diff = now - last
    if diff < 0: diff += 4294967296
    return diff / 1e6

def trilaterate(distances, anchors):
    def residuals(x, anchors, dists):
        return np.linalg.norm(anchors - x, axis=1) - dists
    x0 = np.mean(anchors, axis=0)
    res = least_squares(residuals, x0, args=(anchors, distances),
                        bounds=([-10, -10, -1], [10, 10, 3]))
    return res.x

def get_motion_state_name(state):
    names = {
        MotionState.STATIC: "STATIC", MotionState.JITTER: "JITTER",
        MotionState.SLOW_DRAW: "SLOW_DRAW", MotionState.NORMAL_DRAW: "NORMAL_DRAW",
        MotionState.FAST_DRAW: "FAST_DRAW", MotionState.TRAJECTORY: "TRAJECTORY",
        MotionState.HOVER: "HOVER",
    }
    return names.get(state, "UNKNOWN")

def calibrate_alignment_with_quat(r: R):
    global imu_heading_offset
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

# --- STATUS INDICATOR (Formerly Toggle Button) ---
status_btn = QtWidgets.QPushButton("Wait for Input...", parent=yz_win)
status_btn.setGeometry(10, 10, 260, 30)
status_btn.setStyleSheet("background-color: #555555; color: white; font-weight: bold;")
status_btn.show()

# --- AUTO FOLLOW CHECKBOX ---
auto_follow_cb = QtWidgets.QCheckBox("Auto-Follow", parent=yz_win)
auto_follow_cb.setGeometry(280, 10, 100, 30)
auto_follow_cb.setStyleSheet("color: black; font-weight: bold; background: rgba(255,255,255,0.5);")
auto_follow_cb.setChecked(True)
auto_follow_cb.show()

# --- RESET FUNCTION ---
def reset_position():
    global pos, vel, yz_strokes, current_stroke, stroke_items, last_timestamp_us
    global uwb_pos, motion_analyzer, is_calibrating, calib_count, gravity_component, has_first_fix
    global uwb_imu_fusion, is_writing

    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    gravity_component[:] = 0
    last_timestamp_us = None

    is_calibrating = True
    calib_count = 0
    is_writing = False
    motion_analyzer = MotionAnalyzer()
    uwb_imu_fusion.reset()

    yz_strokes.clear()
    current_stroke.clear()
    for item in stroke_items:
        try: yz_win.removeItem(item)
        except Exception: pass
    stroke_items.clear()

    current_stroke = []
    yz_dot.setData([0], [0])
    has_first_fix = False
    print("--- Reset ---")

# -------------------------
# IMU PROCESSING
# -------------------------
def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, force, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us
    global yz_strokes, current_stroke, stroke_items, imu_heading_offset
    global is_calibrating, calib_count, motion_analyzer, gravity_component
    global is_writing # Logic now based on force

    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us
    if dt is None or dt <= 0 or dt > TRAP_DT_MAX: return False

    # 1. Rotation (Local -> Global)
    r = R.from_quat([qx, qy, qz, qw])
    accel_world = r.apply(accel_local)
    accel_world[1] *= -1

    # 2. Startup Calibration
    if is_calibrating:
        calib_count += 1
        if calib_count >= CALIB_SAMPLES:
            calibrate_alignment_with_quat(r)
            is_calibrating = False
            print("--- Calibration done ---")
        return True
    
    # --- FORCE SENSOR LOGIC ---
    # Determine contact based on threshold
    is_touching = force > FORCE_ACTIVATION_THRESHOLD
    
    # State Transition: Hover -> Write
    if is_touching and not is_writing:
        is_writing = True
        
        # --- NEW CODE: SNAP TO UWB START POINT ---
        # This acts like a "reset" every time you start a new character.
        # We only do this if we actually have a valid UWB fix.
        if has_first_fix and np.linalg.norm(uwb_pos) > 0.1:
            pos[:] = uwb_pos[:] 
            # Optional: Reset velocity to 0 to prevent "sliding" from previous momentum
            vel[:] = 0 
        # -----------------------------------------

        status_btn.setText(f"Writing (Force: {force})")
        status_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        
        # Start new stroke
        current_stroke = []
        yz_strokes.append(current_stroke)
        new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
        yz_win.addItem(new_curve)
        stroke_items.append(new_curve)
        
    # State Transition: Write -> Hover
    elif not is_touching and is_writing:
        is_writing = False
        status_btn.setText(f"Hovering (Force: {force})")
        status_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
    
    # Update Status Text if state implies it
    if is_writing:
         status_btn.setText(f"Writing (Force: {force})")
    else:
         status_btn.setText(f"Hovering (Force: {force})")

    cutoff_freq = 0.05
    RC = 1.0 / (cutoff_freq * 2 * np.pi)
    dynamic_alpha = RC / (RC + dt)

    # 3. ITrackU Filter Phase
    gravity_component[:] = dynamic_alpha * gravity_component + (1 - dynamic_alpha) * accel_world
    accel_high_passed = accel_world - gravity_component
    accel_filtered[:] = accel_high_passed
    accel_mag = np.linalg.norm(accel_filtered)

    # 4. Motion Analysis
    motion_state, features = motion_analyzer.update(vel, accel_filtered, accel_mag, pos, dt)

    # 5. Integration
    if motion_state == MotionState.STATIC:
        accel_bias[:] = (1 - BIAS_ALPHA) * accel_bias + BIAS_ALPHA * accel_filtered
        vel[:] = 0.0
        
    elif motion_state in [MotionState.JITTER, MotionState.HOVER]:
        vel[:] *= VELOCITY_DECAY 

    else:
        accel_corrected = accel_filtered - accel_bias
        if prev_accel is None:
            prev_accel = accel_corrected.copy()
            prev_vel = vel.copy()
        
        vel += 0.5 * (prev_accel + accel_corrected) * dt
        vel *= VELOCITY_DECAY 
        pos += 0.5 * (prev_vel + vel) * dt
        
        prev_accel = accel_corrected.copy()
        prev_vel = vel.copy()
    
    rot_matrix = r.as_matrix()
    rotated_offset = rot_matrix @ PEN_TIP_OFFSET
    tip_pos = pos + rotated_offset

    # 6. Continuous Drawing (Gated by Force Sensor)
    if is_writing and len(stroke_items) > 0:
        current_stroke.append((tip_pos[0], tip_pos[2]))
        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])

    # 7. Visualization Update
    yz_dot.setData([tip_pos[0]], [tip_pos[2]])
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[2]])

    # ---- NEW CODE: AUTO-ZOOM CAMERA ----
    if auto_follow_cb.isChecked():
        view_radius = 0.15 # 15 cm radius
        yz_win.setXRange(tip_pos[0] - view_radius, tip_pos[0] + view_radius, padding=0)
        yz_win.setYRange(tip_pos[2] - view_radius, tip_pos[2] + view_radius, padding=0)
    # ------------------------------------

    state_name = get_motion_state_name(motion_state)
    info_label.setText(
        f"Status: {'CALIBRATING...' if is_calibrating else 'READY'}\n"
        f"Motion State: {state_name}\n"
        f"Pos X: {tip_pos[0]:.3f} | Y: {tip_pos[2]:.3f}\n"
        f"Vel: {np.linalg.norm(vel):.4f} m/s | Accel: {accel_mag:.4f} m/s²\n"
        f"Curvature: {features['curvature']:.3f} | Force: {force}"
    )

    # Buffer management
    if len(current_stroke) > 2000:
        current_stroke = []
        yz_strokes.append(current_stroke)
        new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
        yz_win.addItem(new_curve)
        stroke_items.append(new_curve)
        
    if len(yz_strokes) > 100:
        yz_strokes.pop(0)
        old_item = stroke_items.pop(0)
        yz_win.removeItem(old_item)

    return True

# -------------------------
# UPDATE LOOP
# -------------------------
def update():
    global _serial_buf, uwb_pos, last_uwb_time, last_uwb_pos, vel, pos, has_first_fix
    global uwb_imu_fusion, motion_analyzer
    
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
            if not raw: continue
            toks = raw.split(',')
            
            # New Packet Length Check:
            # 6 header fields (x, y, d0, d1, d2, ts) 
            # + 10 samples * 9 fields (qx, qy, qz, qw, ax, ay, az, force, ts)
            # = 6 + 90 = 96 fields
            if len(toks) < 96: continue

            try:
                # -------------------------------------------------
                # 1. PROCESS UWB
                # -------------------------------------------------
                d0 = float(toks[2]); d1 = float(toks[3]); d2 = float(toks[4])
                if d0 > 0 and d1 > 0 and d2 > 0:
                    current_time = time.time()
                    
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    raw_uwb_3d_transformed = np.array([
                        raw_uwb_3d[0],
                        raw_uwb_3d[2],
                        raw_uwb_3d[1]
                    ])
                    
                    if not has_first_fix:
                        uwb_pos[:] = raw_uwb_3d_transformed[:]
                        pos[:] = raw_uwb_3d_transformed[:]
                        last_uwb_pos[:] = raw_uwb_3d_transformed[:]
                        vel[:] = 0
                        has_first_fix = True
                        print(f"Fix Acquired at: {raw_uwb_3d_transformed}")
                        continue
                    
                    # Outlier Rejection
                    dist_jump = np.linalg.norm(raw_uwb_3d_transformed - uwb_pos)
                    if dist_jump > 1.0 and np.linalg.norm(uwb_pos) > 0.1:
                        pass
                    else:
                        # UWB-IMU Fusion Filter
                        if len(motion_analyzer.vel_history) > 0:
                            recent_imu_vel = motion_analyzer.vel_history[-1]
                        else:
                            recent_imu_vel = np.zeros(3)
                        
                        filtered_pos, kgain = uwb_imu_fusion.update_with_uwb(
                            raw_uwb_3d_transformed,
                            recent_imu_vel,
                            current_time
                        )
                        uwb_pos[:] = filtered_pos
                        
                        pos_error = uwb_pos - pos
                        if is_writing:
                            # When writing, TRUST IMU. Very low correction (1%)
                            # This allows smooth curves without UWB "tugging" the pen
                            correction_strength = 0.00
                        else:
                            # When hovering, snap back to UWB faster (5%)
                            correction_strength = 0.15  # Tuning parameter (0.05 to 0.2)
                        pos[:] += pos_error * correction_strength

                        last_uwb_pos[:] = uwb_pos.copy()
                        last_uwb_time = current_time

                # -------------------------------------------------
                # 2. PROCESS IMU
                # -------------------------------------------------
                for i in range(IMU_SAMPLES_PER_PACKET):
                    # Stride is now 9 (was 8)
                    # Offsets: qx, qy, qz, qw, ax, ay, az, force, ts
                    s = 6 + i * 9
                    qx, qy, qz, qw = map(float, toks[s:s+4])
                    ax, ay, az = map(float, toks[s+4:s+7])
                    force = float(toks[s+7]) # Force value (0-31)
                    t_us = int(toks[s+8])
                    
                    process_imu_sample(qx, qy, qz, qw, ax, ay, az, force, t_us)
            
            except ValueError: continue
    except Exception as ex:
        print(f"Update loop error: {ex}")

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == "__main__":
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), yz_win, reset_position)
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), yz_win, calibrate_alignment_with_quat)
    sys.exit(pg.exec())