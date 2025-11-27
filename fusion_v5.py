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

PEN_TIP_OFFSET = np.array([0.0, 0.0, -0.12])
ALIGNMENT_ANGLE_DEG = 0.0

# System Constants
TRAP_DT_MAX = 0.15
IMU_SAMPLES_PER_PACKET = 10

# Filter & Bias Constants
BIAS_ALPHA = 0.008
VELOCITY_DECAY = 0.97    # Soft decay to prevent runaway when no UWB

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

UWB_SMOOTHING_ALPHA = 0.05  # Lower = Smoother Green Cross, Higher = Faster response
MAX_HANDWRITING_VELOCITY = 1.0 # m/s (Ignore UWB jumps faster than this)

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
        vel_mag = np.linalg.norm(vel)
        self.contact_vel_avg.append(vel_mag)

        features = self._compute_features(vel_mag, accel_mag)
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

# -------------------------
# STATE
# -------------------------
pos = np.zeros(3)
vel = np.zeros(3)
accel_filtered = np.zeros(3)
accel_bias = np.zeros(3)
gravity_component = np.zeros(3) # ITrackU High Pass Filter State

prev_accel = None
prev_vel = None
last_timestamp_us = None

uwb_pos = np.zeros(3)
last_uwb_pos = np.zeros(3)
last_uwb_time = 0

has_first_fix = False
is_simulated_contact = False 

motion_analyzer = MotionAnalyzer()

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

# --- CONTACT BUTTON (REPLACES ZERO POSITION BUTTON) ---
def toggle_contact():
    global is_simulated_contact, current_stroke, yz_strokes, stroke_items
    global pos, vel, prev_vel, prev_accel, last_uwb_pos, has_first_fix

    # remember previous state to detect OFF->ON transition
    prev_contact = is_simulated_contact
    is_simulated_contact = not is_simulated_contact

    if is_simulated_contact:
        toggle_btn.setText("Simulated Contact: ON (Writing)")
        toggle_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")

        # --- NEW: if we are switching from HOVERING -> WRITING, snap IMU to last UWB ---
        # Only perform the teleport when we have a valid UWB fix
        if (not prev_contact) and has_first_fix:
            # Teleport IMU state to last UWB position (trusted)
            pos[:] = last_uwb_pos.copy()
            # zero velocity and previous integration buffers so we don't "launch"
            vel[:] = 0.0
            prev_vel = None
            prev_accel = None

            # Update the visible IMU dot immediately to reflect teleport
            try:
                yz_dot.setData([pos[0]], [pos[2]])
            except Exception:
                # fallback: set to UWB ghost XY if something odd happens
                yz_dot.setData([last_uwb_pos[0]], [last_uwb_pos[2] if len(last_uwb_pos) > 2 else last_uwb_pos[1]])
            print(f"[TOGGLE] Teleported IMU to last UWB position: {last_uwb_pos}")

        # START A NEW STROKE SEGMENT
        current_stroke = []
        yz_strokes.append(current_stroke)
        new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
        yz_win.addItem(new_curve)
        stroke_items.append(new_curve)
    else:
        toggle_btn.setText("Simulated Contact: OFF (Hovering)")
        toggle_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")

# Create the toggle button inside the Plot Window (Overlay)
toggle_btn = QtWidgets.QPushButton("Simulated Contact: OFF (Hovering)", parent=yz_win)
toggle_btn.setGeometry(10, 10, 260, 30) # Top-Left of the plot
toggle_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
toggle_btn.setCheckable(True)
toggle_btn.clicked.connect(toggle_contact)
toggle_btn.show()
# ------------------------


# --- HIDDEN RESET FUNCTION (Keys only) ---
def reset_position():
    global pos, vel, yz_strokes, current_stroke, stroke_items, last_timestamp_us
    global uwb_pos, motion_analyzer, is_calibrating, calib_count, gravity_component, has_first_fix

    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    gravity_component[:] = 0 # Reset HPF state
    last_timestamp_us = None

    is_calibrating = True
    calib_count = 0
    motion_analyzer = MotionAnalyzer()

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
# -------------------------------------------

# -------------------------
# IMU PROCESSING
# -------------------------
def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us
    global yz_strokes, current_stroke, stroke_items, imu_heading_offset
    global is_calibrating, calib_count, motion_analyzer, gravity_component
    global is_simulated_contact 

    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us
    if dt is None or dt <= 0 or dt > TRAP_DT_MAX: return False

    # 1. Rotation (Local -> Global)
    r = R.from_quat([qx, qy, qz, qw])
    accel_world = r.apply(accel_local)
    accel_world[1] *= -1 # Axis correction
    
    # --- ALIGNMENT ROTATION ---
    if ALIGNMENT_ANGLE_DEG != 0:
        align_rad = np.radians(ALIGNMENT_ANGLE_DEG)
        c, s = np.cos(align_rad), np.sin(align_rad)
        ax_new = accel_world[0] * c - accel_world[1] * s
        ay_new = accel_world[0] * s + accel_world[1] * c
        accel_world[0] = ax_new
        accel_world[1] = ay_new
    # --------------------------

    # 2. Startup Calibration
    if is_calibrating:
        calib_count += 1
        if calib_count >= CALIB_SAMPLES:
            calibrate_alignment_with_quat(r)
            is_calibrating = False
            print("--- Calibration done ---")
        return True
    
    cutoff_freq = 0.05 # Hz, per ITrackU paper
    RC = 1.0 / (cutoff_freq * 2 * np.pi)
    dynamic_alpha = RC / (RC + dt)

    # 3. ITrackU Filter Phase
    gravity_component[:] = dynamic_alpha* gravity_component + (1 - dynamic_alpha) * accel_world
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

    # 2. Rotate the offset vector
    rotated_offset = rot_matrix @ PEN_TIP_OFFSET
    
    if ALIGNMENT_ANGLE_DEG != 0:
        rx = rotated_offset[0] * c - rotated_offset[1] * s
        ry = rotated_offset[0] * s + rotated_offset[1] * c
        rotated_offset[0] = rx
        rotated_offset[1] = ry

    # 3. Calculate actual tip position
    tip_pos = pos + rotated_offset

    # 6. Continuous Drawing (Controlled by Toggle Button)
    #    Only append points if Simulated Contact is TRUE
    if is_simulated_contact and len(stroke_items) > 0:
        current_stroke.append((tip_pos[0], tip_pos[2]))
        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])

    # 7. Visualization Update
    yz_dot.setData([tip_pos[0]], [tip_pos[2]])
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[1]])

    state_name = get_motion_state_name(motion_state)
    info_label.setText(
        f"Status: {'CALIBRATING...' if is_calibrating else 'READY'}\n"
        f"Motion State: {state_name}\n"
        f"Pos X: {tip_pos[0]:.3f} | Y: {tip_pos[1]:.3f}\n"
        f"Vel: {np.linalg.norm(vel):.4f} m/s | Accel: {accel_mag:.4f} m/s²\n"
        f"Curvature: {features['curvature']:.3f} | Contact: {is_simulated_contact}"
    )

    # Buffer management
    if len(current_stroke) > 2000:
        current_stroke = []
        yz_strokes.append(current_stroke)
        new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
        yz_win.addItem(new_curve)
        stroke_items.append(new_curve)
        
    if len(yz_strokes) > 5:
        yz_strokes.pop(0)
        old_item = stroke_items.pop(0)
        yz_win.removeItem(old_item)

    return True

# -------------------------
# UPDATE LOOP
# -------------------------
def update():
    global _serial_buf, uwb_pos, last_uwb_time, last_uwb_pos, vel, pos, gravity_component, has_first_fix
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
            if len(toks) < 86: continue

            try:
                # -------------------------------------------------
                # 1. PROCESS UWB (The "Green Cross")
                # -------------------------------------------------
                d0 = float(toks[2]); d1 = float(toks[3]); d2 = float(toks[4])
                if d0 > 0 and d1 > 0 and d2 > 0:
                    current_time = time.time()
                    dt_uwb = current_time - last_uwb_time
                    
                    # A. Raw Trilateration
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    
                    if not has_first_fix:
                        # TELEPORT everything to this coordinate
                        uwb_pos[:] = raw_uwb_3d[:]
                        pos[:] = raw_uwb_3d[:]  # Snap IMU to UWB
                        last_uwb_pos[:] = raw_uwb_3d[:]
                        
                        # Also zero out velocity so we don't "launch" from the snap
                        vel[:] = 0 
                        
                        has_first_fix = True
                        print(f"Fix Acquired at: {raw_uwb_3d}")
                        continue # Skip the rest of the UWB log
                    
                    # B. Outlier Rejection (Glitch Filter)
                    dist_jump = np.linalg.norm(raw_uwb_3d - uwb_pos)
                    if dist_jump > 1.0 and np.linalg.norm(uwb_pos) > 0.1:
                        pass # Ignore glitch
                    else:
                        # C. Low Pass Filter on UWB Position
                        uwb_pos[:] = (UWB_SMOOTHING_ALPHA * raw_uwb_3d) + \
                                     ((1 - UWB_SMOOTHING_ALPHA) * uwb_pos)

                        # D. Calculate UWB Velocity
                        if dt_uwb > 0 and dt_uwb < 0.5:
                            uwb_velocity = (uwb_pos - last_uwb_pos) / dt_uwb
                            uwb_speed = np.linalg.norm(uwb_velocity)

                            # E. Velocity Clamp
                            if uwb_speed < MAX_HANDWRITING_VELOCITY:
                                # F. The "Filter-and-Reset" Fusion
                                reset_factor = 0.2 
                                vel[:] = (1 - reset_factor) * vel + (reset_factor * uwb_velocity)
                                
                                # Position Leash
                                pos_offset = uwb_pos - pos
                                pos[:] += pos_offset * 0.15 

                        last_uwb_pos[:] = uwb_pos.copy()
                        last_uwb_time = current_time

                # -------------------------------------------------
                # 2. PROCESS IMU (The "Red Dot")
                # -------------------------------------------------
                for i in range(IMU_SAMPLES_PER_PACKET):
                    s = 6 + i * 8
                    qx, qy, qz, qw = map(float, toks[s:s+4])
                    ax, ay, az = map(float, toks[s+4:s+7])
                    t_us = int(toks[s+7])
                    process_imu_sample(qx, qy, qz, qw, ax, ay, az, t_us)
            
            except ValueError: continue
    except Exception as ex:
        print(f"Update loop error: {ex}")

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == "__main__":
    # Keyboard shortcut for hidden reset function
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), yz_win, reset_position)
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), yz_win, calibrate_alignment_with_quat)
    sys.exit(pg.exec())