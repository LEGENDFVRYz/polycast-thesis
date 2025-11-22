import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from collections import deque
import sys

# ============================================
# CONFIGURATION
# ============================================
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

ANCHORS = np.array([
    [1.75, 0.00, 0.00],  # Anchor 0
    [1.75, 1.61, 0.00],  # Anchor 1
    [0.00, 0.00, 0.00]   # Anchor 2
])

# --- FUNDAMENTAL PARAMETERS ---
LP_ALPHA = 0.25                 # Accel low-pass filter
BIAS_ALPHA = 0.008             # Bias update rate when static
VELOCITY_DECAY = 0.97          # Velocity damping per frame
TRAP_DT_MAX = 0.15             # Max dt before skip

IMU_SAMPLES_PER_PACKET = 10

# --- FUSION SETTINGS ---
UWB_ALPHA_MOVING = 0.015
UWB_ALPHA_STATIC = 0.03

# --- MOTION STATE DETECTION THRESHOLDS (HIGH SENSITIVITY) ---
# Reduced noise floor to catch subtle movements
ACCEL_NOISE_FLOOR = 0.01       # Was 0.02

# Lowered static threshold so it exits "stop" mode easier
STATIC_ACCEL_THRESH = 0.06     # Was 0.08

# Adjusted motion regimes
SLOW_MOTION_THRESH = 0.15      
NORMAL_MOTION_THRESH = 0.35    
FAST_MOTION_THRESH = 0.8       
EXTREME_MOTION_THRESH = 2.0    

VELOCITY_STATIC_THRESH = 0.005 # Was 0.01 (start moving sooner)
VELOCITY_SLOW_THRESH = 0.04
VELOCITY_NORMAL_THRESH = 0.25
VELOCITY_FAST_THRESH = 0.8

# *** CRITICAL TUNING FOR "FLOATING" ISSUE ***
# Lower this: Speed required to put ink on paper
DRAW_VEL_THRESH = 0.005        # Was 0.018 (Now activates at 5mm/s)

# Lower this: Force required to prove intentionality
# If this is too high, smooth constant-speed writing (which has 0 accel) won't draw.
DRAW_ACCEL_THRESH = 0.015      # Was 0.05 (Drastically reduced)

STATIC_TIME_BRIEF = 0.15       # Brief pause (0.15s)
STATIC_TIME_NORMAL = 0.4       # Normal static (0.4s) - between letters
STATIC_TIME_LONG = 1.0         # Long static (1.0s+) - lifted pen

# --- JITTER DETECTION ---
JITTER_WINDOW = 5              # Last N velocity samples to check
JITTER_VARIANCE_THRESH = 0.001 # Variance of velocity = jitter
JITTER_MAX_VEL = 0.03          # Max velocity in jitter state

# --- TRAJECTORY CURVATURE ANALYSIS ---
CURVATURE_WINDOW = 8           # Points to compute curvature
CURVATURE_SHARP_THRESH = 0.8   # High curvature = sharp corner
CURVATURE_SMOOTH_THRESH = 0.15 # Low curvature = smooth curve

# --- PRESSURE/CONTACT PROXY (use velocity magnitude history) ---
CONTACT_VEL_AVG_WINDOW = 4     # Rolling average window

# ============================================
# CALIBRATION STATE
# ============================================
is_calibrating = True
calib_count = 0
calib_accum = np.zeros(3)
gravity_ref = np.array([0.0, 0.0, 9.81])

# ============================================
# SERIAL
# ============================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Serial Error: {e}")
    sys.exit(1)

_serial_buf = ""

# ============================================
# MOTION STATE MACHINE
# ============================================
class MotionState:
    """Enum for motion detection states"""
    STATIC = 0           # Not moving
    JITTER = 1           # High-frequency noise
    SLOW_DRAW = 2        # Slow intentional writing
    NORMAL_DRAW = 3      # Normal handwriting speed
    FAST_DRAW = 4        # Fast writing (cursive, connecting)
    TRAJECTORY = 5       # Ballistic motion (pen moving but not drawing)
    HOVER = 6            # Hovering above surface

class MotionAnalyzer:
    """Advanced motion detection with history and context"""
    def __init__(self, window_size=20):
        self.vel_history = deque(maxlen=window_size)
        self.accel_history = deque(maxlen=window_size)
        self.accel_mag_history = deque(maxlen=window_size)
        self.state_history = deque(maxlen=50)
        self.pos_history = deque(maxlen=CURVATURE_WINDOW)
        
        self.current_state = MotionState.STATIC
        self.time_in_state = 0.0
        self.state_transition_time = time.time()
        
        self.contact_vel_avg = deque(maxlen=CONTACT_VEL_AVG_WINDOW)
        self.is_contact_detected = False
        
    def update(self, vel, accel, accel_mag, pos, dt):
        """Update motion analyzer with new sensor data"""
        self.vel_history.append(vel.copy())
        self.accel_history.append(accel.copy())
        self.accel_mag_history.append(accel_mag)
        self.pos_history.append(pos.copy())
        
        vel_mag = np.linalg.norm(vel)
        self.contact_vel_avg.append(vel_mag)
        
        # Compute motion features
        features = self._compute_features(vel_mag, accel_mag)
        
        # State machine
        new_state = self._classify_motion(features)
        
        # Smooth transitions (avoid flickering)
        if new_state != self.current_state:
            self.current_state = new_state
            self.state_transition_time = time.time()
            self.time_in_state = 0.0
        else:
            self.time_in_state += dt
        
        self.state_history.append((self.current_state, time.time()))
        
        # Contact detection (rolling average of velocity)
        avg_contact_vel = np.mean(list(self.contact_vel_avg)) if self.contact_vel_avg else 0
        self.is_contact_detected = avg_contact_vel > DRAW_VEL_THRESH * 0.8
        
        return self.current_state, features
    
    def _compute_features(self, vel_mag, accel_mag):
        """Compute motion features from sensor data"""
        features = {
            'vel_mag': vel_mag,
            'accel_mag': accel_mag,
            'vel_variance': np.var(list(self.vel_history)) if len(self.vel_history) > 2 else 0,
            'accel_variance': np.var(list(self.accel_mag_history)) if len(self.accel_mag_history) > 2 else 0,
            'is_jittering': self._detect_jitter(),
            'curvature': self._compute_curvature(),
            'acceleration_direction_change': self._detect_accel_direction_change(),
        }
        return features
    
    def _detect_jitter(self):
        """Detect high-frequency jitter (noise or micro-tremor)"""
        if len(self.vel_history) < 3:
            return False
        
        vels = list(self.vel_history)
        vel_mags = [np.linalg.norm(v) for v in vels[-JITTER_WINDOW:]]
        
        if not vel_mags:
            return False
        
        variance = np.var(vel_mags)
        max_vel = max(vel_mags)
        
        # Jitter: high variance but low absolute velocity
        is_jitter = variance > JITTER_VARIANCE_THRESH and max_vel < JITTER_MAX_VEL
        return is_jitter
    
    def _compute_curvature(self):
        """Compute path curvature (sharp corners vs smooth curves)"""
        if len(self.pos_history) < 3:
            return 0.0
        
        positions = list(self.pos_history)
        
        # Compute direction vectors
        v1 = positions[-2] - positions[-3] if len(positions) >= 3 else None
        v2 = positions[-1] - positions[-2]
        
        if v1 is None:
            return 0.0
        
        # Normalize
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm < 1e-6 or v2_norm < 1e-6:
            return 0.0
        
        v1 = v1 / v1_norm
        v2 = v2 / v2_norm
        
        # Curvature from angle between vectors (higher = sharper turn)
        dot_product = np.clip(np.dot(v1, v2), -1, 1)
        angle = np.arccos(dot_product)
        curvature = angle / np.pi  # Normalize to [0, 1]
        
        return curvature
    
    def _detect_accel_direction_change(self):
        """Detect sudden changes in acceleration direction (sharp strokes)"""
        if len(self.accel_history) < 2:
            return 0.0
        
        accels = list(self.accel_history)[-3:]
        
        if len(accels) < 2:
            return 0.0
        
        a1 = accels[-2]
        a2 = accels[-1]
        
        a1_norm = np.linalg.norm(a1)
        a2_norm = np.linalg.norm(a2)
        
        if a1_norm < 1e-6 or a2_norm < 1e-6:
            return 0.0
        
        a1 = a1 / a1_norm
        a2 = a2 / a2_norm
        
        dot_product = np.clip(np.dot(a1, a2), -1, 1)
        angle = np.arccos(dot_product)
        direction_change = angle / np.pi
        
        return direction_change
    
    def _classify_motion(self, features):
            """Classify motion state using multi-feature decision tree"""
            vel_mag = features['vel_mag']
            accel_mag = features['accel_mag']
            is_jittering = features['is_jittering']
            curvature = features['curvature']
            
            # Priority 1: Detect jitter (noise/tremor)
            if is_jittering and vel_mag < JITTER_MAX_VEL:
                return MotionState.JITTER
            
            # Priority 2: Detect static state (very low motion)
            if vel_mag < VELOCITY_STATIC_THRESH and accel_mag < STATIC_ACCEL_THRESH:
                return MotionState.STATIC
            
            # --- CHANGED LOGIC HERE ---
            
            # Priority 3: Hovering 
            # Only hover if velocity is tiny AND acceleration is essentially zero.
            # We tightened the constraints here so it falls through to "DRAW" easier.
            if vel_mag < VELOCITY_SLOW_THRESH and accel_mag < ACCEL_NOISE_FLOOR:
                return MotionState.HOVER
            
            # Priority 4: Slow deliberate writing
            if accel_mag < SLOW_MOTION_THRESH and vel_mag < VELOCITY_NORMAL_THRESH:
                # If we have ANY significant velocity, assume we are drawing.
                # We removed the strict acceleration check here.
                if vel_mag > DRAW_VEL_THRESH: 
                    return MotionState.SLOW_DRAW
                
                # Fallback: if velocity is low, check acceleration intent
                if accel_mag > DRAW_ACCEL_THRESH:
                    return MotionState.SLOW_DRAW
            
            # Priority 5: Normal handwriting
            if accel_mag < NORMAL_MOTION_THRESH and vel_mag < VELOCITY_FAST_THRESH:
                if curvature > CURVATURE_SHARP_THRESH:
                    return MotionState.NORMAL_DRAW 
                else:
                    return MotionState.NORMAL_DRAW
            
            # Priority 6: Fast/cursive writing
            if accel_mag < FAST_MOTION_THRESH and vel_mag > VELOCITY_NORMAL_THRESH:
                return MotionState.FAST_DRAW
            
            # Priority 7: Ballistic trajectory (High speed, zero accel)
            # This is when you flick the pen across the page in the air
            if vel_mag > VELOCITY_FAST_THRESH and accel_mag < ACCEL_NOISE_FLOOR:
                return MotionState.TRAJECTORY
            
            # Default catch-all: If moving reasonably, draw.
            if vel_mag > DRAW_VEL_THRESH:
                return MotionState.NORMAL_DRAW
            
            return MotionState.STATIC

# ============================================
# CORE STATE VARIABLES
# ============================================
pos = np.zeros(3)
vel = np.zeros(3)
accel_filtered = np.zeros(3)
accel_bias = np.zeros(3)
prev_accel = None
prev_vel = None
last_timestamp_us = None
static_since = None
imu_heading_offset = 0.0

uwb_pos = np.zeros(3)
uwb_center_offset = np.zeros(3)
fused_pos = np.zeros(3)

# Motion analyzer
motion_analyzer = MotionAnalyzer()

# Drawing state
yz_strokes = []
current_stroke = []
is_drawing = False
stroke_items = []

# ============================================
# HELPER FUNCTIONS
# ============================================
def micros_dt(last, now):
    if last is None: return None
    diff = now - last
    if diff < 0: diff += 4294967296
    return diff / 1e6

def rotate_vec2d(x, y, theta):
    cs = np.cos(theta)
    sn = np.sin(theta)
    rx = x * cs - y * sn
    ry = x * sn + y * cs
    return rx, ry

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
    """Convert motion state enum to readable name"""
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

def calibrate_alignment():
    global imu_heading_offset, vel
    imu_angle = np.arctan2(vel[1], vel[0])
    target_angle = 0.0
    new_offset = target_angle - imu_angle
    imu_heading_offset += new_offset
    print(f"--- Heading Calibrated: {np.degrees(imu_heading_offset):.2f}° ---")

# ============================================
# PyQt5 GUI
# ============================================
app = QtWidgets.QApplication([])

yz_win = pg.plot(title="Advanced Motion Tracking - YZ Plane")
yz_win.setLabel('bottom', 'Y (m)')
yz_win.setLabel('left', 'Z (m)')
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
    global is_calibrating, calib_count, calib_accum
    
    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    fused_pos[:] = 0
    uwb_center_offset[:] = 0
    last_timestamp_us = None
    
    is_calibrating = True
    calib_count = 0
    calib_accum[:] = 0
    
    motion_analyzer = MotionAnalyzer()
    
    yz_strokes.clear()
    current_stroke.clear()
    is_drawing = False
    for item in stroke_items:
        try: yz_win.removeItem(item)
        except: pass
    stroke_items.clear()
    
    yz_dot.setData([0], [0])
    print("--- Reset ---")

reset_btn = QtWidgets.QPushButton("Zero Position", parent=yz_win)
reset_btn.clicked.connect(reset_position)
reset_btn.show()

# ============================================
# IMU PROCESSING WITH ADVANCED MOTION DETECTION
# ============================================
def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us, static_since
    global yz_strokes, current_stroke, is_drawing, stroke_items, fused_pos, imu_heading_offset
    global is_calibrating, calib_count, calib_accum, gravity_ref
    global motion_analyzer

    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us

    if dt is None or dt <= 0 or dt > TRAP_DT_MAX: 
        return False

    # 1. Rotate acceleration to world frame
    r = R.from_quat([qx, qy, qz, qw])
    accel_world = r.apply(accel_local)
    accel_world[1] *= -1

    # 2. Align to heading
    ax_aligned, ay_aligned = rotate_vec2d(accel_world[0], accel_world[1], imu_heading_offset)
    accel_world[0] = ax_aligned
    accel_world[1] = ay_aligned
    
    # 3. Calibration phase
    if is_calibrating:
        calib_accum += accel_world
        calib_count += 1
        if calib_count >= 50:
            gravity_ref = calib_accum / 50.0
            is_calibrating = False
            print(f"Calibrated Gravity: {gravity_ref}")
        return True

    # 4. Remove gravity
    accel_world -= gravity_ref

    # 5. Noise floor
    if abs(accel_world[1]) < ACCEL_NOISE_FLOOR: accel_world[1] = 0
    if abs(accel_world[2]) < ACCEL_NOISE_FLOOR: accel_world[2] = 0

    # 6. Low-pass filter
    accel_filtered[:] = LP_ALPHA * accel_world + (1 - LP_ALPHA) * accel_filtered
    accel_mag = np.linalg.norm(accel_filtered)

    # 7. Update motion analyzer and get state
    motion_state, features = motion_analyzer.update(vel, accel_filtered, accel_mag, fused_pos, dt)

    # 8. Handle different motion states
    if motion_state == MotionState.STATIC:
        accel_bias[:] = (1 - BIAS_ALPHA) * accel_bias + BIAS_ALPHA * accel_filtered
        vel[:] = 0.0
        is_drawing = False

    elif motion_state == MotionState.JITTER:
        # Ignore jitter, don't integrate
        vel[:] *= VELOCITY_DECAY
        pass

    elif motion_state == MotionState.HOVER:
        # Light integration but don't draw
        is_drawing = False
        vel[:] *= VELOCITY_DECAY * 0.95
        
    elif motion_state in [MotionState.SLOW_DRAW, MotionState.NORMAL_DRAW, MotionState.FAST_DRAW]:
        # Integrate normally for drawing states
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
        # Coasting - minimal acceleration integration
        accel_corrected = accel_filtered - accel_bias
        vel += accel_corrected * dt * 0.3  # Reduced integration
        vel *= VELOCITY_DECAY * 0.98
        pos += vel * dt
        is_drawing = False

    # 9. Update fused position
    fused_pos[:] = pos[:] + uwb_center_offset

    # 10. Drawing logic (motion-state aware)
    vel_mag = np.linalg.norm(vel)
    
    # Only draw in active drawing states with sufficient velocity
    should_draw = (
        motion_state in [MotionState.SLOW_DRAW, MotionState.NORMAL_DRAW, MotionState.FAST_DRAW] and
        vel_mag > DRAW_VEL_THRESH and
        not features['is_jittering']
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

    # 11. Update visualization
    yz_dot.setData([fused_pos[0]], [fused_pos[2]])
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[1]])

    # 12. Update info display
    state_name = get_motion_state_name(motion_state)
    info_label.setText(
        f"Status: {'CALIBRATING...' if is_calibrating else 'READY'}\n"
        f"Motion State: {state_name}\n"
        f"Pos Y: {fused_pos[0]:.3f} | Z: {fused_pos[2]:.3f}\n"
        f"Vel: {vel_mag:.4f} m/s | Accel: {accel_mag:.4f} m/s²\n"
        f"Curvature: {features['curvature']:.3f} | Contact: {'YES' if motion_analyzer.is_contact_detected else 'NO'}\n"
        f"Vel Var: {features['vel_variance']:.6f} | Accel Var: {features['accel_variance']:.6f}"
    )
    
    if len(yz_strokes) > 100:
        yz_strokes.pop(0)
        old_item = stroke_items.pop(0)
        yz_win.removeItem(old_item)
    
    return True

# ============================================
# UPDATE LOOP
# ============================================
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
                d0 = float(toks[2])
                d1 = float(toks[3])
                d2 = float(toks[4])
                
                if d0 > 0 and d1 > 0 and d2 > 0:
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    uwb_pos[:] = raw_uwb_3d[:]
                    is_moving = motion_analyzer.current_state not in [MotionState.STATIC, MotionState.JITTER]
                    apply_uwb_fusion(is_moving_state=is_moving)
                
                for i in range(IMU_SAMPLES_PER_PACKET):
                    s = 6 + i * 8
                    qx, qy, qz, qw = map(float, toks[s : s+4])
                    ax, ay, az = map(float, toks[s+4 : s+7])
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
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), yz_win, calibrate_alignment)
    sys.exit(pg.exec())