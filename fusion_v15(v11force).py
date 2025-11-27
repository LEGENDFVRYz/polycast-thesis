import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from collections import deque
import sys
from dataclasses import dataclass

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
FORCE_ACTIVATION_THRESHOLD = 3.04 # LOWERED SLIGHTLY FOR BETTER CONTACT

# Filter & Bias Constants
BIAS_ALPHA = 0.01
VELOCITY_DECAY = 0.99

# Motion Thresholds
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
# ADAPTIVE FUSION STATE (KEPT EXACTLY AS YOU WROTE IT)
# -------------------------
@dataclass
class FusionMetrics:
    """Tracks fusion performance and error accumulation"""
    imu_drift_rate: float = 0.0
    uwb_noise_level: float = 0.02
    fusion_confidence: float = 0.5
    position_error: float = 0.0
    velocity_error: float = 0.0
    time_since_uwb: float = 0.0
    samples_since_uwb: int = 0

class AdaptiveUWBIMUFusion:
    """
    Intelligent fusion with self-tuning parameters based on real-time error metrics
    """
    def __init__(self):
        self.filtered_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        self.position_history = deque(maxlen=15)
        self.uwb_error_history = deque(maxlen=20)
        self.measurement_count = 0
        self.metrics = FusionMetrics()
        
        # Adaptive parameters
        self.base_correction = 0.15
        self.kalman_gain = 0.1
        self.velocity_fusion_weight = 0.2
        
    def compute_uwb_quality(self):
        """Assess UWB measurement quality from variance"""
        if len(self.position_history) < 4:
            return 0.5
        
        history_array = np.array(list(self.position_history))
        variance = np.var(history_array, axis=0).mean()
        
        # Quality: low variance = high quality
        quality = 1.0 / (1.0 + variance)
        quality = np.clip(quality, 0.3, 0.95)
        
        self.metrics.uwb_noise_level = variance
        return quality
    
    def estimate_imu_drift(self, imu_error):
        """Estimate how much IMU is drifting based on UWB correction magnitude"""
        if self.measurement_count < 3:
            return 0.05
        
        drift = np.linalg.norm(imu_error) / max(self.metrics.time_since_uwb, 0.01)
        self.metrics.imu_drift_rate = drift
        return drift
    
    def update_with_uwb(self, raw_uwb_pos, imu_vel, current_time, is_contact=False, motion_state=None):
        """Dynamically fuse UWB with IMU based on real-time conditions"""
        
        # 1. Time since last UWB update
        if self.last_uwb_time > 0:
            self.metrics.time_since_uwb = current_time - self.last_uwb_time
            self.metrics.samples_since_uwb += 1
        else:
            self.metrics.time_since_uwb = 0
        
        # 2. Predict using IMU
        dt = current_time - self.last_uwb_time if self.last_uwb_time > 0 else 0.0
        if self.measurement_count > 0 and dt > 0 and dt < 1.0:
            predicted_pos = self.filtered_pos + imu_vel * dt
        else:
            predicted_pos = raw_uwb_pos.copy()
        
        # 3. UWB quality assessment
        self.position_history.append(raw_uwb_pos.copy())
        uwb_quality = self.compute_uwb_quality()
        
        # 4. Innovation (UWB error from prediction)
        innovation = raw_uwb_pos - predicted_pos
        imu_error = np.linalg.norm(innovation)
        self.uwb_error_history.append(imu_error)
        self.metrics.position_error = imu_error
        
        # 5. ADAPTIVE KALMAN GAIN
        # Higher quality UWB + more time passed = stronger correction
        drift_factor = min(self.metrics.time_since_uwb / 0.1, 1.0)  # Max at 100ms
        self.kalman_gain = np.clip(
            uwb_quality * (0.15 + drift_factor * 0.25),
            0.08, 0.6
        )
        
        # 6. ADAPTIVE CORRECTION STRENGTH
        # During contact: trust UWB more if it's stable
        # Not in contact: lighter correction to avoid jumps
        if is_contact:
            self.base_correction = 0.15 + uwb_quality * 0.25
        else:
            self.base_correction = 0.05 + uwb_quality * 0.08
        
        # If UWB is too noisy, reduce correction
        if self.metrics.uwb_noise_level > 0.1:
            self.base_correction *= 0.6
        
        # 7. ADAPTIVE VELOCITY FUSION
        # More weight to UWB velocity if we haven't corrected in a while
        if self.metrics.time_since_uwb > 0.05:
            self.velocity_fusion_weight = 0.4
        elif self.metrics.time_since_uwb > 0.1:
            self.velocity_fusion_weight = 0.6
        else:
            self.velocity_fusion_weight = 0.2
        
        # 8. Multi-stage fusion
        # Stage 1: Predict with UWB-informed Kalman
        corrected_pos = predicted_pos + self.kalman_gain * innovation
        
        # Stage 2: Smooth with previous position (exponential moving average)
        smooth_factor = 0.7
        self.filtered_pos = smooth_factor * self.filtered_pos + (1 - smooth_factor) * corrected_pos
        
        # 9. Update confidence metric
        if imu_error < 0.05:
            self.metrics.fusion_confidence = min(self.metrics.fusion_confidence + 0.02, 0.95)
        else:
            self.metrics.fusion_confidence = max(self.metrics.fusion_confidence - 0.1, 0.3)
        
        # Store for next iteration
        self.last_uwb_pos = raw_uwb_pos.copy()
        self.last_uwb_time = current_time
        self.measurement_count += 1
        self.metrics.samples_since_uwb = 0
        
        return self.filtered_pos.copy(), self.kalman_gain
    
    def predict_imu_only(self, imu_vel, dt, motion_state):
        """Predict position using IMU when no UWB available"""
        self.metrics.time_since_uwb += dt
        self.metrics.samples_since_uwb += 1
        
        # Reduce IMU drift over time using velocity decay
        decay = 0.985 if motion_state in [0, 1, 6] else 0.99  # Heavier decay when static
        predicted = self.filtered_pos + imu_vel * dt * decay
        
        return predicted
    
    def get_metrics(self):
        return self.metrics
    
    def reset(self):
        self.filtered_pos[:] = 0
        self.last_uwb_pos[:] = 0
        self.last_uwb_time = 0
        self.position_history.clear()
        self.uwb_error_history.clear()
        self.measurement_count = 0
        self.metrics = FusionMetrics()

# -------------------------
# MOTION STATE MACHINE (KEPT EXACTLY AS YOU WROTE IT)
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
        self.state_transitions = 0
        
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
            self.state_transitions += 1
        
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
gravity_component = np.zeros(3)
prev_accel = None
prev_vel = None
last_timestamp_us = None
uwb_pos = np.zeros(3)
last_uwb_pos = np.zeros(3)
last_uwb_time = 0
has_first_fix = False
is_writing = False
motion_analyzer = MotionAnalyzer()
fusion_filter = AdaptiveUWBIMUFusion()
is_calibrating = True
calib_count = 0
CALIB_SAMPLES = 30
imu_heading_offset = 0.0

# Visualization
yz_strokes = []
current_stroke = []
stroke_items = []

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
# CHANGED: Label to X-Y (Top Down) instead of Y-Z
yz_win = pg.plot(title="Smart UWB-IMU Fusion Handwriting Tracker")
yz_win.setLabel('bottom', 'X (m)')
yz_win.setLabel('left', 'Y (m)') 
yz_win.setAspectLocked(True)
yz_win.showGrid(x=True, y=True)
yz_dot = yz_win.plot(symbol='o', symbolSize=8, symbolBrush='r')
uwb_ghost = yz_win.plot(symbol='+', symbolSize=10, symbolBrush='g')
yz_win.show()

info_win = QtWidgets.QGroupBox("Adaptive Fusion Metrics")
info_layout = QtWidgets.QVBoxLayout(info_win)
info_label = QtWidgets.QLabel("Status: Waiting...")
info_layout.addWidget(info_label)
info_win.show()

status_btn = QtWidgets.QPushButton("Wait for Input...", parent=yz_win)
status_btn.setGeometry(10, 10, 260, 30)
status_btn.setStyleSheet("background-color: #555555; color: white; font-weight: bold;")
status_btn.show()

def reset_position():
    global pos, vel, yz_strokes, current_stroke, stroke_items, last_timestamp_us
    global uwb_pos, motion_analyzer, is_calibrating, calib_count, gravity_component, has_first_fix
    global fusion_filter, is_writing
    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    gravity_component[:] = 0
    last_timestamp_us = None
    is_calibrating = True
    calib_count = 0
    is_writing = False
    motion_analyzer = MotionAnalyzer()
    fusion_filter.reset()
    for item in stroke_items:
        try: yz_win.removeItem(item)
        except: pass
    yz_strokes.clear()
    current_stroke.clear()
    stroke_items.clear()
    yz_dot.setData([0], [0])
    has_first_fix = False
    print("--- Reset ---")

def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, force, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us
    global yz_strokes, current_stroke, stroke_items, imu_heading_offset
    global is_calibrating, calib_count, motion_analyzer, gravity_component, is_writing
    
    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us
    if dt is None or dt <= 0 or dt > TRAP_DT_MAX: return False
    
    r = R.from_quat([qx, qy, qz, qw])
    
    # --- FIX 1: BETTER GRAVITY SUBTRACTION (QUATERNION BASED) ---
    # Convert Sensor Accel to World Frame
    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
    accel_world = r.apply(accel_local)
    
    # Subtract Gravity Vector (Assuming World Z is up)
    gravity_vec = np.array([0.0, 0.0, 9.81])
    
    # We remove the Low Pass Filter logic here because it creates lag. 
    # Using direct quaternion subtraction is instant and smoother.
    accel_filtered[:] = accel_world - gravity_vec
    
    # Simple deadband to stop noise when completely still
    if np.linalg.norm(accel_filtered) < 0.1:
        accel_filtered[:] = 0

    accel_mag = np.linalg.norm(accel_filtered)
    
    if is_calibrating:
        calib_count += 1
        if calib_count >= CALIB_SAMPLES:
            calibrate_alignment_with_quat(r)
            is_calibrating = False
            print("--- Calibration done ---")
        return True
        
    # --- FORCE SENSOR LOGIC ---
    is_touching = force > FORCE_ACTIVATION_THRESHOLD
    
    # State Transition: Hover -> Write
    if is_touching and not is_writing:
        is_writing = True
        status_btn.setText(f"Writing (Force: {force:.2f})")
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
        status_btn.setText(f"Hovering (Force: {force:.2f})")
        status_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
    
    if is_writing:
         status_btn.setText(f"Writing (Force: {force:.2f})")
    else:
         status_btn.setText(f"Hovering (Force: {force:.2f})")
    
    # Update Analyzer
    motion_state, features = motion_analyzer.update(vel, accel_filtered, accel_mag, pos, dt)
    
    # --- FIX 2: STATIC HANDLING ---
    # Don't force to 0.0 immediately, it breaks the line. Just decay fast.
    if motion_state == MotionState.STATIC:
        accel_bias[:] = (1 - BIAS_ALPHA) * accel_bias + BIAS_ALPHA * accel_filtered
        vel[:] *= 0.8 # Strong decay instead of hard stop
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
    
    # --- FIX 3: AXIS MAPPING (X and Y, not Z) ---
    # Only draw when writing
    if is_writing and len(stroke_items) > 0:
        current_stroke.append((tip_pos[0], tip_pos[2])) # CHANGED Z->Y
        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
    
    # Plot X and Y (Index 0 and 1)
    yz_dot.setData([tip_pos[0]], [tip_pos[2]]) # CHANGED Z->Y
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[2]]) # CHANGED Z->Y
    
    metrics = fusion_filter.get_metrics()
    state_name = get_motion_state_name(motion_state)
    info_label.setText(
        f"Motion: {state_name} | Force: {force:.2f}\n"
        f"Pos: ({tip_pos[0]:.3f}, {tip_pos[2]:.3f}) m\n"
        f"Vel: {np.linalg.norm(vel):.3f} m/s | Accel: {accel_mag:.3f} m/s²\n"
        f"---Fusion Metrics---\n"
        f"Kalman Gain: {fusion_filter.kalman_gain:.3f} | Correction: {fusion_filter.base_correction:.3f}\n"
        f"UWB Quality: {metrics.uwb_noise_level:.4f} | Confidence: {metrics.fusion_confidence:.2f}\n"
        f"IMU Drift Rate: {metrics.imu_drift_rate:.4f} m/s | Pos Error: {metrics.position_error:.4f} m\n"
        f"Time Since UWB: {metrics.time_since_uwb:.3f}s"
    )
    
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

def update():
    global _serial_buf, uwb_pos, last_uwb_time, last_uwb_pos, vel, pos, has_first_fix
    global fusion_filter, motion_analyzer, is_writing
    
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
            
            # Packet Length Check:
            if len(toks) < 96: continue
            
            try:
                # Process UWB
                d0 = float(toks[2]); d1 = float(toks[3]); d2 = float(toks[4])
                if d0 > 0 and d1 > 0 and d2 > 0:
                    current_time = time.time()
                    dt_uwb = current_time - last_uwb_time if last_uwb_time > 0 else 0
                    
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    
                    # Direct Mapping
                    raw_uwb_3d_transformed = np.array([
                        raw_uwb_3d[0],
                        raw_uwb_3d[2], # Y
                        raw_uwb_3d[1]  # Z
                    ])
                    
                    if not has_first_fix:
                        uwb_pos[:] = raw_uwb_3d_transformed[:]
                        pos[:] = raw_uwb_3d_transformed[:]
                        last_uwb_pos[:] = raw_uwb_3d_transformed[:]
                        vel[:] = 0
                        has_first_fix = True
                        last_uwb_time = current_time
                        print(f"Fix Acquired at: {raw_uwb_3d_transformed}")
                        continue
                    
                    dist_jump = np.linalg.norm(raw_uwb_3d_transformed - uwb_pos)
                    if dist_jump > 1.0 and np.linalg.norm(uwb_pos) > 0.1:
                        pass
                    else:
                        recent_imu_vel = motion_analyzer.vel_history[-1] if motion_analyzer.vel_history else np.zeros(3)
                        
                        filtered_pos, kgain = fusion_filter.update_with_uwb(
                            raw_uwb_3d_transformed,
                            recent_imu_vel,
                            current_time,
                            is_contact=is_writing,
                            motion_state=motion_analyzer.current_state
                        )
                        uwb_pos[:] = filtered_pos
                        
                        # Dynamic position correction
                        pos_error = uwb_pos - pos
                        correction = fusion_filter.base_correction
                        pos[:] += pos_error * correction
                        
                        # Adaptive velocity fusion
                        if dt_uwb > 0 and dt_uwb < 0.5:
                            uwb_velocity = (uwb_pos - last_uwb_pos) / dt_uwb
                            uwb_speed = np.linalg.norm(uwb_velocity)
                            if uwb_speed < MAX_HANDWRITING_VELOCITY:
                                vel[:] = (1 - fusion_filter.velocity_fusion_weight) * vel + (fusion_filter.velocity_fusion_weight * uwb_velocity)
                        
                        last_uwb_pos[:] = uwb_pos.copy()
                        last_uwb_time = current_time
                
                # Process IMU
                for i in range(IMU_SAMPLES_PER_PACKET):
                    s = 6 + i * 9
                    qx, qy, qz, qw = map(float, toks[s:s+4])
                    ax, ay, az = map(float, toks[s+4:s+7])
                    force = float(toks[s+7])
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
    sys.exit(pg.exec())