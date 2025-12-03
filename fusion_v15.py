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
SERIAL_PORT = 'COM2'
BAUD_RATE = 115200

# BBOX CONFIG (For Prototype Thread)
BBOX_CONFIG = {
    'b_min': 0.0,
    'b_max': 1.2,
}

ANCHORS = np.array([
    [1.41, 0.00, 0.00],
    [1.41, 1.35, 0.00],
    [0.00, 1.35, 0.00]
])
PEN_TIP_OFFSET = np.array([0.0, 0.0, 0.0])

# System Constants
TRAP_DT_MAX = 0.15
IMU_SAMPLES_PER_PACKET = 10
FORCE_ACTIVATION_THRESHOLD = 3.04 

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
# ADAPTIVE FUSION STATE (LOGIC V15)
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
        drift_factor = min(self.metrics.time_since_uwb / 0.1, 1.0)
        self.kalman_gain = np.clip(
            uwb_quality * (0.15 + drift_factor * 0.25),
            0.08, 0.6
        )
        
        # 6. ADAPTIVE CORRECTION STRENGTH
        if is_contact:
            self.base_correction = 0.15 + uwb_quality * 0.25
        else:
            self.base_correction = 0.05 + uwb_quality * 0.08
        
        # If UWB is too noisy, reduce correction
        if self.metrics.uwb_noise_level > 0.1:
            self.base_correction *= 0.6
        
        # 7. ADAPTIVE VELOCITY FUSION WEIGHT
        if self.metrics.time_since_uwb > 0.05:
            self.velocity_fusion_weight = 0.4
        elif self.metrics.time_since_uwb > 0.1:
            self.velocity_fusion_weight = 0.6
        else:
            self.velocity_fusion_weight = 0.2
        
        # 8. Multi-stage fusion
        corrected_pos = predicted_pos + self.kalman_gain * innovation
        smooth_factor = 0.7
        self.filtered_pos = smooth_factor * self.filtered_pos + (1 - smooth_factor) * corrected_pos
        
        # 9. Update confidence metric
        if imu_error < 0.05:
            self.metrics.fusion_confidence = min(self.metrics.fusion_confidence + 0.02, 0.95)
        else:
            self.metrics.fusion_confidence = max(self.metrics.fusion_confidence - 0.1, 0.3)
        
        self.last_uwb_pos = raw_uwb_pos.copy()
        self.last_uwb_time = current_time
        self.measurement_count += 1
        self.metrics.samples_since_uwb = 0
        
        return self.filtered_pos.copy(), self.kalman_gain
    
    def predict_imu_only(self, imu_vel, dt, motion_state):
        """Predict position using IMU when no UWB available"""
        self.metrics.time_since_uwb += dt
        self.metrics.samples_since_uwb += 1
        
        decay = 0.985 if motion_state in [0, 1, 6] else 0.99
        self.filtered_pos = self.filtered_pos + imu_vel * dt * decay
        return self.filtered_pos.copy()
    
    def reset(self):
        self.filtered_pos[:] = 0
        self.last_uwb_pos[:] = 0
        self.last_uwb_time = 0
        self.position_history.clear()
        self.uwb_error_history.clear()
        self.measurement_count = 0
        self.metrics = FusionMetrics()


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


# ==========================================
# STROKE TRACKER WRAPPER CLASS (V15 Logic)
# ==========================================
class StrokeTracker:
    """
    Wrapper class for _prototype_thread.py compatibility.
    Implements full Fusion V15 logic.
    """
    def __init__(self):
        # State Vectors
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.accel_filtered = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.gravity_component = np.zeros(3)
        
        self.prev_accel = None
        self.prev_vel = None
        self.last_timestamp_us = None
        
        # UWB State
        self.uwb_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        self.has_first_fix = False
        
        # System State
        self.is_writing = False
        self.is_calibrating = True
        self.calib_count = 0
        self.CALIB_SAMPLES = 30
        self.imu_heading_offset = 0.0
        
        # Modules
        self.motion_analyzer = MotionAnalyzer()
        self.fusion = AdaptiveUWBIMUFusion()
        
    def get_bbox(self):
        """Returns the render bounds config for _prototype_thread.py"""
        return BBOX_CONFIG.copy()
    
    def _micros_dt(self, last, now):
        if last is None: return None
        diff = now - last
        if diff < 0: diff += 4294967296
        return diff / 1e6

    def _trilaterate(self, distances):
        """Internal trilateration using SciPy"""
        def residuals(x, anchors, dists):
            return np.linalg.norm(anchors - x, axis=1) - dists
        x0 = np.mean(ANCHORS, axis=0)
        try:
            res = least_squares(residuals, x0, args=(ANCHORS, distances),
                                bounds=([-10, -10, -1], [10, 10, 3]))
            return res.x
        except:
            return np.zeros(3)
            
    def _calibrate_alignment_with_quat(self, r):
        yaw = r.as_euler('xyz', degrees=False)[2]
        self.imu_heading_offset = -yaw
        print(f"Heading Calibrated: offset {np.degrees(self.imu_heading_offset):.2f}°")

    def process_packet(self, line_bytes):
        """
        Process raw serial packet and return (x, y, is_drawing) tuple.
        Incorporates Fusion V15 logic.
        """
        try:
            raw = line_bytes.decode('utf-8', errors='ignore').strip()
            if not raw: return None
            
            parts = raw.split(',')
            vals = [float(x) for x in parts]
            if len(vals) < 96: return None
            
            # ========================================
            # 1. PROCESS UWB DATA
            # ========================================
            d0, d1, d2 = vals[2], vals[3], vals[4]
            
            if d0 > 0 and d1 > 0 and d2 > 0:
                current_time = time.time()
                dt_uwb = current_time - self.last_uwb_time if self.last_uwb_time > 0 else 0
                
                raw_uwb_3d = self._trilaterate([d0, d1, d2])
                
                # MAPPING: Source V15 Maps X->X, Z->Y, Y->Z for 2D top-down
                raw_uwb_transformed = np.array([
                    raw_uwb_3d[0],
                    raw_uwb_3d[2], # Y
                    raw_uwb_3d[1]  # Z
                ])
                
                if not self.has_first_fix:
                    self.pos[:] = raw_uwb_transformed[:]
                    self.uwb_pos[:] = raw_uwb_transformed[:]
                    self.last_uwb_pos[:] = raw_uwb_transformed[:]
                    self.vel[:] = 0
                    self.has_first_fix = True
                    self.last_uwb_time = current_time
                    return None # Wait for first clean frame
                
                # Outlier Rejection
                dist_jump = np.linalg.norm(raw_uwb_transformed - self.uwb_pos)
                if dist_jump <= 1.0 or np.linalg.norm(self.uwb_pos) <= 0.1:
                    
                    # FUSION UPDATE (V15 Logic)
                    fused_pos, kgain = self.fusion.update_with_uwb(
                        raw_uwb_transformed,
                        self.vel,
                        current_time,
                        is_contact=self.is_writing,
                        motion_state=self.motion_analyzer.current_state
                    )
                    self.uwb_pos[:] = fused_pos
                    
                    # Apply base correction to tracked position
                    pos_error = self.uwb_pos - self.pos
                    correction = self.fusion.base_correction
                    self.pos[:] += pos_error * correction
                    
                    # V15 ADAPTIVE VELOCITY FUSION
                    # Mixes UWB-derived velocity with IMU velocity based on fusion weight
                    if dt_uwb > 0 and dt_uwb < 0.5:
                        uwb_velocity = (self.uwb_pos - self.last_uwb_pos) / dt_uwb
                        uwb_speed = np.linalg.norm(uwb_velocity)
                        if uwb_speed < MAX_HANDWRITING_VELOCITY:
                            weight = self.fusion.velocity_fusion_weight
                            self.vel[:] = (1 - weight) * self.vel + (weight * uwb_velocity)
                            
                    self.last_uwb_pos[:] = self.uwb_pos.copy()
                    self.last_uwb_time = current_time
            
            # ========================================
            # 2. PROCESS IMU SAMPLES (BATCH OF 10)
            # ========================================
            last_valid_result = None
            offset = 6
            stride = 9
            
            for i in range(IMU_SAMPLES_PER_PACKET):
                idx = offset + (i * stride)
                if idx + 8 >= len(vals): break
                
                qx, qy, qz, qw = vals[idx:idx+4]
                ax_s, ay_s, az_s = vals[idx+4:idx+7]
                force = vals[idx+7]
                t_us = int(vals[idx+8])
                
                # Internal IMU Step
                result = self._process_imu_sample_internal(qx, qy, qz, qw, ax_s, ay_s, az_s, force, t_us)
                if result:
                    last_valid_result = result
            
            return last_valid_result

        except (ValueError, IndexError, UnicodeDecodeError):
            return None

    def _process_imu_sample_internal(self, qx, qy, qz, qw, ax_s, ay_s, az_s, force, t_us):
        """
        Internal loop step:
        1. Calculates DT
        2. Gravity Subtraction (Quaternion based)
        3. Motion Analysis
        4. Trapezoidal Integration
        5. Fusion Prediction
        """
        dt = self._micros_dt(self.last_timestamp_us, t_us)
        self.last_timestamp_us = t_us
        
        if dt is None or dt <= 0 or dt > TRAP_DT_MAX:
            return None
            
        r = R.from_quat([qx, qy, qz, qw])
        
        # 1. Rotation & Gravity Subtraction (V15 Logic - No HPF)
        accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
        accel_world = r.apply(accel_local)
        gravity_vec = np.array([0.0, 0.0, 9.81])
        
        self.accel_filtered[:] = accel_world - gravity_vec
        
        # Deadband
        if np.linalg.norm(self.accel_filtered) < 0.1:
            self.accel_filtered[:] = 0
            
        accel_mag = np.linalg.norm(self.accel_filtered)
        
        # 2. Calibration
        if self.is_calibrating:
            self.calib_count += 1
            if self.calib_count >= self.CALIB_SAMPLES:
                self._calibrate_alignment_with_quat(r)
                self.is_calibrating = False
            return None
        
        # 3. Force Logic
        is_touching = force > FORCE_ACTIVATION_THRESHOLD
        if is_touching and not self.is_writing:
            self.is_writing = True
            # V15 Snap Logic: If clean fix exists, maybe snap? 
            # V15 code actually removed the snap logic in the loop, so we stick to just state change
            pass
        elif not is_touching and self.is_writing:
            self.is_writing = False
            
        # 4. Motion Analysis
        motion_state, features = self.motion_analyzer.update(
            self.vel, self.accel_filtered, accel_mag, self.pos, dt
        )
        
        # 5. Integration with State Logic (V15 Specifics)
        BIAS_ALPHA = 0.01
        VELOCITY_DECAY_VAL = 0.99 # Updated from V15
        
        if motion_state == MotionState.STATIC:
            self.accel_bias[:] = (1 - BIAS_ALPHA) * self.accel_bias + BIAS_ALPHA * self.accel_filtered
            self.vel[:] *= 0.8 # V15: Rapid decay instead of hard stop
        elif motion_state in [MotionState.JITTER, MotionState.HOVER]:
            self.vel[:] *= VELOCITY_DECAY_VAL
        else:
            accel_corrected = self.accel_filtered - self.accel_bias
            if self.prev_accel is None:
                self.prev_accel = accel_corrected.copy()
                self.prev_vel = self.vel.copy()
            
            # Trapezoidal
            self.vel += 0.5 * (self.prev_accel + accel_corrected) * dt
            self.vel *= VELOCITY_DECAY_VAL
            
            self.pos += 0.5 * (self.prev_vel + self.vel) * dt
            
            self.prev_accel = accel_corrected.copy()
            self.prev_vel = self.vel.copy()
            
        # 6. Fusion Prediction Step (updates drift metrics)
        self.fusion.predict_imu_only(self.vel, dt, motion_state)
        
        # 7. Tip Calculation & Return
        rot_matrix = r.as_matrix()
        rotated_offset = rot_matrix @ PEN_TIP_OFFSET
        tip_pos = self.pos + rotated_offset
        
        final_x = tip_pos[0]
        final_y = tip_pos[2] # Use Z as Y (Top-down view per V15)
        
        return (final_x, final_y, self.is_writing)


# ==========================================
# STANDALONE TESTING (If run directly)
# ==========================================
if __name__ == "__main__":
    print("Starting StrokeTracker V15 Standalone...")
    
    # Initialize Tracker
    tracker = StrokeTracker()
    print(f"BBox: {tracker.get_bbox()}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        sys.exit(1)
        
    app = QtWidgets.QApplication([])
    yz_win = pg.plot(title="Smart UWB-IMU Fusion Handwriting Tracker (V15 Class)")
    yz_win.setLabel('bottom', 'X (m)')
    yz_win.setLabel('left', 'Y (m)')
    yz_win.setAspectLocked(True)
    yz_win.showGrid(x=True, y=True)
    yz_dot = yz_win.plot(symbol='o', symbolSize=8, symbolBrush='r')
    uwb_ghost = yz_win.plot(symbol='+', symbolSize=10, symbolBrush='g')
    yz_win.show()
    
    status_btn = QtWidgets.QPushButton("Wait for Input...", parent=yz_win)
    status_btn.setGeometry(10, 10, 260, 30)
    status_btn.setStyleSheet("background-color: #555555; color: white; font-weight: bold;")
    status_btn.show()
    
    info_win = QtWidgets.QGroupBox("Adaptive Fusion Metrics")
    info_layout = QtWidgets.QVBoxLayout(info_win)
    info_label = QtWidgets.QLabel("Status: Waiting...")
    info_layout.addWidget(info_label)
    info_win.show()
    
    _serial_buf = ""
    yz_strokes = []
    current_stroke = []
    stroke_items = []
    
    def update():
        global _serial_buf, current_stroke, stroke_items, yz_strokes
        
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
                line_bytes = raw.encode('utf-8')
                result = tracker.process_packet(line_bytes)
                
                if result:
                    x, y, is_drawing = result
                    
                    yz_dot.setData([x], [y])
                    uwb_ghost.setData([tracker.uwb_pos[0]], [tracker.uwb_pos[2]]) # X, Z (mapped to Y)
                    
                    if is_drawing:
                        if not current_stroke or len(stroke_items) == 0:
                            current_stroke = []
                            yz_strokes.append(current_stroke)
                            new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
                            yz_win.addItem(new_curve)
                            stroke_items.append(new_curve)
                        
                        current_stroke.append((x, y))
                        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
                        
                        status_btn.setText("Writing")
                        status_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
                    else:
                        status_btn.setText("Hovering")
                        status_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")

                    # Info Update
                    metrics = tracker.fusion.get_metrics()
                    info_label.setText(
                        f"Status: {'CALIBRATING...' if tracker.is_calibrating else 'READY'}\n"
                        f"Pos X: {x:.3f} | Y: {y:.3f}\n"
                        f"Fusion Confidence: {metrics.fusion_confidence:.2f}\n"
                        f"IMU Drift: {metrics.imu_drift_rate:.4f}"
                    )
        except Exception as ex:
            print(f"Update loop error: {ex}")
            
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(10)
    
    sys.exit(pg.exec())