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
    [1.22, 0.00, 0.00],
    [1.22, 1.22, 0.00],
    [0.00, 1.22, 0.00]
])

PEN_TIP_OFFSET = np.array([0.0, 0.0, 0.0])

# System Constants
TRAP_DT_MAX = 0.15
IMU_SAMPLES_PER_PACKET = 5
FORCE_ACTIVATION_THRESHOLD = 8

# Filter & Bias Constants
BIAS_ALPHA = 0.01
VELOCITY_DECAY = 0.98

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
# BOUNDING BOX CONFIG
# -------------------------
BBOX_CONFIG = {
    'b_min': 0.0,
    'b_max': 1.2,
}

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

# [RESTORED ORIGINAL CLASS]
class UWBIMUFusionFilter:
    def __init__(self):
        self.filtered_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        self.uwb_position_variance = np.zeros(3)
        self.position_history = deque(maxlen=10)
        self.measurement_count = 0
        
    def update_with_uwb(self, raw_uwb_pos, imu_vel, current_time):
        if self.measurement_count > 0 and self.last_uwb_time > 0:
            dt = current_time - self.last_uwb_time
            if dt > 0 and dt < 1.0:
                predicted_pos = self.filtered_pos + imu_vel * dt
            else:
                predicted_pos = self.filtered_pos.copy()
        else:
            predicted_pos = raw_uwb_pos.copy()
        
        self.position_history.append(raw_uwb_pos.copy())
        if len(self.position_history) >= 3:
            history_array = np.array(list(self.position_history))
            self.uwb_position_variance = np.var(history_array, axis=0)
        
        base_variance = 0.02
        kalman_gain = np.clip(
            self.uwb_position_variance / (self.uwb_position_variance + base_variance),
            0.001, 0.02
        )
        
        innovation = raw_uwb_pos - predicted_pos
        self.filtered_pos = predicted_pos + kalman_gain * innovation
        
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

class StrokeTracker:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.accel_filtered = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.gravity_component = np.zeros(3)
        
        self.prev_accel = None
        self.prev_vel = None
        self.last_timestamp_us = None
        
        self.raw_uwb_pos = np.zeros(3)
        self.uwb_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        
        self.has_first_fix = False
        self.is_writing = False
        
        self.motion_analyzer = MotionAnalyzer()
        self.uwb_imu_fusion = UWBIMUFusionFilter()
        
        # Calibration state
        self.is_calibrating = True
        self.calib_count = 0
        self.CALIB_SAMPLES = 30
        self.imu_heading_offset = 0.0
        
    def get_bbox(self):
        return BBOX_CONFIG.copy()
    
    def process_packet(self, line_bytes):
        try:
            raw = line_bytes.decode('utf-8', errors='ignore').strip()
            if not raw:
                return None
                
            toks = raw.split(',')
            if len(toks) < 48:
                return None
            
            # 1. PROCESS UWB (With Original Filter Restored)
            d0 = float(toks[0])
            d1 = float(toks[1])
            d2 = float(toks[2])
            
            raw_uwb_3d = self._trilaterate([d0, d1, d2], ANCHORS)
            raw_uwb_3d_transformed = np.array([
                raw_uwb_3d[0],
                raw_uwb_3d[2],
                raw_uwb_3d[1]
            ])
            self.raw_uwb_pos[:] = raw_uwb_3d_transformed[:]
            
            if d0 > 0 and d1 > 0 and d2 > 0:
                current_time = time.time()
                
                if not self.has_first_fix:
                    self.uwb_pos[:] = raw_uwb_3d_transformed[:]
                    self.pos[:] = raw_uwb_3d_transformed[:]
                    self.vel[:] = 0
                    self.has_first_fix = True
                    return None 
                
                dist_jump = np.linalg.norm(raw_uwb_3d_transformed - self.uwb_pos)
                if dist_jump <= 1.0 or np.linalg.norm(self.uwb_pos) <= 0.1:
                    
                    if len(self.motion_analyzer.vel_history) > 0:
                        recent_imu_vel = self.motion_analyzer.vel_history[-1]
                    else:
                        recent_imu_vel = np.zeros(3)
                    
                    # USE RESTORED FILTER LOGIC
                    filtered_pos, kgain = self.uwb_imu_fusion.update_with_uwb(
                        raw_uwb_3d_transformed,
                        recent_imu_vel,
                        current_time
                    )
                    self.uwb_pos[:] = filtered_pos
            
            # 2. PROCESS IMU SAMPLES (With Smart Snap)
            last_valid_result = None
            
            for i in range(IMU_SAMPLES_PER_PACKET):
                s = 3 + i * 9
                qx, qy, qz, qw = map(float, toks[s:s+4])
                ax, ay, az = map(float, toks[s+4:s+7])
                force = float(toks[s+7])
                t_us = int(toks[s+8])
                
                # Pass the TARGET (uwb_pos) to the processor
                result = self._process_imu_sample(qx, qy, qz, qw, ax, ay, az, force, t_us, self.uwb_pos)
                if result:
                    last_valid_result = result
            
            return last_valid_result
            
        except (ValueError, IndexError, UnicodeDecodeError) as e:
            return None
    
    def _trilaterate(self, distances, anchors):
        def residuals(x, anchors, dists):
            return np.linalg.norm(anchors - x, axis=1) - dists
        x0 = np.mean(anchors, axis=0)
        res = least_squares(residuals, x0, args=(anchors, distances),
                            bounds=([-10, -10, -1], [10, 10, 3]))
        return res.x
    
    def _micros_dt(self, last, now):
        if last is None:
            return None
        diff = now - last
        if diff < 0:
            diff += 4294967296 
        return diff / 1e6
    
    def _calibrate_alignment_with_quat(self, r):
        yaw = r.as_euler('xyz', degrees=False)[2]
        self.imu_heading_offset = -yaw
    
    def _process_imu_sample(self, qx, qy, qz, qw, ax_s, ay_s, az_s, force, t_us, target_pos):
        accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
        dt = self._micros_dt(self.last_timestamp_us, t_us)
        self.last_timestamp_us = t_us
        
        if dt is None or dt <= 0 or dt > TRAP_DT_MAX:
            return None
        
        # 1. Rotation
        r = R.from_quat([qx, qy, qz, qw])
        accel_world = r.apply(accel_local)
        accel_world[1] *= -1 
        
        # 2. Calibration
        if self.is_calibrating:
            self.calib_count += 1
            if self.calib_count >= self.CALIB_SAMPLES:
                self._calibrate_alignment_with_quat(r)
                self.is_calibrating = False
            return None
        
        # 3. Force Sensor
        is_touching = force > FORCE_ACTIVATION_THRESHOLD
        
        if is_touching and not self.is_writing:
            self.is_writing = True
            if self.has_first_fix:
                self.vel[:] = 0 # Kill momentum on touchdown
        
        elif not is_touching and self.is_writing:
            self.is_writing = False
        
        # 4. Filter accel
        cutoff_freq = 0.05
        RC = 1.0 / (cutoff_freq * 2 * np.pi)
        dynamic_alpha = RC / (RC + dt)
        
        self.gravity_component[:] = dynamic_alpha * self.gravity_component + (1 - dynamic_alpha) * accel_world
        accel_high_passed = accel_world - self.gravity_component
        self.accel_filtered[:] = accel_high_passed
        accel_mag = np.linalg.norm(self.accel_filtered)
        
        # 5. Motion Analysis
        motion_state, features = self.motion_analyzer.update(
            self.vel, self.accel_filtered, accel_mag, self.pos, dt
        )
        
        # 6. Integration + SMART SNAP LOGIC
        # -----------------------------------------------------------
        # Calculate Gap
        pos_error = target_pos - self.pos
        dist_error = np.linalg.norm(pos_error)

        # ADAPTIVE GAIN (K)
        # >15cm: High Gain (0.3) -> Snaps to target quickly (Fast Transit)
        # <15cm: Low Gain (0.05) -> Gentle guide (Writing/Detail)
        if dist_error > 0.15: 
            K = 0.30 
        else:
            K = 0.05

        if motion_state == MotionState.STATIC:
            self.accel_bias[:] = (1 - BIAS_ALPHA) * self.accel_bias + BIAS_ALPHA * self.accel_filtered
            self.vel[:] = 0.0
            self.pos += pos_error * K
        elif motion_state in [MotionState.JITTER, MotionState.HOVER]:
            self.vel[:] *= VELOCITY_DECAY
            self.pos += (self.vel * dt) + (pos_error * K)
        else:
            accel_corrected = self.accel_filtered - self.accel_bias
            if self.prev_accel is None:
                self.prev_accel = accel_corrected.copy()
                self.prev_vel = self.vel.copy()
            
            self.vel += 0.5 * (self.prev_accel + accel_corrected) * dt
            self.vel *= VELOCITY_DECAY
            
            # Move based on IMU + Smart Nudge towards UWB
            self.pos += 0.5 * (self.prev_vel + self.vel) * dt
            self.pos += pos_error * K 
            
            self.prev_accel = accel_corrected.copy()
            self.prev_vel = self.vel.copy()
        # -----------------------------------------------------------
        
        rot_matrix = r.as_matrix()
        rotated_offset = rot_matrix @ PEN_TIP_OFFSET
        tip_pos = self.pos + rotated_offset
        
        return (tip_pos[0], tip_pos[2], self.is_writing)

if __name__ == "__main__":
    print("Initializing PolyCast (Full Logic + Smart Snap)...")
    tracker = StrokeTracker()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        sys.exit(1)
    
    _serial_buf = ""
    
    app = QtWidgets.QApplication([])
    yz_win = pg.plot(title="PolyCast - Smart Snap")
    yz_win.setLabel('bottom', 'X (m)')
    yz_win.setLabel('left', 'Y (m)')
    yz_win.setAspectLocked(True)
    yz_win.showGrid(x=True, y=True)
    
    yz_dot = yz_win.plot(symbol='o', symbolSize=10, symbolBrush='r', name="Fused")
    uwb_ghost = yz_win.plot(pen=pg.mkPen(color=(0, 255, 0, 120), width=2), name="Target")
    
    yz_win.show()
    
    info_win = QtWidgets.QGroupBox("Status")
    info_layout = QtWidgets.QVBoxLayout(info_win)
    info_label = QtWidgets.QLabel("Waiting...")
    info_layout.addWidget(info_label)
    info_win.show()
    
    status_btn = QtWidgets.QPushButton("Status", parent=yz_win)
    status_btn.setGeometry(10, 10, 100, 30)
    status_btn.show()
    
    yz_strokes = []
    current_stroke = []
    stroke_items = []
    uwb_filt_history = deque(maxlen=300)
    
    def update():
        global _serial_buf, current_stroke, stroke_items
        
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
                    
                    if np.linalg.norm(tracker.uwb_pos) > 0.01:
                        uwb_filt_history.append((tracker.uwb_pos[0], tracker.uwb_pos[2]))
                        fx, fy = zip(*uwb_filt_history)
                        uwb_ghost.setData(fx, fy)

                    if is_drawing:
                        if not current_stroke:
                            current_stroke = []
                            yz_strokes.append(current_stroke)
                            new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=3))
                            yz_win.addItem(new_curve)
                            stroke_items.append(new_curve)
                        
                        current_stroke.append((x, y))
                        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
                        status_btn.setStyleSheet("background-color: green;")
                    else:
                        current_stroke = []
                        status_btn.setStyleSheet("background-color: red;")
                    
                    info_label.setText(f"POS: ({x:.2f}, {y:.2f})")
        
        except Exception as ex:
            pass
    
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(10)
    
    sys.exit(pg.exec())