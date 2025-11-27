import os
# ==============================================================================
#   FORCE PYTHON TO CONTROL SIGNALS (Prevents forrtl error 200)
#   This must be done BEFORE importing numpy/scipy
# ==============================================================================
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import serial
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import signal
from scipy.optimize import least_squares
from collections import deque

# ==============================================================================
#   "v5.1: Perpendicular/Hover Mode Optimization"
#   
#   ADJUSTMENTS FOR PERPENDICULAR HOLDING:
#   - Tip Offset set to [0,0,0] (Sensor X/Y = Tip X/Y in projection)
#   - Lever arm logic disabled (Constant offset assumption)
#   - Gravity filter optimized for constant Z-axis alignment
# ==============================================================================

plt.ion()

CONFIG = {
    'serial_port': 'COM3',
    'baud_rate': 115200,
    
    # --- PHYSICAL SETUP (CRITICAL) ---
    'anchors': np.array([
        [1.41, 0.00, 0.00],  # Anchor 0: BOTTOM RIGHT
        [1.41, 1.35, 0.00],  # Anchor 1: TOP RIGHT
        [0.00, 1.35, 0.00]   # Anchor 2: BOTTOM LEFT (Origin)
    ]),
    
    # [IMPORTANT]: For perpendicular holding, Back and Tip project to same X/Y.
    # We keep this 0.0 to track the sensor directly.
    'tip_offset': [0.0, 0.0, 0.0], 
    
    # --- UWB PROCESSING ---
    'uwb_window_size': 5,        
    'max_accel': 2.0,            
    
    # --- VIRTUAL WHITEBOARD BOUNDS ---
    'bounds_x_min': -0.5, 'bounds_x_max': 2.5,
    'bounds_y_min': -0.5, 'bounds_y_max': 2.5,
    
    # --- IMU STABILIZATION ---
    'calibration_samples': 30,       # Startup calibration window
    'gravity_cutoff_freq': 0.05,     # High-pass filter cutoff (Hz)
    'bias_learning_rate': 0.01,      # Adaptive bias update rate
    'velocity_decay': 0.99,          # Velocity damping during uncertain states
    
    # --- MOTION STATE THRESHOLDS ---
    'static_accel_thresh': 0.06,     # Below this = STATIC candidate
    'jitter_variance_thresh': 0.001, # Variance threshold for jitter detection
    'jitter_window_size': 5,         # Window for jitter analysis
    'motion_history_size': 20,       # Motion analyzer history length
    
    # --- TUNING ---
    'friction': 0.90,            
    'accel_noise_var': 0.05,     
    'stationary_thresh': 0.20,   
    'accel_deadband': 0.12,
    
    # --- KALMAN FILTER ---
    'process_pos_var': 0.01,
    'process_vel_var': 0.1,    
    'uwb_noise_std': 0.5,       
    
    # --- SAFETY ---
    'uwb_jump_thresh': 0.60      
}

# ==============================================================================
# MOTION STATE MACHINE
# ==============================================================================
class MotionState:
    CALIBRATING = -1
    STATIC = 0        # Not moving (learn bias)
    JITTER = 1        # High-freq noise / vibration
    DRAWING = 2       # Active writing motion

class MotionAnalyzer:
    def __init__(self, window_size=None):
        if window_size is None:
            window_size = CONFIG['motion_history_size']
        
        self.vel_history = deque(maxlen=window_size)
        self.accel_history = deque(maxlen=window_size)
        self.accel_mag_history = deque(maxlen=window_size)
        self.current_state = MotionState.CALIBRATING
        
    def update(self, vel, accel, accel_mag):
        self.vel_history.append(vel.copy())
        self.accel_history.append(accel.copy())
        self.accel_mag_history.append(accel_mag)
        
        vel_mag = np.linalg.norm(vel)
        features = self._compute_features(vel_mag, accel_mag)
        
        new_state = self._classify_motion(features)
        self.current_state = new_state
        
        return self.current_state, features
    
    def _compute_features(self, vel_mag, accel_mag):
        features = {
            'vel_mag': vel_mag,
            'accel_mag': accel_mag,
            'vel_variance': 0.0,
            'accel_variance': 0.0,
            'is_jittering': False
        }
        
        if len(self.vel_history) > 2:
            vel_mags = [np.linalg.norm(v) for v in self.vel_history]
            features['vel_variance'] = np.var(vel_mags)
        
        if len(self.accel_mag_history) > 2:
            features['accel_variance'] = np.var(list(self.accel_mag_history))
        
        features['is_jittering'] = self._detect_jitter()
        
        return features
    
    def _detect_jitter(self):
        if len(self.vel_history) < CONFIG['jitter_window_size']:
            return False
        
        recent_vels = list(self.vel_history)[-CONFIG['jitter_window_size']:]
        vel_mags = [np.linalg.norm(v) for v in recent_vels]
        
        if not vel_mags:
            return False
        
        variance = np.var(vel_mags)
        max_vel = max(vel_mags)
        
        return (variance > CONFIG['jitter_variance_thresh'] and max_vel < 0.03)
    
    def _classify_motion(self, features):
        vel_mag = features['vel_mag']
        accel_mag = features['accel_mag']
        is_jittering = features['is_jittering']
        
        if is_jittering:
            return MotionState.JITTER
        
        if vel_mag < 0.005 and accel_mag < CONFIG['static_accel_thresh']:
            return MotionState.STATIC
        
        return MotionState.DRAWING


# ==============================================================================
# UWB-IMU COMPLEMENTARY FILTER
# ==============================================================================
class UWBIMUFusionFilter:
    def __init__(self):
        self.filtered_pos = np.zeros(3)
        self.last_uwb_pos = np.zeros(3)
        self.last_uwb_time = 0
        self.uwb_position_variance = np.zeros(3)
        self.position_history = deque(maxlen=10)
        self.measurement_count = 0
        
    def update_with_uwb(self, raw_uwb_pos, imu_vel, current_time):
        # 1. Predict using IMU velocity
        if self.measurement_count > 0 and self.last_uwb_time > 0:
            dt = current_time - self.last_uwb_time
            if 0 < dt < 1.0:
                predicted_pos = self.filtered_pos + imu_vel * dt
            else:
                predicted_pos = self.filtered_pos.copy()
        else:
            predicted_pos = raw_uwb_pos.copy()
        
        # 2. Measure UWB noise
        self.position_history.append(raw_uwb_pos.copy())
        if len(self.position_history) >= 3:
            history_array = np.array(list(self.position_history))
            self.uwb_position_variance = np.var(history_array, axis=0)
        
        # 3. Adaptive Kalman gain
        base_variance = 0.05
        kalman_gain = np.clip(
            self.uwb_position_variance / (self.uwb_position_variance + base_variance),
            0.01, 0.3
        )
        
        # 4. Fuse
        innovation = raw_uwb_pos - predicted_pos
        self.filtered_pos = predicted_pos + kalman_gain * innovation
        
        # 5. Store
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


# ==============================================================================
# WEIGHTED NON-LINEAR LEAST SQUARES
# ==============================================================================
def trilaterate_wls(distances, anchors, variances, last_known_pos=None):
    distances = np.array(distances)
    variances = np.array(variances)
    valid_mask = distances > 0
    
    if np.sum(valid_mask) < 3:
        return None, None

    active_anchors = anchors[valid_mask]
    active_dists = distances[valid_mask]
    active_vars = variances[valid_mask]

    weights = 1.0 / (active_vars + 1e-6)
    weights = weights / np.mean(weights)

    def residuals(x, anchors, measured_dists, w):
        estimated_dists = np.linalg.norm(anchors - x, axis=1)
        error = estimated_dists - measured_dists
        return error * np.sqrt(w)

    if last_known_pos is not None:
        x0 = last_known_pos
    else:
        x0 = np.mean(active_anchors, axis=0)

    bounds_min = [-10.0, -10.0, -0.05]
    bounds_max = [ 10.0,  10.0,  0.05]

    try:
        res = least_squares(
            residuals, 
            x0, 
            bounds=(bounds_min, bounds_max),
            args=(active_anchors, active_dists, weights),
            loss='soft_l1', 
            f_scale=0.5 
        )
        return res.x, np.mean(np.abs(res.fun))
    except Exception:
        return None, None


# ==============================================================================
# ENHANCED KALMAN FILTER
# ==============================================================================
class KalmanFilter2D:
    def __init__(self, initial_pos):
        self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0], dtype=float)
        self.P = np.diag([0.1**2, 0.1**2, 1.0**2, 1.0**2])
        self.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.R_default = np.diag([CONFIG['uwb_noise_std']**2, CONFIG['uwb_noise_std']**2])

    def predict(self, ax, ay, dt):
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        half_dt2 = 0.5 * dt * dt
        B = np.array([
            [half_dt2, 0.0],
            [0.0, half_dt2],
            [dt,      0.0],
            [0.0,     dt ]
        ])
        u = np.array([ax, ay])
        self.x = F.dot(self.x) + B.dot(u)
        
        Q_accel = B.dot(np.eye(2) * CONFIG['accel_noise_var']).dot(B.T)
        Q_base = np.diag([
            CONFIG['process_pos_var'], CONFIG['process_pos_var'],
            CONFIG['process_vel_var'], CONFIG['process_vel_var']
        ])
        self.P = F.dot(self.P).dot(F.T) + (Q_accel + Q_base)

    def update(self, z_meas, R_override=None):
        R = R_override if R_override is not None else self.R_default
        y = z_meas - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        I = np.eye(4)
        self.P = (I - K.dot(self.H)).dot(self.P)

    def apply_zupt(self):
        self.x[2] = 0.0 
        self.x[3] = 0.0 
        self.P[2,2] = min(self.P[2,2], 1e-4)
        self.P[3,3] = min(self.P[3,3], 1e-4)
    
    def reset_state(self, new_pos):
        self.x[0] = new_pos[0]
        self.x[1] = new_pos[1]
        self.x[2] = 0.0
        self.x[3] = 0.0


# ==============================================================================
# ENHANCED TRACKING PIPELINE
# ==============================================================================
class StrokeTracker:
    def __init__(self):
        self.kf = None
        self.kf_comp = None
        
        self.prev_uwb_tip = None 
        self.prev_uwb_comp_tip = None
        self.consecutive_rejects = 0
        self.consecutive_rejects_comp = 0
        
        self.path_x = []         
        self.path_y = []
        self.path_comp_fused_x = []
        self.path_comp_fused_y = []
        self.is_drawing = False 
        
        self.uwb_hw_x = []       
        self.uwb_hw_y = []
        self.uwb_computed_x = [] 
        self.uwb_computed_y = []
        
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False
        self.dist_windows = [[], [], []]
        self.uwb_window_x = []
        self.uwb_window_y = []
        
        self.gravity_component = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.prev_ts_micros = 0
        self.accel_magnitudes = [] 
        self.ZUPT_WINDOW_SIZE = 10
        
        # We assume OFFSET is ZERO for Perpendicular/Hover mode
        self.offset_local = np.array(CONFIG['tip_offset'])
        
        self.is_calibrating = True
        self.calib_count = 0
        self.imu_heading_offset = 0.0
        
        self.motion_analyzer = MotionAnalyzer()
        self.uwb_imu_fusion_hw = UWBIMUFusionFilter()
        self.uwb_imu_fusion_comp = UWBIMUFusionFilter()

    def is_in_bounds(self, x, y):
        return (x >= CONFIG['bounds_x_min'] and 
                x <= CONFIG['bounds_x_max'] and 
                y >= CONFIG['bounds_y_min'] and 
                y <= CONFIG['bounds_y_max'])

    def calibrate_heading(self, q: Quaternion):
        # Even in perpendicular mode, we calibrate heading to zero out 
        # any mounting misalignment around the Z-axis.
        roll, pitch, yaw = q.yaw_pitch_roll 
        self.imu_heading_offset = -yaw
        print(f"[CALIB] Heading offset: {np.degrees(self.imu_heading_offset):.2f}°")

    def process_packet(self, line_bytes):
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return 
            
        if len(vals) < 10: return

        # --- UWB DATA ---
        hw_x, hw_y = vals[0], vals[1]
        dists = [vals[2], vals[3], vals[4]]
        
        idx_first = 6 
        qx_init, qy_init, qz_init, qw_init = vals[idx_first : idx_first+4]
        q_init = Quaternion(qw_init, qx_init, qy_init, qz_init)

        # --- VARIANCES ---
        current_variances = []
        for i in range(3):
            d = dists[i]
            if d > 0:
                self.dist_windows[i].append(d)
                if len(self.dist_windows[i]) > CONFIG['uwb_window_size']:
                    self.dist_windows[i].pop(0)
                if len(self.dist_windows[i]) > 2:
                    current_variances.append(max(np.var(self.dist_windows[i]), 1e-5))
                else:
                    current_variances.append(1.0)

        # --- PIPELINE A: WEIGHTED NLLS ---
        last_pos_guess = np.array([self.kf_comp.x[0], self.kf_comp.x[1], 0.0]) if self.kf_comp else None
        comp_pos_3d, _ = trilaterate_wls(dists, CONFIG['anchors'], current_variances, last_pos_guess)
        
        has_comp_data = False
        if comp_pos_3d is not None:
            raw_x, raw_y = comp_pos_3d[0], comp_pos_3d[1]
            if not self.sim_initialized:
                self.sim_filter_x, self.sim_filter_y = raw_x, raw_y
                self.sim_initialized = True
            else:
                self.sim_filter_x = (0.75 * self.sim_filter_x) + (0.25 * raw_x)
                self.sim_filter_y = (0.75 * self.sim_filter_y) + (0.25 * raw_y)

            self.uwb_computed_x.append(self.sim_filter_x)
            self.uwb_computed_y.append(self.sim_filter_y)
            has_comp_data = True
        
        # --- PIPELINE B: HARDWARE UWB ---
        self.uwb_hw_x.append(hw_x)
        self.uwb_hw_y.append(hw_y)
        self.uwb_window_x.append(hw_x)
        self.uwb_window_y.append(hw_y)
        if len(self.uwb_window_x) > CONFIG['uwb_window_size']:
            self.uwb_window_x.pop(0)
            self.uwb_window_y.pop(0)
            
        uwb_sensor_clean_x = np.median(self.uwb_window_x)
        uwb_sensor_clean_y = np.median(self.uwb_window_y)
        
        # --- TIP TRANSFORMATION (NO LEVER ARM LOGIC) ---
        # For perpendicular holding, Offset is effectively zero in 2D projection
        offset_world = q_init.rotate(self.offset_local)
        
        uwb_tip_pos_hw = np.array([
            uwb_sensor_clean_x + offset_world[0],
            uwb_sensor_clean_y + offset_world[1]
        ])
        
        uwb_tip_pos_comp = np.array([
            self.sim_filter_x + offset_world[0],
            self.sim_filter_y + offset_world[1]
        ])

        # --- UWB SMOOTHING ---
        alpha = 0.3
        if self.prev_uwb_tip is None:
            self.prev_uwb_tip = uwb_tip_pos_hw
            self.prev_uwb_comp_tip = uwb_tip_pos_comp
        else:
            self.prev_uwb_tip = alpha * uwb_tip_pos_hw + (1.0 - alpha) * self.prev_uwb_tip
            self.prev_uwb_comp_tip = alpha * uwb_tip_pos_comp + (1.0 - alpha) * self.prev_uwb_comp_tip

        # --- INITIALIZATION ---
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_tip_pos_hw) 
            self.kf_comp = KalmanFilter2D(uwb_tip_pos_comp) 
            self.prev_ts_micros = vals[13] 
            return

        # --- IMU BATCH PREDICTION ---
        offset = 6
        stride = 8
        has_predicted = False

        for i in range(10):
            idx = offset + (i * stride)
            if idx + 7 >= len(vals): break

            qx, qy, qz, qw = vals[idx : idx+4]
            ax, ay, az     = vals[idx+4 : idx+7]
            ts_micros      = int(vals[idx+7])

            dt = (ts_micros - self.prev_ts_micros) / 1_000_000.0
            self.prev_ts_micros = ts_micros
            if dt <= 0 or dt > 0.2: dt = 0.01

            q = Quaternion(qw, qx, qy, qz)
            accel_local = np.array([ax, ay, az])
            accel_world = q.rotate(accel_local)
            
            # --- STARTUP CALIBRATION ---
            if self.is_calibrating:
                self.calib_count += 1
                if self.calib_count >= CONFIG['calibration_samples']:
                    self.calibrate_heading(q)
                    self.is_calibrating = False
                    print("[READY] Calibration complete. Perpendicular mode active.")
                continue 
            
            # --- DYNAMIC GRAVITY FILTER ---
            # Ideal for Perpendicular mode where gravity vector is constant in X/Y plane
            cutoff_freq = CONFIG['gravity_cutoff_freq']
            RC = 1.0 / (cutoff_freq * 2 * np.pi)
            dynamic_alpha = RC / (RC + dt)
            
            self.gravity_component = (dynamic_alpha * self.gravity_component + 
                                     (1 - dynamic_alpha) * accel_world)
            accel_high_passed = accel_world - self.gravity_component
            
            accel_filtered_2d = np.array([accel_high_passed[0], accel_high_passed[1]])
            accel_mag = np.linalg.norm(accel_filtered_2d)
            
            # --- MOTION STATE ANALYSIS ---
            current_vel_2d = np.array([self.kf.x[2], self.kf.x[3]])
            motion_state, _ = self.motion_analyzer.update(current_vel_2d, accel_filtered_2d, accel_mag)
            
            # --- ADAPTIVE BIAS ---
            if motion_state == MotionState.STATIC:
                self.accel_bias[:2] = ((1 - CONFIG['bias_learning_rate']) * self.accel_bias[:2] + 
                                      CONFIG['bias_learning_rate'] * accel_filtered_2d)
                self.kf.x[2:4] = 0.0
                self.kf_comp.x[2:4] = 0.0
                
            elif motion_state == MotionState.JITTER:
                self.kf.x[2:4] *= CONFIG['velocity_decay']
                self.kf_comp.x[2:4] *= CONFIG['velocity_decay']
            
            # --- PREDICT ---
            accel_corrected = accel_filtered_2d - self.accel_bias[:2]
            
            limit = CONFIG['max_accel']
            accel_corrected = np.clip(accel_corrected, -limit, limit)
            
            if np.linalg.norm(accel_corrected) < CONFIG['accel_deadband']:
                accel_corrected[:] = 0.0

            self.kf.predict(accel_corrected[0], accel_corrected[1], dt)
            self.kf_comp.predict(accel_corrected[0], accel_corrected[1], dt)

            # Friction
            self.kf.x[2:4] *= CONFIG['friction']
            self.kf_comp.x[2:4] *= CONFIG['friction']

            # ZUPT
            self.accel_magnitudes.append(np.linalg.norm(accel_corrected))
            if len(self.accel_magnitudes) > self.ZUPT_WINDOW_SIZE:
                self.accel_magnitudes.pop(0)

            if len(self.accel_magnitudes) == self.ZUPT_WINDOW_SIZE:
                if np.mean(self.accel_magnitudes) < CONFIG['stationary_thresh'] and np.var(self.accel_magnitudes) < 0.05:
                    self.kf.apply_zupt()
                    self.kf_comp.apply_zupt()
            
            has_predicted = True
            
            # --- PATH STORAGE ---
            curr_x, curr_y = self.kf.x[0], self.kf.x[1]
            if self.is_in_bounds(curr_x, curr_y):
                self.path_x.append(curr_x)
                self.path_y.append(curr_y)
            else:
                self.path_x.append(np.nan)
                self.path_y.append(np.nan)
            
            comp_curr_x, comp_curr_y = self.kf_comp.x[0], self.kf_comp.x[1]
            if self.is_in_bounds(comp_curr_x, comp_curr_y):
                self.path_comp_fused_x.append(comp_curr_x)
                self.path_comp_fused_y.append(comp_curr_y)
            else:
                self.path_comp_fused_x.append(np.nan)
                self.path_comp_fused_y.append(np.nan)

        # --- UWB UPDATE STEP ---
        if has_predicted and not self.is_calibrating:
            # Hardware Filter
            pred_pos = self.kf.x[:2]
            innovation = np.linalg.norm(self.prev_uwb_tip - pred_pos)
            
            if innovation > CONFIG['uwb_jump_thresh']:
                self.consecutive_rejects += 1
                if self.consecutive_rejects >= 5:
                    self.kf.reset_state(self.prev_uwb_tip)
                    self.consecutive_rejects = 0
            else:
                self.kf.update(self.prev_uwb_tip)
                self.consecutive_rejects = 0
            
            # Computed Filter
            if has_comp_data: 
                pred_pos_comp = self.kf_comp.x[:2]
                innovation_comp = np.linalg.norm(self.prev_uwb_comp_tip - pred_pos_comp)

                if innovation_comp > CONFIG['uwb_jump_thresh']:
                    self.consecutive_rejects_comp += 1
                    if self.consecutive_rejects_comp >= 5:
                        self.kf_comp.reset_state(self.prev_uwb_comp_tip)
                        self.consecutive_rejects_comp = 0
                else:
                    self.kf_comp.update(self.prev_uwb_comp_tip)
                    self.consecutive_rejects_comp = 0


# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
is_running = True

def signal_handler(sig, frame):
    global is_running
    print("\nStopping capture...")
    is_running = False

def main():
    signal.signal(signal.SIGINT, signal_handler)
    tracker = StrokeTracker()
    
    print(f"Opening {CONFIG['serial_port']}...")
    print("[CALIB] Collecting initial samples for calibration...")

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    line_raw, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.4,
                        linewidth=1.0, color="#E6AC3F", label='Hardware UWB')
    line_comp, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.6,
                          linewidth=1.0, color="#E63F3F", label='NLLS Trilateration')
    line_fused, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#3F92E6",
                           label='Fused (Hardware)')
    line_fused_comp, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#2A2A94",
                           label='Fused (Computed)')

    ax.set_title("v5.1: Perpendicular/Hover Mode")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.show()

    try:
        ser = serial.Serial(CONFIG['serial_port'], CONFIG['baud_rate'], timeout=1)
        ser.flushInput()
        
        while is_running:
            line = ser.readline()
            if line:
                tracker.process_packet(line)

                if tracker.uwb_hw_x:
                    line_raw.set_xdata(tracker.uwb_hw_x)
                    line_raw.set_ydata(tracker.uwb_hw_y)
                
                if tracker.uwb_computed_x:
                    line_comp.set_xdata(tracker.uwb_computed_x)
                    line_comp.set_ydata(tracker.uwb_computed_y)
                
                if tracker.path_x:
                    line_fused.set_xdata(tracker.path_x)
                    line_fused.set_ydata(tracker.path_y)
                    
                if tracker.path_comp_fused_x:
                    line_fused_comp.set_xdata(tracker.path_comp_fused_x)
                    line_fused_comp.set_ydata(tracker.path_comp_fused_y)

                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("Capture finished.")
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()