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

# ==============================================================================
#   "v4.3: Weighted Non-Linear Least Squares (WNLLS)"
# ==============================================================================

plt.ion()

CONFIG = {
    'serial_port': 'COM2',
    'baud_rate': 115200,
    
    # --- PHYSICAL SETUP (CRITICAL) ---
    # 2 Anchors at Bottom (Left/Right), 1 Anchor at Right (Top).
    'anchors': np.array([
        [1.75, 0.00, 0.00],  # Anchor 0: BOTTOM RIGHT
        [1.75, 1.61, 0.00],  # Anchor 1: TOP RIGHT
        [0.00, 0.00, 0.00]   # Anchor 2: BOTTOM LEFT (Origin)
    ]),
    # Note: This setup is "Right-Heavy". Tracking on the far Left side 
    # relies heavily on Anchor 2 (Bottom Left) and Anchor 1 (Top Right).
    
    # 2. LEVER ARM (Sensor -> Tip)
    'tip_offset': [0.0, 0.0, 0.0], 
    
    # --- LAYERS 1 & 2 (Defense) ---
    'uwb_window_size': 5,        
    'max_accel': 2.0,            
    
    # --- VIRTUAL WHITEBOARD BOUNDS ---
    'bounds_x_min': -0.5, 'bounds_x_max': 2.5,
    'bounds_y_min': -0.5, 'bounds_y_max': 2.5,
    
    # --- FINE TUNING ---
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


# ==========================================
# WEIGHTED NON-LINEAR LEAST SQUARES (WNLLS)
# ==========================================
def trilaterate_wls(distances, anchors, variances, last_known_pos=None):
    """
    Solves min sum( weight_i * (|x - Ai| - di)^2 )
    Where weight_i = 1 / variance_i
    """
    
    distances = np.array(distances)
    variances = np.array(variances)
    valid_mask = distances > 0
    
    if np.sum(valid_mask) < 3:
        return None, None

    active_anchors = anchors[valid_mask]
    active_dists = distances[valid_mask]
    active_vars = variances[valid_mask]

    # Calculate Weights: w = 1 / sigma^2
    # Add small epsilon (1e-6) to prevent division by zero if variance is 0
    weights = 1.0 / (active_vars + 1e-6)
    
    # Normalize weights so they don't explode the cost function
    weights = weights / np.mean(weights)

    # 2. Define Residuals with Weights
    def residuals(x, anchors, measured_dists, w):
        estimated_dists = np.linalg.norm(anchors - x, axis=1)
        error = estimated_dists - measured_dists
        # Multiply by sqrt(w) because least_squares minimizes sum(res^2)
        # (err * sqrt(w))^2 = err^2 * w
        return error * np.sqrt(w)

    # 3. Smart Initial Guess
    if last_known_pos is not None:
        x0 = last_known_pos
    else:
        x0 = np.mean(active_anchors, axis=0)

    # 4. Strong Bounds (Wall Mode)
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
        
        avg_error = np.mean(np.abs(res.fun))
        return res.x, avg_error

    except Exception as e:
        return None, None

# ==========================================
# CLASS: KALMAN FILTER
# ==========================================
class KalmanFilter2D:
    def __init__(self, initial_pos):
        # State: [x, y, vx, vy] of the TIP
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
            [dt,       0.0],
            [0.0,      dt ]
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

# ==========================================
# CLASS: TRACKING PIPELINE
# ==========================================

class StrokeTracker:
    def __init__(self):
        self.kf = None
        self.prev_uwb_tip = None 
        self.path_x = []         
        self.path_y = []
        self.consecutive_rejects = 0

        self.kf_comp = None
        self.prev_uwb_comp_tip = None
        self.path_comp_fused_x = []
        self.path_comp_fused_y = []
        self.consecutive_rejects_comp = 0

        self.prev_ts_micros = 0
        self.is_drawing = False 
        
        self.uwb_hw_x = []       
        self.uwb_hw_y = []
        self.uwb_computed_x = [] 
        self.uwb_computed_y = []
        
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False

        # --- VARIANCE BUFFERS (For WLS) ---
        # Stores last N raw distances for Anchor 0, 1, 2
        self.dist_windows = [[], [], []] 
        
        # Median Filter Window (For Hardware pipeline)
        self.uwb_window_x = []
        self.uwb_window_y = []
        
        self.accel_magnitudes = [] 
        self.ZUPT_WINDOW_SIZE = 10
        
        self.offset_local = np.array(CONFIG['tip_offset'])

    def is_in_bounds(self, x, y):
        return (x >= CONFIG['bounds_x_min'] and 
                x <= CONFIG['bounds_x_max'] and 
                y >= CONFIG['bounds_y_min'] and 
                y <= CONFIG['bounds_y_max'])

    def process_packet(self, line_bytes):
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return 
            
        if len(vals) < 10: return

        # 1. EXTRACT DATA
        hw_x, hw_y = vals[0], vals[1]
        dists = [vals[2], vals[3], vals[4]] # d0, d1, d2
        
        idx_first = 6 
        qx_init, qy_init, qz_init, qw_init = vals[idx_first : idx_first+4]
        q_init = Quaternion(qw_init, qx_init, qy_init, qz_init)

        # ====================================================
        #    CALCULATE VARIANCES FOR WEIGHTING
        # ====================================================
        current_variances = []
        
        for i in range(3):
            d = dists[i]
            if d > 0:
                self.dist_windows[i].append(d)
                
            # Maintain window size
            if len(self.dist_windows[i]) > CONFIG['uwb_window_size']:
                self.dist_windows[i].pop(0)
            
            # Calculate Variance (std^2)
            if len(self.dist_windows[i]) > 2:
                var = np.var(self.dist_windows[i])
                # If variance is dangerously low (perfect signal?), clamp it
                current_variances.append(max(var, 1e-5))
            else:
                current_variances.append(1.0) # High uncertainty if no history

        # ====================================================
        #    PIPELINE A: WEIGHTED NLLS (RED DATA)
        # ====================================================
        last_pos_guess = None
        if self.kf_comp is not None:
            last_pos_guess = np.array([self.kf_comp.x[0], self.kf_comp.x[1], 0.0])

        # Pass Variances to the Solver
        comp_pos_3d, error_metric = trilaterate_wls(
            dists, 
            CONFIG['anchors'], 
            current_variances,
            last_known_pos=last_pos_guess
        )
        
        has_comp_data = False
        
        if comp_pos_3d is not None:
            raw_x = comp_pos_3d[0]
            raw_y = comp_pos_3d[1]

            # Imitation Filter
            if not self.sim_initialized:
                self.sim_filter_x = raw_x
                self.sim_filter_y = raw_y
                self.sim_initialized = True
            else:
                self.sim_filter_x = (0.75 * self.sim_filter_x) + (0.25 * raw_x)
                self.sim_filter_y = (0.75 * self.sim_filter_y) + (0.25 * raw_y)

            self.uwb_computed_x.append(self.sim_filter_x)
            self.uwb_computed_y.append(self.sim_filter_y)
            has_comp_data = True
        
        # ====================================================
        #    PIPELINE B: HARDWARE UWB (ORANGE DATA)
        # ====================================================
        self.uwb_hw_x.append(hw_x)
        self.uwb_hw_y.append(hw_y)

        self.uwb_window_x.append(hw_x)
        self.uwb_window_y.append(hw_y)
        if len(self.uwb_window_x) > CONFIG['uwb_window_size']:
            self.uwb_window_x.pop(0)
            self.uwb_window_y.pop(0)
            
        uwb_sensor_clean_x = np.median(self.uwb_window_x)
        uwb_sensor_clean_y = np.median(self.uwb_window_y)
        
        # ====================================================
        #    TRANSFORM BOTH TO TIP SPACE
        # ====================================================
        offset_world = q_init.rotate(self.offset_local)

        # 1. Hardware Tip
        tip_hw_x = uwb_sensor_clean_x + offset_world[0]
        tip_hw_y = uwb_sensor_clean_y + offset_world[1]
        uwb_tip_pos_hw = np.array([tip_hw_x, tip_hw_y])

        # 2. Computed Tip
        tip_comp_x = self.sim_filter_x + offset_world[0]
        tip_comp_y = self.sim_filter_y + offset_world[1]
        uwb_tip_pos_comp = np.array([tip_comp_x, tip_comp_y])

        # UWB Smoothing (Low Pass)
        alpha = 0.3
        
        # Hardware Smoothing
        if self.prev_uwb_tip is None:
            self.prev_uwb_tip = uwb_tip_pos_hw
        else:
            self.prev_uwb_tip = alpha * uwb_tip_pos_hw + (1.0 - alpha) * self.prev_uwb_tip

        # Computed Smoothing
        if self.prev_uwb_comp_tip is None:
            self.prev_uwb_comp_tip = uwb_tip_pos_comp
        else:
            self.prev_uwb_comp_tip = alpha * uwb_tip_pos_comp + (1.0 - alpha) * self.prev_uwb_comp_tip


        # ====================================================
        #    INITIALIZATION
        # ====================================================
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_tip_pos_hw) 
            self.kf_comp = KalmanFilter2D(uwb_tip_pos_comp) 
            self.prev_ts_micros = vals[13] 
            return

        # ====================================================
        #    IMU BATCH PREDICTION
        # ====================================================
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
            lin_acc = q.rotate(np.array([ax, ay, az]))
            
            # Clamping & Deadband
            limit = CONFIG['max_accel']
            input_acc_x = np.clip(lin_acc[0], -limit, limit)
            input_acc_y = np.clip(lin_acc[1], -limit, limit)
            input_acc = np.array([input_acc_x, input_acc_y])

            if np.linalg.norm(input_acc) < CONFIG['accel_deadband']:
                input_acc[:] = 0.0

            # ZUPT Logic
            acc_mag = np.linalg.norm(input_acc)
            self.accel_magnitudes.append(acc_mag)
            
            if len(self.accel_magnitudes) > self.ZUPT_WINDOW_SIZE:
                self.accel_magnitudes.pop(0)

            is_stationary = False
            if len(self.accel_magnitudes) == self.ZUPT_WINDOW_SIZE:
                avg_acc = np.mean(self.accel_magnitudes)
                var_acc = np.var(self.accel_magnitudes)
                if avg_acc < CONFIG['stationary_thresh'] and var_acc < 0.05:
                    is_stationary = True

            # --- PREDICT BOTH FILTERS ---
            self.kf.predict(input_acc[0], input_acc[1], dt)
            self.kf_comp.predict(input_acc[0], input_acc[1], dt)

            # Apply Friction
            self.kf.x[2] *= CONFIG['friction']
            self.kf.x[3] *= CONFIG['friction']
            self.kf_comp.x[2] *= CONFIG['friction']
            self.kf_comp.x[3] *= CONFIG['friction']

            # Apply ZUPT
            if is_stationary:
                self.kf.apply_zupt()
                self.kf_comp.apply_zupt()
            
            has_predicted = True
            
            # --- RECORD PATHS ---
            curr_x, curr_y = self.kf.x[0], self.kf.x[1]
            if self.is_in_bounds(curr_x, curr_y):
                self.path_x.append(curr_x)
                self.path_y.append(curr_y)
                self.is_drawing = True
            else:
                if self.is_drawing:
                    self.path_x.append(np.nan)
                    self.path_y.append(np.nan)
                    self.is_drawing = False
            
            comp_curr_x, comp_curr_y = self.kf_comp.x[0], self.kf_comp.x[1]
            if self.is_in_bounds(comp_curr_x, comp_curr_y):
                self.path_comp_fused_x.append(comp_curr_x)
                self.path_comp_fused_y.append(comp_curr_y)
            else:
                self.path_comp_fused_x.append(np.nan)
                self.path_comp_fused_y.append(np.nan)

        # ====================================================
        #    UPDATE STEP
        # ====================================================
        if has_predicted:
            # --- Update Filter 1 (Hardware) ---
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
            
            # --- Update Filter 2 (Computed) ---
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

# ==========================================
# MAIN EXECUTION
# ==========================================
is_running = True

def signal_handler(sig, frame):
    global is_running
    print("\nStopping capture...")
    is_running = False

def main():
    signal.signal(signal.SIGINT, signal_handler)
    tracker = StrokeTracker()
    
    print(f"Opening {CONFIG['serial_port']}...")

    # =====================================================
    #        LIVE PLOT
    # =====================================================
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

    ax.set_title("v4.3: NLLS + Strong Wall Bounds")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.show()

    try:
        ser = serial.Serial(CONFIG['serial_port'], CONFIG['baud_rate'], timeout=1)
        ser.flushInput()
        print("System Ready. Draw on the wall.")
        
        while is_running:
            line = ser.readline()
            if line:
                tracker.process_packet(line)

                # ====== UPDATE PLOTS (UWB) ======
                # if tracker.uwb_hw_x:
                #     line_raw.set_xdata(tracker.uwb_hw_x)
                #     line_raw.set_ydata(tracker.uwb_hw_y)
                #
                # if tracker.uwb_computed_x:
                #     line_comp.set_xdata(tracker.uwb_computed_x)
                #     line_comp.set_ydata(tracker.uwb_computed_y)
                
                # # ====== UPDATE PLOTS (COORDINATE) ======
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