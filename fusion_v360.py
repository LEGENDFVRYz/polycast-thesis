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
from trilateration import solve_position_from_distances

# ==============================================================================
#   "v4.2.1: Non-Linear Least Squares (NLLS) optimization"
# ==============================================================================
#   
#   UPDATED LOGIC:
#   - Added 'anchor_offsets' to CONFIG.
#   - Applied (Distance + Offset) before trilateration.
#   - This aligns perfectly with the manufacturer's 'parse_uwb_data' function.
#
# ==============================================================================

plt.ion()

CONFIG = {
    'serial_port': 'COM2',
    'baud_rate': 115200,
    
    # --- PHYSICAL SETUP (CRITICAL) ---
    # 1. ANCHOR POSITIONS (From your reference code)
    #    Format: Numpy Array [x, y, z]
    #    Order: Anchor 0, Anchor 1, Anchor 2 (Matches d0, d1, d2)
    'anchors': np.array([
        [1.75, 0.00, 0.00],  # Anchor 0
        [1.75, 1.61, 0.00],  # Anchor 1
        [0.00, 0.00, 0.00]   # Anchor 2
    ]),
    
    # 2. LEVER ARM (Sensor -> Tip)
    #    Vector from Sensor Center -> Pen Tip (in meters)
    'tip_offset': [0.0, 0.0, 0.0], 
    
    # --- LAYERS 1 & 2 (Defense) ---
    'uwb_window_size': 5,        
    'max_accel': 2.0,            
    
    # --- VIRTUAL WHITEBOARD BOUNDS ---
    'bounds_x_min': -0.5, 'bounds_x_max': 2.5, # Expanded bounds for testing
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
# MANUFACTURER TRILATERATION LOGIC
# ==========================================
# ==========================================
# NEW: NON-LINEAR LEAST SQUARES TRILATERATION
# ==========================================
def trilaterate_nlls(distances, anchors):
    """
    Solves for position using Non-Linear Least Squares optimization.
    Minimizes the error: sum(|x - Ai| - di)^2
    Includes Bounds to force Z near 0 (Wall mode).
    """
    
    # 1. Filter out invalid distances (Reference code treats 0.0 as error)
    distances = np.array(distances)
    valid_mask = distances > 0
    
    # We need at least 3 anchors for a solid fix, but NLLS can attempt with fewer
    # given the bounds, though 3 is recommended.
    if np.sum(valid_mask) < 3:
        return None

    active_anchors = anchors[valid_mask]
    active_dists = distances[valid_mask]

    # 2. Define the Residual Function (The error to minimize)
    def residuals(x, anchors, measured_dists):
        # x is the guessed position [x, y, z]
        # np.linalg.norm(anchors - x, axis=1) calculates distance from guess to each anchor
        estimated_dists = np.linalg.norm(anchors - x, axis=1)
        return estimated_dists - measured_dists

    # 3. Initial Guess (Centroid of anchors is a safe starting point)
    x0 = np.mean(active_anchors, axis=0)

    # 4. Optimization with Bounds (Option B from your notes)
    # Since we are writing on a wall, we constrain Z heavily.
    # X/Y bounds can be loose (room size). Z bounds are tight (-0.2 to 0.2).
    bounds_min = [-10.0, -10.0, -0.2]
    bounds_max = [ 10.0,  10.0,  0.2]

    try:
        res = least_squares(
            residuals, 
            x0, 
            bounds=(bounds_min, bounds_max),
            args=(active_anchors, active_dists),
            loss='soft_l1', # Robust loss: reduces effect of gross outliers
            f_scale=0.5     # Sensitivity of the robust loss
        )
        return res.x # Returns [x, y, z]
    except Exception as e:
        # Fallback or logging if solver fails
        return None


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
        # NOTE: Ideally we would add Coriolis terms here using Gyro, 
        # but for handwriting speeds, assuming a_tip ≈ a_sensor is acceptable.
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
        # --- FILTER 1: Standard Hardware Fused (Light Blue) ---
        self.kf = None
        self.prev_uwb_tip = None 
        self.path_x = []         
        self.path_y = []
        self.consecutive_rejects = 0

        # --- FILTER 2: Computed/Manual Fused (Dark Blue) ---
        self.kf_comp = None
        self.prev_uwb_comp_tip = None
        self.path_comp_fused_x = []
        self.path_comp_fused_y = []
        self.consecutive_rejects_comp = 0

        # --- General State ---
        self.prev_ts_micros = 0
        self.is_drawing = False 
        
        # Debugging/Raw Lines
        self.uwb_hw_x = []       # Orange
        self.uwb_hw_y = []
        self.uwb_computed_x = [] # Red
        self.uwb_computed_y = []
        
        # Imitation Filter State
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False

        # Median Filter Window
        self.uwb_window_x = []
        self.uwb_window_y = []
        
        # ZUPT
        self.accel_magnitudes = [] 
        self.ZUPT_WINDOW_SIZE = 10
        
        # Pre-compute offset vector
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
        d0, d1, d2 = vals[2], vals[3], vals[4]
        
        idx_first = 6 
        qx_init, qy_init, qz_init, qw_init = vals[idx_first : idx_first+4]
        q_init = Quaternion(qw_init, qx_init, qy_init, qz_init)

        # ====================================================
        #    PIPELINE A: MANUALLY COMPUTED UWB (RED DATA)
        # ====================================================
        comp_pos_3d = trilaterate_nlls([d0, d1, d2], CONFIG['anchors'])
        
        has_comp_data = False
        
        if comp_pos_3d is not None:
            raw_x = comp_pos_3d[0]
            raw_y = comp_pos_3d[1]

            # Imitation Filter (Matches Arduino)
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

        # 2. Computed Tip (From Manual Trilateration)
        # We use self.sim_filter_x/y which is the smoothed RED data
        tip_comp_x = self.sim_filter_x + offset_world[0]
        tip_comp_y = self.sim_filter_y + offset_world[1]
        uwb_tip_pos_comp = np.array([tip_comp_x, tip_comp_y])

        # UWB Smoothing (Low Pass) for both inputs
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
            self.kf = KalmanFilter2D(uwb_tip_pos_hw) # Filter 1
            self.kf_comp = KalmanFilter2D(uwb_tip_pos_comp) # Filter 2 (New)
            self.prev_ts_micros = vals[13] 
            return

        # ====================================================
        #    IMU BATCH PREDICTION (Shared by both filters)
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
            # Path 1: Light Blue (Hardware)
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
            
            # Path 2: Dark Blue (Computed)
            comp_curr_x, comp_curr_y = self.kf_comp.x[0], self.kf_comp.x[1]
            # We use the same 'is_drawing' flag logic roughly, or check bounds again
            if self.is_in_bounds(comp_curr_x, comp_curr_y):
                self.path_comp_fused_x.append(comp_curr_x)
                self.path_comp_fused_y.append(comp_curr_y)
            else:
                # Add gaps if out of bounds
                self.path_comp_fused_x.append(np.nan)
                self.path_comp_fused_y.append(np.nan)

        # ====================================================
        #    UPDATE STEP (Correction)
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
            if has_comp_data: # Only update if trilateration succeeded this frame
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

    # 1. ORANGE: Hardware Output (Sensor)
    line_raw, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.4,
                        linewidth=1.0, color="#E6AC3F", label='Hardware UWB')

    # 2. RED: Manual Trilateration (Computed Raw)
    line_comp, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.6,
                          linewidth=1.0, color="#E63F3F", label='Computed Trilateration')

    # 3. LIGHT BLUE: Fused Track (Hardware Tip)
    line_fused, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#3F92E6",
                           label='Fused (Hardware)')
    
    # 4. DARK BLUE: Fused Track (Computed Tip) - NEW!
    line_fused_comp, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#2A2A94",
                           label='Fused (Computed)')

    ax.set_title("v4.2.1: Non-Linear Least Squares (NLLS) optimization")
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

                # ====== UPDATE PLOTS ======
                # if tracker.uwb_hw_x:
                #     line_raw.set_xdata(tracker.uwb_hw_x)
                #     line_raw.set_ydata(tracker.uwb_hw_y)

                # if tracker.uwb_computed_x:
                #     line_comp.set_xdata(tracker.uwb_computed_x)
                #     line_comp.set_ydata(tracker.uwb_computed_y)

                if tracker.path_x:
                    line_fused.set_xdata(tracker.path_x)
                    line_fused.set_ydata(tracker.path_y)
                    
                if tracker.path_comp_fused_x:
                    line_fused_comp.set_xdata(tracker.path_comp_fused_x)
                    line_fused_comp.set_ydata(tracker.path_comp_fused_y)

                # ====== Auto scale ======
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