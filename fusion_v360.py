import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import signal
import sys

# ==========================================
# CONFIGURATION - v3.2 "Robust Defense"
# ==========================================
CONFIG = {
    'serial_port': 'COM2',       
    'baud_rate': 115200,
    
    # --- LAYER 1: SPIKE EATER SETTINGS ---
    'uwb_window_size': 5,        # Number of samples for Median Filter (odd numbers best)
    
    # --- LAYER 2: PHYSICS GATE SETTINGS ---
    'max_accel': 3.0,            # Max allowed acceleration (m/s^2). Clips handling noise.
    'warmup_packets': 50,        # Ignore first ~1 sec of data to skip "pickup" noise.
    
    # --- VIRTUAL WHITEBOARD BOUNDS (Meters) ---
    # Adjust these to match your actual wall drawing area
    'bounds_x_min': 0.2,
    'bounds_x_max': 3,
    'bounds_y_min': -0.1,
    'bounds_y_max': 2.5,
    
    # --- TUNING PARAMETERS ---
    'accel_deadband': 0.12,      
    'stationary_thresh': 0.30,   
    'friction': 0.90,            
    
    # --- KALMAN FILTER NOISE ---
    'process_pos_var': 1e-5,     
    'process_vel_var': 0.005,    
    'accel_noise_var': 0.2,      
    'uwb_noise_std': 0.15,       
    
    # --- LAYER 3: STATISTICAL GATE SETTINGS ---
    'uwb_jump_thresh': 0.60      # Max allowed jump (meters) per update
}

# ==========================================
# CLASS: KALMAN FILTER
# ==========================================
class KalmanFilter2D:
    """
    Linear Kalman Filter for tracking Position (px, py) and Velocity (vx, vy).
    State Vector x = [px, py, vx, vy]
    """
    def __init__(self, initial_pos):
        self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0], dtype=float)
        
        # Initial Covariance (P)
        self.P = np.diag([0.1**2, 0.1**2, 1.0**2, 1.0**2])
        
        # Measurement Matrix (H)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        
        # Default Measurement Noise (R)
        std = CONFIG['uwb_noise_std']
        self.R_default = np.diag([std**2, std**2])

    def predict(self, ax, ay, dt):
        """ Prediction Step: x = Fx + Bu """
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

        # 1. Predict State
        self.x = F.dot(self.x) + B.dot(u)

        # 2. Predict Covariance
        Q_accel = B.dot(np.eye(2) * CONFIG['accel_noise_var']).dot(B.T)
        Q_base = np.diag([
            CONFIG['process_pos_var'], CONFIG['process_pos_var'],
            CONFIG['process_vel_var'], CONFIG['process_vel_var']
        ])
        Q = Q_accel + Q_base
        
        self.P = F.dot(self.P).dot(F.T) + Q

    def update(self, z_meas, R_override=None):
        """ Update Step: Corrects state using UWB measurement. """
        R = R_override if R_override is not None else self.R_default
        
        y = z_meas - self.H.dot(self.x) # Residual
        S = self.H.dot(self.P).dot(self.H.T) + R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S)) # Kalman Gain
        
        self.x = self.x + K.dot(y)
        
        I = np.eye(4)
        self.P = (I - K.dot(self.H)).dot(self.P)

    def apply_zupt(self):
        """ Zero Velocity Update: Forces velocity to 0. """
        self.x[2] = 0.0 # vx
        self.x[3] = 0.0 # vy
        self.P[2,2] = min(self.P[2,2], 1e-4)
        self.P[3,3] = min(self.P[3,3], 1e-4)

# ==========================================
# CLASS: TRACKING PIPELINE
# ==========================================
class StrokeTracker:
    def __init__(self):
        self.kf = None
        self.prev_ts_micros = 0
        self.prev_uwb = None
        
        # --- Data Storage ---
        self.path_x = []
        self.path_y = []
        self.uwb_raw_x = []
        self.uwb_raw_y = []
        
        # --- State Flags ---
        self.is_drawing = False 
        self.packet_count = 0 
        
        # --- LAYER 1: MEDIAN FILTER BUFFERS ---
        self.uwb_window_x = []
        self.uwb_window_y = []

    def is_in_bounds(self, x, y):
        """Geofencing: Check if coordinate is inside the Virtual Whiteboard."""
        return (x >= CONFIG['bounds_x_min'] and 
                x <= CONFIG['bounds_x_max'] and 
                y >= CONFIG['bounds_y_min'] and 
                y <= CONFIG['bounds_y_max'])

    def process_packet(self, line_bytes):
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return # Bad packet/encoding
            
        if len(vals) < 10: return

        # Increment packet counter for Warmup Logic
        self.packet_count += 1

        # =========================================================
        # LAYER 1: THE SPIKE EATER (Median Filter)
        # =========================================================
        raw_x, raw_y = vals[0], vals[1]
        
        # 1. Add new value to window
        self.uwb_window_x.append(raw_x)
        self.uwb_window_y.append(raw_y)
        
        # 2. Maintain window size (Slide the window)
        if len(self.uwb_window_x) > CONFIG['uwb_window_size']:
            self.uwb_window_x.pop(0)
            self.uwb_window_y.pop(0)
            
        # 3. Calculate Median (This rejects the single-point glitches)
        uwb_clean_x = np.median(self.uwb_window_x)
        uwb_clean_y = np.median(self.uwb_window_y)
        
        uwb_pos = np.array([uwb_clean_x, uwb_clean_y])

        # Store RAW data for comparison in plot, but use CLEAN data for math
        self.uwb_raw_x.append(raw_x)
        self.uwb_raw_y.append(raw_y)

        # Simple exponential smoothing (Low Pass) on the CLEAN data
        alpha = 0.4
        if self.prev_uwb is None:
            self.prev_uwb = uwb_pos
        else:
            self.prev_uwb = alpha * uwb_pos + (1.0 - alpha) * self.prev_uwb

        # --- Initialize KF on first run ---
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_pos)
            self.prev_ts_micros = vals[13] 
            return

        # =========================================================
        # LAYER 2 (Part A): WARMUP LOGIC
        # =========================================================
        # If we are just starting, run the math but do NOT draw/record.
        is_warming_up = self.packet_count < CONFIG['warmup_packets']

        # --- Iterate through IMU Batch ---
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

            # Transform & Gravity Removal
            q = Quaternion(qw, qx, qy, qz)
            acc_world = q.rotate(np.array([ax, ay, az]))
            lin_acc = acc_world - np.array([0.0, 0.0, 9.80665])
            
            # =========================================================
            # LAYER 2 (Part B): PHYSICS GATE (Input Clamping)
            # =========================================================
            # Clip acceleration to realistic handwriting limits
            limit = CONFIG['max_accel']
            input_acc_x = np.clip(lin_acc[0], -limit, limit)
            input_acc_y = np.clip(lin_acc[1], -limit, limit)
            input_acc = np.array([input_acc_x, input_acc_y])

            # Deadband & ZUPT Check
            if np.linalg.norm(input_acc) < CONFIG['accel_deadband']:
                input_acc[:] = 0.0
            is_stationary = np.linalg.norm(input_acc) < CONFIG['stationary_thresh']

            # KF Predict
            self.kf.predict(input_acc[0], input_acc[1], dt)
            self.kf.x[2] *= CONFIG['friction']
            self.kf.x[3] *= CONFIG['friction']

            if is_stationary:
                self.kf.apply_zupt()

            has_predicted = True
            
            # --- VIRTUAL PEN / GEOFENCING ---
            if is_warming_up:
                continue # Skip recording

            curr_x, curr_y = self.kf.x[0], self.kf.x[1]
            if self.is_in_bounds(curr_x, curr_y):
                self.path_x.append(curr_x)
                self.path_y.append(curr_y)
                self.is_drawing = True
            else:
                if self.is_drawing:
                    self.path_x.append(np.nan) # Break the line
                    self.path_y.append(np.nan)
                    self.is_drawing = False

        # =========================================================
        # LAYER 3: STATISTICAL GATE (Innovation Gating)
        # =========================================================
        if has_predicted:
            pred_pos = self.kf.x[:2]
            
            # How far is the UWB measurement from where physics says we are?
            innovation = np.linalg.norm(self.prev_uwb - pred_pos)
            
            if innovation > CONFIG['uwb_jump_thresh']:
                # REJECT: The jump is physically impossible for this dt
                print(f"Update REJECTED. Jump: {innovation:.2f}m")
                pass 
            else:
                # ACCEPT: Update filter
                self.kf.update(self.prev_uwb)

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
    try:
        ser = serial.Serial(CONFIG['serial_port'], CONFIG['baud_rate'], timeout=1)
        ser.flushInput()
        print("System Ready. (Warmup active for first ~1s)")
        
        while is_running:
            try:
                line = ser.readline()
                if line:
                    tracker.process_packet(line)
            except serial.SerialException:
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        
        if len(tracker.path_x) > 0:
            plt.figure(figsize=(8, 8))
            
            # Raw UWB (Orange dashed line with dots)
            plt.plot(
                tracker.uwb_raw_x,
                tracker.uwb_raw_y,
                linestyle='--',
                marker='o',
                markersize=2,
                alpha=0.6,
                color='orange',
                label='Raw UWB'
            )
            
            # Filtered Path (Blue solid line)
            plt.plot(
                tracker.path_x,
                tracker.path_y,
                linestyle='-',
                linewidth=1.5,
                color="#3F92E6",
                label='Fused Track'
            )
            
            plt.title("v3.2.1: Robust Fusion (Median + Clamped + Gated)")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.show()

if __name__ == "__main__":
    main()