import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import signal
import sys

# ==========================================
# CONFIGURATION
# ==========================================
CONFIG = {
    'serial_port': 'COM2',       # Update this for your machine
    'baud_rate': 115200,
    
    # --- Tuning Parameters ---
    'accel_deadband': 0.15,      # Ignore accel below this (m/s^2)
    'stationary_thresh': 0.25,   # Threshold to trigger ZUPT (Zero Velocity)
    'friction': 0.95,            # Damping factor for velocity when coasting
    
    # --- Kalman Filter Noise Profiles ---
    # Process Noise (How much we trust the physics prediction)
    'process_pos_var': 1e-6,
    'process_vel_var': 1e-3,
    'accel_noise_var': 0.5,      # How noisy is the IMU acceleration?
    
    # Measurement Noise (How much we trust the UWB)
    'uwb_noise_std': 0.05,       # 5cm standard deviation
    'uwb_jump_thresh': 0.5       # Outlier rejection threshold (meters)
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
        # Initial State: [x, y, vx, vy]
        self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0], dtype=float)
        
        # Initial Covariance (P): High uncertainty in velocity, low in pos
        self.P = np.diag([0.1**2, 0.1**2, 1.0**2, 1.0**2])
        
        # Measurement Matrix (H): We only measure Position [px, py]
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        
        # Measurement Noise Covariance (R)
        std = CONFIG['uwb_noise_std']
        self.R_default = np.diag([std**2, std**2])

    def predict(self, ax, ay, dt):
        """
        Prediction Step: Moves state forward using physics + IMU acceleration.
        x = Fx + Bu
        """
        # State Transition Matrix (F) - Kinematics
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Control Matrix (B) - How Accel affects Pos and Vel
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
        # Q involves continuous process noise + accel noise injection
        Q_accel = B.dot(np.eye(2) * CONFIG['accel_noise_var']).dot(B.T)
        Q_base = np.diag([
            CONFIG['process_pos_var'], CONFIG['process_pos_var'],
            CONFIG['process_vel_var'], CONFIG['process_vel_var']
        ])
        Q = Q_accel + Q_base
        
        self.P = F.dot(self.P).dot(F.T) + Q

    def update(self, z_meas, R_override=None):
        """
        Update Step: Corrects state using UWB measurement.
        """
        R = R_override if R_override is not None else self.R_default
        
        # Residual (y) = Measurement - Prediction
        y = z_meas - self.H.dot(self.x)
        
        # Kalman Gain (K)
        S = self.H.dot(self.P).dot(self.H.T) + R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        
        # Update State
        self.x = self.x + K.dot(y)
        
        # Update Covariance
        I = np.eye(4)
        self.P = (I - K.dot(self.H)).dot(self.P)

    def apply_zupt(self):
        """
        Zero Velocity Update: Forces velocity to 0 and reduces uncertainty.
        Crucial for stopping drift when the pen stops.
        """
        self.x[2] = 0.0 # vx
        self.x[3] = 0.0 # vy
        
        # Trust this "stop" heavily -> reduce velocity covariance
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
        
        # Data storage for plotting
        self.path_x = []
        self.path_y = []
        self.uwb_raw_x = []
        self.uwb_raw_y = []

    def process_packet(self, line_bytes):
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return # Bad packet
            
        if len(vals) < 10: return

        # --- 1. Parse UWB ---
        uwb_pos = np.array([vals[0], vals[1]])
        self.uwb_raw_x.append(uwb_pos[0])
        self.uwb_raw_y.append(uwb_pos[1])

        # Simple smoothing for UWB (Low Pass Filter)
        alpha = 0.4
        if self.prev_uwb is None:
            self.prev_uwb = uwb_pos
        else:
            self.prev_uwb = alpha * uwb_pos + (1.0 - alpha) * self.prev_uwb

        # --- 2. Initialize KF on first run ---
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_pos)
            # Set initial timestamp from the first IMU sample in packet
            self.prev_ts_micros = vals[6 + 7] 
            self.path_x.append(self.kf.x[0])
            self.path_y.append(self.kf.x[1])
            return

        # --- 3. Iterate through IMU Batch (10 samples) ---
        # Protocol assumes: UWB_X, UWB_Y, [Qx, Qy, Qz, Qw, Ax, Ay, Az, TS] x 10
        offset = 6
        stride = 8
        
        has_predicted = False

        for i in range(10):
            idx = offset + (i * stride)
            if idx + 7 >= len(vals): break

            # Extract Raw Data
            qx, qy, qz, qw = vals[idx : idx+4]
            ax, ay, az     = vals[idx+4 : idx+7]
            ts_micros      = int(vals[idx+7])

            # Calculate DT
            dt = (ts_micros - self.prev_ts_micros) / 1_000_000.0
            self.prev_ts_micros = ts_micros
            
            # Sanity check dt
            if dt <= 0 or dt > 0.2:
                dt = 0.01 # fallback nominal

            # Coordinate Transform: Body -> World
            q = Quaternion(qw, qx, qy, qz)
            acc_world = q.rotate(np.array([ax, ay, az]))
            
            # Remove Gravity (Assume Z is up/down)
            gravity = np.array([0.0, 0.0, 9.80665])
            lin_acc = acc_world - gravity
            
            # We only care about Wall Plane (X, Y)
            input_acc = np.array([lin_acc[0], lin_acc[1]])

            # --- Heuristic: Deadband ---
            # If accel is tiny, assume it's sensor noise and clamp to 0
            if np.linalg.norm(input_acc) < CONFIG['accel_deadband']:
                input_acc[:] = 0.0
            
            # --- Heuristic: ZUPT ---
            # If totally stationary, tell KF to stop velocity
            is_stationary = np.linalg.norm(input_acc) < CONFIG['stationary_thresh']

            # --- KF Predict ---
            self.kf.predict(input_acc[0], input_acc[1], dt)
            
            # Apply Friction (prevents velocity from persisting forever)
            self.kf.x[2] *= CONFIG['friction']
            self.kf.x[3] *= CONFIG['friction']

            if is_stationary:
                self.kf.apply_zupt()

            has_predicted = True
            
            # Store path (Predicted)
            self.path_x.append(self.kf.x[0])
            self.path_y.append(self.kf.x[1])

        # --- 4. KF Update (UWB Correction) ---
        # We update once per batch using the smoothed UWB value
        if has_predicted:
            # Outlier Rejection Check
            pred_pos = self.kf.x[:2]
            jump = np.linalg.norm(self.prev_uwb - pred_pos)
            
            R_current = None
            if jump > CONFIG['uwb_jump_thresh']:
                # If UWB jump is massive, trust it less (high variance)
                R_current = np.diag([1.0, 1.0]) 
            
            self.kf.update(self.prev_uwb, R_override=R_current)

            # Correct the last point in the path list with the updated position
            self.path_x[-1] = self.kf.x[0]
            self.path_y[-1] = self.kf.x[1]

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
        
        print("System Ready. Draw on the wall.")
        
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
        
        # --- Plotting Results ---
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
            
            plt.title("v2.0: Linear Kalman Filter (KF)")
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.show()

if __name__ == "__main__":
    main()
