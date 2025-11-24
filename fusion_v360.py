import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import signal
import sys

# ==========================================
# CONFIGURATION - v5.0 "Production Ready"
# ==========================================
plt.ion()

CONFIG = {
    'serial_port': 'COM2',       
    'baud_rate': 115200,
    
    # --- LAYERS 1 & 2 (Defense) ---
    'uwb_window_size': 5,        # Median filter size
    
    # [FIX] Tighter Clamp to catch the 2.93 spike
    'max_accel': 2.0,            
    
    # [FIX] REMOVED WARMUP PACKETS (Was 50, now effectively 0)
    
    # --- VIRTUAL WHITEBOARD BOUNDS ---
    'bounds_x_min': 0.2, 'bounds_x_max': 3.0,
    'bounds_y_min': -0.1, 'bounds_y_max': 2.5,
    
    # --- FINE TUNING (The Ruler Profile) ---
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
# CLASS: KALMAN FILTER
# ==========================================
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
        self.prev_ts_micros = 0
        self.prev_uwb = None
        
        # Buffers
        self.path_x = []
        self.path_y = []
        self.uwb_raw_x = []
        self.uwb_raw_y = []
        
        # State
        self.is_drawing = False 
        
        # [FIX] Added Rescue Counter
        self.consecutive_rejects = 0
        
        # Filters
        self.uwb_window_x = []
        self.uwb_window_y = []
        
        # [FIX] Added Windowed ZUPT Buffer
        self.accel_window = [] 
        # --- NEW: ZUPT Buffer ---
        self.accel_magnitudes = [] 
        self.ZUPT_WINDOW_SIZE = 10

    def is_in_bounds(self, x, y):
        # NOTE: Once you have a pressure sensor, replace this function
        # with a simple "return True" or "return pressure > threshold" check.
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

        # --- LAYER 1: MEDIAN FILTER ---
        self.uwb_window_x.append(vals[0])
        self.uwb_window_y.append(vals[1])
        if len(self.uwb_window_x) > CONFIG['uwb_window_size']:
            self.uwb_window_x.pop(0)
            self.uwb_window_y.pop(0)
            
        uwb_clean_x = np.median(self.uwb_window_x)
        uwb_clean_y = np.median(self.uwb_window_y)
        uwb_pos = np.array([uwb_clean_x, uwb_clean_y])
        
        self.uwb_raw_x.append(vals[0])
        self.uwb_raw_y.append(vals[1])

        # # UWB Smoothing
        alpha = 0.3
        if self.prev_uwb is None:
            self.prev_uwb = uwb_pos
        else:
            self.prev_uwb = alpha * uwb_pos + (1.0 - alpha) * self.prev_uwb


        # --- INITIALIZATION ---
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_pos)
            self.prev_ts_micros = vals[13] 
            return

        # [FIX] Warmup check removed. We process everything immediately.

        # --- IMU BATCH PROCESSING ---
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

            # [FIX] GRAVITY BUG REMOVED
            # We assume sensor provides Linear Acceleration (Gravity removed internally)
            # So we only rotate, we do NOT subtract 9.81 again.
            q = Quaternion(qw, qx, qy, qz)
            lin_acc = q.rotate(np.array([ax, ay, az]))
            
            # --- CLAMPING ---
            limit = CONFIG['max_accel']
            input_acc_x = np.clip(lin_acc[0], -limit, limit)
            input_acc_y = np.clip(lin_acc[1], -limit, limit)
            input_acc = np.array([input_acc_x, input_acc_y])

            # Deadband
            if np.linalg.norm(input_acc) < CONFIG['accel_deadband']:
                input_acc[:] = 0.0

            # --- WINDOWED ZUPT ---

            # --- NEW ROBUST ZUPT LOGIC ---
            # 1. Calculate magnitude of current acceleration
            acc_mag = np.linalg.norm(input_acc)
            self.accel_magnitudes.append(acc_mag)
            
            # Keep window size fixed
            if len(self.accel_magnitudes) > self.ZUPT_WINDOW_SIZE:
                self.accel_magnitudes.pop(0)

            # 2. Check if we are stationary
            is_stationary = False
            if len(self.accel_magnitudes) == self.ZUPT_WINDOW_SIZE:
                # Calculate mean (how much force?) and variance (how much shaking?)
                avg_acc = np.mean(self.accel_magnitudes)
                var_acc = np.var(self.accel_magnitudes)
                
                # TUNING: 
                # thresh 0.2 means "very little movement"
                # var 0.05 means "steady hand"
                if avg_acc < CONFIG['stationary_thresh'] and var_acc < 0.05:
                    is_stationary = True

            # Predict
            self.kf.predict(input_acc[0], input_acc[1], dt)
            
            # Apply Friction
            self.kf.x[2] *= CONFIG['friction']
            self.kf.x[3] *= CONFIG['friction']

            if is_stationary:
                self.kf.apply_zupt()
                # OPTIONAL: If we are truly stationary, we can force the Velocity to 0
                self.kf.x[2] = 0.0
                self.kf.x[3] = 0.0
            
            has_predicted = True
            # ... rest of code ...
            
            # --- RECORD PATH ---
            # No warmup check here anymore. 
            curr_x, curr_y = self.kf.x[0], self.kf.x[1]
            
            # Virtual Pen / Geofencing
            if self.is_in_bounds(curr_x, curr_y):
                self.path_x.append(curr_x)
                self.path_y.append(curr_y)
                self.is_drawing = True
            else:
                if self.is_drawing:
                    self.path_x.append(np.nan) # Break the line
                    self.path_y.append(np.nan)
                    self.is_drawing = False

        # --- UPDATE + RESCUE LOGIC ---
        if has_predicted:
            pred_pos = self.kf.x[:2]
            innovation = np.linalg.norm(self.prev_uwb - pred_pos)
            
            if innovation > CONFIG['uwb_jump_thresh']:
                self.consecutive_rejects += 1
                # [FIX] Rescue: If 5 packets rejected, assume filter is lost -> FORCE RESET
                if self.consecutive_rejects >= 5:
                    print(f"Rescue: Resetting Filter (Diff {innovation:.2f}m)")
                    self.kf.x[0] = self.prev_uwb[0]
                    self.kf.x[1] = self.prev_uwb[1]
                    self.kf.x[2] = 0.0 
                    self.kf.x[3] = 0.0
                    self.kf.P = np.diag([0.5, 0.5, 1.0, 1.0])
                    self.consecutive_rejects = 0
            else:
                self.kf.update(self.prev_uwb)
                self.consecutive_rejects = 0

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
    #        LIVE PLOT – CREATED ONLY ONCE
    # =====================================================
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    # Raw UWB (Orange dashed line with dots)
    line_raw, = ax.plot(
        [], [],
        linestyle='--',
        marker='o',
        markersize=2,
        alpha=0.6,
        color='orange',
        label='Raw UWB'
    )

    # Filtered Path (Blue solid line)
    line_fused, = ax.plot(
        [], [],
        linestyle='-',
        linewidth=1.5,
        color="#3F92E6",
        label='Fused Track'
    )

    ax.set_title("v5.0: Live Tracking")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.show()

    # =====================================================
    #           START SERIAL CAPTURE LOOP
    # =====================================================
    try:
        ser = serial.Serial(CONFIG['serial_port'], CONFIG['baud_rate'], timeout=1)
        ser.flushInput()
        print("System Ready. Draw on the wall.")
        
        while is_running:
            line = ser.readline()
            if line:
                tracker.process_packet(line)

                # Update plot data
                if len(tracker.uwb_raw_x) > 1:
                    line_raw.set_xdata(tracker.uwb_raw_x)
                    line_raw.set_ydata(tracker.uwb_raw_y)

                if len(tracker.path_x) > 1:
                    line_fused.set_xdata(tracker.path_x)
                    line_fused.set_ydata(tracker.path_y)

                # Update limits dynamically (optional)
                ax.relim()
                ax.autoscale_view()

                fig.canvas.draw()
                fig.canvas.flush_events()
                # Optional: smooth CPU use
                # time.sleep(0.001)

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