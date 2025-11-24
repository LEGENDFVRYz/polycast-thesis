import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import signal
import sys

# ==============================================================================
#   v5.1: LEVER ARM CORRECTION (Current Status & Future Optimization)
# ==============================================================================
#
#   CURRENT SITUATION (Testing Phase):
#   - We are currently testing with the pen held PERPENDICULAR to the wall
#     (hovering/robot-arm style). 
#   - In this specific case, the "Lever Arm" effect is just a constant offset.
#   - The IMU acceleration matches the Tip acceleration because there is 
#     no wrist rotation.
#
#   THE PROBLEM (Real Handwriting):
#   - Real handwriting involves pivoting the wrist.
#   - PROBLEM A: The sensor moves in an arc even when the tip is stationary.
#     -> FIXED by the 'tip_offset' logic below (Transforming Sensor -> Tip).
#   - PROBLEM B: The IMU measures "Centripetal Acceleration" from the rotation.
#     -> PENDING: Future optimization will need to subtract these forces
#        using Gyroscope data in the Kalman Filter prediction step.
#
# ==============================================================================

plt.ion()

CONFIG = {
    'serial_port': 'COM2',       
    'baud_rate': 115200,
    
    # --- PHYSICAL SETUP (CRITICAL) ---
    # Vector from Sensor Center -> Pen Tip (in meters)
    # [x, y, z] relative to the sensor board (Local Frame).
    #
    # ACTION: Measure your hardware! 
    # Usually, the sensor axis pointing to the tip is ~0.14m.
    # If mounted sideways, it might be Y or Z.
    'tip_offset': [0.14, 0.0, 0.0], 
    
    # --- LAYERS 1 & 2 (Defense) ---
    'uwb_window_size': 5,        
    'max_accel': 2.0,            
    
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
        self.kf = None
        self.prev_ts_micros = 0
        self.prev_uwb_tip = None # Track the TIP, not the sensor
        
        self.path_x = []
        self.path_y = []
        self.uwb_raw_x = []
        self.uwb_raw_y = []
        
        self.is_drawing = False 
        self.consecutive_rejects = 0
        
        self.uwb_window_x = []
        self.uwb_window_y = []
        
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

        # --- EXTRACT QUATERNION FIRST (Needed for Position Transform) ---
        # We grab the FIRST available quaternion in the batch to correct the UWB position
        # (Technically UWB happened before these IMU samples, but it's the closest estimate)
        idx_first = 6 
        qx_init, qy_init, qz_init, qw_init = vals[idx_first : idx_first+4]
        q_init = Quaternion(qw_init, qx_init, qy_init, qz_init)

        # --- LAYER 1: MEDIAN FILTER (Raw UWB Sensor) ---
        self.uwb_window_x.append(vals[0])
        self.uwb_window_y.append(vals[1])
        if len(self.uwb_window_x) > CONFIG['uwb_window_size']:
            self.uwb_window_x.pop(0)
            self.uwb_window_y.pop(0)
            
        uwb_sensor_clean_x = np.median(self.uwb_window_x)
        uwb_sensor_clean_y = np.median(self.uwb_window_y)
        
        # --- LAYER 1.5: TRANSFORM SENSOR -> TIP ---
        # Rotate the offset vector into world frame
        offset_world = q_init.rotate(self.offset_local)
        
        # Calculate where the TIP is
        tip_x = uwb_sensor_clean_x + offset_world[0]
        tip_y = uwb_sensor_clean_y + offset_world[1]
        uwb_tip_pos = np.array([tip_x, tip_y])

        # Store Raw Sensor data for debug plotting
        self.uwb_raw_x.append(vals[0])
        self.uwb_raw_y.append(vals[1])

        # UWB Smoothing (Applied to the TIP position)
        alpha = 0.3
        if self.prev_uwb_tip is None:
            self.prev_uwb_tip = uwb_tip_pos
        else:
            self.prev_uwb_tip = alpha * uwb_tip_pos + (1.0 - alpha) * self.prev_uwb_tip

        # --- INITIALIZATION ---
        if self.kf is None:
            self.kf = KalmanFilter2D(uwb_tip_pos)
            self.prev_ts_micros = vals[13] 
            return

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

            q = Quaternion(qw, qx, qy, qz)
            lin_acc = q.rotate(np.array([ax, ay, az]))
            
            # --- CLAMPING ---
            limit = CONFIG['max_accel']
            input_acc_x = np.clip(lin_acc[0], -limit, limit)
            input_acc_y = np.clip(lin_acc[1], -limit, limit)
            input_acc = np.array([input_acc_x, input_acc_y])

            if np.linalg.norm(input_acc) < CONFIG['accel_deadband']:
                input_acc[:] = 0.0

            # --- ROBUST ZUPT LOGIC ---
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

            # Predict (The KF now tracks TIP state)
            # Assumption: Tip Acceleration ~= Sensor Acceleration (ignoring rotational terms)
            self.kf.predict(input_acc[0], input_acc[1], dt)
            
            self.kf.x[2] *= CONFIG['friction']
            self.kf.x[3] *= CONFIG['friction']

            if is_stationary:
                self.kf.apply_zupt()
                self.kf.x[2] = 0.0
                self.kf.x[3] = 0.0
            
            has_predicted = True
            
            # --- RECORD PATH ---
            # The KF state is already the TIP position
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

        # --- UPDATE + RESCUE LOGIC ---
        if has_predicted:
            pred_pos = self.kf.x[:2]
            # Innovation is now: (Smoothed UWB TIP) - (Predicted KF TIP)
            innovation = np.linalg.norm(self.prev_uwb_tip - pred_pos)
            
            if innovation > CONFIG['uwb_jump_thresh']:
                self.consecutive_rejects += 1
                if self.consecutive_rejects >= 5:
                    print(f"Rescue: Resetting Filter (Diff {innovation:.2f}m)")
                    self.kf.x[0] = self.prev_uwb_tip[0]
                    self.kf.x[1] = self.prev_uwb_tip[1]
                    self.kf.x[2] = 0.0 
                    self.kf.x[3] = 0.0
                    self.kf.P = np.diag([0.5, 0.5, 1.0, 1.0])
                    self.consecutive_rejects = 0
            else:
                self.kf.update(self.prev_uwb_tip)
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
    #        LIVE PLOT
    # =====================================================
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    # Raw UWB (Orange dashed line) - SENSOR Position
    line_raw, = ax.plot(
        [], [],
        linestyle='--',
        marker='o',
        markersize=2,
        alpha=0.6,
        color='orange',
        label='Raw UWB (Sensor)'
    )

    # Filtered Path (Blue solid line) - TIP Position
    line_fused, = ax.plot(
        [], [],
        linestyle='-',
        linewidth=1.5,
        color="#3F92E6",
        label='Fused Track (Tip)'
    )

    ax.set_title("v5.1: Lever Arm Corrected")
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

                # Update plot data
                if len(tracker.uwb_raw_x) > 1:
                    line_raw.set_xdata(tracker.uwb_raw_x)
                    line_raw.set_ydata(tracker.uwb_raw_y)

                if len(tracker.path_x) > 1:
                    line_fused.set_xdata(tracker.path_x)
                    line_fused.set_ydata(tracker.path_y)

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