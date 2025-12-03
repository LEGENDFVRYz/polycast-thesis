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

TRACKER_CONFIG = {
    # --- PHYSICAL SETUP (CRITICAL) ---
    # 2 Anchors at Bottom (Left/Right), 1 Anchor at Right (Top).
    'anchors': np.array([
        [1.41, 0.00, 0.00],  # Anchor 0: BOTTOM RIGHT
        [1.41, 1.35, 0.00],  # Anchor 1: TOP RIGHT
        [0.00, 1.35, 0.00]   # Anchor 2: BOTTOM LEFT (Origin)
    ]),
    
    'tip_offset': [0.0, 0.0, 0.0],
    'uwb_window_size': 5,
    
    # --- VIRTUAL WHITEBOARD BOUNDS (FOR CALIBRATION) ---
    'bounds_x_min': -0.5, 'bounds_x_max': 2.5,
    'bounds_y_min': -0.5, 'bounds_y_max': 2.5,
    
    # --- SCALING OF RENDERED DRAWING
    'bmin': 0.0,
    'bmax': 1.2,
    
    # --- FINE TUNING ---
    'uwb_noise_std': 0.5,
    'accel_noise_var': 0.05,
    'process_pos_var': 0.01,
    'process_vel_var': 0.1,
    'uwb_jump_thresh': 0.60,
    'friction': 0.90,
    'stationary_thresh': 0.20,
    'accel_deadband': 0.12,
    'max_accel': 2.0
}

# Add these config params near TRACKER_CONFIG (or inside it)
FORCE_CONFIG = {
    'contact_on_thresh': 1.0,       # tune to your hardware units
    'contact_off_thresh': 0.6,
    'contact_on_count': 2,
    'contact_off_count': 3,
    'use_max_force': True,          # use max(force_i) OR mean(force_i)
    'R_contact_scale': 0.25,        # multiply default R by this when contact (trust UWB)
    'R_no_contact_scale': 4.0,      # multiply default R by this when no contact (distrust UWB)
    'min_force_samples_for_decision': 2,
    'spike_reject_window': 3        # number of recent uwb packets to consider for spike rejections
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
        self.R_default = np.diag([TRACKER_CONFIG['uwb_noise_std']**2, TRACKER_CONFIG['uwb_noise_std']**2])

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
        
        Q_accel = B.dot(np.eye(2) * TRACKER_CONFIG['accel_noise_var']).dot(B.T)
        Q_base = np.diag([
            TRACKER_CONFIG['process_pos_var'], TRACKER_CONFIG['process_pos_var'],
            TRACKER_CONFIG['process_vel_var'], TRACKER_CONFIG['process_vel_var']
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
# MAIN CLASS: STROKE TRACKER
# ==========================================
class StrokeTracker:
    def __init__(self):
        self.kf_comp = None
        self.prev_uwb_comp_tip = None
        self.prev_ts_micros = 0
        self.dist_windows = [[], [], []] 
        self.uwb_window_x = []
        self.uwb_window_y = []
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False
        self.accel_magnitudes = []
        self.offset_local = np.array(TRACKER_CONFIG['tip_offset'])
        self.consecutive_rejects_comp = 0
        self.is_drawing = False

        # --- Debug/Plotting History ---
        self.uwb_hw_x = []
        self.uwb_hw_y = []
        self.uwb_computed_x = []
        self.uwb_computed_y = []
        self.path_x = []        
        self.path_y = []
        self.path_comp_fused_x = []
        self.path_comp_fused_y = []
        self.prev_uwb_tip = None
        self.consecutive_rejects = 0
        self.kf = None
        
        # Pressure/contact detection state
        self.contact_state = False
        self._contact_on_counter = 0
        self._contact_off_counter = 0
        self._force_history = []    # short-term smoothing
        self._recent_uwb_positions = []  # keep for spike rejection

    def get_bounds(self):
        """Returns the physical bounds config so the UI thread knows the range."""
        return {
            'x_min': TRACKER_CONFIG['bounds_x_min'],
            'x_max': TRACKER_CONFIG['bounds_x_max'],
            'y_min': TRACKER_CONFIG['bounds_y_min'],
            'y_max': TRACKER_CONFIG['bounds_y_max']
        }

    def get_bbox(self):
        """Returns the render bounds config so the PT thread know what to rendered."""
        return {
            'b_min': TRACKER_CONFIG['bmin'],
            'b_max': TRACKER_CONFIG['bmax'],
        }
    
    def is_in_bounds(self, x, y):
        return (x >= TRACKER_CONFIG['bounds_x_min'] and 
                x <= TRACKER_CONFIG['bounds_x_max'] and 
                y >= TRACKER_CONFIG['bounds_y_min'] and 
                y <= TRACKER_CONFIG['bounds_y_max'])

    def process_packet(self, line_bytes):
        """
        Input: Raw Serial Bytes
        Output: (x_meters, y_meters, is_drawing_bool) OR None
        """
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return None
            
        if len(vals) < 10: return None

        # --- 1. EXTRACT DATA ---
        hw_x, hw_y = vals[0], vals[1]
        dists = [vals[2], vals[3], vals[4]]
        qx_init, qy_init, qz_init, qw_init = vals[6:10]
        q_init = Quaternion(qw_init, qx_init, qy_init, qz_init)

        # --- 2. CALCULATE VARIANCES ---
        current_variances = []
        for i in range(3):
            d = dists[i]
            if d > 0: self.dist_windows[i].append(d)
            if len(self.dist_windows[i]) > TRACKER_CONFIG['uwb_window_size']: self.dist_windows[i].pop(0)
            if len(self.dist_windows[i]) > 2:
                current_variances.append(max(np.var(self.dist_windows[i]), 1e-5))
            else:
                current_variances.append(1.0)

        # --- 3. TRILATERATION (WNLLS) ---
        last_pos_guess = np.array([self.kf_comp.x[0], self.kf_comp.x[1], 0.0]) if self.kf_comp else None
        comp_pos_3d, _ = trilaterate_wls(dists, TRACKER_CONFIG['anchors'], current_variances, last_pos_guess)
        
        # --- 4. DATA LOGGING (For Plotting) ---
        # Record Raw Hardware
        self.uwb_hw_x.append(hw_x)
        self.uwb_hw_y.append(hw_y)

        has_comp_data = False
        if comp_pos_3d is not None:
            raw_x, raw_y = comp_pos_3d[0], comp_pos_3d[1]

            # Imitation Low-Pass Filter
            if not self.sim_initialized:
                self.sim_filter_x, self.sim_filter_y = raw_x, raw_y
                self.sim_initialized = True
            else:
                self.sim_filter_x = (0.75 * self.sim_filter_x) + (0.25 * raw_x)
                self.sim_filter_y = (0.75 * self.sim_filter_y) + (0.25 * raw_y)
            
            self.uwb_computed_x.append(self.sim_filter_x)
            self.uwb_computed_y.append(self.sim_filter_y)
            has_comp_data = True
        else:
            raw_x, raw_y = 0.0, 0.0

        # --- 5. TRANSFORM TO TIP ---
        offset_world = q_init.rotate(self.offset_local)

        # Hardware Tip
        # Simple median filter for hardware path
        self.uwb_window_x.append(hw_x)
        self.uwb_window_y.append(hw_y)
        if len(self.uwb_window_x) > TRACKER_CONFIG['uwb_window_size']:
             self.uwb_window_x.pop(0)
             self.uwb_window_y.pop(0)
        
        tip_hw_x = np.median(self.uwb_window_x) + offset_world[0]
        tip_hw_y = np.median(self.uwb_window_y) + offset_world[1]
        uwb_tip_pos_hw = np.array([tip_hw_x, tip_hw_y])

        # Computed Tip
        tip_comp_x = self.sim_filter_x + offset_world[0]
        tip_comp_y = self.sim_filter_y + offset_world[1]
        uwb_tip_pos_comp = np.array([tip_comp_x, tip_comp_y])

        # Pre-filter Smoothing
        alpha = 0.3
        if self.prev_uwb_comp_tip is None:
            self.prev_uwb_comp_tip = uwb_tip_pos_comp
        else:
            self.prev_uwb_comp_tip = alpha * uwb_tip_pos_comp + (1.0 - alpha) * self.prev_uwb_comp_tip
        
        if self.prev_uwb_tip is None:
            self.prev_uwb_tip = uwb_tip_pos_hw
        else:
            self.prev_uwb_tip = alpha * uwb_tip_pos_hw + (1.0 - alpha) * self.prev_uwb_tip


        # --- 6. INITIALIZE OR PREDICT (KALMAN) ---
        if self.kf_comp is None:
            self.kf_comp = KalmanFilter2D(uwb_tip_pos_comp)
            self.kf = KalmanFilter2D(uwb_tip_pos_hw)
            self.prev_ts_micros = vals[13]
            return (tip_comp_x, tip_comp_y, False)

        # Extract force values from packet (each IMU has force at idx+? in your stride)
        offset = 6
        stride = 8
        has_predicted = False

        # Gather forces for this packet
        forces = []
        for i in range(10):
            idx = offset + (i * stride)
            if idx + 7 >= len(vals): break
            # force is at idx+6 in your earlier packet pattern (qx,qy,qz,qw,ax,ay,az,force,ts)
            f = vals[idx+6]  # adjust if indexing differs
            forces.append(float(f))

        # compute contact metric
        if len(forces) >= FORCE_CONFIG['min_force_samples_for_decision']:
            contact_metric = max(forces) if FORCE_CONFIG['use_max_force'] else (sum(forces)/len(forces))
        else:
            contact_metric = 0.0

        # short smoothing history
        self._force_history.append(contact_metric)
        if len(self._force_history) > 5:
            self._force_history.pop(0)
        smooth_force = sum(self._force_history)/len(self._force_history)

        # Hysteresis / debounce decision
        if smooth_force >= FORCE_CONFIG['contact_on_thresh']:
            self._contact_on_counter += 1
            self._contact_off_counter = 0
        elif smooth_force <= FORCE_CONFIG['contact_off_thresh']:
            self._contact_off_counter += 1
            self._contact_on_counter = 0

        if self._contact_on_counter >= FORCE_CONFIG['contact_on_count']:
            # go to contact state
            if not self.contact_state:
                # pen just touched -> do any on-contact initialization here
                # e.g. apply ZUPT and reset small velocities
                if self.kf is not None:
                    self.kf.apply_zupt()
                    self.kf_comp.apply_zupt()
                # record time if needed, e.g. self.pen_down_ts = current packet ts
            self.contact_state = True
            # keep counters bounded
            self._contact_on_counter = FORCE_CONFIG['contact_on_count']
        elif self._contact_off_counter >= FORCE_CONFIG['contact_off_count']:
            # go to no-contact
            if self.contact_state:
                # pen just lifted -> optional cleanup
                # add nan to path to break stroke
                if self.is_drawing:
                    self.path_x.append(np.nan)
                    self.path_y.append(np.nan)
                    self.path_comp_fused_x.append(np.nan)
                    self.path_comp_fused_y.append(np.nan)
                    self.is_drawing = False
            self.contact_state = False
            self._contact_off_counter = FORCE_CONFIG['contact_off_count']

        
        # Batched IMU processing
        for i in range(10):
            idx = offset + (i * stride)
            if idx + 7 >= len(vals): break
            qx, qy, qz, qw = vals[idx:idx+4]
            ax, ay, az = vals[idx+4:idx+7]
            ts_micros = int(vals[idx+7])

            dt = (ts_micros - self.prev_ts_micros) / 1_000_000.0
            self.prev_ts_micros = ts_micros
            if dt <= 0 or dt > 0.2: dt = 0.01

            q = Quaternion(qw, qx, qy, qz)
            lin_acc = q.rotate(np.array([ax, ay, az]))
            limit = TRACKER_CONFIG['max_accel']
            input_acc = np.clip(lin_acc[:2], -limit, limit)
            if np.linalg.norm(input_acc) < TRACKER_CONFIG['accel_deadband']: input_acc[:] = 0.0

            # ZUPT
            self.accel_magnitudes.append(np.linalg.norm(input_acc))
            if len(self.accel_magnitudes) > 10: self.accel_magnitudes.pop(0)
            is_stationary = False
            if len(self.accel_magnitudes) == 10 and np.mean(self.accel_magnitudes) < TRACKER_CONFIG['stationary_thresh']:
                is_stationary = True

            self.kf.predict(input_acc[0], input_acc[1], dt)
            self.kf_comp.predict(input_acc[0], input_acc[1], dt)

            self.kf.x[2] *= TRACKER_CONFIG['friction']
            self.kf.x[3] *= TRACKER_CONFIG['friction']
            self.kf_comp.x[2] *= TRACKER_CONFIG['friction']
            self.kf_comp.x[3] *= TRACKER_CONFIG['friction']

            if is_stationary: 
                self.kf.apply_zupt()
                self.kf_comp.apply_zupt()

            has_predicted = True
            
            # --- RECORD INTERMEDIATE PATHS ---
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


        # --- 7. UPDATE STEP (modified to use contact_state) ---
        if has_predicted:
            # Choose R scales depending on contact confidence
            # For hardware (kf), if contact -> trust UWB more (smaller R)
            if self.contact_state:
                R_override_hw = np.diag([TRACKER_CONFIG['uwb_noise_std']**2, TRACKER_CONFIG['uwb_noise_std']**2]) * FORCE_CONFIG['R_contact_scale']
                R_override_comp = R_override_hw.copy()
            else:
                R_override_hw = np.diag([TRACKER_CONFIG['uwb_noise_std']**2, TRACKER_CONFIG['uwb_noise_std']**2]) * FORCE_CONFIG['R_no_contact_scale']
                R_override_comp = R_override_hw.copy()

            # Update Hardware Filter (with adaptive R)
            pred_pos = self.kf.x[:2]
            innovation = np.linalg.norm(self.prev_uwb_tip - pred_pos)
            if innovation > TRACKER_CONFIG['uwb_jump_thresh'] and not self.contact_state:
                # reject UWB jump while not contacting
                self.consecutive_rejects += 1
                if self.consecutive_rejects >= 5:
                    self.kf.reset_state(self.prev_uwb_tip)
                    self.consecutive_rejects = 0
            else:
                # Use R_override to control how much we trust the UWB measurement
                self.kf.update(self.prev_uwb_tip, R_override=R_override_hw)
                self.consecutive_rejects = 0

            # Update Computed Filter (with adaptive R)
            pred_pos = self.kf_comp.x[:2]
            innovation = np.linalg.norm(self.prev_uwb_comp_tip - pred_pos)
            if innovation > TRACKER_CONFIG['uwb_jump_thresh'] and not self.contact_state:
                self.consecutive_rejects_comp += 1
                if self.consecutive_rejects_comp >= 5:
                    self.kf_comp.reset_state(self.prev_uwb_comp_tip)
                    self.consecutive_rejects_comp = 0
            else:
                self.kf_comp.update(self.prev_uwb_comp_tip, R_override=R_override_comp)
                self.consecutive_rejects_comp = 0

        # --- 8. FINALIZE (Return Data for Webserver) ---
        final_x = self.kf_comp.x[0]
        final_y = self.kf_comp.x[1]
        
        # Determine Status based on bounds (used by PrototypeManager)
        is_drawing_now = self.is_in_bounds(final_x, final_y)
                        
        return (final_x, final_y, is_drawing_now)


# ==========================================
# TESTING BLOCK (Run Directly)
# ==========================================
if __name__ == "__main__":
    import serial
    import matplotlib.pyplot as plt
    import signal

    os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'
    
    # Testing Config (Override if needed, else uses TRACKER_CONFIG)
    SERIAL_PORT = 'COM3'
    BAUD_RATE = 115200

    is_running = True

    def signal_handler(sig, frame):
        global is_running
        print("\nStopping capture...")
        is_running = False

    signal.signal(signal.SIGINT, signal_handler)
    
    tracker = StrokeTracker()
    print(f"Opening {SERIAL_PORT}...")
    
    # Setup Plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    
    line_raw, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.4, linewidth=1.0, color="#E6AC3F", label='Hardware UWB')
    line_comp, = ax.plot([], [], linestyle=':', marker='o', markersize=2, alpha=0.6, linewidth=1.0, color="#E63F3F", label='NLLS Trilateration')
    line_fused, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#3F92E6", label='Fused (Hardware)')
    line_fused_comp, = ax.plot([], [], linestyle='-', linewidth=1.25, color="#2A2A94", label='Fused (Computed)')

    ax.set_title("Stroke Processor Testing")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.flushInput()
        print("System Ready. Draw on the wall.")
        
        while is_running:
            line = ser.readline()
            if line:
                # We ignore return value here as we use internal lists for plotting
                tracker.process_packet(line) 

                # Update Plots
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
        if ser and ser.is_open:
            ser.close()
        print("Test finished.")
        plt.ioff()
        plt.show()