import os
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'

import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import signal

# ==============================================================================
# STRIPPED DOWN UWB COORDINATES
# ==============================================================================
# ==============================================================================
# CONFIGURATION (Only what is needed for Solver)
# ==============================================================================
plt.ion()
TRACKER_CONFIG = {
    # The physical location of the UWB anchors in meters [X, Y, Z]
    'anchors': np.array([
        [1.41, 0.00, 0.00],  # Anchor 0: BOTTOM RIGHT
        [1.41, 1.35, 0.00],  # Anchor 1: TOP RIGHT
        [0.00, 1.35, 0.00]   # Anchor 2: TOP LEFT
    ]),
}

# ==============================================================================
# PART D: THE TRILATERATION SOLVER
# ==============================================================================
def trilaterate_wls(distances, anchors, variances, last_known_pos=None):
    """
    Input: 3 Distances, 3 Anchor Coordinates
    Output: The Calculated [X, Y, Z] position
    """
    distances = np.array(distances)
    variances = np.array(variances)
    
    # 1. Filter invalid data (distance must be > 0)
    valid_mask = distances > 0
    if np.sum(valid_mask) < 3:
        return None, None # Need at least 3 anchors to solve 3D

    active_anchors = anchors[valid_mask]
    active_dists = distances[valid_mask]
    active_vars = variances[valid_mask]

    # 2. Calculate Weights based on variance (Higher variance = Lower trust)
    weights = 1.0 / (active_vars + 1e-6)
    weights = weights / np.mean(weights)

    # 3. Define the Cost Function (The "Error" we want to minimize)
    #    Error = (Calculated_Distance - Measured_Distance) * Weight
    def residuals(x, anchors, measured_dists, w):
        estimated_dists = np.linalg.norm(anchors - x, axis=1)
        error = estimated_dists - measured_dists
        return error * np.sqrt(w)

    # 4. Initial Guess (Start searching from the average of anchors or last pos)
    if last_known_pos is not None:
        x0 = last_known_pos
    else:
        x0 = np.mean(active_anchors, axis=0)

    # 5. Run the Least Squares Solver
    bounds_min = [-10.0, -10.0, -0.05]
    bounds_max = [ 10.0,  10.0,  0.30]

    try:
        res = least_squares(
            residuals, 
            x0, 
            bounds=(bounds_min, bounds_max),
            args=(active_anchors, active_dists, weights),
            loss='soft_l1',  # Ignores massive outliers
            f_scale=0.5
        )
        return res.x, None
    except Exception as e:
        print(f"Solver Failed: {e}")
        return None, None

# ==============================================================================
# MAIN LOGIC CLASS
# ==============================================================================
class StrokeTracker:
    def __init__(self):
        # History for plotting
        self.uwb_hw_x = []         # Reported by hardware
        self.uwb_hw_y = []
        self.uwb_computed_x = []   # Calculated by Python Solver
        self.uwb_computed_y = []
        self.last_pos = None
        self.last_dists = None     # For computation of trilateration
        
        self.sim_filter_x = 0.0
        self.sim_filter_y = 0.0
        self.sim_initialized = False
        self.warmup_counter = 0


    def process_packet(self, line_bytes):
        try:
            parts = line_bytes.decode('utf-8').strip().split(',')
            vals = [float(x) for x in parts]
        except:
            return None
        
        if len(vals) < 10: return None
        
        # Skip for the first packet
        if self.warmup_counter < 5:
            self.warmup_counter += 1
            return None
        

        # Extract Hardware Prototype Reported Position (Just for comparison)
        hw_x, hw_y = vals[0], vals[1]

        # Extract Raw Distances (for Manual Computed Position)
        dists = [vals[2], vals[3], vals[4]]

        # --- DYNAMIC VARIANCE (Velocity Based) ---
        current_variances = []
        base_variance = 0.05 
        
        # Handle the very first packet
        if self.last_dists is None:
            self.last_dists = dists
            
            current_variances = [base_variance, base_variance, base_variance]
        else:
            for i in range(3):
                current_dist = dists[i]
                prev_dist = self.last_dists[i]
                
                delta = abs(current_dist - prev_dist)
                
                if delta > 0.5: 
                    # 1. Variance High (Solver ignores this)
                    current_variances.append(10.0) 
                    
                else:
                    # 1. Variance Normal
                    current_variances.append(base_variance)
                    self.last_dists[i] = current_dist

        # Trilateration
        comp_pos_3d, _ = trilaterate_wls(dists, TRACKER_CONFIG['anchors'], current_variances, self.last_pos)
        
        
        if comp_pos_3d is not None:
            raw_x = comp_pos_3d[0]
            raw_y = comp_pos_3d[1]

            # --- APPLY THE "IMITATION" FILTER ---
            # This replicates the Arduino logic: uwbXf = 0.8*old + 0.2*new
            
            if not self.sim_initialized:
                # Initialize with the first raw value to prevent flying in from 0,0
                self.sim_filter_x = raw_x
                self.sim_filter_y = raw_y
                self.sim_initialized = True
            else:
                # EMA FILTER RATIO
                self.sim_filter_x = (0.80 * self.sim_filter_x) + (0.20 * raw_x)
                self.sim_filter_y = (0.80 * self.sim_filter_y) + (0.20 * raw_y)

            # OUTPUT OF THE MANUAL COMPUTED UWB
            self.uwb_computed_x.append(self.sim_filter_x)
            self.uwb_computed_y.append(self.sim_filter_y)
        
        # OUTPUT OF THE HARDWARE PROTOTYPE UWB
        self.uwb_hw_x.append(hw_x)
        self.uwb_hw_y.append(hw_y)



# ==============================================================================
# TESTING (VISUALIZATION)
# ==============================================================================
if __name__ == "__main__":
    # SERAIL COMMS SETUP
    SERIAL_PORT = 'COM2' # Change this to your port
    BAUD_RATE = 115200
    
    # Listener
    is_running = True
    def signal_handler(sig, frame):
        global is_running
        is_running = False
    signal.signal(signal.SIGINT, signal_handler)

    # Start Object Tracker
    tracker = StrokeTracker()
    print(f"Listening on {SERIAL_PORT}...")
    
    # MATPLOTLIB SETUP
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # UWB Tracked "Points" (Raw and Manual Computed)
    line_raw, = ax.plot([], [], linestyle=':', marker='o', markersize=4, alpha=0.4, color="#E6AC3F", label='Hardware UWB')
    line_comp, = ax.plot([], [], linestyle=':', marker='x', markersize=6, alpha=0.8, color="#E63F3F", label='Computed UWB')

    ax.set_title("Trilateration Reset: ")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.flushInput()
        print("System Ready... (waiting)")
        
        while is_running:
            line = ser.readline()
            if line:
                # Run the flow
                tracker.process_packet(line) 

                # Update Plot
                if tracker.uwb_hw_x:
                    line_raw.set_xdata(tracker.uwb_hw_x)
                    line_raw.set_ydata(tracker.uwb_hw_y)
                
                if tracker.uwb_computed_x:
                    line_comp.set_xdata(tracker.uwb_computed_x)
                    line_comp.set_ydata(tracker.uwb_computed_y)

                # Refresh chart
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

    except Exception as e:
        print(f"Stopped: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print("Test finished.")
        plt.ioff()
        plt.show()