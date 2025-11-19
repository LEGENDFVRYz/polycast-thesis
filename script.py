import serial
import time
from scipy.spatial.transform import Rotation

# --- Serial Port Configuration ---
PORT = "COM3"
BAUD = 115200

# --- Adaptive Filter Configuration ---
STATIONARY_ALPHA = 0.005
MOVING_ALPHA = 0.3
MOTION_THRESHOLD = 0.15

# --- Fusion Configuration (NEW) ---
# This controls the 'strength' of the UWB's correction on the IMU.
# A small value (e.g., 0.01) 'trusts' the IMU more.
# A larger value (e.g., 0.05) 'trusts' the UWB more.
# This needs to be tuned! Start with 0.02
FUSION_FACTOR = 0.02
# ---

GRAVITY_VECTOR = [0, 0, 9.81]

class JitterFilter:
    """Implements an EMA low-pass filter with a DYNAMIC alpha."""
    def __init__(self):
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.is_initialized = False

    def filter(self, raw_x, raw_y, alpha):
        """Applies the EMA filter using the provided (dynamic) alpha."""
        if not self.is_initialized:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.is_initialized = True
        else:
            self.filtered_x = (alpha * raw_x) + (1.0 - alpha) * self.filtered_x
            self.filtered_y = (alpha * raw_y) + (1.0 - alpha) * self.filtered_y
            
        return self.filtered_x, self.filtered_y

class IMUProcessor:
    """
    Processes raw IMU data and fuses it with UWB data
    using a complementary filter.
    """
    def __init__(self):
        self.position = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.last_timestamp_ms = None
        self.linear_accel_magnitude = 0.0

    def process(self, timestamp_ms, accel_body, quat_body):
        """
        Updates the IMU's relative position estimate (Prediction step).
        
        :return: (float, float) The IMU's *predicted* position (x, y)
        """
        
        if self.last_timestamp_ms is None:
            self.last_timestamp_ms = timestamp_ms
            return self.position

        dt_ms = timestamp_ms - self.last_timestamp_ms
        if dt_ms < 0: dt_ms += 2**32
        self.last_timestamp_ms = timestamp_ms
        dt_s = dt_ms / 1000.0

        if dt_s <= 0 or dt_s > 0.5: # Ignore large time gaps or errors
            if dt_s > 0.5: self.last_timestamp_ms = None # Reset timestamp
            return self.position

        # 2. Remove Gravity
        r = Rotation.from_quat(quat_body)
        accel_world = r.apply(accel_body)
        linear_accel_world = accel_world - GRAVITY_VECTOR
        
        self.linear_accel_magnitude = (
            linear_accel_world[0]**2 + 
            linear_accel_world[1]**2 + 
            linear_accel_world[2]**2
        ) ** 0.5
        
        # 3. Double Integrate (Wall Plane X, Y)
        accel_x = linear_accel_world[0]
        accel_y = linear_accel_world[1]

        self.velocity[0] += accel_x * dt_s
        self.velocity[1] += accel_y * dt_s
        
        self.position[0] += self.velocity[0] * dt_s
        self.position[1] += self.velocity[1] * dt_s
        
        # This returned position is the *drifted* prediction
        return self.position[0], self.position[1]

    def fuse_with_uwb(self, uwb_x, uwb_y, fusion_factor):
        """
        Corrects the IMU's internal position with the UWB's
        absolute position. (Correction step)
        
        :return: (float, float) The new *fused* position (x, y)
        """
        
        # Calculate the error (drift) between IMU and UWB
        error_x = uwb_x - self.position[0]
        error_y = uwb_y - self.position[1]
        
        # Apply the 'nudge' (the complementary filter) to the IMU's
        # internal position state.
        self.position[0] += error_x * fusion_factor
        self.position[1] += error_y * fusion_factor
        
        # This is a good place to also dampen velocity,
        # preventing "velocity windup" from the correction.
        # This scales velocity down by the same factor.
        self.velocity[0] *= (1.0 - fusion_factor)
        self.velocity[1] *= (1.0 - fusion_factor)
        
        # Return the newly corrected (fused) position
        return self.position[0], self.position[1]

    def get_motion_state(self):
        """Returns the last calculated linear acceleration magnitude."""
        return self.linear_accel_magnitude

    # We no longer need reset_position, as fuse_with_uwb handles all corrections

def parse_csv_line(line):
    parts = line.strip().split(",")
    if len(parts) != 12: return None
    try:
        data = [float(p) for p in parts[:11]] + [int(parts[11])]
        return data
    except ValueError:
        return None

def main():
    print(f"Opening {PORT} @ {BAUD}")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port {PORT}: {e}")
        return
        
    time.sleep(2)

    uwb_filter = JitterFilter()
    imu_processor = IMUProcessor()
    
    print("Sensor fusion system initialized.")
    print(f"Config: FUSION_FACTOR={FUSION_FACTOR}, MOTION_THRESHOLD={MOTION_THRESHOLD}")

    header = [
        "raw_x", "raw_y",
        "filtered_uwb_x", "filtered_uwb_y",
        "fused_x", "fused_y", # <-- THIS IS YOUR FINAL STROKE COORDINATE
        "current_alpha", "accel_mag",
        "ax", "ay", "az", "timestamp"
    ]
    print(",".join(header))

    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line: continue
            data = parse_csv_line(line)
            
            if data:
                raw_x, raw_y = data[0], data[1]
                quat_body = [data[4], data[5], data[6], data[7]]
                accel_body = [data[8], data[9], data[10]]
                timestamp = data[11]

                # --- Step 1: PROCESS IMU (Prediction) ---
                # This moves the IMU position based on its last-known state
                # and the new accelerometer data. It *will* drift.
                imu_x, imu_y = imu_processor.process(timestamp, accel_body, quat_body)
                accel_mag = imu_processor.get_motion_state()

                # --- Step 2: PROCESS UWB (Measurement) ---
                # Get the stable, adaptively-filtered UWB position.
                if accel_mag < MOTION_THRESHOLD:
                    current_alpha = STATIONARY_ALPHA
                else:
                    current_alpha = MOVING_ALPHA
                
                filtered_x, filtered_y = uwb_filter.filter(raw_x, raw_y, current_alpha)

                # --- Step 3: FUSE (Correction) ---
                # We replace the "Temporary Fusion" block with this:
                # This 'nudges' the IMU's internal state (position and velocity)
                # towards the UWB's measured state.
                fused_x, fused_y = imu_processor.fuse_with_uwb(
                    filtered_x, 
                    filtered_y, 
                    FUSION_FACTOR
                )
                # ---

                # Prepare the full line of data for output
                output_data = [
                    raw_x, raw_y,
                    filtered_x, filtered_y,
                    fused_x, fused_y,  # <-- This is your final data!
                    current_alpha, accel_mag,
                    *accel_body,
                    timestamp
                ]
                
                formatted_line = ",".join(
                    [f"{v:.6f}" if isinstance(v, float) else str(v) for v in output_data]
                )

                # Print Output
                if accel_mag < MOTION_THRESHOLD:
                    # Stationary
                    print(f"S{formatted_line}")
                else:
                    # Moving
                    print(f"M{formatted_line}")
                
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            break
        except KeyboardInterrupt:
            print("\nExiting by user request.")
            break

    ser.close()
    print("Serial port closed.")

if __name__ == "__main__":
    main()