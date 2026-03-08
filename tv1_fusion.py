import numpy as np

class FusionEngine:
    def __init__(self, k_gain=0.05, vel_decay=0.98):
        self.K_GAIN = k_gain
        self.VEL_DECAY = vel_decay
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.last_imu_time = None

    def predict_imu(self, clean_acc, timestamp):
        """
        Input: clean_acc (np.array [ax, ay, az]), timestamp (seconds)
        """
        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            return self.pos

        dt = timestamp - self.last_imu_time
        self.last_imu_time = timestamp

        if dt <= 0 or dt > 1.0: return self.pos

        # --- THE FIX IS HERE ---
        # Take only the first 2 elements (X and Y) from the 3D acceleration
        acc_2d = clean_acc[:2] 

        # 1. Integrate Acceleration -> Velocity
        self.vel += acc_2d * dt

        # 2. Apply High-Pass Filter
        self.vel *= self.VEL_DECAY

        # 3. Integrate Velocity -> Position
        self.pos += self.vel * dt
        
        return self.pos

    def correct_uwb(self, clean_uwb_pos, status):
        if status == "VALID":
            error = clean_uwb_pos - self.pos
            self.pos += error * self.K_GAIN
        return self.pos