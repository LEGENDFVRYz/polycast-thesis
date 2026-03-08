import time
import os
import numpy as np
from scipy.optimize import least_squares
from pyquaternion import Quaternion
from tv1_unpacker import SerialStreamer

# --- MASTER CONFIGURATION (Validated) ---
CONFIG = {
    # Validated in Phase 1
    'imu_threshold': 0.2,       # m/s^2 (Noise floor)
    
    # Validated in Phase 2
    'anchors': np.array([
        [1.41, 0.00, 0.00], 
        [1.41, 1.35, 0.00], 
        [0.00, 1.35, 0.00]
    ]),
    
    # New for Phase 3
    'max_human_speed': 3.0,     # m/s (Hard limit)
    'ghost_speed_limit': 0.1    # m/s (Limit when IMU is static)
}

streamer = SerialStreamer(port='COM3', baud=115200)

# --- STATE ---
last_valid_pos = None
last_valid_time = 0
is_imu_moving = False
imu_mag_debug = 0.0

def solve_position(dists, anchors):
    def residuals(guess, anchors, measured_dists):
        return np.linalg.norm(anchors - guess, axis=1) - measured_dists
    
    x0 = np.mean(anchors, axis=0)
    res = least_squares(residuals, x0, bounds=([-10, -10, -0.1], [10, 10, 0.1]), 
                        args=(anchors, dists), loss='soft_l1')
    return res.x

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Phase 2 Step 3: The Physics Gate")
time.sleep(1)

try:
    while True:
        packets = streamer.read_new_packets()
        latest_dists = None
        current_time = time.time()

        # 1. PROCESS PACKETS
        for pkt in packets:
            # --- IMU CHECK (The "Truth" about movement) ---
            if pkt['type'] == 'IMU':
                s = pkt['samples'][-1]
                # Scale & Rotate (Standard Mode)
                q = Quaternion(s['quat'][3], s['quat'][0], s['quat'][1], s['quat'][2])
                acc_world = q.rotate(np.array(s['acc']))
                
                # ZUPT Logic
                imu_mag_debug = np.linalg.norm(acc_world)
                if imu_mag_debug > CONFIG['imu_threshold']:
                    is_imu_moving = True
                else:
                    is_imu_moving = False

            # --- UWB READ ---
            if 'dists' in pkt:
                d = pkt['dists']
                if d[0] > 0 and d[1] > 0 and d[2] > 0:
                    latest_dists = np.array(d)

        # 2. APPLY LOGIC (Throttled Display)
        if latest_dists is not None and (current_time - last_valid_time > 0.1):
            
            # A. Geometric Solve
            raw_pos = solve_position(latest_dists, CONFIG['anchors'])
            
            # B. The Velocity Gate
            status = "VALID"
            rejected_reason = ""
            final_pos = raw_pos

            if last_valid_pos is not None:
                dt = current_time - last_valid_time
                dist_moved = np.linalg.norm(raw_pos - last_valid_pos)
                speed = dist_moved / dt

                # CHECK 1: The "Superman" Check (Hard Limit)
                if speed > CONFIG['max_human_speed']:
                    status = "REJECTED"
                    rejected_reason = f"Too Fast ({speed:.1f} m/s)"
                    final_pos = last_valid_pos # Teleport rejected; stay put

                # CHECK 2: The "Ghost" Check (IMU vs UWB)
                # If IMU says "I am still", but UWB says "I am moving 0.5m/s" -> Reject
                elif not is_imu_moving and speed > CONFIG['ghost_speed_limit']:
                    status = "REJECTED"
                    rejected_reason = "Ghost Jump (IMU Static)"
                    final_pos = last_valid_pos # Stay put

            # Update State
            if status == "VALID":
                last_valid_pos = final_pos
                last_valid_time = current_time

            # --- VISUALIZATION ---
            if time.time() % 0.1 < 0.02: # 10Hz Display
                clear_screen()
                print(f"=== PHYSICS GATE (IMU + UWB) ===")
                print("-" * 50)
                
                # IMU STATUS
                imu_stat = "MOVING" if is_imu_moving else "STATIONARY"
                print(f"IMU STATE:  [{imu_stat}]  (Mag: {imu_mag_debug:.2f})")
                print("-" * 50)
                
                # UWB STATUS
                print(f"UWB RAW POS: X={raw_pos[0]:.3f}  Y={raw_pos[1]:.3f}")
                print(f"DECISION:    {status} {rejected_reason}")
                print(f"FINAL POS:   X={final_pos[0]:.3f}  Y={final_pos[1]:.3f}")
                print("-" * 50)
                
                print("INSTRUCTIONS:")
                print("1. Hold pen STILL. UWB Pos should lock (Ghost Rejection).")
                print("2. Move pen NORMALLY. UWB should track.")
                print("3. Cover UWB antenna (create glitch). Logic should reject jumps.")

except KeyboardInterrupt:
    streamer.close()