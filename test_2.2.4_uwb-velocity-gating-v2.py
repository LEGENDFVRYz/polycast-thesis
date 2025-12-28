import time
import os
import numpy as np
from scipy.optimize import least_squares
from pyquaternion import Quaternion
from tv1_unpacker import SerialStreamer

# --- MASTER CONFIGURATION ---
CONFIG = {
    'imu_threshold': 0.2,       # m/s^2 (Noise floor)
    'anchors': np.array([
        [0.90, 0.90, 0.00], 
        [0.00, 0.90, 0.00], 
        [0.00, 0.00, 0.00]
    ]),
    
    # --- UPDATED TUNING ---
    'max_human_speed': 6.0,      # INCREASED: 3.0 -> 6.0 (Allows fast scribbles)
    'ghost_speed_limit': 0.1,    # m/s (Limit when TRULY static)
    'motion_cooldown': 0.3       # NEW: Keep "Moving" status for 0.3s after accel stops
}

streamer = SerialStreamer(port='COM3', baud=115200)

# --- STATE ---
last_valid_pos = None
last_valid_time = 0

# Motion Memory State
last_imu_spike_time = 0          # When was the last time we felt force?
is_moving_extended = False       # True if within the cooldown window

def solve_position(dists, anchors):
    def residuals(guess, anchors, measured_dists):
        return np.linalg.norm(anchors - guess, axis=1) - measured_dists
    
    x0 = np.mean(anchors, axis=0)
    res = least_squares(residuals, x0, bounds=([-10, -10, -0.1], [10, 10, 0.1]), 
                        args=(anchors, dists), loss='soft_l1')
    return res.x

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Phase 2 Step 3: Physics Gate (Fixed for Speed)")
time.sleep(1)

try:
    while True:
        packets = streamer.read_new_packets()
        latest_dists = None
        current_time = time.time()

        # 1. PROCESS PACKETS
        for pkt in packets:
            # --- IMU CHECK ---
            if pkt['type'] == 'IMU':
                s = pkt['samples'][-1]
                q = Quaternion(s['quat'][3], s['quat'][0], s['quat'][1], s['quat'][2])
                acc_world = q.rotate(np.array(s['acc']))
                
                # ZUPT Logic
                imu_mag = np.linalg.norm(acc_world)
                
                # If we feel force, update the timestamp
                if imu_mag > CONFIG['imu_threshold']:
                    last_imu_spike_time = current_time

            # --- UWB READ ---
            if 'dists' in pkt:
                d = pkt['dists']
                if d[0] > 0 and d[1] > 0 and d[2] > 0:
                    latest_dists = np.array(d)

        # 2. UPDATE MOTION STATE (The "Memory")
        # We are "Moving" if we felt force recently (within cooldown)
        if (current_time - last_imu_spike_time) < CONFIG['motion_cooldown']:
            is_moving_extended = True
        else:
            is_moving_extended = False

        # 3. APPLY GATE LOGIC
        if latest_dists is not None and (current_time - last_valid_time > 0.1):
            
            # A. Geometric Solve
            raw_pos = solve_position(latest_dists, CONFIG['anchors'])
            
            # B. The Velocity Gate
            status = "VALID"
            rejected_reason = ""
            final_pos = raw_pos
            speed = 0.0

            if last_valid_pos is not None:
                dt = current_time - last_valid_time
                dist_moved = np.linalg.norm(raw_pos - last_valid_pos)
                speed = dist_moved / dt

                # CHECK 1: The "Superman" Check (Hard Limit)
                if speed > CONFIG['max_human_speed']:
                    status = "REJECTED"
                    rejected_reason = f"Too Fast ({speed:.1f} m/s)"
                    final_pos = last_valid_pos 

                # CHECK 2: The "Ghost" Check (Revised)
                # Only reject if we have been COMPLETELY STILL for > 0.5 seconds
                elif not is_moving_extended and speed > CONFIG['ghost_speed_limit']:
                    status = "REJECTED"
                    rejected_reason = "Ghost Jump (IMU Static)"
                    final_pos = last_valid_pos 

            # Update State
            if status == "VALID":
                last_valid_pos = final_pos
                last_valid_time = current_time

            # --- VISUALIZATION ---
            if time.time() % 0.1 < 0.02: 
                clear_screen()
                print(f"=== PHYSICS GATE (Fixed) ===")
                print("-" * 50)
                
                # IMU STATUS
                imu_stat = "MOVING (Active)" if is_moving_extended else "STATIONARY (Idle)"
                print(f"IMU STATE:   {imu_stat}")
                print(f"Last Force:  {(current_time - last_imu_spike_time):.2f}s ago")
                print("-" * 50)
                
                # UWB STATUS
                print(f"UWB RAW POS: X={raw_pos[0]:.3f}  Y={raw_pos[1]:.3f}")
                print(f"SPEED:       {speed:.2f} m/s")
                print(f"DECISION:    {status} {rejected_reason}")
                print("-" * 50)
                
                print("INSTRUCTIONS:")
                print("1. WAVE FAST: Should NOT reject (IMU Memory active).")
                print("2. STOP: Wait 0.5s. Status becomes STATIONARY.")
                print("3. GHOST TEST: When Stationary, UWB jumps should be REJECTED.")

except KeyboardInterrupt:
    streamer.close()