import time
import os
import numpy as np
from tv1_unpacker import SerialStreamer
from pyquaternion import Quaternion

# --- CONFIG ---
streamer = SerialStreamer(port='COM3', baud=115200)

# --- TUNING ---
# ZUPT THRESHOLD: Any movement smaller than this is treated as noise.
# You saw noise of +/- 0.1. We set this to 0.2 to be safe.
NOISE_THRESHOLD = 0.2 

# --- STATE ---
latest_sample = None
last_print_time = 0
display_interval = 0.1 

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Step 3 & 4: Gravity Check + Noise Filter")
time.sleep(1)

try:
    while True:
        packets = streamer.read_new_packets()
        for pkt in packets:
            if pkt['type'] == 'IMU':
                latest_sample = pkt['samples'][-1]

        if latest_sample and (time.time() - last_print_time > display_interval):
            
            # 1. UNPACK & SCALE
            ax, ay, az = latest_sample['acc']
            qx, qy, qz, qw = latest_sample['quat']
            
            # 2. ROTATE (Standard Mode)
            q = Quaternion(qw, qx, qy, qz)
            acc_body = np.array([ax, ay, az])
            acc_world = q.rotate(acc_body)
            
            # 3. GRAVITY REMOVAL (Hardware handles this)
            # We just copy the value. We do NOT subtract 9.81.
            acc_linear = acc_world.copy()
            
            # 4. ZUPT (The Noise Filter)
            # Calculate total magnitude of movement
            mag = np.linalg.norm(acc_linear)
            
            clean_acc = np.array([0.0, 0.0, 0.0])
            status = ""

            if mag < NOISE_THRESHOLD:
                # CASE A: STATIONARY
                # Force everything to exactly 0.0
                clean_acc = np.array([0.0, 0.0, 0.0])
                status = "[ IDLE ] Silence"
            else:
                # CASE B: MOVING
                # Allow the data through
                clean_acc = acc_linear
                status = ">>> MOVING <<<"

            # --- VISUALIZATION ---
            clear_screen()
            print(f"=== PHASE 1 COMPLETE: PRE-CLEANING ===")
            print("-" * 50)
            print(f"{'AXIS':<10} | {'RAW':<10} | {'WORLD':<10} | {'CLEAN (Final)':<15}")
            print("-" * 50)
            print(f"X          | {ax:<10.2f} | {acc_world[0]:<10.2f} | {clean_acc[0]:<10.2f}")
            print(f"Y          | {ay:<10.2f} | {acc_world[1]:<10.2f} | {clean_acc[1]:<10.2f}")
            print(f"Z          | {az:<10.2f} | {acc_world[2]:<10.2f} | {clean_acc[2]:<10.2f}")
            print("-" * 50)
            print(f"Noise Threshold: {NOISE_THRESHOLD}")
            print(f"Current Mag:     {mag:.3f}")
            print(f"Status:          {status}")

            last_print_time = time.time()

except KeyboardInterrupt:
    streamer.close()