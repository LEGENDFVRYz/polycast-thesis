import time
import os
import numpy as np
from tv1_unpacker import SerialStreamer
from pyquaternion import Quaternion  # <--- NEW IMPORT

# --- CONFIG ---
streamer = SerialStreamer(port='COM3', baud=115200)

# --- STATE ---
latest_sample = None
last_print_time = 0
display_interval = 0.1 

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Step 2: World Frame Rotation")
print("Loading...")
time.sleep(1)

try:
    while True:
        # 1. FAST READ
        packets = streamer.read_new_packets()
        for pkt in packets:
            if pkt['type'] == 'IMU':
                latest_sample = pkt['samples'][-1]
                latest_packet_id = pkt['id']

        # 2. SLOW DISPLAY
        if latest_sample and (time.time() - last_print_time > display_interval):
            
            # --- UNPACK ---
            ax, ay, az = latest_sample['acc']
            qx, qy, qz, qw = latest_sample['quat']
            
            # --- STEP 2 LOGIC: FRAME ROTATION ---
            # A. Construct Quaternion (pyquaternion needs w, x, y, z)
            q = Quaternion(qw, qx, qy, qz)
            
            # B. Create Vector
            acc_body = np.array([ax, ay, az])
            
            # C. Rotate to World (STANDARD MODE)
            # This applies q * v * q_inv
            acc_world = q.rotate(acc_body)
            
            # --- VISUALIZATION ---
            clear_screen()
            print(f"=== STEP 2: FRAME ROTATION (Standard Mode) ===")
            print(f"Packet ID: {latest_packet_id}")
            print("-" * 50)
            
            # COMPARISON TABLE
            print(f"{'AXIS':<10} | {'RAW (Body)':<15} | {'ROTATED (World)':<15}")
            print("-" * 50)
            print(f"{'X':<10} | {ax:10.3f}      | {acc_world[0]:10.3f}")
            print(f"{'Y':<10} | {ay:10.3f}      | {acc_world[1]:10.3f}")
            print(f"{'Z':<10} | {az:10.3f}      | {acc_world[2]:10.3f}")
            print("-" * 50)

            # --- SANITY CHECK DIAGNOSTICS ---
            # Logic: If you are stationary, World X and Y should be near 0.
            # World Z should be near 9.8 (Gravity).
            
            is_flat = abs(acc_world[0]) < 1.0 and abs(acc_world[1]) < 1.0
            has_gravity = 8.0 < abs(acc_world[2]) < 11.0
            
            if is_flat and has_gravity:
                print("\n[ PASS ] Gravity correctly isolated on Z-Axis.")
            else:
                print("\n[ WAIT ] Aligning... or moving?")

            print("\nINSTRUCTIONS:")
            print("1. Hold pen STATIONARY.")
            print("2. Rotate your WRIST in all directions.")
            print("3. Verify 'ROTATED' columns stay constant while 'RAW' goes crazy.")

            last_print_time = time.time()

except KeyboardInterrupt:
    print("\nStopping...")
    streamer.close()