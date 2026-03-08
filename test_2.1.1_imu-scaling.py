import time
import os
import numpy as np
from tv1_unpacker import SerialStreamer

# --- CONFIG ---
streamer = SerialStreamer(port='COM3', baud=115200)

# --- STATE ---
latest_sample = None
last_print_time = 0
display_interval = 0.1 # Update screen every 100ms (10fps)

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Step 1: Scaling ")
time.sleep(1)

try:
    while True:
        # ---------------------------------------------------------
        # 1. CONSUME ALL DATA (FAST)
        # ---------------------------------------------------------
        packets = streamer.read_new_packets()
        
        for pkt in packets:
            if pkt['type'] == 'IMU':
                # Just update the variable. Do NOT print here.
                # Do NOT sleep here.
                latest_sample = pkt['samples'][-1]
                latest_packet_id = pkt['id']

        # ---------------------------------------------------------
        # 2. UPDATE DISPLAY (THROTTLED)
        # ---------------------------------------------------------
        if latest_sample and (time.time() - last_print_time > display_interval):
            
            # Unpack the latest snapshot
            ax, ay, az = latest_sample['acc']
            qx, qy, qz, qw = latest_sample['quat']
            force = latest_sample['force']
            
            # Calc Magnitudes
            acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
            quat_mag = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            
            # Print
            clear_screen()
            print(f"=== STEP 1: SCALING (REAL-TIME) ===")
            print(f"Packet ID: {latest_packet_id} ")
            print("-" * 30)
            
            print(f"1. LINEAR ACCELERATION")
            print(f"   Values:   X={ax:.3f}  Y={ay:.3f}  Z={az:.3f}")
            print(f"   Magnitude: {acc_mag:.3f}")
            if acc_mag < 0.2:
                print("   [PASS] Near Zero (Stationary)")
            else:
                print("   [WARN] Moving or Noisy")
            
            print("-" * 30)
            print(f"2. QUATERNION")
            print(f"   Values:   x={qx:.2f} y={qy:.2f} z={qz:.2f} w={qw:.2f}")
            print(f"   Magnitude: {quat_mag:.4f}")
            if 0.98 < quat_mag < 1.02:
                print("   [PASS] Valid Rotation")
            else:
                print("   [FAIL] Math Error (Check Q_SCALE)")

            print("-" * 30)
            print(f"3. FORCE SENSOR (Baseline ~2.9)")
            print(f"   Value: {force:.2f}")
            if force > 3:
                print("   [DETECTED] Pressure Applied")
            else:
                print("   [IDLE] No Pressure")
            
            last_print_time = time.time()

except KeyboardInterrupt:
    print("\nStopping...")
    streamer.close()