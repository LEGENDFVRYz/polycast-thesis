import time
import os
import numpy as np
from collections import deque
from tv1_unpacker import SerialStreamer

# --- CONFIG ---
streamer = SerialStreamer(port='COM3', baud=115200)
WINDOW_SIZE = 50  # How many samples to use for jitter calc

# --- STATE ---
# Buffers to hold the last N distance readings for each anchor
anchors_history = {
    0: deque(maxlen=WINDOW_SIZE),
    1: deque(maxlen=WINDOW_SIZE),
    2: deque(maxlen=WINDOW_SIZE)
}

display_interval = 0.1
last_print_time = 0

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Phase 2 - Step 1: UWB Signal Integrity")
print("Collecting initial samples...")
time.sleep(1)

try:
    while True:
        packets = streamer.read_new_packets()
        latest_uwb = None

        for pkt in packets:
            # Look for UWB packets (ID 82 or derived)
            # Assuming 'pos' type carries the dists in your parser
            # Adjust if your parser labels them differently
            if pkt['type'] == 'UWB' or pkt['type'] == 'POS': 
                latest_uwb = pkt
                
                # Extract distances (assuming they are in 'dists' or 'samples')
                # Check your parser structure. Usually it's in 'dists' list [d0, d1, d2]
                if 'dists' in pkt:
                    dists = pkt['dists']
                    for i in range(3):
                        if dists[i] > 0: # Ignore 0/error readings
                            anchors_history[i].append(dists[i])

        # UPDATE DISPLAY
        if latest_uwb and (time.time() - last_print_time > display_interval):
            clear_screen()
            print(f"=== UWB SIGNAL HEALTH CHECK ===")
            print(f"Packet ID: {latest_uwb['id']}")
            print("-" * 60)
            print(f"{'ANCHOR':<10} | {'DIST (m)':<10} | {'JITTER (StdDev)':<20} | {'STATUS':<10}")
            print("-" * 60)

            for i in range(3):
                if len(anchors_history[i]) > 2:
                    curr_dist = anchors_history[i][-1]
                    # Standard Deviation represents "Noise/Jitter"
                    jitter = np.std(anchors_history[i])
                    
                    status = "OK"
                    if jitter > 0.05: status = "NOISY"     # > 5cm jitter
                    if jitter > 0.20: status = "UNSTABLE"  # > 20cm jitter
                    
                    print(f"Anchor {i}   | {curr_dist:.3f}      | {jitter:.4f} m             | {status}")
                else:
                    print(f"Anchor {i}   | WAITING... | ...                  | ...")
            
            print("-" * 60)
            print("INSTRUCTIONS:")
            print("1. Place pen STATIONARY on the board.")
            print("2. Check 'JITTER'. Ideally it should be < 0.0200 (2cm).")
            print("3. Move pen 10cm. Verify 'DIST' updates smoothly.")

            last_print_time = time.time()

except KeyboardInterrupt:
    print("\nStopping...")
    streamer.close()