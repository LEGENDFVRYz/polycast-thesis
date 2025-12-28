import time
import os
import numpy as np
from scipy.optimize import least_squares
from tv1_unpacker import SerialStreamer

# --- CONFIGURATION ---
streamer = SerialStreamer(port='COM3', baud=115200)

# [CRITICAL] VERIFY THESE MATCH YOUR PHYSICAL WALL SETUP!
# Based on your previous code:
ANCHORS = np.array([
    [0.90, 0.90, 0.00],  # Anchor 0
    [0.00, 0.90, 0.00],  # Anchor 1
    [0.00, 0.00, 0.00]   # Anchor 2
])

# --- TRILATERATION SOLVER ---
def solve_position(dists, anchors):
    # Function to minimize: (Distance_Calc - Distance_Measured)^2
    def residuals(guess, anchors, measured_dists):
        # Calculate distance from 'guess' to each anchor
        calc_dists = np.linalg.norm(anchors - guess, axis=1)
        return calc_dists - measured_dists

    # Initial Guess: Center of the board
    x0 = np.mean(anchors, axis=0)
    
    # Run Optimizer
    # We constrain Z to be close to 0 (Planar writing)
    result = least_squares(
        residuals, 
        x0, 
        bounds=([-10, -10, -0.1], [10, 10, 0.1]), 
        args=(anchors, dists),
        loss='soft_l1'
    )
    return result.x, result.cost # Cost = Sum of squared errors

# --- MAIN LOOP ---
display_interval = 0.1
last_print_time = 0

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

print("[TEST] Phase 2 - Step 2: Geometric Solve")
print(f"Using Anchors:\n{ANCHORS}")
time.sleep(2)

try:
    while True:
        packets = streamer.read_new_packets()
        latest_dists = None

        # 1. READ DISTANCES
        for pkt in packets:
            if 'dists' in pkt: # Adjust based on your parser key
                d = pkt['dists']
                # Ensure we have 3 valid distances
                if d[0] > 0 and d[1] > 0 and d[2] > 0:
                    latest_dists = np.array([d[0], d[1], d[2]])

        # 2. SOLVE & DISPLAY
        if latest_dists is not None and (time.time() - last_print_time > display_interval):
            
            # Run Solver
            pos, error_score = solve_position(latest_dists, ANCHORS)
            
            clear_screen()
            print(f"=== UWB GEOMETRY CHECK ===")
            print("-" * 50)
            print(f"INPUT DISTANCES: {latest_dists[0]:.2f}, {latest_dists[1]:.2f}, {latest_dists[2]:.2f}")
            print("-" * 50)
            print(f"CALCULATED POS : X={pos[0]:.3f}  Y={pos[1]:.3f}")
            print("-" * 50)
            
            # --- THE JUDGMENT ---
            # Error Score is the sum of squared residuals.
            # < 0.01: Perfect intersection
            # > 0.10: The anchors are wrong (Circles don't touch)
            
            quality = "PERFECT"
            if error_score > 0.05: quality = "OKAY (Noisy)"
            if error_score > 0.20: quality = "BAD (Check Anchors!)"
            
            print(f"GEOMETRIC ERROR: {error_score:.4f}  [{quality}]")
            print("-" * 50)
            
            # Bounds Check
            if 0 <= pos[0] <= 1.41 and 0 <= pos[1] <= 1.35:
                print("STATUS: Inside Board Bounds")
            else:
                print("STATUS: >> OUT OF BOUNDS <<")

            last_print_time = time.time()

except KeyboardInterrupt:
    streamer.close()