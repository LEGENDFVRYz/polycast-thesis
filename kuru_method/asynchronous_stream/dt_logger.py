import argparse
import csv
import os
from typing import List, Tuple
import numpy as np

# ===============================================================
# CONFIGURATION (Set your defaults here)
# ===============================================================
# This line finds the folder where dt_logger.py actually lives
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Now we use ./ relative to the script's own folder
DEFAULT_CSV_PATH = os.path.join(BASE_DIR, "datasets_str_50hz", "middle-.csv")

USE_PLOT = True                                   
MAX_PACKETS = None                           
# ===============================================================

def compute_deltas(csv_path: str, max_packets: int | None = None) -> Tuple[List[float], List[float]]:
    """Parse the CSV and return lists of IMU and UWB Δt values in seconds."""
    imu_dts: List[float] = []
    uwb_dts: List[float] = []
    last_imu_ts: int | None = None
    last_uwb_ts: int | None = None
    count = 0

    if not os.path.exists(csv_path):
        print(f"ERROR: File not found at {os.path.abspath(csv_path)}")
        return [], []

    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if row[0] == 'I' and len(row) >= 11:
                try:
                    ts = int(row[10])
                except ValueError:
                    continue
                if last_imu_ts is not None:
                    dt = (ts - last_imu_ts) / 1_000_000.0
                    imu_dts.append(dt)
                last_imu_ts = ts
                count += 1
            elif row[0] == 'U' and len(row) >= 7:
                try:
                    ts = int(row[6])
                except ValueError:
                    continue
                if last_uwb_ts is not None:
                    dt = (ts - last_uwb_ts) / 1_000_000.0
                    uwb_dts.append(dt)
                last_uwb_ts = ts
                count += 1
            
            if max_packets is not None and count >= max_packets:
                break
    return imu_dts, uwb_dts

def print_stats(name: str, dts: List[float]) -> None:
    if not dts:
        print(f"{name}: no data")
        return
    arr = np.array(dts, dtype=float)
    print(f"\n{name} Δt (s) Statistics:")
    print(f"  count: {len(arr)}")
    print(f"  mean:  {arr.mean():.6f}")
    print(f"  std:   {arr.std(ddof=1):.6f}")
    print(f"  min:   {arr.min():.6f}")
    print(f"  max:   {arr.max():.6f}")
    print(f"  median:{np.median(arr):.6f}")

def maybe_plot(imu_dts: List[float], uwb_dts: List[float], csv_path: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping plots.")
        return
    
    plt.figure(figsize=(10, 4))
    bins = 50
    if imu_dts:
        plt.subplot(1, 2, 1)
        plt.hist(imu_dts, bins=bins, color='blue', alpha=0.7)
        plt.title('IMU Δt distribution')
        plt.xlabel('Δt (s)')
        plt.ylabel('Count')
    if uwb_dts:
        plt.subplot(1, 2, 2)
        plt.hist(uwb_dts, bins=bins, color='green', alpha=0.7)
        plt.title('UWB Δt distribution')
        plt.xlabel('Δt (s)')
        plt.ylabel('Count')
    plt.suptitle(f"Dataset: {os.path.basename(csv_path)}")
    plt.tight_layout()
    plt.show()

def main() -> None:
    # Use the hardcoded variables instead of argparse
    print(f"Analyzing: {DEFAULT_CSV_PATH}")
    
    imu_dts, uwb_dts = compute_deltas(DEFAULT_CSV_PATH, MAX_PACKETS)
    
    if not imu_dts and not uwb_dts:
        print("No data processed. Please check your file path or CSV format.")
        return

    print_stats('IMU', imu_dts)
    print_stats('UWB', uwb_dts)
    
    if USE_PLOT:
        maybe_plot(imu_dts, uwb_dts, DEFAULT_CSV_PATH)

if __name__ == '__main__':
    main()