import time
import numpy as np
from tv1_unpacker import SerialStreamer

# --- CONFIG ---
streamer = SerialStreamer(port='COM3', baud=115200)
SAMPLE_TIME = 10.0  # seconds to collect static data
PRINT_INTERVAL = 1.0
K_STD = 3  # threshold = mean + K*std

# --- STATE ---
acc_buffer = []
jerk_buffer = []
last_acc = None
last_ts = None
start_time = time.time()
last_print = start_time

print("[AUTO-CALIBRATOR] Place pen still for static calibration...")

try:
    while time.time() - start_time < SAMPLE_TIME:
        packets = streamer.read_new_packets()
        for pkt in packets:
            if pkt['type'] != 'IMU':
                continue
            sample = pkt['samples'][-1]
            acc = np.array(sample['acc'])
            ts = sample['ts']

            # dt in seconds
            if last_ts is None:
                dt = 0.0
            else:
                dt = max((ts - last_ts) * 1e-6, 1e-4)

            # store acceleration
            acc_buffer.append(np.linalg.norm(acc))

            # compute jerk in body frame
            if last_acc is None or dt == 0.0:
                jerk = 0.0
            else:
                jerk = np.linalg.norm(acc - last_acc) / dt
            jerk_buffer.append(jerk)

            last_acc = acc
            last_ts = ts

        # optional live print
        if time.time() - last_print > PRINT_INTERVAL:
            print(f"Collected {len(acc_buffer)} samples...")
            last_print = time.time()

    # --- COMPUTE THRESHOLDS ---
    acc_mean = np.mean(acc_buffer)
    acc_std = np.std(acc_buffer)
    jerk_mean = np.mean(jerk_buffer)
    jerk_std = np.std(jerk_buffer)

    zupt_acc_thresh = acc_mean + K_STD * acc_std
    zupt_jerk_thresh = jerk_mean + K_STD * jerk_std
    zupt_time_thresh = 0.08  # keep default

    print("\n[AUTO-CALIBRATION COMPLETE]")
    print(f"STATIC ACC MAG: mean={acc_mean:.3f}, std={acc_std:.3f}")
    print(f"STATIC JERK:   mean={jerk_mean:.3f}, std={jerk_std:.3f}")
    print("\n--- Suggested ZUPT thresholds ---")
    print(f"ZUPT_ACC_THRESH  = {zupt_acc_thresh:.3f}  # m/s²")
    print(f"ZUPT_JERK_THRESH = {zupt_jerk_thresh:.3f}  # m/s³")
    print(f"ZUPT_TIME_THRESH = {zupt_time_thresh:.3f}  # seconds")

except KeyboardInterrupt:
    print("Calibration interrupted.")

finally:
    streamer.close()
