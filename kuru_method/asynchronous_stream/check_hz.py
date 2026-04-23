"""
check_hz.py — PolyCast Stream Frequency Monitor
===============================================
A simple diagnostic tool to calculate the real-time Hz
of the IMU and UWB streams, and display their ratio.
"""

import time
import sys
import config
from data_parser import AsyncDataParser

def main():
    # Initialize your existing async parser
    parser = AsyncDataParser(port=config.SERIAL_PORT, baud=config.BAUD_RATE)
    
    if not parser.connect():
        sys.exit(1)

    print(f"📡 Connected to {config.SERIAL_PORT}. Calculating real-time Hz...")
    print("Press Ctrl+C to stop.\n")

    imu_count = 0
    uwb_count = 0
    last_time = time.time()

    try:
        while True:
            pkt = parser.get_packet()
            
            if pkt == 'EOF':
                break
            if not pkt:
                continue

            # Increment counters based on packet type
            if pkt['type'] == 'imu':
                imu_count += 1
            elif pkt['type'] == 'uwb':
                uwb_count += 1

            # Check if 1 second has passed
            current_time = time.time()
            elapsed = current_time - last_time

            if elapsed >= 1.0:
                # Calculate Hz
                imu_hz = imu_count / elapsed
                uwb_hz = uwb_count / elapsed
                
                # Calculate Ratio (protect against division by zero)
                if uwb_hz > 0:
                    ratio = imu_hz / uwb_hz
                    ratio_str = f"{ratio:.1f}:1"
                else:
                    ratio_str = "N/A (No UWB)"

                # Print the live stats
                print(f"⏱️ [1s Window]  IMU: {imu_hz:>5.1f} Hz  |  UWB: {uwb_hz:>5.1f} Hz  |  Ratio: {ratio_str}")

                # Reset counters for the next second
                imu_count = 0
                uwb_count = 0
                last_time = current_time

    except KeyboardInterrupt:
        print("\n🛑 Stopped monitoring.")
    finally:
        parser.close()

if __name__ == '__main__':
    main()