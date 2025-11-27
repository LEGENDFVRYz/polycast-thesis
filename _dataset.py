import csv
import time
import serial
import sys

# Configuration
CSV_FILE = "_circle.csv"  # Ensure this matches your new filename
TIMESTAMP_COL = "packet_ts"
SERIAL_PORT = 'COM1'
BAUD_RATE = 115200

try:
    # Open serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Successfully connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

try:
    with open(CSV_FILE, newline="") as f:
        reader = csv.DictReader(f)
        
        # 1. Validation: Check if the new structure is detected
        if 'force_0' in reader.fieldnames:
            print("New CSV structure detected (Force columns found).")
        else:
            print("Warning: Standard CSV structure detected (No Force columns).")

        # 2. Send header first
        # This dynamically builds the header based on the ACTUAL file content
        header = ",".join(reader.fieldnames) + "\n"
        ser.write(header.encode())

        prev_ts = None

        # 3. Stream rows (Memory efficient: doesn't load whole file to RAM)
        for row in reader:
            try:
                current_ts = int(row[TIMESTAMP_COL])
            except ValueError:
                continue # Skip rows with bad timestamp data

            # Calculate real-time delay
            if prev_ts is not None:
                delta_us = current_ts - prev_ts
                
                # Handle potential negative delta (out of order packets)
                if delta_us > 0:
                    delta_s = delta_us / 1_000_000.0
                    time.sleep(delta_s)

            # Format row as CSV line including the new 'force' columns
            # This works because reader.fieldnames includes the new columns automatically
            line = ",".join(row[field] for field in reader.fieldnames) + "\n"

            # Send over serial
            ser.write(line.encode())
            
            # Print status every 100 lines (optional, for debugging)
            # print(f"Sent packet: {current_ts}") 

            prev_ts = current_ts

    print("CSV playback finished.")
    ser.close()

except FileNotFoundError:
    print(f"Error: The file '{CSV_FILE}' was not found.")
except KeyboardInterrupt:
    print("\nPlayback stopped by user.")
    ser.close()