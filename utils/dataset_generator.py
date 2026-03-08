import serial
import csv
import time
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'   # <--- CHANGE THIS to your specific Port
BAUD_RATE = 115200     # Must match Serial.begin in ESP32
OUTPUT_FILE = 'Hline.csv'
IMU_SAMPLES_PER_PACKET = 10

def generate_header():
    """
    Creates the header row names based on the ESP32 output logic:
    1 set of UWB/Packet data + 10 sets of IMU data
    """
    # The base packet data
    header = ["filtered_x", "filtered_y", "dist0", "dist1", "dist2", "packet_ts"]
    
    # The looped IMU data (10 samples per row)
    for i in range(IMU_SAMPLES_PER_PACKET):
        suffix = f"_{i}" # e.g., qx_0, qx_1...
        # Updated order matches receiver: qx, qy, qz, qw, ax, ay, az, force, ts
        header.extend([
            f"qx{suffix}", f"qy{suffix}", f"qz{suffix}", f"qw{suffix}",
            f"ax{suffix}", f"ay{suffix}", f"az{suffix}", 
            f"force{suffix}", # Added force column
            f"ts{suffix}"
        ])
    return header

def main():
    # 1. Generate the header list
    csv_header = generate_header()
    print(f"Columns detected: {len(csv_header)}")

    # 2. Open Serial Connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # 3. Open CSV file and start logging
    try:
        with open(OUTPUT_FILE, mode='w', newline='') as f:
            writer = csv.writer(f)
            
            # Write the Header Row
            writer.writerow(csv_header)
            
            print(f"Logging started. Saving to '{OUTPUT_FILE}'")
            print("Press Ctrl+C to stop recording.\n")

            while True:
                if ser.in_waiting > 0:
                    try:
                        # Read a line from the serial port
                        line_bytes = ser.readline()
                        
                        # Decode bytes to string and strip whitespace (\r\n)
                        line_str = line_bytes.decode('utf-8', errors='ignore').strip()

                        # Skip empty lines or debug messages (optional check)
                        # We assume valid data lines contain commas
                        if ',' in line_str:
                            data_list = line_str.split(',')
                            
                            # Safety check: ensure columns match expectations (Optional)
                            if len(data_list) == len(csv_header):
                                writer.writerow(data_list)
                                print(f"Logged Packet: {data_list[5]} (Size: {len(data_list)})")
                            else:
                                print(f"Skipping malformed line (Got {len(data_list)} cols, expected {len(csv_header)})")
                        else:
                            # Print debug messages from ESP32 (like "Receiver ready...")
                            print(f"ESP32 Info: {line_str}")

                    except UnicodeDecodeError:
                        # Sometimes the first few bytes are garbage
                        pass
                        
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()