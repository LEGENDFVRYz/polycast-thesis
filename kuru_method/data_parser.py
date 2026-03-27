import serial
import time
import sys

# --- CONFIGURATION ---
# UPDATE THIS to match your ESP32 Receiver's Port
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

def parse_packet(line):
    """
    Parses a single line of CSV data from the Receiver.
    Expected Format:
    SEQ, BATCH_TS, UWB_TS, DIST0, DIST1, DIST2, DIST3, [QX, QY, QZ, QW, AX, AY, AZ, FORCE, TS] x 5
    Total Columns: 3 (Headers) + 4 (UWB) + (9 * 5) = 52 columns
    """
    try:
        # 1. Remove whitespace and split by comma
        parts = line.strip().split(',')
        
        # 2. Basic Validation
        # We expect exactly 52 data points based on your new 4-anchor loop structure
        if len(parts) != 52:
            print(f"[ESP32 MSG]: {line.strip()}")
            return

        # 3. Extract Headers (First 3 items)
        seq = int(parts[0])
        batch_ts = int(parts[1])
        uwb_ts = int(parts[2])

        # 4. Extract UWB Distances (Next 4 items)
        d0 = float(parts[3])
        d1 = float(parts[4])
        d2 = float(parts[5])
        d3 = float(parts[6])

        print("-" * 65)
        print(f"📦 PACKET SEQ: {seq} | BATCH TS: {batch_ts} | UWB TS: {uwb_ts}")
        print(f"📡 UWB ANCHORS | D0: {d0}m | D1: {d1}m | D2: {d2}m | D3: {d3}m")
        print("-" * 65)

        # 5. Extract the 5 Batched IMU Samples
        # Each sample has 9 values: QX, QY, QZ, QW, AX, AY, AZ, FORCE, TS
        samples_per_packet = 5
        data_per_sample = 9
        start_index = 7 # IMU Data now starts after the 3 headers + 4 distances

        for i in range(samples_per_packet):
            # Calculate where this specific sample starts in the list
            offset = start_index + (i * data_per_sample)
            
            # Extract values safely
            qx = float(parts[offset + 0])
            qy = float(parts[offset + 1])
            qz = float(parts[offset + 2])
            qw = float(parts[offset + 3])
            
            ax = float(parts[offset + 4])
            ay = float(parts[offset + 5])
            az = float(parts[offset + 6])
            
            force = float(parts[offset + 7])
            ts = int(parts[offset + 8])

            # Print readable output for verification
            print(f"   Sample {i+1}: T={ts} | Quat:({qx}, {qy}, {qz}, {qw}) | Acc: ({ax}, {ay}, {az}) | Force: {force}")

        print("✅ Packet Parsed Successfully\n")

    except ValueError as e:
        print(f"[ERROR] Parsing Error (Non-numeric data?): {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected Error: {e}")

def main():
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Give connection a moment to settle
        print(f"Connected to {SERIAL_PORT}!")
        print("Waiting for data... (Press Ctrl+C to stop)")

        while True:
            if ser.in_waiting > 0:
                # Read line, decode bytes to string (utf-8), strip newline characters
                line = ser.readline().decode('utf-8', errors='replace').strip()
                
                # Only process if line is not empty
                if line:
                    # Optional: Print raw line for debugging
                    # print(f"RAW: {line}") 
                    parse_packet(line)
                    
    except serial.SerialException:
        print(f"[FATAL] Could not open port {SERIAL_PORT}. Is another program (Arduino IDE) using it?")
    except KeyboardInterrupt:
        print("\nExiting program.")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit()

if __name__ == "__main__":
    main()