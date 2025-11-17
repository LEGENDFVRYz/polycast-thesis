import serial
import re
import time
import os
import platform

# ---------------------------------------
# CONFIGURE YOUR SERIAL PORT HERE
# ---------------------------------------
PORT = "COM3"
BAUD = 115200

# --------------------
# ** UPDATED REGEX **
# Regex to parse the NEW ESP32 output format
# Format: From: ... | X: ... | Y: ... | F_X: ... | F_Y: ...
# --------------------
pattern = re.compile(
    r"From:\s*([0-9A-F:]+)\s*\|\s*"      # Group 1: MAC Address
    r"X:\s*(-?[0-9.]+)\s*\|\s*"         # Group 2: X
    r"Y:\s*(-?[0-9.]+)\s*\|\s*"         # Group 3: Y
    r"F_X:\s*(-?[0-9.]+)\s*\|\s*"       # Group 4: F_X
    r"F_Y:\s*(-?[0-9.]+)",              # Group 5: F_Y
    re.IGNORECASE
)

def clear_screen():
    """Clear terminal for Windows, Linux, Mac."""
    if platform.system() == "Windows":
        os.system("cls")
    else:
        os.system("clear")

def parse_line(line):
    """Extract structured data."""
    match = pattern.search(line)
    if not match:
        return None

    # Unpack the new groups
    mac, x_val, y_val, fx_val, fy_val = match.groups()

    # --------------------
    # ** UPDATED PARSING **
    # Return a dictionary with the new float values
    # --------------------
    return {
        "mac": mac.strip(),
        "x": float(x_val),
        "y": float(y_val),
        "f_x": float(fx_val),
        "f_y": float(fy_val)
    }

def main():
    print(f"Opening serial port {PORT} at {BAUD} baud...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Error: Could not open port {PORT}. {e}")
        print("Please check your port configuration.")
        return
        
    time.sleep(2) # Wait for serial to initialize

    print("Listening for ESP-NOW sensor packets...\n")

    try:
        while True:
            if ser.in_waiting:
                raw = ser.readline().decode(errors='ignore').strip()
                data = parse_line(raw)

                # --------------------
                # ** UPDATED PRINT LOGIC **
                # Display the new data structure
                # --------------------
                if data:
                    clear_screen()
                    print("===== ESP-NOW SENSOR PACKET =====")
                    print(f"From MAC : {data['mac']}")
                    # Using f-string formatting to align the numbers
                    print(f"X        : {data['x']:>7.3f}") 
                    print(f"Y        : {data['y']:>7.3f}")
                    print(f"F_X      : {data['f_x']:>7.3f}")
                    print(f"F_Y      : {data['f_y']:>7.3f}")
                    print("===================================")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()

if __name__ == "__main__":
    main()