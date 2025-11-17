import serial
import re
import time
import os
import platform

PORT = "COM3"
BAUD = 115200

# NEW REGEX FOR ALL SENSOR FIELDS
pattern = re.compile(
    r"From:\s*([0-9A-F:]+)\s*\|\s*"
    r"X:\s*(-?[0-9.]+)\s*\|\s*"
    r"Y:\s*(-?[0-9.]+)\s*\|\s*"
    r"FX:\s*(-?[0-9.]+)\s*\|\s*"
    r"FY:\s*(-?[0-9.]+)\s*\|\s*"
    r"QX:\s*(-?[0-9.]+)\s*\|\s*"
    r"QY:\s*(-?[0-9.]+)\s*\|\s*"
    r"QZ:\s*(-?[0-9.]+)\s*\|\s*"
    r"QW:\s*(-?[0-9.]+)\s*\|\s*"
    r"AX:\s*(-?[0-9.]+)\s*\|\s*"
    r"AY:\s*(-?[0-9.]+)\s*\|\s*"
    r"AZ:\s*(-?[0-9.]+)\s*\|\s*"
    r"TS:\s*([0-9]+)",
    re.IGNORECASE
)

def clear_screen():
    os.system("cls" if platform.system() == "Windows" else "clear")

def parse_line(line):
    match = pattern.search(line)
    if not match:
        return None

    (mac, x, y, fx, fy,
     qx, qy, qz, qw,
     ax, ay, az,
     ts) = match.groups()

    return {
        "mac": mac,
        "x": float(x),
        "y": float(y),
        "fx": float(fx),
        "fy": float(fy),
        "qx": float(qx),
        "qy": float(qy),
        "qz": float(qz),
        "qw": float(qw),
        "ax": float(ax),
        "ay": float(ay),
        "az": float(az),
        "timestamp": int(ts)
    }

def main():
    print(f"Opening {PORT} @ {BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    # time.sleep(2)

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        data = parse_line(line)

        if data:
            # clear_screen()
            print("========== SENSOR PACKET ==========")
            for key, val in data.items():
                print(f"{key:10}: {val}")
            print("===================================")

        time.sleep(0.01)

if __name__ == "__main__":
    main()
