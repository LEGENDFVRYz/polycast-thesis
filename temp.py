import serial
import time
import os
import platform

PORT = "COM3"
BAUD = 115200

# Optional: clear screen function
def clear_screen():
    os.system("cls" if platform.system() == "Windows" else "clear")

def parse_csv_line(line):
    # Split by comma
    parts = line.strip().split(",")
    if len(parts) != 12:  # 11 floats + 1 timestamp
        return None
    try:
        x, y, fx, fy = map(float, parts[0:4])
        qx, qy, qz, qw = map(float, parts[4:8])
        ax, ay, az = map(float, parts[8:11])
        timestamp = int(parts[11])
        return {
            "x": x, "y": y, "fx": fx, "fy": fy,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "ax": ax, "ay": ay, "az": az,
            "timestamp": timestamp
        }
    except ValueError:
        return None

def main():
    print(f"Opening {PORT} @ {BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    # time.sleep(2)

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        data = parse_csv_line(line)

        if data:
            # clear_screen()
            print("========== SENSOR PACKET ==========")
            for key, val in data.items():
                print(f"{key:10}: {val}")
            print("===================================")

        time.sleep(0.01)

if __name__ == "__main__":
    main()
