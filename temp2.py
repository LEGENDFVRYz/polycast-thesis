import serial
import time

PORT = "COM3"
BAUD = 115200

def parse_csv_line(line):
    """Parse a comma-separated line into floats/ints, return as list."""
    parts = line.strip().split(",")
    if len(parts) != 12:  # 11 floats + 1 timestamp
        return None
    try:
        # Convert first 11 to float, last to int
        data = [float(p) for p in parts[:11]] + [int(parts[11])]
        return data
    except ValueError:
        return None

def main():
    print(f"Opening {PORT} @ {BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Allow serial to initialize

    # Print CSV header
    header = ["x","y","filtered_x","filtered_y",
              "qx","qy","qz","qw",
              "ax","ay","az","timestamp"]
    print(",".join(header))

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        data = parse_csv_line(line)
        if data:
            # Print raw CSV line
            print(",".join(map(str, data)))

        time.sleep(0.01)

if __name__ == "__main__":
    main()
