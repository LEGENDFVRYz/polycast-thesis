import serial
import time
import csv  # <-- Import the csv module

PORT = "COM3"
BAUD = 115200
OUTPUT_FILE = "sample_data.csv"  # <-- Define the output filename

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
    
    # These will be initialized inside the try block
    ser = None
    csv_file = None 

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Allow serial to initialize

        # Open the CSV file for writing
        # newline='' is important to prevent extra blank lines
        csv_file = open(OUTPUT_FILE, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        
        print(f"Saving data to {OUTPUT_FILE}. Press Ctrl+C to stop.")

        # Write the CSV header to the file
        header = ["x","y","filtered_x","filtered_y",
                  "qx","qy","qz","qw",
                  "ax","ay","az","timestamp"]
        csv_writer.writerow(header)

        while True:
            line = ser.readline().decode(errors='ignore').strip()
            data = parse_csv_line(line)
            if data:
                # Write the data row to the file
                csv_writer.writerow(data)
            
            # This sleep is optional but can be good to yield time
            # back to the OS if data isn't coming extremely fast.
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping data collection.")
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    finally:
        # This block ensures resources are closed no matter what
        if csv_file:
            csv_file.close()
            print(f"Closed {OUTPUT_FILE}.")
        if ser and ser.is_open:
            ser.close()
            print(f"Closed {PORT}.")

if __name__ == "__main__":
    main()  