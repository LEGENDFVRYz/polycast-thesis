import serial
import csv
import os
import time

# --- CONFIGURATION ---
SERIAL_PORT = "COM5"   # Check your Device Manager
BAUD_RATE = 115200

# 1. NAME YOUR CSV FILE HERE
FILE_NAME = "../datasets_v2/abcde.csv" 

# --- FOLDER SETUP ---
# Get the directory where THIS script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define the datasets folder path relative to the script
dataset_folder = os.path.join(script_dir, "datasets_v2")

# Create the 'datasets' folder if it doesn't exist
if not os.path.exists(dataset_folder):
    os.makedirs(dataset_folder)
    print(f"Created new folder: {dataset_folder}")

# Add .csv extension if missing
if not FILE_NAME.endswith(".csv"):
    FILE_NAME += ".csv"

# Full path to the file
output_path = os.path.join(dataset_folder, FILE_NAME)

# --- CSV HEADER ---
# 3 Headers + 4 UWB Distances + 5 IMU Samples (9 values each) = 52 Columns
header = ["Seq", "BatchTS", "UwbTS", "Dist0", "Dist1", "Dist2", "Dist3"]
for i in range(5):
    prefix = f"S{i}_"
    header.extend([
        f"{prefix}Qx", f"{prefix}Qy", f"{prefix}Qz", f"{prefix}Qw",
        f"{prefix}Ax", f"{prefix}Ay", f"{prefix}Az",
        f"{prefix}Force", f"{prefix}TS"
    ])

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Error connecting to serial: {e}")
        return

    print(f"Saving data to: {output_path}")
    print("Waiting for data... (Press Ctrl+C to stop)")

    with open(output_path, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        try:
            while True:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        # Only save valid data lines. 
                        # 52 columns means exactly 51 commas.
                        if line.count(',') == 51:
                            data = line.split(',')
                            writer.writerow(data)
                            
                            # data[0] = seq, data[3:7] = dist0 to dist3
                            print(f"\rSaved Seq: {data[0]} | UWB: {data[3]}m, {data[4]}m, {data[5]}m, {data[6]}m", end="")
                        else:
                            # Print debug messages (like "Receiver Ready") without saving
                            if line:
                                print(f"\nMsg: {line}")
                            
                    except ValueError:
                        pass
        except KeyboardInterrupt:
            print(f"\n\nStopping... File saved successfully.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()