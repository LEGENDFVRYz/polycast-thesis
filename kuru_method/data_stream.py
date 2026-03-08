import os
import serial
import time
from preprocessor import UWBPreprocessor, IMUPreprocessor

class DataStream:
    def __init__(self, port, baud_rate, dataset_filename=""):
        self.port = port
        self.baud_rate = baud_rate
        self.dataset_filename = dataset_filename
        
        # Determine mode based on whether a filename was provided
        self.mode = "csv" if dataset_filename.strip() else "live"
        
        self.ser = None
        self.csv_file = None
        
        # Initialize the cleaners
        self.uwb_cleaner = UWBPreprocessor()
        self.imu_cleaner = IMUPreprocessor()

    def connect(self):
        if self.mode == "live":
            try:
                self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
                time.sleep(2) # Wait for Arduino reset
                print(f"✅ Connected to LIVE stream on {self.port}")
                return True
            except Exception as e:
                print(f"❌ Connection Failed: {e}")
                return False
        else:
            try:
                # 1. Get the directory where data_stream.py is located
                script_dir = os.path.dirname(os.path.abspath(__file__))
                
                # 2. Join it with your dataset filename
                full_path = os.path.join(script_dir, self.dataset_filename)
                
                # 3. Open the file using the absolute path
                self.csv_file = open(full_path, 'r')
                # -----------------------------------
                
                # --- AUTO HEADER DETECTION ---
                first_pos = self.csv_file.tell()
                first_line = self.csv_file.readline()
                try:
                    float(first_line.split(',')[0])
                    self.csv_file.seek(first_pos)
                except ValueError:
                    pass 
                # -----------------------------
                
                print(f"✅ Opened CSV dataset: {full_path}")
                return True
            except Exception as e:
                print(f"❌ Failed to open CSV: {e}")
                return False

    def data_available(self):
        """Helper to check if there is data in the serial buffer."""
        if self.mode == "live":
            return self.ser and self.ser.in_waiting > 0
        return False

    def get_packet(self):
        """Reads one line, parses it, cleans it, and returns structured data."""
        line = ""
        
        # 1. READ LINE based on mode
        if self.mode == "live":
            if not self.ser or self.ser.in_waiting == 0:
                return None
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
            except Exception:
                return None
        else:
            if not self.csv_file:
                return None
            line = self.csv_file.readline().strip()
            if not line:
                return "EOF" # Signal end of file

        if not line: 
            return None

        # 2. PARSE AND CLEAN
        try:
            parts = line.split(',')
            
            # Validation: 3 UWB + (9 IMU * 5 samples) = 48 columns
            if len(parts) != 48:
                return None 

            # --- PARSE & CLEAN UWB ---
            raw_d0 = float(parts[0])
            raw_d1 = float(parts[1])
            raw_d2 = float(parts[2]) 
            
            # The cleaner returns the median-filtered distances
            d0, d1, d2 = self.uwb_cleaner.process(raw_d0, raw_d1, raw_d2)

            # --- PARSE & CLEAN IMU BATCH ---
            imu_batch = []
            start_idx = 3
            for i in range(5):
                off = start_idx + (i * 9)
                
                qx, qy, qz, qw = float(parts[off]), float(parts[off+1]), float(parts[off+2]), float(parts[off+3])
                ax, ay, az = float(parts[off+4]), float(parts[off+5]), float(parts[off+6])
                force = float(parts[off+7])
                ts = int(parts[off+8])

                clean_sample = self.imu_cleaner.process_sample(qx, qy, qz, qw, ax, ay, az)
                
                imu_batch.append({
                    'quat': clean_sample[0:4],
                    'acc': clean_sample[4:7],
                    'force': force,
                    'ts': ts
                })

            return {
                'uwb': (d0, d1, d2),
                'uwb_raw': (raw_d0, raw_d1, raw_d2),
                'imu': imu_batch
            }

        except ValueError:
            return None # Skip corrupt lines
            
    def close(self):
        if self.ser:
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()