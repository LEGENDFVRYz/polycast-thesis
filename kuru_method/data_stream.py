import serial
import time
from preprocessor import UWBPreprocessor, IMUPreprocessor

class DataStream:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Initialize the cleaners
        self.uwb_cleaner = UWBPreprocessor()
        self.imu_cleaner = IMUPreprocessor()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            print(f"✅ Connected to {self.port}")
            return True
        except Exception as e:
            print(f"❌ Connection Failed: {e}")
            return False

    def get_packet(self):
        """Reads one line, parses it, cleans it, and returns structured data."""
        if not self.ser or self.ser.in_waiting == 0:
            return None

        try:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            if not line: return None

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
                # Parse Raw
                qx, qy, qz, qw = float(parts[off]), float(parts[off+1]), float(parts[off+2]), float(parts[off+3])
                ax, ay, az = float(parts[off+4]), float(parts[off+5]), float(parts[off+6])
                force = float(parts[off+7])
                ts = int(parts[off+8])

                # Clean
                clean_sample = self.imu_cleaner.process_sample(qx, qy, qz, qw, ax, ay, az)
                
                # Append formatted object or dict
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