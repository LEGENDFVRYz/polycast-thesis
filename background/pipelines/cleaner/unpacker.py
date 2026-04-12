"""
Module 1 — Stream Unpacker (Async CSV Version)

Responsibility:
    - Read all raw data coming from the receiver module.
    - Parse the asynchronous CSV string streams into flat Python dictionaries.
    - Prevent partial-line reads using safe `readline()` logic.

Input (from the ESP32 Receiver, fetched via serial com on COM3):
    - Interleaved CSV lines:
      IMU: I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
      UWB: U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>

Output (Flat event dictionaries ready for normalizer/preprocessor):
    IMU Event: { 'sensor': 'IMU', 'packet_id': int, 'sample_idx': 0, 'quat': (qx,qy,qz,qw), 'acc': (ax,ay,az), 'force': float, 'ts_hw': int }
    UWB Event: { 'sensor': 'UWB', 'packet_id': int, 'sample_idx': 0, 'dists': (d0,d1,d2,d3), 'ts_hw': int }
"""

import serial
import time
import os

class SerialStreamer:
    def __init__(self, port='COM3', baud=115200):
        # --- SERIAL COM ---
        self.port = port
        self.baud = baud
        self.ser = None
        self.connect()

    def connect(self):
        try:
            # Using timeout=1 allows readline() to successfully wait for the \n char
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.ser.reset_input_buffer()
            print(f"[STREAMER] Connected to LIVE stream on {self.port}")
        except Exception as e:
            print(f"[STREAMER] Connection Error: {e}")
            self.ser = None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def read_new_packets(self):
        """
        Reads all complete CSV lines currently in the serial buffer, 
        parses them, and returns a list of standardized event dictionaries.
        """
        packets_found = []
        
        if not self.ser or not self.ser.is_open:
            return packets_found
            
        try:
            # Only process if there are bytes waiting, preventing blocking
            while self.ser.in_waiting > 0:
                # readline() guarantees we get a full line up to '\n'
                raw_line = self.ser.readline()
                line = raw_line.decode('utf-8', errors='replace').strip()
                
                if not line:
                    continue
                    
                parts = line.split(',')
                if not parts:
                    continue
                    
                type_char = parts[0]

                try:
                    # ── Parse IMU Event ────────────────────────────────────────
                    if type_char == 'I' and len(parts) == 11:
                        packets_found.append({
                            'sensor':     'IMU',
                            'packet_id':  int(parts[1]),
                            'sample_idx': 0,  # Always 0
                            'quat':       (float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])),
                            'acc':        (float(parts[6]), float(parts[7]), float(parts[8])),
                            'force':      float(parts[9]),
                            'ts_hw':      int(parts[10])
                        })

                    # ── Parse UWB Event ────────────────────────────────────────
                    elif type_char == 'U' and len(parts) == 7:
                        packets_found.append({
                            'sensor':     'UWB',
                            'packet_id':  int(parts[1]),
                            'sample_idx': 0,  # Always 0
                            'dists':      (float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])),
                            'ts_hw':      int(parts[6])
                        })
                except (ValueError, IndexError):
                    # Corrupt line mid-stream — safely ignore and continue
                    continue
                    
        except Exception as e:
            print(f"[STREAMER] Read Error: {e}")
            
        return packets_found


# ==============================================================================
# DEBUG MODE
#   - Enable to print all the collected data immediately as possible
# ==============================================================================
if __name__ == "__main__":
    
    # --- CONFIGURATION ---
    # VIEW MODE OPTIONS: 
    #   > 'HISTORY' (Scrolls / Stack Outputs) 
    #   > 'LIVE'    (Clears screen for every packet)
    VIEW_MODE = 'LIVE' 

    # FILTER OPTIONS: 
    #   > 'BOTH',   (Shows both sensor data)
    #   > 'IMU',    (Shows IMU data)
    #   > 'UWB'     (Shows UWB data)
    FILTER_MODE = 'BOTH'

    # --- DISPLAY SETTINGS ---
    #   > 0  =  real-speed of transfer between sender and reciever + unpacker.py
    #   > 1  =  10 updates per second (Smooth, readable)
    DISPLAY_RATE = 0.1
    
    # Dashboard Data Store
    latest = {'IMU': None, 'UWB': None}
    last_draw_time = 0
    
    # --- MAIN DEBUGGER  ---
    streamer = SerialStreamer(port='COM3', baud=115200)
    
    if not streamer.ser:
        exit(1)
        
    try:
        while True:
            new_packets = streamer.read_new_packets()
            
            # PROCESS / STORE
            for pkt in new_packets:
                sensor_type = pkt['sensor']
                
                if sensor_type == 'IMU':
                    latest['IMU'] = pkt
                    # In History mode, print immediately
                    if VIEW_MODE == 'HISTORY' and FILTER_MODE in ['BOTH', 'IMU']:
                        print(f"[IMU #{pkt['packet_id']}] Acc: {pkt['acc']}")
                        
                elif sensor_type == 'UWB':
                    latest['UWB'] = pkt
                    if VIEW_MODE == 'HISTORY' and FILTER_MODE in ['BOTH', 'UWB']:
                        d = pkt['dists']
                        print(f">>> [UWB #{pkt['packet_id']}] Dists: {d[0]:.2f}, {d[1]:.2f}, {d[2]:.2f}, {d[3]:.2f}")

            # LIVE VISUALIZATION (Throttled via DISPLAY RATE)
            if VIEW_MODE == 'LIVE' and (time.time() - last_draw_time > DISPLAY_RATE):
                os.system('cls' if os.name == 'nt' else 'clear')
                print(f"=========== STREAMER DEBUG ({DISPLAY_RATE}s) ==========")
                
                if FILTER_MODE in ['BOTH', 'IMU'] and latest['IMU']:
                    imu = latest['IMU']
                    q = imu['quat']
                    print(f"\n[IMU #{imu['packet_id']}]")
                    print(f"  Force: {imu['force']:.2f}")
                    print(f"  Accel: {imu['acc'][0]:.2f}, {imu['acc'][1]:.2f}, {imu['acc'][2]:.2f}")
                    print(f"  Quat:  {q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}")

                if FILTER_MODE in ['BOTH', 'UWB'] and latest['UWB']:
                    uwb = latest['UWB']
                    d = uwb['dists']
                    print(f"\n[UWB #{uwb['packet_id']}]")
                    print(f"  Dists: {d[0]:.2f}, {d[1]:.2f}, {d[2]:.2f}, {d[3]:.2f}")
                
                print("\n============================================")
                last_draw_time = time.time()
                
            time.sleep(0.005)
                
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.close()