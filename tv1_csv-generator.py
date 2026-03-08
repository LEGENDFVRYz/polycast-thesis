import serial
import struct
import time
import csv
import os
from datetime import datetime

# ==============================================================================
# SERIAL STREAMER CLASS
#    - Handles connection and binary unpacking
# ==============================================================================
class SerialStreamer:
    def __init__(self, port='COM5', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.buffer = bytearray()
        
        # Scaling Factors (Must match Arduino Sender)
        self.Q_SCALE = 32767.0
        self.A_SCALE = 1000.0
        self.F_SCALE = 100.0
        
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ser.reset_input_buffer()
            print(f"[STREAMER] Connected to {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[STREAMER] Connection Error: {e}")
            self.ser = None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _parse_imu(self, payload, rx_ts):
        """
        Unpacks IMU bytes.
        Returns a dict containing a list of 3 samples.
        """
        try:
            # Payload Structure: [Type (1)] [PacketID (4)] [Sample1 (20)] [Sample2 (20)] [Sample3 (20)]
            packet_id = struct.unpack('<I', payload[1:5])[0] 
            offset = 5
            sample_size = 20 
            
            samples = []
            
            # Extract all 3 samples in the batch
            for i in range(3):
                if offset + sample_size > len(payload): break
                chunk = payload[offset : offset + sample_size]
                offset += sample_size
                
                # Unpack: qx,qy,qz,qw (shorts), ax,ay,az (shorts), force (short), ts (uint)
                data = struct.unpack('<hhhhhhhhI', chunk)
                
                sample = {
                    'rx_ts': rx_ts,          # Receiver Timestamp (ms)
                    'packet_id': packet_id,
                    'sample_idx': i,
                    'sender_ts': data[8],    # Sender Timestamp (micros)
                    'qx': data[0]/self.Q_SCALE, 
                    'qy': data[1]/self.Q_SCALE, 
                    'qz': data[2]/self.Q_SCALE, 
                    'qw': data[3]/self.Q_SCALE,
                    'ax': data[4]/self.A_SCALE, 
                    'ay': data[5]/self.A_SCALE, 
                    'az': data[6]/self.A_SCALE,
                    'force': data[7]/self.F_SCALE
                }
                samples.append(sample)
                
            return {
                'type': 'IMU',
                'samples': samples 
            }
        except Exception as e:
            print(f"[Parse Error IMU] {e}")
            return None

    def _parse_uwb(self, payload, rx_ts):
        """
        Unpacks UWB bytes.
        """
        try:
            # Payload Structure: [Type (1)] [PacketID (4)] [X(4)] [Y(4)] [D0(4)] [D1(4)] [D2(4)] [TS(4)]
            # Note: The format string '<BIfffffI' matches the structure (Byte, Uint, Float*5, Uint)
            data = struct.unpack('<BIfffffI', payload)
            return {
                'type': 'UWB',
                'data': {
                    'rx_ts': rx_ts,         # Receiver Timestamp (ms)
                    'packet_id': data[1],
                    'sender_ts': data[7],   # Sender Timestamp (micros)
                    'x': data[2],
                    'y': data[3],
                    'd0': data[4],
                    'd1': data[5],
                    'd2': data[6]
                }
            }
        except Exception as e:
            print(f"[Parse Error UWB] {e}")
            return None

    def read_new_packets(self):
        packets_found = []
        if not self.ser: return packets_found
        
        try:
            if self.ser.in_waiting:
                self.buffer.extend(self.ser.read(self.ser.in_waiting))
            
            while len(self.buffer) >= 7: # Min header size
                # 1. Header Check (0xAA 0x55)
                if self.buffer[0] != 0xAA or self.buffer[1] != 0x55:
                    self.buffer.pop(0)
                    continue
                
                # 2. Frame Length Check
                # Frame: [AA] [55] [LEN] [RX_TS(4)] [PAYLOAD...] [FF]
                payload_len_wrapper = self.buffer[2] # This length includes RX_TS(4) + Actual Data
                total_frame_size = 3 + payload_len_wrapper + 1 # Header(2)+Len(1) + wrapper + Footer(1)
                
                if len(self.buffer) < total_frame_size:
                    break 
                
                # 3. Footer Check
                if self.buffer[total_frame_size - 1] != 0xFF:
                    self.buffer.pop(0)
                    continue
                
                # 4. Extraction
                frame = self.buffer[:total_frame_size]
                self.buffer = self.buffer[total_frame_size:]
                
                # Extract Receiver Timestamp (Bytes 3,4,5,6)
                rx_ts = struct.unpack('<I', frame[3:7])[0]
                
                # Extract Actual Payload (Bytes 7 to End-1)
                actual_payload = frame[7:-1]
                
                if len(actual_payload) > 0:
                    pkt = None
                    if actual_payload[0] == 0x01:   
                        pkt = self._parse_imu(actual_payload, rx_ts)
                    elif actual_payload[0] == 0x02: 
                        pkt = self._parse_uwb(actual_payload, rx_ts)
                    
                    if pkt: packets_found.append(pkt)
                    
        except Exception as e:
            print(f"[STREAMER] Read Error: {e}")
            
        return packets_found


# ==============================================================================
# MAIN LOGGING SCRIPT
# ==============================================================================
if __name__ == "__main__":
    
    # --- FILES SETUP ---
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    imu_filename = f"imu_abc_wforce.csv"
    uwb_filename = f"uwb_abc_wforce.csv"
    
    # Define CSV Columns
    imu_columns = ['rx_ts', 'packet_id', 'sample_idx', 'sender_ts', 
                   'qx', 'qy', 'qz', 'qw', 'ax', 'ay', 'az', 'force']
    
    uwb_columns = ['rx_ts', 'packet_id', 'sender_ts', 
                   'x', 'y', 'd0', 'd1', 'd2']
    
    print(f"--- STARTING LOGGING ---")
    print(f"Creating: {imu_filename}")
    print(f"Creating: {uwb_filename}")
    
    # Initialize Streamer
    # !!! CHANGE 'COM5' TO YOUR ACTUAL PORT !!!
    streamer = SerialStreamer(port='COM5', baud=115200) 
    
    # Open Files
    with open(imu_filename, 'w', newline='') as f_imu, \
         open(uwb_filename, 'w', newline='') as f_uwb:
        
        imu_writer = csv.DictWriter(f_imu, fieldnames=imu_columns)
        uwb_writer = csv.DictWriter(f_uwb, fieldnames=uwb_columns)
        
        imu_writer.writeheader()
        uwb_writer.writeheader()
        
        packet_count = 0
        
        try:
            while True:
                packets = streamer.read_new_packets()
                
                for pkt in packets:
                    packet_count += 1
                    
                    # LOG IMU (1 Packet = 3 Rows)
                    if pkt['type'] == 'IMU':
                        for sample in pkt['samples']:
                            imu_writer.writerow(sample)
                            
                        # Visual Feedback (every 100 packets)
                        if packet_count % 50 == 0:
                            s = pkt['samples'][-1]
                            print(f"[IMU] ID:{s['packet_id']} Force:{s['force']:.2f} | Written batch")

                    # LOG UWB (1 Packet = 1 Row)
                    elif pkt['type'] == 'UWB':
                        data = pkt['data']
                        uwb_writer.writerow(data)
                        print(f"[UWB] ID:{data['packet_id']} Pos:({data['x']:.2f}, {data['y']:.2f})")

        except KeyboardInterrupt:
            print("\n--- STOPPING LOG ---")
            streamer.close()
            print("Files closed safely.")