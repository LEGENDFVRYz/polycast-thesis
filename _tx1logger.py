import serial
import struct
import time
import csv
import os

# ==============================================================================
# PASTE YOUR SerialStreamer CLASS HERE 
# (Included entirely for a copy-paste ready script)
# ==============================================================================
class SerialStreamer:
    def __init__(self, port='COM3', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.buffer = bytearray()
        
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

    def _parse_imu(self, payload):
        try:
            packet_id = struct.unpack('<I', payload[1:5])[0] 
            offset = 5
            sample_size = 20 
            samples = []
            
            for i in range(3):
                if offset + sample_size > len(payload): break
                chunk = payload[offset : offset + sample_size]
                offset += sample_size
                data = struct.unpack('<hhhhhhhhI', chunk)
                
                sample = {
                    'ts': data[8],
                    'quat': (data[0]/self.Q_SCALE, data[1]/self.Q_SCALE, data[2]/self.Q_SCALE, data[3]/self.Q_SCALE),
                    'acc':  (data[4]/self.A_SCALE, data[5]/self.A_SCALE, data[6]/self.A_SCALE),
                    'force': data[7]/self.F_SCALE
                }
                samples.append(sample)
                
            return {'type': 'IMU', 'id': packet_id, 'samples': samples}
        except Exception as e:
            return None

    def _parse_uwb(self, payload):
        try:
            data = struct.unpack('<BIfffffI', payload)
            return {
                'type': 'UWB', 'id': data[1], 'pos': (data[2], data[3]),
                'dists': (data[4], data[5], data[6]), 'ts': data[7]
            }
        except Exception as e:
            return None

    def read_new_packets(self):
        packets_found = []
        if not self.ser: return packets_found
        
        try:
            if self.ser.in_waiting:
                self.buffer.extend(self.ser.read(self.ser.in_waiting))
            
            while len(self.buffer) >= 4:
                if self.buffer[0] != 0xAA or self.buffer[1] != 0x55:
                    self.buffer.pop(0) 
                    continue
                
                payload_len = self.buffer[2]
                total_frame = 2 + 1 + payload_len + 1 
                
                if len(self.buffer) < total_frame: break 
                
                if self.buffer[total_frame - 1] != 0xFF:
                    self.buffer.pop(0) 
                    continue
                
                frame = self.buffer[:total_frame]
                self.buffer = self.buffer[total_frame:] 
                
                data_payload = frame[7:-1] # Adjust based on your header len
                
                if len(data_payload) > 0:
                    pkt = None
                    if data_payload[0] == 0x01:   pkt = self._parse_imu(data_payload)
                    elif data_payload[0] == 0x02: pkt = self._parse_uwb(data_payload)
                    
                    if pkt: packets_found.append(pkt)
                    
        except Exception as e:
            pass # Silently pass read errors during high-speed logging
            
        return packets_found

# ==============================================================================
# CSV CAPTURE ROUTINE
# ==============================================================================
if __name__ == "__main__":
    # CHANGE THIS to your actual physical COM port connected to the sender
    CAPTURE_PORT = 'COM3' 
    streamer = SerialStreamer(port=CAPTURE_PORT, baud=115200)
    
    filename = f"default.csv"
    
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        # Unified Header for both sensor types
        writer.writerow([
            'System_Time', 'Type', 'Packet_ID', 'Sample_Idx', 'HW_TS',
            'Q_x', 'Q_y', 'Q_z', 'Q_w', 'A_x', 'A_y', 'A_z', 'Force',
            'Pos_x', 'Pos_y', 'Dist_0', 'Dist_1', 'Dist_2'
        ])
        
        print(f"[*] Capturing serial data to {filename}...")
        print("[*] Press Ctrl+C to stop recording.")
        
        try:
            while True:
                packets = streamer.read_new_packets()
                for pkt in packets:
                    sys_t = time.time()
                    
                    if pkt['type'] == 'IMU':
                        # IMU has 3 samples per packet, we write 3 rows
                        for idx, s in enumerate(pkt['samples']):
                            writer.writerow([
                                sys_t, 'IMU', pkt['id'], idx, s['ts'],
                                s['quat'][0], s['quat'][1], s['quat'][2], s['quat'][3],
                                s['acc'][0], s['acc'][1], s['acc'][2], s['force'],
                                '', '', '', '', '' # Leave UWB fields blank
                            ])
                            
                    elif pkt['type'] == 'UWB':
                        # UWB has 1 sample per packet
                        writer.writerow([
                            sys_t, 'UWB', pkt['id'], 0, pkt['ts'],
                            '', '', '', '', '', '', '', '', # Leave IMU fields blank
                            pkt['pos'][0], pkt['pos'][1],
                            pkt['dists'][0], pkt['dists'][1], pkt['dists'][2]
                        ])
                        
        except KeyboardInterrupt:
            print("\n[*] Capture finished successfully.")
            streamer.close()