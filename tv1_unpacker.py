import serial
import struct
import time
import os


# ==============================================================================
# SERIAL STREAMER
#    - Enable to read all the raw data came from the reciever module
# ==============================================================================
class SerialStreamer:
    def __init__(self, port='COM3', baud=115200):
        # --- SERIAL COM ---
        self.port = port
        self.baud = baud
        self.ser = None
        self.buffer = bytearray()
        
        # --- SCALING FACTORS ---
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
        """
        Unpacks IMU bytes into a usable dictionary.
        """
        try:
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
                    'ts': data[8],
                    'quat': (data[0]/self.Q_SCALE, data[1]/self.Q_SCALE, data[2]/self.Q_SCALE, data[3]/self.Q_SCALE),
                    'acc':  (data[4]/self.A_SCALE, data[5]/self.A_SCALE, data[6]/self.A_SCALE),
                    'force': data[7]/self.F_SCALE
                }
                samples.append(sample)
                
            return {
                'type': 'IMU',
                'id': packet_id,
                'samples': samples # List of 3 samples
            }
        except Exception as e:
            return None

    def _parse_uwb(self, payload):
        """
        Unpacks UWB bytes into a usable dictionary.
        """
        try:
            # Format: Type(1), ID(4), x(4), y(4), d0..d2(12), ts(4)
            data = struct.unpack('<BIfffffI', payload)
            return {
                'type': 'UWB',
                'id': data[1],
                'pos': (data[2], data[3]),
                'dists': (data[4], data[5], data[6]),
                'ts': data[7] # Hardware Timestamp
            }
        except Exception as e:
            return None

    def read_new_packets(self):
        """
        Main interface function.
        Returns: A list of packet dictionaries found in the buffer.
        """
        packets_found = []
        
        if not self.ser: return packets_found
        
        try:
            # Read Raw Bytes (As fast as possible)
            if self.ser.in_waiting:
                self.buffer.extend(self.ser.read(self.ser.in_waiting))
            
            # Parsing the Frames
            while len(self.buffer) >= 4:
                # Header Check (0xAA 0x55)
                if self.buffer[0] != 0xAA or self.buffer[1] != 0x55:
                    self.buffer.pop(0) # Invalid, slide 1 byte
                    continue
                
                payload_len = self.buffer[2]
                total_frame = 2 + 1 + payload_len + 1 # Head + Len + Payload + Foot
                
                if len(self.buffer) < total_frame:
                    break # Wait for more data
                
                # Footer Check (0xFF)
                if self.buffer[total_frame - 1] != 0xFF:
                    self.buffer.pop(0) # Corrupt, slide 1 byte
                    continue
                
                # Extract Payload
                frame = self.buffer[:total_frame]
                self.buffer = self.buffer[total_frame:] # Remove from buffer
                
                # payload starts at index 7 (Header=2, Len=1, RecvTS=4)
                data_payload = frame[7:-1]
                
                if len(data_payload) > 0:
                    pkt = None
                    if data_payload[0] == 0x01:   pkt = self._parse_imu(data_payload)
                    elif data_payload[0] == 0x02: pkt = self._parse_uwb(data_payload)
                    
                    if pkt: packets_found.append(pkt)
                    
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
    latest = {'imu': None, 'uwb': None}
    last_draw_time = 0
    
    
    # --- MAIN DEBUGGER  ---
    streamer = SerialStreamer(port='COM3', baud=115200)
    
    try:
        while True:
            new_packets = streamer.read_new_packets()
            
            # PROCESS / STORE
            for pkt in new_packets:
                if pkt['type'] == 'IMU':
                    latest['imu'] = pkt
                    # In History mode, print immediately
                    if VIEW_MODE == 'HISTORY' and FILTER_MODE in ['BOTH', 'IMU']:
                        # Print last sample of batch
                        s = pkt['samples'][-1]
                        print(f"[IMU #{pkt['id']}] Acc: {s['acc']}")
                        
                elif pkt['type'] == 'UWB':
                    latest['uwb'] = pkt
                    if VIEW_MODE == 'HISTORY' and FILTER_MODE in ['BOTH', 'UWB']:
                        print(f">>> [UWB #{pkt['id']}] Pos: {pkt['pos']}")

            # LIVE VISUALIZATION (Throttled via DISPLAY RATE)
            if VIEW_MODE == 'LIVE' and (time.time() - last_draw_time > DISPLAY_RATE):
                os.system('cls' if os.name == 'nt' else 'clear')
                print(f"=========== STREAMER DEBUG ({DISPLAY_RATE}s) ==========")
                
                if FILTER_MODE in ['BOTH', 'IMU'] and latest['imu']:
                    s = latest['imu']['samples'][-1]
                    q = s['quat']
                    print(f"\n[IMU #{latest['imu']['id']}]")
                    print(f"  Force: {s['force']:.2f}")
                    print(f"  Accel: {s['acc']}")
                    print(f"  Quat:  {q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}")

                if FILTER_MODE in ['BOTH', 'UWB'] and latest['uwb']:
                    u = latest['uwb']
                    print(f"\n[UWB #{u['id']}]")
                    print(f"  Pos  : X={u['pos'][0]:.2f}, Y={u['pos'][1]:.2f}")
                    print(f"  Dists: {u['dists'][0]:.2f}, {u['dists'][1]:.2f}, {u['dists'][2]:.2f}")
                
                print("\n============================================")
                last_draw_time = time.time()
                
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.close()
        