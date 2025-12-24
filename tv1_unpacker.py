import serial
import struct
import time
import os

# --- CONFIGURATION (EDIT THESE TO CHANGE MODES) ---
# SERIAL COM: 
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# VIEW MODE OPTIONS: 
#   > 'HISTORY' (Scrolls / Stack Outputs) 
#   > 'LIVE'    (Clears screen for every packet)
VIEW_MODE = 'LIVE' 

# FILTER OPTIONS: 
#   > 'BOTH',   (Shows both sensor data)
#   > 'IMU',    (Shows IMU data)
#   > 'UWB'     (Shows UWB data)
FILTER_MODE = 'IMU'

# --- DISPLAY SETTINGS ---
#   > 0  =  real-speed of transfer between sender and reciever + unpacker.py
#   > 1  =  10 updates per second (Smooth, readable)
DISPLAY_RATE = 0.1


# --- SCALING FACTORS ---
Q_SCALE = 32767.0
A_SCALE = 1000.0
F_SCALE = 100.0


# --- GLOBAL DATA STORE ---
latest_data = {
    'imu_id': 0,
    'acc': (0.0, 0.0, 0.0),
    'force': 0.0,
    'uwb_id': 0,
    'pos': (0.0, 0.0),
    'dists': (0.0, 0.0, 0.0)
}


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def parse_imu_packet(payload):
    try:
        packet_id = struct.unpack('<I', payload[1:5])[0] 
        offset = 5
        sample_size = 20 
        
        # We only care about the LAST sample in the batch for "Live View"
        # But we process all to ensure we don't miss any logic later
        for i in range(3):
            if offset + sample_size > len(payload): break
            chunk = payload[offset : offset + sample_size]
            offset += sample_size
            
            data = struct.unpack('<hhhhhhhhI', chunk)
            
            # Update Global Data
            latest_data['imu_id'] = packet_id
            latest_data['acc'] = (data[4]/A_SCALE, data[5]/A_SCALE, data[6]/A_SCALE)
            latest_data['force'] = data[7]/F_SCALE
            
            # If HISTORY mode, we print every sample immediately
            if VIEW_MODE == 'HISTORY' and (FILTER_MODE in ['BOTH', 'IMU']):
                 if i == 2: # Print once per batch to save space
                    print(f"[IMU #{packet_id}] Force: {latest_data['force']:.2f} | Acc: {latest_data['acc']}")

    except Exception as e:
        pass # Silently ignore partial corruptions to prevent scroll jitter

def parse_uwb_packet(payload):
    try:
        data = struct.unpack('<BIfffffI', payload)
        
        # Update Global Data
        latest_data['uwb_id'] = data[1]
        latest_data['pos'] = (data[2], data[3])
        latest_data['dists'] = (data[4], data[5], data[6])
        
        # If HISTORY mode, print immediately
        if VIEW_MODE == 'HISTORY' and (FILTER_MODE in ['BOTH', 'UWB']):
            print(f">>> [UWB #{data[1]}] Pos: {latest_data['pos']} <<<")
            
    except Exception as e:
        pass

def print_live_dashboard():
    """Prints a static dashboard that doesn't flicker"""
    clear_screen()
    print(f"==========================================")
    print(f" LIVE MONITORING (Update Rate: {DISPLAY_RATE}s)")
    print(f"==========================================")
    
    if FILTER_MODE in ['BOTH', 'IMU']:
        acc = latest_data['acc']
        print(f"\n [IMU #{latest_data['imu_id']}]")
        print(f"   Force: {latest_data['force']:.2f}")
        print(f"   Accel: X={acc[0]:.2f}, Y={acc[1]:.2f}, Z={acc[2]:.2f}")
    
    if FILTER_MODE in ['BOTH', 'UWB']:
        pos = latest_data['pos']
        dists = latest_data['dists']
        print(f"\n [UWB #{latest_data['uwb_id']}]")
        print(f"   Pos  : X={pos[0]:.2f}, Y={pos[1]:.2f}")
        print(f"   Dists: {dists[0]:.2f}, {dists[1]:.2f}, {dists[2]:.2f}")
    
    print("\n==========================================")
    print(" (Data is streaming 100% speed in background)")

def read_serial_stream():
    last_update_time = 0
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}...")
        time.sleep(1)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Error: {e}")
        return

    buffer = bytearray()
    
    while True:
        try:
            # 1. READ DATA (As fast as possible)
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
            
            while len(buffer) >= 4: 
                # Header Check
                if buffer[0] != 0xAA or buffer[1] != 0x55:
                    buffer.pop(0); continue
                
                payload_len = buffer[2]
                total_frame_size = 2 + 1 + payload_len + 1 
                
                if len(buffer) < total_frame_size: break 
                
                if buffer[total_frame_size - 1] != 0xFF:
                    buffer.pop(0); continue
                
                # Extract
                frame = buffer[:total_frame_size]
                buffer = buffer[total_frame_size:] 
                data_payload = frame[7:-1] 
                
                if len(data_payload) > 0:
                    if data_payload[0] == 0x01: parse_imu_packet(data_payload)
                    elif data_payload[0] == 0x02: parse_uwb_packet(data_payload)

            # 2. UPDATE DISPLAY (Throttled)
            if VIEW_MODE == 'LIVE':
                current_time = time.time()
                if (current_time - last_update_time) > DISPLAY_RATE:
                    print_live_dashboard()
                    last_update_time = current_time

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
            break

if __name__ == "__main__":
    read_serial_stream()