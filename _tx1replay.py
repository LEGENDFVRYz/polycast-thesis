import csv
import serial
import struct
import time
import os

# ==============================================================================
# CONFIGURATION
# ==============================================================================
VIRTUAL_COM_PORT = 'COM19'  # The port the replayer sends TO
BAUD_RATE = 115200
CSV_FILE = 'sc1_loop.csv'  # <--- CHANGE THIS to your actual CSV filename

# Scaling factors (Must match the original unpacker)
Q_SCALE = 32767.0
A_SCALE = 1000.0

# In your latest snippet, Force wasn't divided by F_SCALE in _parse_imu, 
# but if it was saved as a decimal, we might need to multiply it back.
F_SCALE = 100.0 

# ==============================================================================
# PAYLOAD BUILDERS
# ==============================================================================
def build_imu_payload(rows):
    """Rebuilds the IMU binary payload from 3 CSV rows"""
    packet_id = int(rows[0]['Packet_ID'])
    
    # Payload starts with Type(0x01) + ID(4 bytes)
    payload = struct.pack('<BI', 0x01, packet_id)
    
    for row in rows:
        # Reverse the scaling to get back to the original short integers
        qx = int(float(row['Q_x']) * Q_SCALE)
        qy = int(float(row['Q_y']) * Q_SCALE)
        qz = int(float(row['Q_z']) * Q_SCALE)
        qw = int(float(row['Q_w']) * Q_SCALE)
        
        ax = int(float(row['A_x']) * A_SCALE)
        ay = int(float(row['A_y']) * A_SCALE)
        az = int(float(row['A_z']) * A_SCALE)
        
        # Safely handle force (multiply by scale if it was logged as a float)
        force_val = float(row['Force'])
        if abs(force_val) < 1000: # Assuming it was scaled down
            force = int(force_val * F_SCALE)
        else:
            force = int(force_val)
            
        ts = int(float(row['HW_TS']))
        
        # Pack: qx,qy,qz,qw (shorts), ax,ay,az (shorts), force (short), ts (uint)
        chunk = struct.pack('<hhhhhhhhI', qx, qy, qz, qw, ax, ay, az, force, ts)
        payload += chunk
        
    return payload

def build_uwb_payload(row):
    """Rebuilds the UWB binary payload from 1 CSV row"""
    packet_id = int(row['Packet_ID'])
    pos_x = float(row['Pos_x'])
    pos_y = float(row['Pos_y'])
    d0 = float(row['Dist_0'])
    d1 = float(row['Dist_1'])
    d2 = float(row['Dist_2'])
    ts = int(float(row['HW_TS']))
    
    # Pack: Type(0x02) + ID(4) + x(4) + y(4) + d0..d2(12) + ts(4)
    payload = struct.pack('<BIfffffI', 0x02, packet_id, pos_x, pos_y, d0, d1, d2, ts)
    return payload

def wrap_frame(payload):
    """Wraps payload with Header, Length, RecvTS, and Footer"""
    header = b'\xAA\x55'
    
    # FIX: The receiver's payload_len calculation includes the 4 bytes of RecvTS!
    length_val = len(payload) + 4 
    length_byte = struct.pack('<B', length_val)
    
    recv_ts = struct.pack('<I', int(time.time() * 1000) % 0xFFFFFFFF)
    footer = b'\xFF'
    
    # Structure: 0xAA 0x55 | Length(1) | RecvTS(4) | Payload(var) | 0xFF
    return header + length_byte + recv_ts + payload + footer

# ==============================================================================
# MAIN REPLAY LOOP
# ==============================================================================
if __name__ == "__main__":
    if not os.path.exists(CSV_FILE):
        print(f"[!] Error: Could not find {CSV_FILE}.")
        print("Please check the filename and try again.")
        exit()

    print(f"[*] Opening Virtual COM Port {VIRTUAL_COM_PORT}...")
    try:
        ser = serial.Serial(VIRTUAL_COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"[!] Error opening port: {e}")
        exit()

    print(f"[*] Reading '{CSV_FILE}' and streaming to {VIRTUAL_COM_PORT}...")
    print("[*] Switch to your receiver window (COM20) now!\n")
    
    packets_sent = 0
    
    with open(CSV_FILE, mode='r') as f:
        reader = csv.DictReader(f)
        
        imu_batch = []
        last_sys_time = None
        
        try:
            for row in reader:
                current_sys_time = float(row['System_Time'])
                
                # Recreate the exact original timing delays
                if last_sys_time is not None:
                    time_diff = current_sys_time - last_sys_time
                    if time_diff > 0:
                        time.sleep(time_diff)
                
                last_sys_time = current_sys_time
                
                if row['Type'] == 'IMU':
                    imu_batch.append(row)
                    # Once we have 3 samples, pack and send the batch
                    if len(imu_batch) == 3:
                        payload = build_imu_payload(imu_batch)
                        frame = wrap_frame(payload)
                        ser.write(frame)
                        packets_sent += 1
                        imu_batch = [] # Reset batch
                        
                elif row['Type'] == 'UWB':
                    payload = build_uwb_payload(row)
                    frame = wrap_frame(payload)
                    ser.write(frame)
                    packets_sent += 1
                    
                # Print a status update every 50 packets so you know it's working
                if packets_sent % 50 == 0:
                    print(f" -> Streamed {packets_sent} packets...", end='\r')

        except KeyboardInterrupt:
            print("\n\n[*] Replay stopped manually.")
            
    print(f"\n\n[*] Replay Complete. Total Packets Sent: {packets_sent}")
    ser.close()