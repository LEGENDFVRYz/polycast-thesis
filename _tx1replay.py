from lzma import MODE_FAST

import serial
import time
import os
import re

# ==============================================================================
# CONFIGURATION
# ==============================================================================
VIRTUAL_COM_PORT    = 'COM19'
BAUD_RATE           = 921600
MODE                = "abc"
TESTNAME            = "force_test"
LOG_FILE            = f'logs/{TESTNAME}.csv'

# ==============================================================================
# MAIN REPLAY LOOP
# ==============================================================================
if __name__ == "__main__":
    if not os.path.exists(LOG_FILE):
        print(f"[!] Error: Could not find {LOG_FILE}.")
        print("Please check the filename and try again.")
        exit()

    print(f"[*] Opening Virtual COM Port {VIRTUAL_COM_PORT}...")
    try:
        ser = serial.Serial(VIRTUAL_COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"[!] Error opening port: {e}")
        exit()

    print(f"[*] Reading '{LOG_FILE}' and streaming to {VIRTUAL_COM_PORT}...")
    print("[*] Switch to your receiver window (COM20) now!\n")
    
    packets_sent = 0
    last_hw_ts = None
    
    with open(LOG_FILE, mode='r', encoding='utf-8') as f:
        try:
            for line in f:
                # 1. Clean the line entirely of whitespace/newlines
                line = line.strip()
                if not line:
                    continue
                    
                # 2. Split by ANY combination of tabs, spaces, or commas
                # Using list comprehension with "if p" removes all empty string artifacts 
                # caused by multiple trailing tabs.
                parts = [p for p in re.split(r'[\t, ]+', line) if p]
                
                if not parts:
                    continue
                
                type_char = parts[0]
                
                # 3. Strict Pre-Flight Check (Mimic the unpacker's requirements)
                if type_char == 'I' and len(parts) == 11:
                    pass # Valid IMU
                elif type_char == 'U' and len(parts) == 7:
                    pass # Valid UWB
                else:
                    # Ignore headers or malformed lines entirely
                    continue 
                
                # 4. Extract hardware timestamp for realistic timing replay
                try:
                    current_hw_ts = int(parts[-1])
                except ValueError:
                    continue
                
                if last_hw_ts is not None:
                    # Hardware timestamps appear to be microseconds
                    delay_us = current_hw_ts - last_hw_ts
                    
                    # Apply delay if it's sensible (e.g., between 0 and 1 second)
                    if 0 < delay_us < 1_000_000:  
                        time.sleep(delay_us / 1_000_000.0)
                
                last_hw_ts = current_hw_ts
                
                # 5. Reconstruct as a STRICT comma-separated string ending in newline
                # Example Output: I,127894,-0.0253,-0.0126,-0.7144,0.6992,0.0703,0.0664,-0.0078,125,1282498145\n
                csv_string = ",".join(parts) + "\n"
                
                # 6. Send over Virtual COM Port and flush buffer immediately
                ser.write(csv_string.encode('utf-8'))
                ser.flush() 
                packets_sent += 1
                
                # Visual heartbeat
                if packets_sent % 50 == 0:
                    print(f" -> Streamed {packets_sent} packets...", end='\r')

        except KeyboardInterrupt:
            print("\n\n[*] Replay stopped manually.")
            
    print(f"\n\n[*] Replay Complete. Total Packets Sent: {packets_sent}")
    ser.close()