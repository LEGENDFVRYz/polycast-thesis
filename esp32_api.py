import serial
import xp_pen_api
import time
import threading

COM_PORT = "COM3"
BAUD_RATE = 115200

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

def pen_callback(pkt_ptr):
    pkt = pkt_ptr.contents
    x = pkt.x
    y = pkt.pressure
    p = pkt.button
    line = f"{x},{y},{p}\n"
    ser.write(line.encode())
    print("→", line.strip())
    return 0

def run_api():
    api = xp_pen_api.XPPenAPI()
    try:
        api.start(pen_callback)
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        api.stop()

if __name__ == "__main__":
    print("XP-Pen → ESP32 (CSV Bridge)")
    threading.Thread(target=run_api, daemon=True).start()
    while True:
        time.sleep(1)
