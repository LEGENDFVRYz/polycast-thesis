import serial
import json
import time
import threading
import xp_pen_api  # your XP-Pen API module

# ------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------
COM_PORT = "COM3"       # change to your ESP32 port
BAUD_RATE = 115200

# Open serial connection to ESP32
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# ------------------------------------------------------------
# XP-Pen Callback
# ------------------------------------------------------------
def pen_callback(pkt_ptr):
    pkt = pkt_ptr.contents

    # Remap according to your usage
    x = pkt.x
    y = pkt.pressure      # remap (pressure field = Y)
    pressure = pkt.button # remap (button field = pressure)

    data = {
        "x": x,
        "y": y,
        "pressure": pressure
    }

    # Send JSON line to ESP32
    try:
        ser.write((json.dumps(data) + "\n").encode())
    except Exception as e:
        print("Serial write error:", e)

    # Optional: debug print
    print(f"→ Sent: {data}")
    return 0

# ------------------------------------------------------------
# XP-Pen API Thread
# ------------------------------------------------------------
def run_api():
    api = xp_pen_api.XPPenAPI()
    try:
        api.start(pen_callback)
        while True:
            time.sleep(0.01)  # keep callback alive
    except KeyboardInterrupt:
        api.stop()

# ------------------------------------------------------------
# Run everything
# ------------------------------------------------------------
if __name__ == "__main__":
    print("Starting XP-Pen → ESP32 serial bridge...")
    threading.Thread(target=run_api, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping bridge...")
