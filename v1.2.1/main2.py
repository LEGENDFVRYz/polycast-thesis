"""
Run:
    pip install flask pillow websocket-client websocket-server
    python main.py
"""

import threading
import time
import io
import os
import json
import struct
from datetime import datetime

from flask import Flask, Response, render_template
from PIL import Image, ImageDraw
import websocket
from websocket_server import WebsocketServer  # pip package: websocket-server

# CONFIG
ESP32_WS_URL = "ws://192.168.1.19/ws"  # <-- replace with your ESP32 ws address
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 600
ARCHIVE_DIR = "archive"
MJPEG_FPS = 20  # roughly (frame sleep ~1/20)
BROWSER_WS_PORT = 5001
FLASK_PORT = 5000

# Shared canvas and synchronization
canvas = Image.new('L', (CANVAS_WIDTH, CANVAS_HEIGHT), 'white')  # 'L' grayscale
draw = ImageDraw.Draw(canvas)
image_lock = threading.Lock()
last_point = None

# Websocket-server instance placeholder (will be created later)
ws_server = None


# ------------ Helper: draw stroke ----------------
def draw_segment(x0, y0, x1, y1, pressure):
    # pressure is expected as an int; scale to line width
    line_width = max(1, int(pressure / 100)) if pressure is not None else 1
    # clamp coords to canvas
    x0 = max(0, min(CANVAS_WIDTH - 1, int(x0)))
    y0 = max(0, min(CANVAS_HEIGHT - 1, int(y0)))
    x1 = max(0, min(CANVAS_WIDTH - 1, int(x1)))
    y1 = max(0, min(CANVAS_HEIGHT - 1, int(y1)))
    draw.line([(x0, y0), (x1, y1)], fill=0, width=line_width)


# ------------ WebSocket client that connects to ESP32 ----------------
def on_esp_open(wsapp):
    print("[ESP WS] connected to ESP32")


def on_esp_error(wsapp, err):
    print("[ESP WS] error:", err)


def on_esp_close(wsapp, close_status_code, close_msg):
    print("[ESP WS] connection closed:", close_status_code, close_msg)
    # websocket-client's run_forever will reconnect if you call it again; we keep it simple and exit/restart thread if needed


def on_esp_message(wsapp, message):
    """
    message can be bytes (binary) or text (JSON).
    Handle both cases:
        - if bytes: we expect little-endian signed x,y int16 and unsigned p uint16 like your original JS sample:
            struct: <hhH  => x:int16, y:int16, p:uint16
        - if text: expect JSON {"x":..., "y":..., "p":...}
    For each packet: draw on canvas, then broadcast to connected browser clients (as JSON).
    """
    global last_point, ws_server

    try:
        if isinstance(message, (bytes, bytearray)):
            # parse binary: little-endian int16, int16, uint16
            if len(message) >= 6:
                x = struct.unpack_from('<h', message, 0)[0]
                y = struct.unpack_from('<h', message, 2)[0]
                p = struct.unpack_from('<H', message, 4)[0]
            else:
                print("[ESP WS] received too-short binary message:", len(message))
                return
        else:
            # text
            payload = json.loads(message)
            x = int(payload.get("x", 0))
            y = int(payload.get("y", 0))
            p = int(payload.get("p", 0))
    except Exception as e:
        print("[ESP WS] parse error:", e)
        return

    # Drawing logic: if last_point is set and pressure > 0, draw line from last_point to (x,y)
    with image_lock:
        if p == 0:
            # pen up — reset
            last_point = None
        else:
            if last_point is not None:
                draw_segment(last_point[0], last_point[1], x, y, p)
            last_point = (x, y)

    # Broadcast to browser clients: send JSON so frontends not requiring binary still work
    if ws_server is not None:
        try:
            msg = json.dumps({"x": x, "y": y, "p": p})
            ws_server.send_message_to_all(msg)
        except Exception as e:
            print("[ESP WS] broadcast error:", e)


def ws_client_thread():
    """Connects to ESP32 and listens forever; will attempt to reconnect on failure."""
    while True:
        try:
            print("[ESP WS] attempting connection to", ESP32_WS_URL)
            wsapp = websocket.WebSocketApp(
                ESP32_WS_URL,
                on_open=on_esp_open,
                on_message=on_esp_message,
                on_error=on_esp_error,
                on_close=on_esp_close,
            )
            # run_forever blocks until connection dies
            wsapp.run_forever()
        except Exception as e:
            print("[ESP WS] run_forever exception:", e)
        print("[ESP WS] sleeping 2s before reconnect attempt...")
        time.sleep(2)


# ------------ WebSocket server for browsers (simple) ----------------
# We'll use websocket-server library which is threaded and simple.
def new_browser_client(client, server):
    print(f"[BROWSER WS] new client connected: {client['id']}")


def browser_client_left(client, server):
    print(f"[BROWSER WS] client disconnected: {client['id']}")


def browser_message_received(client, server, message):
    # if the browser sends something (text), we just print it. Not used in this demo.
    print(f"[BROWSER WS] msg from {client['id']}: {message}")


def start_browser_ws_server():
    global ws_server
    ws_server = WebsocketServer(host='0.0.0.0', port=BROWSER_WS_PORT, loglevel=20)
    ws_server.set_fn_new_client(new_browser_client)
    ws_server.set_fn_client_left(browser_client_left)
    ws_server.set_fn_message_received(browser_message_received)
    print(f"[BROWSER WS] listening on port {BROWSER_WS_PORT}")
    ws_server.run_forever()


# ------------ Archiver thread: save image every second ----------------
def archiver_thread():
    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    while True:
        time.sleep(1)
        with image_lock:
            snapshot = canvas.copy()
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        fname = os.path.join(ARCHIVE_DIR, f"note_{timestamp}.jpg")
        try:
            snapshot.save(fname, quality=85)
            # print saved occasionally to avoid noisy logs every second
            print(f"[ARCHIVER] saved {fname}")
        except Exception as e:
            print("[ARCHIVER] save error:", e)


# ------------ Flask app for MJPEG streaming ----------------
app = Flask(__name__, template_folder="templates")


@app.route('/')
def index():
    return render_template('index.html', ws_port=BROWSER_WS_PORT)


def generate_frames():
    # MJPEG frames generator
    frame_interval = 1.0 / MJPEG_FPS
    while True:
        start = time.time()
        with image_lock:
            frame = canvas.copy()
        buf = io.BytesIO()
        # convert to RGB on save to keep compatibility with browsers — though canvas is grayscale
        frame.convert('RGB').save(buf, format='JPEG')
        jpg = buf.getvalue()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
        # throttle
        elapsed = time.time() - start
        to_sleep = frame_interval - elapsed
        if to_sleep > 0:
            time.sleep(to_sleep)


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ------------ Main ----------------
if __name__ == '__main__':
    # Start browser WebSocket server (thread)
    t_ws_server = threading.Thread(target=start_browser_ws_server, daemon=True)
    t_ws_server.start()

    # Start the ESP32 WebSocket client (thread)
    t_esp = threading.Thread(target=ws_client_thread, daemon=True)
    t_esp.start()

    # Start the archiver (thread)
    t_arch = threading.Thread(target=archiver_thread, daemon=True)
    t_arch.start()

    # Run Flask (blocking)
    print(f"[FLASK] serving on 0.0.0.0:{FLASK_PORT}")
    app.run(host='0.0.0.0', port=FLASK_PORT, threaded=True)
