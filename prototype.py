import time, json, struct, websocket
from config import prototype_config, CONN_LOG, ERROR_LOG
from app.utils.utils import log_message
from image_processing import draw_segment, image_lock
from websocket_server import WebsocketServer

last_point = None
ws_server = None


def parse_esp_binary(msg: bytes):
    """Parse binary packets of absolute coordinates (type=0)."""
    points = []
    i = 0
    while i + 6 <= len(msg):
        header = msg[i]
        if header != 0:
            i += 1
            continue
        x = struct.unpack_from("<h", msg, i + 1)[0]
        y = struct.unpack_from("<h", msg, i + 3)[0]
        p = msg[i + 5]
        points.append((x, y, p))
        i += 6
    return points


def on_esp_message(wsapp, message):
    global last_point, ws_server
    try:
        points = parse_esp_binary(message)
        if not points:
            return
        with image_lock:
            for (x, y, p) in points:
                if p < 3:
                    last_point = None
                else:
                    if last_point:
                        draw_segment(last_point[0], last_point[1], x, y, p)
                    last_point = (x, y)
        if ws_server:
            ws_server.send_message_to_all(json.dumps(points[-1]))
    except Exception as e:
        print("[ESP WS] parse/draw error:", e)


def on_esp_open(wsapp):
    log_message(CONN_LOG, "[ESP WS] connected")

def on_esp_error(wsapp, err):
    log_message(ERROR_LOG, f"[ESP WS] error: {err}")

def on_esp_close(wsapp, code, msg):
    log_message(CONN_LOG, f"[ESP WS] closed: {code} {msg}")


def ws_client_thread():
    while True:
        try:
            print(f"[ESP WS] connecting to {prototype_config.ESP32_WS_URL}")
            wsapp = websocket.WebSocketApp(
                prototype_config.ESP32_WS_URL,
                on_open=on_esp_open,
                on_message=on_esp_message,
                on_error=on_esp_error,
                on_close=on_esp_close,
            )
            wsapp.run_forever()
        except Exception as e:
            print("[ESP WS] client exception:", e)
        print("[ESP WS] reconnecting in 2s...")
        time.sleep(2)


def start_browser_ws_server(port):
    """Start a WebSocket server for browser clients."""
    global ws_server
    def new_client(c, s): print(f"[BROWSER WS] + Client {c['id']}")
    def client_left(c, s): print(f"[BROWSER WS] - Client {c['id']}")
    def msg_received(c, s, m): pass

    ws_server = WebsocketServer(host="0.0.0.0", port=port, loglevel=20)
    ws_server.set_fn_new_client(new_client)
    ws_server.set_fn_client_left(client_left)
    ws_server.set_fn_message_received(msg_received)
    print(f"[BROWSER WS] serving on {port}")
    ws_server.run_forever()
