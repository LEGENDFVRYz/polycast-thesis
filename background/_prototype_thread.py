import threading, time, json, struct, websocket
from config import prototype_config, CONN_LOG, ERROR_LOG
from app.utils.utils import log_message
from image_processing import draw_segment, image_lock


class PrototypeWSThread(threading.Thread):
    def __init__(self, stop_event, ws_server=None):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.ws_server = ws_server
        self.wsapp = None
        self.last_point = None

    def parse_esp_binary(self, msg: bytes):
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

    # WebSocket event handlers -------------------
    def on_open(self, wsapp):
        log_message(CONN_LOG, "[ESP WS] connected")

    def on_message(self, wsapp, message):
        try:
            points = self.parse_esp_binary(message)
            if not points:
                return

            with image_lock:
                for (x, y, p) in points:
                    if p < 3:
                        self.last_point = None
                    else:
                        if self.last_point:
                            draw_segment(self.last_point[0], self.last_point[1], x, y, p)
                        self.last_point = (x, y)

            if self.ws_server:
                self.ws_server.send_message_to_all(json.dumps(points[-1]))

        except Exception as e:
            print("[ESP WS] parse/draw error:", e)

    def on_error(self, wsapp, err):
        log_message(ERROR_LOG, f"[ESP WS] error: {err}")

    def on_close(self, wsapp, code, msg):
        log_message(CONN_LOG, f"[ESP WS] closed: {code} {msg}")

    # Main loop ---------------------------------
    def run(self):
        while not self.stop_event.is_set():
            try:
                print(f"[ESP WS] connecting to {prototype_config.ESP32_WS_URL}")
                self.wsapp = websocket.WebSocketApp(
                    prototype_config.ESP32_WS_URL,
                    on_open=self.on_open,
                    on_message=self.on_message,
                    on_error=self.on_error,
                    on_close=self.on_close,
                )
                self.wsapp.run_forever()
            except Exception as e:
                print("[ESP WS] client exception:", e)

            if not self.stop_event.is_set():
                print("[ESP WS] reconnecting in 1s...")
                time.sleep(1)

        print("[ESP WS] thread exited cleanly.")

    def stop(self):
        self.stop_event.set()
        if self.wsapp:
            try:
                self.wsapp.close()
            except Exception as e:
                print("[ESP WS] error closing ws:", e)
