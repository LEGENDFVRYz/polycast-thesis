import threading
from websocket_server import WebsocketServer
from app.webserver import run_webserver
import app.globals as g
from config import FLASK_PORT, BROWSER_WS_PORT


def _make_browser_ws_server(port):
    def new_client(c, _s): print(f"[BROWSER WS] + Client {c['id']}")
    def client_left(c, _s): print(f"[BROWSER WS] - Client {c['id']}")
    def msg_received(_c, _s, _m): pass

    ws = WebsocketServer(host="0.0.0.0", port=port, loglevel=20)
    ws.set_fn_new_client(new_client)
    ws.set_fn_client_left(client_left)
    ws.set_fn_message_received(msg_received)
    return ws


if __name__ == "__main__":
    ws_server = _make_browser_ws_server(BROWSER_WS_PORT)
    g.thread_manager.ws_server = ws_server
    threading.Thread(target=ws_server.run_forever, daemon=True).start()

    run_webserver(FLASK_PORT)
