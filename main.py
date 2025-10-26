import threading
from config import *
from prototype import ws_client_thread, start_browser_ws_server
from image_processing import archiver_thread
from webserver import run_webserver


if __name__ == "__main__":
    threading.Thread(target=start_browser_ws_server, args=(BROWSER_WS_PORT,), daemon=True).start()
    threading.Thread(target=ws_client_thread, daemon=True).start()

    if IS_AUTO_ARCHIVING:
        threading.Thread(target=archiver_thread, daemon=True).start()
        print("[ARCHIVER] auto-archiving enabled")
    else:
        print("[ARCHIVER] auto-archiving disabled")

    run_webserver(FLASK_PORT)
