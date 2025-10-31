import threading
import socket
from config import *
from prototype import ws_client_thread, start_browser_ws_server
from image_processing import archiver_thread
from webserver import run_webserver


def prototype_manager_thread():
    """
    This thread waits for the admin to configure the system
    via the web interface before starting the ESP32 websocket client.
    """
    print("[MANAGER] Waiting for admin configuration...")
    
    # --- 3. Wait on the event (imported from config.py) ---
    config_done_event.wait()
    
    print("[MANAGER] Admin configured. Starting ESP32 client thread...")
    threading.Thread(target=ws_client_thread, daemon=True).start()




if __name__ == "__main__":
    threading.Thread(target=start_browser_ws_server, args=(BROWSER_WS_PORT,), daemon=True).start()
    threading.Thread(target=prototype_manager_thread, daemon=True).start() # prototype thread

    if IS_AUTO_ARCHIVING:
        threading.Thread(target=archiver_thread, daemon=True).start()
        print("[ARCHIVER] auto-archiving enabled")
    else:
        print("[ARCHIVER] auto-archiving disabled")

    run_webserver(FLASK_PORT)
