import threading
from app.webserver import run_webserver
from prototype import ws_client_thread, start_browser_ws_server
# from image_processing import archiver_thread
from image_processing import archiver_thread, archiving_enabled_event
from config import IS_AUTO_ARCHIVING, FLASK_PORT, BROWSER_WS_PORT, config_done_event





if __name__ == "__main__":
    threading.Thread(target=start_browser_ws_server, args=(BROWSER_WS_PORT,), daemon=True).start()
    threading.Thread(target=archiver_thread, args=(archiving_enabled_event,), daemon=True).start()
    
    
    # Flags
    if IS_AUTO_ARCHIVING:
        threading.Thread(target=archiver_thread, daemon=True).start()
        print("[ARCHIVER] auto-archiving enabled")
    else:
        print("[ARCHIVER] auto-archiving disabled")

    # MAIN THREAD: Run the webserver independently
    run_webserver(FLASK_PORT)
