import threading
from app.webserver import run_webserver
from prototype import ws_client_thread, start_browser_ws_server
from config import IS_AUTO_ARCHIVING, FLASK_PORT, BROWSER_WS_PORT, config_done_event





if __name__ == "__main__":
    threading.Thread(target=start_browser_ws_server, args=(BROWSER_WS_PORT,), daemon=True).start()

    # MAIN THREAD: Run the webserver independently
    run_webserver(FLASK_PORT)
