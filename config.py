import os

# Prototype endpoint
ESP32_WS_URL = "ws://192.168.1.8/ws"

# Canvas dimensions
CANVAS_WIDTH, CANVAS_HEIGHT = 35560, 22219
MJPEG_WIDTH, MJPEG_HEIGHT = 600, 400
MJPEG_FPS = 20

# Ports
BROWSER_WS_PORT = 5001
FLASK_PORT = 5050

# Paths
ARCHIVE_DIR = "archive"
LOG_DIR = "logs"

# Flags
IS_AUTO_ARCHIVING = False

# Create directories
os.makedirs(ARCHIVE_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Log paths
CONN_LOG = os.path.join(LOG_DIR, "conn.logs")
ERROR_LOG = os.path.join(LOG_DIR, "error.logs")