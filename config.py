import os
import threading
import socket
import sys
import glob
import serial

# --- 2. Define the shared event here ---
config_done_event = threading.Event()

# Background Task: Thread Reference
prototype_reader_thread_stop_event = threading.Event()
prototype_reader_thread = None


# Canvas dimensions
CANVAS_WIDTH, CANVAS_HEIGHT = 1980, 1980
MJPEG_WIDTH, MJPEG_HEIGHT = 600, 600
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


class _Config:
    # ==================================================
    # SERIAL CONFIGURATION (New)
    # ==================================================
    # Windows: 'COM3', 'COM4', etc.
    # Linux/Mac: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    SERIAL_PORT = 'COM3' 
    
    # Must match the Serial.begin() in your ESP32 code
    BAUD_RATE = 115200

    # ==================================================
    # DEPRECATED / REMOVED
    # ==================================================
    # UDP_PORT = 12345        <-- No longer listening for UDP broadcasts
    # PROTOTYPE_IP = None     <-- Wired connection doesn't use IP
    # ESP32_WS_URL = ...      <-- We are reading Serial, not WS

    def list_serial_ports(self):
        """Helper to print available ports if you don't know which one to use."""
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

prototype_config = _Config()