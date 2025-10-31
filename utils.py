import socket
import websocket
from datetime import datetime


def log_message(filepath, message):
    """Log messages with timestamps."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(filepath, "a", encoding="utf-8") as f:
        f.write(f"[{timestamp}] {message}\n")


def find_esp_ip(timeout=10):
    """
    Scans the network for the ESP32 broadcast for a limited time.
    Returns the IP string if found, else None.
    """
    UDP_PORT = 12345
    PROTOTYPE_IP = None
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Set a timeout on the socket itself
    sock.settimeout(timeout) 
    
    try:
        sock.bind(("", UDP_PORT))
        print(f"Waiting for ESP32 broadcast on port {UDP_PORT} for {timeout}s...")
        
        # Wait for a packet
        data, addr = sock.recvfrom(1024) 
        message = data.decode()
        
        if message.startswith("ESP_IP:"):
            PROTOTYPE_IP = message.split(":")[1]
            print(f"Got IP from ESP32 at {addr}: {PROTOTYPE_IP}")
            
    except socket.timeout:
        print("Socket timed out. No ESP32 IP found.")
        PROTOTYPE_IP = None # Explicitly set to None
        
    finally:
        sock.close()
        
    return PROTOTYPE_IP


def check_esp_ws_connection(ws_url):
    """
    Attempts a one-time, synchronous connection to a WebSocket URL.
    Returns True if successful, False otherwise.
    """
    try:
        # Create a connection with a short timeout
        print(f"[ADMIN CHECK] Attempting to connect to {ws_url}...")
        ws = websocket.create_connection(ws_url, timeout=3)
        
        # If we got this far, the handshake was successful.
        print(f"[ADMIN CHECK] Connection to {ws_url} SUCCEEDED.")
        ws.close()
        return True

    # This catches "Connection refused"
    except (ConnectionRefusedError, socket.error):
        print(f"[ADMIN CHECK] Connection to {ws_url} FAILED (Connection Refused).")
        return False
        
    # This catches "Operation timed out"
    except websocket.WebSocketTimeoutException:
        print(f"[ADMIN CHECK] Connection to {ws_url} FAILED (Timeout).")
        return False
        
    # This catches DNS errors (e.g., "invalid.hostname")
    except socket.gaierror:
        print(f"[ADMIN CHECK] Connection to {ws_url} FAILED (Invalid IP/Hostname).")
        return False
        
    # Catch any other WebSocket or general exceptions
    except Exception as e:
        print(f"[ADMIN CHECK] Connection to {ws_url} FAILED (Error: {e}).")
        return False