import threading
import json
import time

class AdminStatusManager:
    def __init__(self):
        self._status = {
            "admin_name": None,     # If none, "NO ADMIN YET"
            "status": "IDLE",       # IDLE, CONFIGURING, CONFIGURED, STARTING, HOSTING
            "hosting_active": False
        }
        
        self._lock = threading.Lock()
        self.status_changed = threading.Event()     # Signals when status changes

    
    # ------------------------------
    # READ operations
    # ------------------------------
    def get_status(self):
        """
        Return a copy of the current admin status.
        """
        with self._lock:
            return self._status.copy()
    
    def get_field(self, key):
        """
        Return specific field of the admin status
        """
        with self._lock:
            return self._status.get(key)
    
    
    # ------------------------------
    # WRITE operations
    # ------------------------------
    def _update_state(self, status=None, admin_name=None, hosting_active=None):
        """
        Update a field in the status and notify others.
        """
        changed = False
        
        with self._lock:
            updates = {}
            if status is not None:
                updates["status"] = status
            if admin_name is not None:
                updates["admin_name"] = admin_name
            if hosting_active is not None:
                updates["hosting_active"] = hosting_active

            if admin_name == "":
                updates["admin_name"] = None    # Force reset the admin name if empty
            
            # Apply updates only if something actually changed
            for key, new_value in updates.items():
                old_value = self._status.get(key)
                if old_value != new_value:
                    self._status[key] = new_value
                    changed = True
            
            if changed:
                self.status_changed.set()       # Trigger event
                self.status_changed.clear()     # Reset trigger
    
    
    def login(self, username):
        """
        Attempts to log in an admin.
        Returns False if another admin is active.
        """
        if self._status["admin_name"] and self._status["admin_name"] != username:
            return False 
        
        self._update_state(
            status="IDLE",
            admin_name=username,
            hosting_active=False
        )
        return True
    
    def reset(self):
        """
        Resets the admin state.
        """
        self._update_state(
            status="IDLE",
            admin_name="",
            hosting_active=False
        )