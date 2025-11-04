from threading import Event
from background._prototype_thread import PrototypeWSThread

class PrototypeManager:
    def __init__(self):
        self.thread = None
        self.stop_event = Event()

    def start(self):
        if self.thread and self.thread.is_alive():
            print("Thread already running.")
            return False
        self.stop_event.clear()
        self.thread = PrototypeWSThread(self.stop_event)
        self.thread.start()
        print("Thread started.")
        return True 

    def stop(self):
        if not self.thread or not self.thread.is_alive():
            print("No active thread to stop.")
            return False
        print("Stopping thread...")
        self.thread.stop()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            print("Warning: thread did not stop cleanly.")
        else:
            print("Thread stopped successfully.")
        return True

