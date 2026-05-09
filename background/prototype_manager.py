from threading import Event
from background._prototype_thread import PrototypeSerialThread
from background.image_generator import get_stroke_segments, clear_stroke_segments, reset_canvas


class PrototypeManager:
    def __init__(self, ws_server=None, prefer_extraction=True):
        self.thread = None
        self.stop_event = Event()
        self.ws_server = ws_server
        # True means: when the tracker exposes recognition/extraction coordinates,
        # render those black output coordinates instead of the legacy blue pen-tip path.
        self.prefer_extraction = prefer_extraction

    def start(self, ws_server=None, prefer_extraction=None, clear_coordinates=True):
        if self.thread and self.thread.is_alive():
            print("Thread already running.")
            return False

        if ws_server is not None:
            self.ws_server = ws_server
        if prefer_extraction is not None:
            self.prefer_extraction = bool(prefer_extraction)
        if clear_coordinates:
            clear_stroke_segments()

        self.stop_event.clear()
        self.thread = PrototypeSerialThread(
            self.stop_event,
            ws_server=self.ws_server,
            prefer_extraction=self.prefer_extraction,
        )
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
            self.thread = None
        return True

    def get_stroke_coordinates(self, space="canvas"):
        """Return rendered black-line stroke coordinates.

        space may be "canvas", "mjpeg", "meters", or "all". Each item is a
        segment, not just a point, so callers can replay/export the exact black
        lines that were rendered.
        """
        return get_stroke_segments(space=space)

    def clear_stroke_coordinates(self):
        clear_stroke_segments()

    def reset_rendered_image(self, clear_coordinates=True):
        reset_canvas(clear_coordinates=clear_coordinates)
