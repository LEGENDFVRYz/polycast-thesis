import os
from app import create_app, db
from config import FLASK_PORT, IS_PROD
from background.image_generator import start_encoder_thread


# ---------------------------------------------------------------------
# Flask app initialization
# ---------------------------------------------------------------------
app = create_app()

# Start the single encoder thread once per worker process.
# Skip the Werkzeug reloader's parent watcher (it never serves requests).
if not (os.environ.get("FLASK_ENV") == "development" and os.environ.get("WERKZEUG_RUN_MAIN") != "true" and not IS_PROD):
    start_encoder_thread()


# ---------------------------------------------------------------------
# UTILITIES
# ---------------------------------------------------------------------
def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=not IS_PROD,
        threaded=True # CRITICAL for SSE
    )

if __name__ == "__main__":
    run_webserver(FLASK_PORT)