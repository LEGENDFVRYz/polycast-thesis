from app import create_app, db
from config import FLASK_PORT, IS_PROD


# ---------------------------------------------------------------------
# Flask app initialization
# ---------------------------------------------------------------------
app = create_app()


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