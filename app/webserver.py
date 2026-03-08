from app import create_app, db



# ---------------------------------------------------------------------
# Flask app initialization
# ---------------------------------------------------------------------
app = create_app()
app.secret_key = "polycast-creator_BatsiKuruSyaniOmit"


# ---------------------------------------------------------------------
# UTILITIES
# ---------------------------------------------------------------------
def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,
        threaded=True # CRITICAL for SSE
    )

if __name__ == "__main__":
    run_webserver(5000)