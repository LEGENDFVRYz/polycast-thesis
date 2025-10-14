from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import asyncio
import threading
import xp_pen_api  # <-- your real API wrapper

app = FastAPI()

# Shared variable for latest datapoint
# x: 35560, y: 22219, pressure: 8191
latest_point = {"x": 0, "y": 0, "pressure": 0}

# Callback for XP-Pen SDK
def pen_callback(pkt_ptr):
    pkt = pkt_ptr.contents
    global latest_point
    latest_point = {
        "x": pkt.x,
        "y": pkt.pressure,      # remap
        "pressure": pkt.button  # remap
    }
    return 0

# Start XP-Pen API in background thread
api = xp_pen_api.XPPenAPI()
def run_api():
    api.start(pen_callback)
    while True:
        asyncio.sleep(0.1)

threading.Thread(target=run_api, daemon=True).start()

# REST endpoint (single read)
@app.get("/pen_api")
def get_pen_data():
    return latest_point

# Mount static folder
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def root():
    return FileResponse("static/index.html")

# WebSocket streaming
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(latest_point)
            await asyncio.sleep(0.005)  # ~20Hz update
    except Exception as e:
        print("WebSocket error:", e)
        await websocket.close()
