from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import random
import asyncio
import httpx

app = FastAPI()

# Dummy API (source of truth)
@app.get("/dummy_api")
def dummy_api():
    return {
        "x": random.randint(0, 500),
        "y": random.randint(0, 500),
        "pressure": round(random.random(), 2)
    }

# Mount static folder to serve HTML, JS, CSS, etc.
app.mount("/static", StaticFiles(directory="static"), name="static")

# Default route loads index.html
@app.get("/")
async def root():
    return FileResponse("static/index.html")

# WebSocket streaming dummy_api data
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    async with httpx.AsyncClient() as client:
        try:
            while True:
                resp = await client.get("http://127.0.0.1:8000/dummy_api")
                data = resp.json()
                await websocket.send_json(data)
                await asyncio.sleep(0.5)  # adjust refresh rate
        except Exception as e:
            print("WebSocket error:", e)
            await websocket.close()
