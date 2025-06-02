#
# host_backend.py  – runs on your laptop / dev PC.
#
#   • POST  /start       → proxy to Pi’s /start
#   • POST  /stop        → proxy to Pi’s /stop
#   • GET   /status      → proxy to Pi’s /status
#   • POST  /set_motor   → call the ROS‐2 /set_motor service via roslibpy
#   • WS    /ws/motor_state → fan‐out /motor_X/state → browser
#

import asyncio, httpx, signal
from typing import Literal, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from ros_bridge import RosBridge   # you must have this in the same folder

# ───────────────────────── Pi backend address ──────────────────────────── #
PI_API = "http://192.168.2.4:5000"          # ← replace with your Pi’s IP/port
TIMEOUT = 5.0                               # seconds for HTTP hop

# ───────────────────────── FastAPI plumbing ─────────────────────────────── #
app = FastAPI(title="ROS-2 Motor Dashboard – Main backend")
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

ros = RosBridge()  # real-time ROS traffic via roslibpy against rosbridge_websocket

# ───────────────────────────── Helpers ──────────────────────────────────── #
async def _pi_cmd(endpoint: str) -> dict:
    """
    Send a POST to http://<PI>/endpoint.  Raise HTTPException(502) if it fails.
    """
    url = f"{PI_API}/{endpoint.lstrip('/')}"
    async with httpx.AsyncClient() as client:
        try:
            r = await client.post(url, timeout=TIMEOUT)
            r.raise_for_status()
        except Exception as exc:
            raise HTTPException(502, f"Pi backend error: {exc}") from exc
    return r.json()

# ───────────────────────────── Models ───────────────────────────────────── #
class MotorCmd(BaseModel):
    motor_id: int = Field(ge=0, le=3)
    direction: Literal["forward", "backward", "brake"]
    speed: int = Field(ge=0, le=65535)

# ───────────────────────────── Routes ───────────────────────────────────── #
@app.get("/", response_class=PlainTextResponse)
def root() -> str:
    return "Main backend alive. Endpoints: /start /stop /set_motor /status /ws/motor_state"

@app.post("/start")
async def start() -> dict:
    return await _pi_cmd("start")

@app.post("/stop")
async def stop() -> dict:
    return await _pi_cmd("stop")

@app.get("/status")
async def status() -> dict:
    return await _pi_cmd("status")

@app.post("/set_motor")
async def set_motor(cmd: MotorCmd) -> dict:
    """
    Proxy for the ROS 2 /set_motor service — send via rosbridge to Pi.
    """
    try:
        result = await ros.set_motor(cmd.motor_id, cmd.direction, cmd.speed)
    except Exception as exc:
        raise HTTPException(500, f"ROS service error: {exc}") from exc
    return {"result": result}

@app.websocket("/ws/motor_state")
async def motor_state(ws: WebSocket):
    """
    Push every MotorState msg as JSON to connected browsers.
    """
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue()

    # Fan-out callback from roslibpy
    def _enqueue(msg: dict):
        queue.put_nowait(msg)

    ros.register_state_callback(_enqueue)

    try:
        while True:
            msg = await queue.get()
            await ws.send_json(msg)
    except WebSocketDisconnect:
        pass