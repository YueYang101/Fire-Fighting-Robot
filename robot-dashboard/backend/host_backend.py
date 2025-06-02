"""
Main backend – runs on your laptop / dev PC.

* POST /start  ┐   proxies to the tiny Pi backend
* POST /stop   │   (HTTP hop)
* GET  /status ┘
* POST /set_motor      -> calls /set_motor via rosbridge
* WS   /ws/motor_state -> streams /motor_* /state to browsers
"""

import asyncio, httpx, signal
from typing import Literal, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from ros_bridge import RosBridge

# ───────────────────────── Pi backend address ──────────────────────────── #
PI_API = "http://192.168.2.4:5000"          # ← replace if Pi IP/port differ
TIMEOUT = 5.0                               # seconds for HTTP hop

# ───────────────────────── FastAPI plumbing ─────────────────────────────── #
app = FastAPI(title="ROS-2 Motor Dashboard – Main backend")
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

ros = RosBridge()                           # real-time ROS traffic

# ───────────────────────────── Helpers ──────────────────────────────────── #
async def _pi_cmd(endpoint: str) -> dict:
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
async def start():   return await _pi_cmd("start")

@app.post("/stop")
async def stop():    return await _pi_cmd("stop")

@app.get("/status")
async def status():  return await _pi_cmd("status")

@app.post("/set_motor")
async def set_motor(cmd: MotorCmd) -> dict:
    try:
        res = await ros.set_motor(cmd.motor_id, cmd.direction, cmd.speed)
    except Exception as exc:
        raise HTTPException(500, f"ROS service error: {exc}") from exc
    return {"result": res}

@app.websocket("/ws/motor_state")
async def motor_state(ws: WebSocket):
    await ws.accept()
    q: asyncio.Queue = asyncio.Queue()
    ros.register_state_callback(q.put_nowait)
    try:
        while True:
            msg = await q.get()
            await ws.send_json(msg)
    except WebSocketDisconnect:
        pass