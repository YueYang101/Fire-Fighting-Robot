"""
FastAPI backend (auto-reload friendly) — exposes:

POST /start          -> launch motor_driver_node
POST /stop           -> terminate node
POST /set_motor      -> call /set_motor service via Python
WS   /ws/motor_state -> push live motor-state JSON frames

Run with:
    uvicorn server:app --reload --host 0.0.0.0 --port 8000
"""

import asyncio, signal, subprocess
from pathlib import Path
from typing import Literal, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from ros_bridge import RosBridge

# ───────────────────────── ROS 2 launch command ─────────────────────────── #

ROS_SETUP = "/opt/ros/humble/setup.bash"                      # adjust to taste
PARAM_FILE = Path(
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

CMD = [
    "bash",
    "-lc",
    f"source {ROS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ─────────────────────────── FastAPI plumbing ───────────────────────────── #

app = FastAPI(title="ROS 2 Motor Dashboard API")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

ros = RosBridge()  # <-- the Python side of rosbridge

_proc: Optional[subprocess.Popen] = None


def _running() -> bool:
    return _proc is not None and _proc.poll() is None


# ─────────────────────────────── Models ─────────────────────────────────── #

class MotorCmd(BaseModel):
    motor_id: int = Field(ge=0, le=3)
    direction: Literal["forward", "backward", "brake"]
    speed: int = Field(ge=0, le=65535)


# ───────────────────────────── Routes ───────────────────────────────────── #

@app.get("/", response_class=PlainTextResponse)
def root() -> str:
    return "Backend alive. Endpoints: /start /stop /set_motor /status /ws/motor_state"


@app.get("/status")
def status() -> dict:
    return {"running": _running(), "pid": _proc.pid if _running() else None}


@app.post("/start")
async def start() -> dict:
    global _proc
    if _running():
        return {"status": "already running", "pid": _proc.pid}

    try:
        _proc = subprocess.Popen(
            CMD,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
        )
    except FileNotFoundError as exc:
        raise HTTPException(500, f"ros2 launch failed: {exc}") from exc

    await asyncio.sleep(1)
    return {"status": "started", "pid": _proc.pid}


@app.post("/stop")
def stop() -> dict:
    global _proc
    if not _running():
        return {"status": "not running"}
    _proc.terminate()
    _proc.wait(5)
    return {"status": "stopped"}


@app.post("/set_motor")
async def set_motor(cmd: MotorCmd) -> dict:
    """
    Proxy for the ROS 2 /set_motor service — all Python, no JS.
    """
    try:
        result = await ros.set_motor(cmd.motor_id, cmd.direction, cmd.speed)
    except Exception as exc:  # broad: pass through any rosbridge issues
        raise HTTPException(500, f"ROS service error: {exc}") from exc
    return {"result": result}


@app.websocket("/ws/motor_state")
async def motor_state(ws: WebSocket):
    """
    Pushes every MotorState message as JSON to connected browsers.
    """
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue()

    # fan-out callback
    def _enqueue(msg: dict):
        queue.put_nowait(msg)

    ros.register_state_callback(_enqueue)

    try:
        while True:
            msg = await queue.get()
            await ws.send_json(msg)
    except WebSocketDisconnect:
        pass