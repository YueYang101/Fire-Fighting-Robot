"""
pi_backend.py  – FastAPI micro-backend that runs ON THE PI

• POST /start  → start rosbridge_websocket + motor_driver_node
• POST /stop   → terminate both
• GET  /status → show pids / running state

Runs under systemd (see pi-backend.service). No roslibpy needed.
"""

from __future__ import annotations
import asyncio, os, signal, subprocess
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException

# ───────────────────────── configurable paths ──────────────────────────────
ROS_SETUP  = "/opt/ros/humble/setup.bash"

PARAM_FILE = Path(
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

# ───────────────────────── commands to launch ──────────────────────────────
BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]

MOTOR_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

app = FastAPI(title="Pi micro-backend")

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None


# ───────────────────────── helpers ───────────────────────────────
def _running(p: Optional[subprocess.Popen]) -> bool:
    return p is not None and p.poll() is None


async def _launch(cmd: list[str]) -> subprocess.Popen:
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,  # detach from FastAPI signal handling
    )
    await asyncio.sleep(1)     # give it a moment to fail if it's going to
    return p


def _terminate(p: Optional[subprocess.Popen]) -> None:
    if _running(p):
        p.terminate()
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


# ───────────────────────── routes ────────────────────────────────
@app.post("/start")
async def start():
    global bridge_proc, motor_proc
    if _running(bridge_proc) and _running(motor_proc):
        return {"status": "already running",
                "bridge_pid": bridge_proc.pid,
                "motor_pid":  motor_proc.pid}

    try:
        bridge_proc = await _launch(BRIDGE_CMD)
        motor_proc  = await _launch(MOTOR_CMD)
    except FileNotFoundError as exc:
        raise HTTPException(500, f"ROS command failed: {exc}") from exc

    if not _running(bridge_proc):
        raise HTTPException(500, "rosbridge failed to start")
    if not _running(motor_proc):
        _terminate(bridge_proc)
        raise HTTPException(500, "motor_driver_node failed to start")

    return {"status": "started",
            "bridge_pid": bridge_proc.pid,
            "motor_pid":  motor_proc.pid}


@app.post("/stop")
def stop():
    global bridge_proc, motor_proc
    if not (_running(bridge_proc) or _running(motor_proc)):
        return {"status": "not running"}

    _terminate(motor_proc)
    _terminate(bridge_proc)
    return {"status": "stopped"}


@app.get("/status")
def status():
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid": bridge_proc.pid if _running(bridge_proc) else None,
        "motor_running": _running(motor_proc),
        "motor_pid":  motor_proc.pid if _running(motor_proc) else None,
    }