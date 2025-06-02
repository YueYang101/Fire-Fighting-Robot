# pi_backend.py
# FastAPI micro‐backend that runs ON THE PI (port 5000).
# On uvicorn startup, it auto‐"POST /start" so rosbridge + motor_driver_node launch immediately.
#
# Endpoints:
#   POST   /start   → launch rosbridge_websocket (9090) & motor_driver_node
#   POST   /stop    → kill both rosbridge & motor_driver
#   GET    /status  → JSON state + reason
#   POST   /status  → same JSON (for convenience, so host can POST)
#
# All stdout/stderr from rosbridge and motor_driver_node get streamed to
# LOGGER → sys.stdout → journalctl -u pi-backend -f

from __future__ import annotations
import asyncio
import os
import subprocess
import logging
import sys
from logging.handlers import TimedRotatingFileHandler
from typing import Optional

from fastapi import FastAPI, HTTPException

# ────────────────────────── Logging Setup ──────────────────────────

LOG_PATH = "/var/log/pi_backend.log"
LOGGER = logging.getLogger("pi_backend")
LOGGER.setLevel(logging.INFO)

fmt = logging.Formatter(
    "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    "%Y-%m-%d %H:%M:%S",
)

# 1) Stream to stdout/journal
_stream = logging.StreamHandler(sys.stdout)
_stream.setFormatter(fmt)
LOGGER.addHandler(_stream)

# 2) File logger (rotates daily, keep 5 days) – may fail if no permission
try:
    _file = TimedRotatingFileHandler(LOG_PATH, when="midnight", backupCount=5)
    _file.setFormatter(fmt)
    LOGGER.addHandler(_file)
except PermissionError:
    LOGGER.warning("cannot write %s (permission denied) – file logging disabled", LOG_PATH)

# ─────────────────────── Paths & ROS Commands ───────────────────────

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"

PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

# Launch rosbridge over WebSocket (port 9090):
BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]

# Launch the PCA9685 motor_driver node:
MOTOR_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ────────────────────── FastAPI App Definition ──────────────────────

app = FastAPI(title="Pi micro‐backend (auto‐start)")

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None

bridge_reason: str = "not started"
motor_reason:  str = "not started"


# ───────────────────── Helper Functions ───────────────────────────

def _running(p: Optional[subprocess.Popen]) -> bool:
    return (p is not None and p.poll() is None)

def _stream_output(name: str, pipe) -> None:
    """
    Read each stdout/stderr line from 'pipe' and log it with LOGGER.
    """
    for line in iter(pipe.readline, b""):
        LOGGER.info("%s | %s", name, line.decode(errors="replace").rstrip())
    pipe.close()

async def _launch(name: str, cmd: list[str]) -> subprocess.Popen:
    """
    Spawn a subprocess (rosbridge or motor_driver).  Pipe its stdout/stderr
    into a background thread that logs to LOGGER.  Return the Popen.
    """
    LOGGER.info("%s: launching…", name)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,  # detach from FastAPI signal handling
        bufsize=1
    )
    # Spawn a background thread to read/forward all lines from p.stdout
    asyncio.get_event_loop().run_in_executor(None, _stream_output, name, p.stdout)
    await asyncio.sleep(1)  # allow immediate crash if it’s going to fail
    if p.poll() is not None:
        LOGGER.error("%s exited early with code %s", name, p.returncode)
    else:
        LOGGER.info("%s is running (pid %d)", name, p.pid)
    return p

def _terminate(name: str, p: Optional[subprocess.Popen]) -> None:
    """
    If process 'p' is still running, terminate it.  If it refuses after 5 s,
    kill it.  Log everything.
    """
    if _running(p):
        LOGGER.info("%s: terminating (pid %d)…", name, p.pid)
        p.terminate()
        try:
            p.wait(5)
            LOGGER.info("%s terminated with code %s", name, p.returncode)
        except subprocess.TimeoutExpired:
            LOGGER.warning("%s did not exit – killing", name)
            p.kill()


# ───────────────────────── FastAPI Routes ──────────────────────────

@app.post("/start")
async def start():
    """
    POST /start → launch rosbridge_websocket + motor_driver_node.
    If they’re already running, just return their PIDs + “already running.”
    On failure, shut down any partial children and throw HTTP 500.
    """
    global bridge_proc, motor_proc, bridge_reason, motor_reason

    if _running(bridge_proc) and _running(motor_proc):
        return {
            "status": "already running",
            "bridge_pid": bridge_proc.pid,
            "motor_pid": motor_proc.pid,
        }

    # 1) Launch rosbridge:
    bridge_proc = await _launch("rosbridge", BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"

    # 2) Launch motor_driver:
    motor_proc = await _launch("motor_driver", MOTOR_CMD)
    motor_reason = "running" if _running(motor_proc) else "failed"

    # If either failed, clean up:
    if not (_running(bridge_proc) and _running(motor_proc)):
        _terminate("motor_driver", motor_proc)
        _terminate("rosbridge", bridge_proc)
        raise HTTPException(500, "one or both subprocesses failed to start")

    return {
        "status":      "started",
        "bridge_pid":  bridge_proc.pid,
        "motor_pid":   motor_proc.pid,
    }


@app.post("/stop")
def stop():
    """
    POST /stop → terminate rosbridge & motor_driver if they’re running.
    """
    global bridge_proc, motor_proc, bridge_reason, motor_reason

    if not (_running(bridge_proc) or _running(motor_proc)):
        return {"status": "not running"}

    _terminate("motor_driver", motor_proc)
    motor_reason = "stopped"

    _terminate("rosbridge", bridge_proc)
    bridge_reason = "stopped"

    return {"status": "stopped"}


@app.get("/status")
def status():
    """
    GET /status → return JSON about whether rosbridge/motor_driver are running,
    plus their PIDs and textual “reason”.
    """
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid":     bridge_proc.pid if _running(bridge_proc) else None,
        "bridge_reason":  bridge_reason,
        "motor_running":  _running(motor_proc),
        "motor_pid":      motor_proc.pid if _running(motor_proc) else None,
        "motor_reason":   motor_reason,
    }

# Allow POST /status as well, so that the Host can do POST → /status:
@app.post("/status")
def status_post():
    return status()


# ───────────────────── Auto‐Start Hook ─────────────────────────────

@app.on_event("startup")
async def _auto_start_on_boot():
    """
    As soon as uvicorn is up and listening on port 5000,
    wait 2 s, then call our own `start()` coroutine so that
    rosbridge_websocket + motor_driver_node launch immediately.
    """
    await asyncio.sleep(2.0)
    try:
        LOGGER.info("auto_start: invoking internal /start() …")
        result = await start()  # directly call the POST /start logic
        LOGGER.info("auto_start: /start returned %s", result)
    except Exception as e:
        LOGGER.error("auto_start: failed to launch rosbridge+motor_driver: %s", e)