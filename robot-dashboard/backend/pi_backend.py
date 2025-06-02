"""
pi_backend.py
FastAPI micro-backend that runs ON THE PI

POST /start   → start rosbridge_websocket + motor_driver_node
POST /stop    → terminate both
GET  /status  → JSON state + reason

All stdout/stderr from the two subprocesses is streamed into the logger so
`journalctl -u pi-backend` shows every crash instantly.
"""

from __future__ import annotations
import asyncio, os, signal, subprocess, sys
from pathlib import Path
from typing import Optional

import logging
from logging.handlers import TimedRotatingFileHandler

from fastapi import FastAPI, HTTPException

# ───────────────────────── logging ───────────────────────────────────────────
LOG_PATH = "/var/log/pi_backend.log"

LOGGER = logging.getLogger("pi_backend")
LOGGER.setLevel(logging.INFO)

fmt = logging.Formatter(
    "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    "%Y-%m-%d %H:%M:%S",
)

# journal / stdout
_stream = logging.StreamHandler(sys.stdout)
_stream.setFormatter(fmt)
LOGGER.addHandler(_stream)

# rotating file (daily, keep 5)
try:
    _file = TimedRotatingFileHandler(LOG_PATH, when="midnight", backupCount=5)
    _file.setFormatter(fmt)
    LOGGER.addHandler(_file)
except PermissionError:
    LOGGER.warning("cannot write %s (permission denied) – file logging disabled", LOG_PATH)

# ───────────────────────── paths & commands ──────────────────────────────────
ROS_SETUP  = "/opt/ros/humble/setup.bash"
WS_SETUP   = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"

PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]

MOTOR_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ───────────────────────── FastAPI app ───────────────────────────────────────
app = FastAPI(title="Pi micro-backend with logging")

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None
bridge_reason: str = "not started"
motor_reason:  str = "not started"

# ───────────────────────── helpers ───────────────────────────────────────────
def _running(p: Optional[subprocess.Popen]) -> bool:
    return p is not None and p.poll() is None

def _stream_output(name: str, pipe) -> None:
    """Read a subprocess pipe line-by-line and log it."""
    for line in iter(pipe.readline, b''):
        LOGGER.info("%s | %s", name, line.decode(errors="replace").rstrip())
    pipe.close()

async def _launch(name: str, cmd: list[str]) -> subprocess.Popen:
    LOGGER.info("%s: launching…", name)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,
        bufsize=1,
    )
    # background thread to forward log lines
    asyncio.get_event_loop().run_in_executor(
        None, _stream_output, name, p.stdout)
    await asyncio.sleep(1)
    if p.poll() is not None:
        LOGGER.error("%s exited early with code %s", name, p.returncode)
    else:
        LOGGER.info("%s is running (pid %d)", name, p.pid)
    return p

def _terminate(name: str, p: Optional[subprocess.Popen]) -> None:
    if _running(p):
        LOGGER.info("%s: terminating (pid %d)…", name, p.pid)
        p.terminate()
        try:
            p.wait(5)
            LOGGER.info("%s terminated with code %s", name, p.returncode)
        except subprocess.TimeoutExpired:
            LOGGER.warning("%s did not exit – killing", name)
            p.kill()

# ───────────────────────── routes ────────────────────────────────────────────
@app.post("/start")
async def start():
    global bridge_proc, motor_proc, bridge_reason, motor_reason
    if _running(bridge_proc) and _running(motor_proc):
        return {
            "status": "already running",
            "bridge_pid": bridge_proc.pid,
            "motor_pid":  motor_proc.pid,
        }

    bridge_proc  = await _launch("rosbridge", BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"

    motor_proc   = await _launch("motor_driver", MOTOR_CMD)
    motor_reason  = "running" if _running(motor_proc) else "failed"

    if not _running(bridge_proc) or not _running(motor_proc):
        _terminate("motor_driver", motor_proc)
        _terminate("rosbridge",   bridge_proc)
        raise HTTPException(500, "one or both subprocesses failed to start")

    return {
        "status":      "started",
        "bridge_pid":  bridge_proc.pid,
        "motor_pid":   motor_proc.pid,
    }

@app.post("/stop")
def stop():
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
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid":     bridge_proc.pid if _running(bridge_proc) else None,
        "bridge_reason":  bridge_reason,
        "motor_running":  _running(motor_proc),
        "motor_pid":      motor_proc.pid if _running(motor_proc) else None,
        "motor_reason":   motor_reason,
    }