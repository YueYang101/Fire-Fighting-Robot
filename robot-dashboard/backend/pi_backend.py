# pi_backend.py
# FastAPI micro‐backend that runs ON THE PI.  Now with an automatic "startup" hook
# to begin rosbridge_websocket + motor_driver_node without having to POST /start.

from __future__ import annotations
import asyncio
import os
import signal
import subprocess
from pathlib import Path
from typing import Optional

import logging
import sys
from logging.handlers import TimedRotatingFileHandler

from fastapi import FastAPI, HTTPException

# ────────────────────────── Logging Setup ──────────────────────────

LOG_PATH = "/var/log/pi_backend.log"
LOGGER = logging.getLogger("pi_backend")
LOGGER.setLevel(logging.INFO)

fmt = logging.Formatter(
    "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    "%Y-%m-%d %H:%M:%S",
)

# 1) Stream handler → stdout/journal
_stream = logging.StreamHandler(sys.stdout)
_stream.setFormatter(fmt)
LOGGER.addHandler(_stream)

# 2) Optional file logger (rotates daily, keeps 5 days’ logs)
try:
    _file = TimedRotatingFileHandler(LOG_PATH, when="midnight", backupCount=5)
    _file.setFormatter(fmt)
    LOGGER.addHandler(_file)
except PermissionError:
    LOGGER.warning("cannot write %s (permission denied) – file logging disabled", LOG_PATH)

# ─────────────────────── Paths & ROS Commands ───────────────────────

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"

# Make sure this path matches exactly where your .yaml lives:
PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

# Launch rosbridge over WebSocket (port 9090 by default)
BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]

# Launch the low‐level PCA9685 motor_driver node
MOTOR_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ─────────────────────── FastAPI App Definition ───────────────────────

app = FastAPI(title="Pi micro‐backend (auto‐start)")

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None

bridge_reason: str = "not started"
motor_reason:  str = "not started"

# ──────────────────────── Helper Functions ──────────────────────────

def _running(p: Optional[subprocess.Popen]) -> bool:
    return (p is not None and p.poll() is None)

def _stream_output(name: str, pipe) -> None:
    """
    Reads a subprocess’s stdout/stderr line by line and logs it under 'name'.
    Since we passed stdout=PIPE, we must consume it here.
    """
    for line in iter(pipe.readline, b''):
        LOGGER.info("%s | %s", name, line.decode(errors="replace").rstrip())
    pipe.close()

async def _launch(name: str, cmd: list[str]) -> subprocess.Popen:
    """
    Actually spawn the child process (rosbridge or motor_driver). Then
    spin up a background thread to forward stdout → our logger.
    """
    LOGGER.info("%s: launching…", name)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,  # detach signal handling from FastAPI
        bufsize=1,
    )
    # Run a thread to read and log each line from p.stdout
    asyncio.get_event_loop().run_in_executor(None, _stream_output, name, p.stdout)

    # Give it a moment to fail immediately if it’s going to fail
    await asyncio.sleep(1)
    if p.poll() is not None:
        LOGGER.error("%s exited early with code %s", name, p.returncode)
    else:
        LOGGER.info("%s is running (pid %d)", name, p.pid)
    return p

def _terminate(name: str, p: Optional[subprocess.Popen]) -> None:
    """
    If still running, terminate gracefully; if it refuses, kill.
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

# ────────────────────────── FastAPI Routes ──────────────────────────

@app.post("/start")
async def start():
    """
    POST /start → spawn rosbridge_websocket & motor_driver_node. If
    already running, simply return current PIDs. Otherwise, try to launch
    both; on failure, shut down any partial children and raise 500.
    """
    global bridge_proc, motor_proc, bridge_reason, motor_reason

    if _running(bridge_proc) and _running(motor_proc):
        return {
            "status": "already running",
            "bridge_pid": bridge_proc.pid,
            "motor_pid": motor_proc.pid,
        }

    # Launch rosbridge and motor_driver in sequence
    bridge_proc = await _launch("rosbridge", BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"

    motor_proc = await _launch("motor_driver", MOTOR_CMD)
    motor_reason = "running" if _running(motor_proc) else "failed"

    if not _running(bridge_proc) or not _running(motor_proc):
        # If either failed, clean up and report error
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
    POST /stop → terminate both rosbridge & motor_driver (if running).
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
    GET /status → return JSON {bridge_running, bridge_pid, bridge_reason,
    motor_running, motor_pid, motor_reason}.
    """
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid":     bridge_proc.pid if _running(bridge_proc) else None,
        "bridge_reason":  bridge_reason,
        "motor_running":  _running(motor_proc),
        "motor_pid":      motor_proc.pid if _running(motor_proc) else None,
        "motor_reason":   motor_reason,
    }

# ───────────────────── Automatic “Startup → POST /start” ────────────────────

@app.on_event("startup")
async def _auto_start_on_boot():
    """
    As soon as Uvicorn finishes its startup, this function will run—
    which in turn calls our own `/start` logic. That ensures that
    rosbridge and motor_driver are already running, without needing a
    manual curl. If either child process errors, you will see it in the logs.
    """
    # Delay a bit to make sure Uvicorn is fully listening on port 5000
    await asyncio.sleep(2.0)
    try:
        LOGGER.info("auto_start: invoking internal /start() …")
        # Directly call our own handler. We do not need an HTTP request here,
        # because start() is just a normal Python coroutine.
        result = await start()
        LOGGER.info("auto_start: /start returned %s", result)
    except Exception as e:
        LOGGER.error("auto_start: failed to launch rosbridge+motor_driver: %s", e)