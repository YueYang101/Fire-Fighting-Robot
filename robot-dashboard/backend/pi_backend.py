#
# pi_backend.py  – FastAPI micro-backend that runs ON THE PI
#
#   • POST  /start      → start rosbridge_websocket + motor_driver_node
#   • POST  /stop       → terminate both
#   • GET   /status     → show “running” booleans + last reason
#   • POST  /status     → same as GET /status (avoids 405 if the client does POST)
#   • WS    /ws/logs    → stream the Pi‐side backend’s own logs (stdout/stderr from rosbridge + motor_driver_node)
#
# Pros:
#   – No roslibpy here – we just launch subprocesses directly,
#     grab their stdout/stderr, and push lines to a WebSocket endpoint
#     plus into the systemd journal (via logging.StreamHandler).
#
# Usage (systemd):
#   $ sudo cp pi_backend.py /home/ubuntu-robot-pi4/robot-dashboard/backend
#   $ cp /home/ubuntu-robot-pi4/robot-dashboard/scripts/run-pi-backend.sh /home/ubuntu-robot-pi4/robot-dashboard/scripts/run-pi-backend.sh
#   $ sudo cp pi-backend.service /etc/systemd/system/pi-backend.service
#   $ sudo systemctl daemon-reload
#   $ sudo systemctl enable --now pi-backend
#

from __future__ import annotations
import asyncio, os, signal, subprocess, sys
from pathlib import Path
from typing import Optional

import logging
from logging.handlers import TimedRotatingFileHandler

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect

# ───────────────────────── logging ───────────────────────────────────────────
LOG_PATH = "/var/log/pi_backend.log"

LOGGER = logging.getLogger("pi_backend")
LOGGER.setLevel(logging.INFO)

fmt = logging.Formatter(
    "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    "%Y-%m-%d %H:%M:%S",
)

# 1) journal/stdout
_stream = logging.StreamHandler(sys.stdout)
_stream.setFormatter(fmt)
LOGGER.addHandler(_stream)

# 2) rotating file (daily, keep 5 backups); might fail if permission denied
try:
    _file = TimedRotatingFileHandler(LOG_PATH, when="midnight", backupCount=5)
    _file.setFormatter(fmt)
    LOGGER.addHandler(_file)
except PermissionError:
    LOGGER.warning("cannot write %s (permission denied) – file logging disabled", LOG_PATH)

# ───────────────────────── paths & commands ──────────────────────────────────
ROS_SETUP  = "/opt/ros/humble/setup.bash"      # adjust if your ROS distro differs
WS_SETUP   = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"

PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
]

MOTOR_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}"
]

# ───────────────────────── FastAPI app ───────────────────────────────────────
app = FastAPI(title="Pi micro-backend with logging")

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None

# track “reason” strings for status
bridge_reason: str = "not started"
motor_reason:  str = "not started"

# ───────────────────────── helpers ────────────────────────────────────────────
def _running(p: Optional[subprocess.Popen]) -> bool:
    return p is not None and p.poll() is None

def _stream_output(name: str, pipe) -> None:
    """
    Read one subprocess pipe (stdout/stderr) line‐by‐line and log it as INFO.
    Each line is tagged with the process name, e.g. "rosbridge | [INFO] ...".
    """
    for raw in iter(pipe.readline, b""):
        try:
            text = raw.decode(errors="replace").rstrip()
        except Exception:
            text = raw.decode("utf-8", "ignore").rstrip()
        LOGGER.info("%s | %s", name, text)
    pipe.close()

async def _launch(name: str, cmd: list[str]) -> subprocess.Popen:
    """
    Spawn the given command.  Redirect its stdout+stderr to a PIPE, then
    spin up a background thread to forward lines into LOGGER.  Return the Popen.
    """
    LOGGER.info("%s: launching…", name)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,
        bufsize=1,         # line buffering (note: on Python 3.10+, line buffering on binary isn't exactly honored)
    )
    # Launch a background task that reads p.stdout line-by-line and logs it
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, _stream_output, name, p.stdout)

    # Give it a second to fail early if it must
    await asyncio.sleep(1.0)
    if p.poll() is not None:
        LOGGER.error("%s exited early with code %s", name, p.returncode)
    else:
        LOGGER.info("%s is running (pid %d)", name, p.pid)
    return p

def _terminate(name: str, p: Optional[subprocess.Popen]) -> None:
    """ If process is running, terminate + wait up to 5 s; kill if it hangs."""
    if _running(p):
        LOGGER.info("%s: terminating (pid %d)…", name, p.pid)
        p.terminate()
        try:
            p.wait(5.0)
            LOGGER.info("%s terminated with code %s", name, p.returncode)
        except subprocess.TimeoutExpired:
            LOGGER.warning("%s did not exit – killing", name)
            p.kill()

# ───────────────────────────── Routes ────────────────────────────────────────
@app.post("/start")
async def start():
    """
    Launch both rosbridge and motor_driver_node (if not already running).
    """
    global bridge_proc, motor_proc, bridge_reason, motor_reason

    if _running(bridge_proc) and _running(motor_proc):
        return {
            "status":     "already running",
            "bridge_pid": bridge_proc.pid,
            "motor_pid":  motor_proc.pid,
        }

    # Launch rosbridge_websocket
    bridge_proc  = await _launch("rosbridge", BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"

    # Launch motor_driver_node
    motor_proc   = await _launch("motor_driver", MOTOR_CMD)
    motor_reason  = "running" if _running(motor_proc) else "failed"

    if not _running(bridge_proc) or not _running(motor_proc):
        # If either failed to start, shut down whichever launched
        _terminate("motor_driver", motor_proc)
        _terminate("rosbridge",   bridge_proc)
        raise HTTPException(500, "one or both subprocesses failed to start")

    return {
        "status":     "started",
        "bridge_pid": bridge_proc.pid,
        "motor_pid":  motor_proc.pid,
    }

@app.post("/stop")
@app.get("/stop")
def stop():
    """
    Terminate both rosbridge and motor_driver_node (if they are running).
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
@app.post("/status")
def status():
    """
    Return JSON describing whether each subprocess is running, its PID, and the last “reason”.
    Example:
      {
        "bridge_running": true,
        "bridge_pid": 48653,
        "bridge_reason": "running",
        "motor_running": true,
        "motor_pid": 48668,
        "motor_reason": "running"
      }
    """
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid":     bridge_proc.pid if _running(bridge_proc) else None,
        "bridge_reason":  bridge_reason,
        "motor_running":  _running(motor_proc),
        "motor_pid":      motor_proc.pid if _running(motor_proc) else None,
        "motor_reason":   motor_reason,
    }

@app.websocket("/ws/logs")
async def logs_stream(ws: WebSocket):
    """
    Stream the pi_backend logger’s lines (as they appear in the journal) to the client.
    We simply forward the same lines that were logged to stdout/stdin when launching.
    """
    await ws.accept()
    queue: asyncio.Queue[str] = asyncio.Queue()

    # Helper: whenever the LOGGER emits a record, push it onto the queue.
    # Note: this is a very simplistic “fan‐out” – every time LOGGER.info(...) is called,
    # we also do `queue.put_nowait(formatted_line)`.
    class QHandler(logging.Handler):
        def emit(self, record):
            try:
                line = self.format(record)
                if line:
                    queue.put_nowait(line)
            except Exception:
                pass

    handler = QHandler()
    handler.setFormatter(fmt)
    LOGGER.addHandler(handler)

    try:
        while True:
            line = await queue.get()
            await ws.send_text(line)
    except WebSocketDisconnect:
        # remove our temporary handler
        LOGGER.removeHandler(handler)
        pass