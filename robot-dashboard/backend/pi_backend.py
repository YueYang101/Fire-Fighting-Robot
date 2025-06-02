#!/usr/bin/env python3
"""
pi_backend.py
FastAPI micro-backend that runs ON THE PI and now offers

POST /start        start rosbridge_websocket + motor_driver_node
POST /stop         terminate both
GET  /status       JSON state + reasons
WS   /ws/logs      every log line as a text frame        <── NEW
"""

from __future__ import annotations
import asyncio, os, signal, subprocess, sys
from pathlib import Path
from typing import Optional, Deque

import logging
from logging.handlers import TimedRotatingFileHandler
from collections import deque

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect

# ───────────────────────── logging ─────────────────────────────────────────
LOG_PATH = "/var/log/pi_backend.log"
LOGGER   = logging.getLogger("pi_backend")
LOGGER.setLevel(logging.INFO)

fmt = logging.Formatter(
    "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    "%Y-%m-%d %H:%M:%S",
)

# stdout → journalctl
_stream = logging.StreamHandler(sys.stdout); _stream.setFormatter(fmt)
LOGGER.addHandler(_stream)

# optional rotating file
try:
    _file = TimedRotatingFileHandler(LOG_PATH, when="midnight", backupCount=5)
    _file.setFormatter(fmt); LOGGER.addHandler(_file)
except PermissionError:
    LOGGER.warning("cannot write %s (permission denied) – file logging disabled", LOG_PATH)

# **NEW** in-memory ring-buffer feeds connected WebSocket clients
_subscribers: set[asyncio.Queue[str]] = set()
_RING: Deque[str] = deque(maxlen=200)        # last 200 lines for late joiners
class _WSHandler(logging.Handler):
    def emit(self, record):
        line = fmt.format(record)
        _RING.append(line)
        for q in list(_subscribers):
            try:           q.put_nowait(line)
            except asyncio.QueueFull: pass
LOGGER.addHandler(_WSHandler())

# ───────────────────────── paths & commands (unchanged) ───────────────────
ROS_SETUP  = "/opt/ros/humble/setup.bash"
WS_SETUP   = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"
PARAM_FILE = "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/" \
             "pca9685_motor_driver_py/config/motor_map.yaml"

BRIDGE_CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]
MOTOR_CMD  = [
    "bash", "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ───────────────────────── FastAPI app ─────────────────────────────────────
app = FastAPI(title="Pi micro-backend with live logs")
bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None
bridge_reason = "not started"
motor_reason  = "not started"

# ───────────────────────── helpers (unchanged except logging) ─────────────
def _running(p): return p and p.poll() is None
def _stream_output(name, pipe):
    for line in iter(pipe.readline, b''):
        LOGGER.info("%s | %s", name, line.decode(errors="replace").rstrip())
    pipe.close()

async def _launch(name, cmd):
    LOGGER.info("%s: launching…", name)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, preexec_fn=os.setpgrp,
                         bufsize=1)
    asyncio.get_event_loop().run_in_executor(None, _stream_output, name, p.stdout)
    await asyncio.sleep(1)
    if p.poll() is not None:
        LOGGER.error("%s exited early with code %s", name, p.returncode)
    else:
        LOGGER.info("%s is running (pid %d)", name, p.pid)
    return p

def _terminate(name, p):
    if _running(p):
        LOGGER.info("%s: terminating (pid %d)…", name, p.pid)
        p.terminate()
        try: p.wait(5)
        except subprocess.TimeoutExpired:
            LOGGER.warning("%s didn’t exit – killing", name); p.kill()

# ───────────────────────── routes (HTTP) ──────────────────────────────────
@app.post("/start")
async def start():
    global bridge_proc, motor_proc, bridge_reason, motor_reason
    if _running(bridge_proc) and _running(motor_proc):
        return {"status": "already running", "bridge_pid": bridge_proc.pid,
                "motor_pid": motor_proc.pid}

    bridge_proc  = await _launch("rosbridge", BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"
    motor_proc   = await _launch("motor_driver", MOTOR_CMD)
    motor_reason  = "running" if _running(motor_proc) else "failed"

    if not _running(bridge_proc) or not _running(motor_proc):
        _terminate("motor_driver", motor_proc)
        _terminate("rosbridge", bridge_proc)
        raise HTTPException(500, "one or both subprocesses failed")

    return {"status": "started", "bridge_pid": bridge_proc.pid,
            "motor_pid": motor_proc.pid}

@app.post("/stop")
def stop():
    _terminate("motor_driver", motor_proc); _terminate("rosbridge", bridge_proc)
    return {"status": "stopped"}

@app.get("/status")
def status():
    return {"bridge_running": _running(bridge_proc),
            "bridge_pid": bridge_proc.pid if _running(bridge_proc) else None,
            "bridge_reason": bridge_reason,
            "motor_running": _running(motor_proc),
            "motor_pid": motor_proc.pid if _running(motor_proc) else None,
            "motor_reason": motor_reason}

# ───────────────────────── WebSocket /ws/logs ─────────────────────────────
@app.websocket("/ws/logs")
async def ws_logs(ws: WebSocket):
    await ws.accept()
    q: asyncio.Queue[str] = asyncio.Queue(100)
    # send recent history
    for line in _RING: await ws.send_text(line)
    _subscribers.add(q)
    try:
        while True:
            msg = await q.get()
            await ws.send_text(msg)
    except WebSocketDisconnect:
        pass
    finally:
        _subscribers.discard(q)