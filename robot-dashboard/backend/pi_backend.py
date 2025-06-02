"""
pi_backend.py
FastAPI micro-backend that runs **on the Pi**.

POST /start   – start rosbridge_websocket + motor_driver_node
POST /stop    – stop both
GET|POST /status – JSON state (+ last reason)

All stdout/stderr from the two subprocesses is streamed into the logger,
so `sudo journalctl -u pi-backend -f` shows every crash immediately.
"""

from __future__ import annotations
import asyncio, os, subprocess, sys, time
from pathlib import Path
from typing import Optional

import logging
from logging.handlers import TimedRotatingFileHandler

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

# ───────────────────────── logging ──────────────────────────────────────────
def _make_logger() -> logging.Logger:
    log = logging.getLogger("pi_backend")
    log.setLevel(logging.INFO)

    fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
        "%Y-%m-%d %H:%M:%S",
    )

    # journalctl / stdout
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    log.addHandler(sh)

    # rotating file (daily, keep 5)
    for path in ("/var/log/pi_backend.log", "/tmp/pi_backend.log"):
        try:
            fh = TimedRotatingFileHandler(path, when="midnight", backupCount=5)
            fh.setFormatter(fmt)
            log.addHandler(fh)
            break
        except PermissionError:
            continue
    return log


L = _make_logger()

# ───────────────────────── paths & commands ─────────────────────────────────
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/ubuntu-robot-pi4/ros2_ws/install/setup.bash"

PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

BRIDGE_CMD = [
    "bash",
    "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
]

MOTOR_CMD = [
    "bash",
    "-lc",
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

# ───────────────────────── FastAPI app ──────────────────────────────────────
app = FastAPI(title="ROS-2 Motor Pi backend")
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

bridge_proc: Optional[subprocess.Popen] = None
motor_proc:  Optional[subprocess.Popen] = None
bridge_reason = "not started"
motor_reason  = "not started"
_last_start   = 0.0   # debounce timer

# ───────────────────────── helpers ──────────────────────────────────────────
def _running(p: Optional[subprocess.Popen]) -> bool:
    return p is not None and p.poll() is None


def _stream_output(name: str, pipe) -> None:
    for line in iter(pipe.readline, b""):
        L.info("%s | %s", name, line.decode(errors="replace").rstrip())
    pipe.close()


async def _launch(name: str, cmd: list[str]) -> subprocess.Popen:
    L.info("%s: launching…", name)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setpgrp,
        bufsize=1,
    )
    # background task → forward every log line into python logger
    asyncio.get_running_loop().run_in_executor(None, _stream_output, name, p.stdout)
    await asyncio.sleep(1)          # give it a moment to crash if mis-configured
    if _running(p):
        L.info("%s is running (pid %d)", name, p.pid)
    else:
        L.error("%s exited immediately with code %s", name, p.returncode)
    return p


def _terminate(name: str, p: Optional[subprocess.Popen]) -> None:
    if not _running(p):
        return
    L.info("%s: terminating (pid %d)…", name, p.pid)
    p.terminate()
    try:
        p.wait(5)
    except subprocess.TimeoutExpired:
        L.warning("%s did not exit -> SIGKILL", name)
        p.kill()

# ───────────────────────── routes ───────────────────────────────────────────
@app.post("/start")
async def start():
    global bridge_proc, motor_proc, bridge_reason, motor_reason, _last_start

    # 2-second debounce (double-click protection)
    now = time.time()
    if now - _last_start < 2:
        return {"status": "ignored (debounce)"}
    _last_start = now

    if _running(bridge_proc) and _running(motor_proc):
        return {"status": "already running",
                "bridge_pid": bridge_proc.pid, "motor_pid": motor_proc.pid}

    bridge_proc  = await _launch("rosbridge",  BRIDGE_CMD)
    bridge_reason = "running" if _running(bridge_proc) else "failed"

    motor_proc   = await _launch("motor_driver", MOTOR_CMD)
    motor_reason  = "running" if _running(motor_proc) else "failed"

    if not _running(bridge_proc) or not _running(motor_proc):
        _terminate("motor_driver", motor_proc)
        _terminate("rosbridge",   bridge_proc)
        raise HTTPException(500, "one or both subprocesses failed to start")

    return {"status": "started",
            "bridge_pid": bridge_proc.pid, "motor_pid": motor_proc.pid}


@app.post("/stop")
def stop():
    global bridge_proc, motor_proc, bridge_reason, motor_reason
    if not (_running(bridge_proc) or _running(motor_proc)):
        return {"status": "not running"}
    _terminate("motor_driver", motor_proc);  motor_reason  = "stopped"
    _terminate("rosbridge",   bridge_proc);  bridge_reason = "stopped"
    return {"status": "stopped"}


@app.get("/status")
@app.post("/status")        # <-- added POST endpoint
def status():
    return {
        "bridge_running": _running(bridge_proc),
        "bridge_pid":     bridge_proc.pid if _running(bridge_proc) else None,
        "bridge_reason":  bridge_reason,
        "motor_running":  _running(motor_proc),
        "motor_pid":      motor_proc.pid if _running(motor_proc) else None,
        "motor_reason":   motor_reason,
    }