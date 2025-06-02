"""
Pi-micro-backend: launch / stop the motor_driver_node locally.
No roslibpy here — just subprocess + ROS.
"""

import asyncio, signal, subprocess, os
from fastapi import FastAPI, HTTPException

ROS_SETUP  = "/opt/ros/humble/setup.bash"
PARAM_FILE = (
    "/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/"
    "pca9685_motor_driver_py/config/motor_map.yaml"
)

CMD = [
    "bash", "-lc",
    f"source {ROS_SETUP} && "
    "ros2 run pca9685_motor_driver_py motor_driver_node "
    f"--ros-args --params-file {PARAM_FILE}",
]

proc: subprocess.Popen | None = None
app = FastAPI()

def running(): return proc and proc.poll() is None

@app.post("/start")
async def start():
    global proc
    if running(): return {"status":"already running","pid":proc.pid}
    proc = subprocess.Popen(CMD, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.STDOUT,
                                  preexec_fn=os.setpgrp)  # detach from API
    await asyncio.sleep(1)
    if running(): return {"status":"started","pid":proc.pid}
    raise HTTPException(500, "motor_driver_node quit immediately")

@app.post("/stop")
def stop():
    global proc
    if not running(): return {"status":"not running"}
    proc.terminate(); proc.wait(5)
    return {"status":"stopped"}

@app.get("/status")
def status(): return {"running": running(), "pid": proc.pid if running() else None}