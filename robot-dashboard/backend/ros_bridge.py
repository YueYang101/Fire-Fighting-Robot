#
# ros_bridge.py
#   Connects to rosbridge_websocket (on your Pi) and provides:
#     • async set_motor(motor_id, direction, speed)
#     • register_state_callback(cb) → cb(msg_dict) on every /motor_X/state
#
# Requires:   pip install roslibpy
#

from __future__ import annotations
import asyncio, threading
from typing import Callable, List

import roslibpy

class RosBridge:
    def __init__(self, host: str = "192.168.2.4", port: int = 9090) -> None:
        self._ros = roslibpy.Ros(host=host, port=port)

        # event hooks
        self._ros.on_ready(lambda: print("✅  Connected to rosbridge"))
        self._ros.on("close",  lambda: print("🔌  rosbridge closed"))
        self._ros.on("error",  lambda err: print("⛔  rosbridge error:", err))

        # start connection (threaded or via run())
        if hasattr(self._ros, "run_threaded"):
            self._ros.run_threaded()
        else:
            threading.Thread(target=self._ros.run, daemon=True).start()

        # /set_motor service proxy
        self._set_motor_srv = roslibpy.Service(
            self._ros,
            "/set_motor",
            "pca9685_motor_driver_py/srv/SetMotor",
        )

        # fan-out plumbing
        self._callbacks: List[Callable[[dict], None]] = []
        self._subscribe_states()

    async def set_motor(self, motor_id: int, direction: str, speed: int) -> dict:
        """Awaitable proxy for the /set_motor ROS-2 service."""
        req = roslibpy.ServiceRequest(
            {"motor_id": motor_id, "direction": direction, "speed": speed}
        )
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self._set_motor_srv.call, req)

    def register_state_callback(self, cb: Callable[[dict], None]) -> None:
        """Register a callback to receive every /motor_X/state message."""
        self._callbacks.append(cb)

    def _subscribe_states(self) -> None:
        for i in range(4):
            topic = roslibpy.Topic(
                self._ros,
                f"/motor_{i}/state",
                "pca9685_motor_driver_py/msg/MotorState",
            )
            topic.subscribe(self._fan_out_state)

    def _fan_out_state(self, msg: dict) -> None:
        for cb in self._callbacks:
            cb(msg)