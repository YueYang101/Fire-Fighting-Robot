"""
Main backend – runs on your laptop / desktop.

 * POST /start      → proxies to the Pi backend’s /start
 * POST /stop       → proxies to the Pi backend’s /stop
 * GET  /status     → proxies to the Pi backend’s /status
 * POST /set_motor  → calls /set_motor via rosbridge (roslibpy)
 * WS   /ws/motor_state → streams /motor_X/state to browsers
 * WS   /ws/logs    → streams host‐backend logs to the browser
"""

import asyncio
import logging
import threading
import time                        # ← needed for timeout loop
from typing import Callable, List, Optional

import httpx
import roslibpy
from fastapi import (
    FastAPI,
    HTTPException,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

# ─────────────────────────── Configuration ──────────────────────────── #

# Change this to your Pi’s IP address:
PI_API = "http://192.168.2.4:5000"

# The Pi’s rosbridge_websocket is at port 9090 by default:
PI_ROSBRIDGE_HOST = "192.168.2.4"
PI_ROSBRIDGE_PORT = 9090

# HTTP timeout for calls to the Pi backend:
TIMEOUT = 5.0

# ─────────────────────────── Logging setup ───────────────────────────── #

logger = logging.getLogger("host_backend")
logger.setLevel(logging.DEBUG)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(
    logging.Formatter(
        "[%(asctime)s] %(levelname)s %(name)s: %(message)s",
        "%Y-%m-%d %H:%M:%S",
    )
)
logger.addHandler(stream_handler)

# ───────────────────────── RosBridge helper ──────────────────────────── #

class RosBridge:
    """
    Wraps roslibpy.Ros.  Spawns its event loop in a background thread.
    Provides:
      • async set_motor(motor_id, direction, speed)
      • register_state_callback(cb) → cb(msg_dict) on every /motor_X/state
    """

    def __init__(self, host: str, port: int):
        self._ros = roslibpy.Ros(host=host, port=port)

        # Log connection events:
        self._ros.on_ready(lambda: logger.info("✅ Connected to rosbridge"))
        self._ros.on("close", lambda _: logger.warning("🔌 Rosbridge closed"))
        self._ros.on("error", lambda err: logger.error("⛔ Rosbridge error: %s", err))

        # Start the ros client in a daemon thread:
        thread = threading.Thread(target=self._ros.run, daemon=True)
        thread.start()
        logger.debug("→ RosBridge.run() started in a daemon thread")

        # Wait up to `timeout` seconds for the WebSocket to connect:
        if not self._wait_for_connection(timeout=5.0):
            logger.error("❌ Failed to connect to rosbridge within 5s")

        # Prepare the /set_motor service proxy
        self._set_motor_srv = roslibpy.Service(
            self._ros,
            "/set_motor",
            "pca9685_motor_driver_py/srv/SetMotor",
        )

        # Hold all callbacks that want motor state updates:
        self._callbacks: List[Callable[[dict], None]] = []
        self._subscribe_states()

    def _wait_for_connection(self, timeout: float) -> bool:
        """
        Poll up to `timeout` seconds for rosbridge to say `is_connected == True`.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._ros.is_connected:
                return True
            time.sleep(0.1)
        return False

    def _subscribe_states(self) -> None:
        """
        Create 4 topic subscribers: /motor_0/state, /motor_1/state, /motor_2/state, /motor_3/state.
        Fan the incoming messages out to any registered callbacks.
        """
        for motor_id in range(4):
            topic_name = f"/motor_{motor_id}/state"
            topic = roslibpy.Topic(
                self._ros,
                topic_name,
                "pca9685_motor_driver_py/msg/MotorState",
            )
            topic.subscribe(self._fan_out_state)
            logger.debug(f"→ Subscribed to {topic_name}")

    def _fan_out_state(self, msg: dict) -> None:
        """
        Whenever any /motor_X/state message arrives, call every callback in self._callbacks(msg).
        """
        for cb in self._callbacks:
            try:
                cb(msg)
            except Exception as e:
                logger.error("Error in state callback: %s", e)

    def register_state_callback(self, cb: Callable[[dict], None]) -> None:
        """
        A front-end route can call this to register a callback.  Whenever a motor_state
        message arrives, `cb(msg_dict)` will be invoked (on a Twisted thread).
        """
        self._callbacks.append(cb)

    async def set_motor(self, motor_id: int, direction: str, speed: int) -> dict:
        """
        Call the /set_motor ROS service via rosbridge.  This is awaitable from FastAPI.
        """
        request = roslibpy.ServiceRequest({
            "motor_id": motor_id,
            "direction": direction,
            "speed": speed
        })
        loop = asyncio.get_running_loop()
        # Run the synchronous .call(...) in a thread pool so we don't block asyncio
        return await loop.run_in_executor(None, self._set_motor_srv.call, request)


# ─────────────────────────── FastAPI app ─────────────────────────────── #

app = FastAPI(title="ROS-2 Motor Dashboard – Main backend")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Instantiate RosBridge (it will immediately start ros.run() in a background thread).
ros = RosBridge(host=PI_ROSBRIDGE_HOST, port=PI_ROSBRIDGE_PORT)


# ───────────────────────── HTTP → Pi-backend helpers ────────────────── #

async def _pi_cmd(endpoint: str) -> dict:
    """
    Issue an HTTP POST to the Pi-backend at http://<Pi-IP>:5000/<endpoint>.
    If it fails (network error or non-2xx), raise HTTPException(502).
    """
    url = f"{PI_API}/{endpoint.lstrip('/')}"
    logger.debug(f"→ Calling Pi-backend: POST {url}")
    async with httpx.AsyncClient() as client:
        try:
            r = await client.post(url, timeout=TIMEOUT)
            r.raise_for_status()
        except Exception as exc:
            logger.error("❌ Pi backend error calling %s: %s", endpoint, exc)
            raise HTTPException(502, f"Pi backend error: {exc}") from exc

    data = r.json()
    logger.debug("← Pi-backend responded: %s", data)
    return data


# ───────────────────────────── Models ─────────────────────────────────── #

class MotorCmd(BaseModel):
    motor_id: int = Field(ge=0, le=3)
    direction: str = Field(pattern="^(forward|backward|brake)$")
    speed: int = Field(ge=0, le=65535)


# ───────────────────────────── Routes ───────────────────────────────────── #

@app.get("/", response_class=PlainTextResponse)
def root() -> str:
    return "Main backend alive. Endpoints: /start /stop /set_motor /status /ws/motor_state /ws/logs"


@app.post("/start")
async def start():
    """
    1) Forward POST /start to the Pi-backend → Pi starts rosbridge + motor_driver_node.
    2) If rosbridge wasn’t already connected, spin up a background thread for RosBridge.
    """
    result = await _pi_cmd("start")

    # If the RosBridge client is not already connected, spin up another thread to run it.
    if not ros._ros.is_connected:
        logger.info("→ rosbridge not connected; launching a new thread to run .run() …")
        thread = threading.Thread(target=ros._ros.run, daemon=True)
        thread.start()

    return {"status": result}


@app.post("/stop")
async def stop():
    """
    Forward POST /stop to the Pi-backend → Pi shuts down rosbridge + motor_driver_node.
    """
    return await _pi_cmd("stop")


@app.get("/status")
async def status():
    """
    Forward GET /status to the Pi-backend → Pi returns JSON with pids + running state.
    """
    return await _pi_cmd("status")


@app.post("/set_motor")
async def set_motor(cmd: MotorCmd) -> dict:
    """
    Called by the front-end's “Send command” button.  This method calls the
    Pi’s /set_motor ROS service over rosbridge (via roslibpy).
    """
    try:
        result = await ros.set_motor(cmd.motor_id, cmd.direction, cmd.speed)
    except Exception as exc:
        logger.error("❌ ROS service error: %s", exc)
        raise HTTPException(500, f"ROS service error: {exc}") from exc

    return {"result": result}


@app.websocket("/ws/motor_state")
async def motor_state(ws: WebSocket):
    """
    Upgrade to a WebSocket.  Whenever rosbridge pushes a /motor_X/state message,
    we funnel it to the browser in JSON form.
    """
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue()

    def _enqueue(msg: dict) -> None:
        # Run on the Twisted event loop thread—post to our asyncio queue
        asyncio.get_event_loop().call_soon_threadsafe(queue.put_nowait, msg)

    ros.register_state_callback(_enqueue)
    logger.info("→ New WebSocket client connected to /ws/motor_state")

    try:
        while True:
            msg = await queue.get()
            await ws.send_json(msg)
    except WebSocketDisconnect:
        logger.info("→ WebSocket /ws/motor_state disconnected")


@app.websocket("/ws/logs")
async def websocket_logs(ws: WebSocket):
    """
    Upgrade to a WebSocket.  Stream all host-backend logger.info/…
    messages to the browser, one line at a time.
    """
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue()

    # When the host-backend’s logger emits a record, push into our queue:
    def _log_hook(record: logging.LogRecord) -> None:
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(record.created))
        payload = f"[{ts}] {record.levelname}: {record.getMessage()}"
        asyncio.get_event_loop().call_soon_threadsafe(queue.put_nowait, payload)

    class QueueHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            _log_hook(record)

    handler = QueueHandler()
    handler.setLevel(logging.INFO)
    logger.addHandler(handler)
    logger.info("→ WebSocket /ws/logs client connected")

    try:
        while True:
            line = await queue.get()
            await ws.send_text(line)
    except WebSocketDisconnect:
        logger.info("→ WebSocket /ws/logs client disconnected")
    finally:
        logger.removeHandler(handler)