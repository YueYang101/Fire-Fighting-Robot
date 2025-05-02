from __future__ import annotations

import socket
import signal
import sys
from typing import Dict, Tuple

try:
    import board  # type: ignore
    import busio  # type: ignore
    import adafruit_pca9685  # type: ignore
    HARDWARE = True
except ImportError:
    # Dummy fallback so the script is testable on non‑Pi hosts.
    HARDWARE = False

    class _DummyCh:
        def __init__(self):
            self.duty_cycle = 0
    class _DummyPCA:
        def __init__(self):
            self.channels = [_DummyCh() for _ in range(16)]
            self.frequency = 100
    class _DummyI2C: pass
    class _DummyBusio:
        @staticmethod
        def I2C(*_):
            return _DummyI2C()
    class _DummyBoard:
        SCL = None
        SDA = None
    board = _DummyBoard()  # type: ignore
    busio = _DummyBusio()  # type: ignore
    adafruit_pca9685 = type("adafruit_pca9685", (), {"PCA9685": _DummyPCA})  # type: ignore

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
FORWARD_POLARITY = (0, 0xFFFF)
BACKWARD_POLARITY = (0xFFFF, 0)

MOTOR_MAP: Dict[int, Tuple[int, int]] = {}

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)  # type: ignore
pca = adafruit_pca9685.PCA9685(i2c)    # type: ignore
pca.frequency = 100

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp(val: int, lo: int = 0, hi: int = 65535) -> int:
    return max(lo, min(hi, val))


def set_motor(motor_id: int, direction: str, speed: int) -> str:
    if motor_id not in MOTOR_MAP:
        return f"ERROR: motor {motor_id} not mapped"

    speed_ch, dir_ch = MOTOR_MAP[motor_id]
    speed = clamp(speed)

    if direction == "forward":
        pca.channels[dir_ch].duty_cycle = FORWARD_POLARITY[1]
    elif direction == "backward":
        pca.channels[dir_ch].duty_cycle = BACKWARD_POLARITY[1]
    elif direction == "brake":
        pca.channels[dir_ch].duty_cycle = 0
        speed = 0
    else:
        return "ERROR: direction must be forward/backward/brake"

    pca.channels[speed_ch].duty_cycle = speed
    return f"OK: motor={motor_id},dir={direction},speed={speed}"


# ---------------------------------------------------------------------------
# Command parser
# ---------------------------------------------------------------------------

def parse_and_execute(cmd: str) -> str:
    cmd = cmd.strip()
    if cmd == "ping":
        return "pong"

    parts = cmd.split(",")

    # ---- mapping command ----------------------------------------------------
    if parts[0] == "map":
        if len(parts) != 4:
            return "ERROR: format map,<motor_id>,<speed_ch>,<dir_ch>"
        try:
            motor_id, speed_ch, dir_ch = map(int, parts[1:])
        except ValueError:
            return "ERROR: non‑integer values in map command"
        MOTOR_MAP[motor_id] = (speed_ch, dir_ch)
        return (f"OK: mapped motor {motor_id} → speed={speed_ch},dir={dir_ch}")

    # ---- motion command -----------------------------------------------------
    if len(parts) != 3:
        return "ERROR: format <motor_id>,<direction>,<speed>"
    try:
        m_id = int(parts[0])
        dirn = parts[1].lower()
        spd  = int(parts[2])
    except ValueError:
        return "ERROR: non‑integer motor_id or speed"

    return set_motor(m_id, dirn, spd)


# ---------------------------------------------------------------------------
# TCP server
# ---------------------------------------------------------------------------
server_socket: socket.socket | None = None


def clean_exit(_sig, _frm):
    global server_socket
    print("\nShutting down motor server …")
    if server_socket:
        server_socket.close()
    sys.exit(0)

signal.signal(signal.SIGINT, clean_exit)
signal.signal(signal.SIGTERM, clean_exit)


def main() -> None:
    global server_socket
    HOST, PORT = "0.0.0.0", 12345
    print(f"Motor server listening on {HOST}:{PORT}")
    if not HARDWARE:
        print("⚠️  Dummy hardware mode – no real PCA9685 detected.")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        while True:
            client, _addr = server_socket.accept()
            with client:
                data = client.recv(1024).decode()
                if data:
                    resp = parse_and_execute(data)
                    client.sendall(resp.encode())
    finally:
        if server_socket:
            server_socket.close()


if __name__ == "__main__":
    main()