#!/usr/bin/env python3
"""
motor_driver_node.py
--------------------
ROS 2 (rclpy) node that drives DC motors through an Adafruit PCA9685
16-channel PWM board.  Each motor needs two output channels:

    • speed_ch – duty-cycle (0-65535) → PWM duty (speed)
    • dir_ch   – duty-cycle (0 % or 100 %) → H-bridge direction pin

The mapping is provided at run-time via the *motor_map* parameter,
encoded as a flat integer array:

    motor_map: [speed0, dir0,  speed1, dir1,  …]

Example for two motors:

    motor_map: [0, 1, 2, 3]      # motor 0 → CH0/1,  motor 1 → CH2/3

Service API
===========

Service  :  /set_motor   (pca9685_interfaces/srv/SetMotor)

Request   { int32  motor_id,
            string direction   # "forward" | "backward" | "brake"
            uint16 speed       # 0-65535 }
Response  { bool   success,
            string message     }

––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
Author :  Yang Yue  <zcemuex@ucl.ac.uk>         MIT License
"""

from __future__ import annotations
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterDescriptor
from pca9685_interfaces.srv import SetMotor

# ────────────────────────────────────────────────── hardware layer
try:
    import board                # type: ignore
    import busio                # type: ignore
    import adafruit_pca9685     # type: ignore

    HARDWARE = True
except ImportError:
    # fallback so unit-tests / laptops can import the module
    HARDWARE = False

    class _DummyChannel:
        def __init__(self):
            self.duty_cycle = 0

    class _DummyPCA:
        def __init__(self):
            self.channels = [_DummyChannel() for _ in range(16)]
            self.frequency = 100

    class _DummyI2C:  # pylint: disable=too-few-public-methods
        pass

    busio = type("busio", (), {"I2C": lambda *_: _DummyI2C()})
    board = type("board", (), {"SCL": None, "SDA": None})
    adafruit_pca9685 = type("adafruit_pca9685", (), {"PCA9685": _DummyPCA})

# ────────────────────────────────────────────────── constants
FORWARD_POLARITY  = (0, 0xFFFF)   # (Lo,Hi) on dir_ch  → forward
BACKWARD_POLARITY = (0xFFFF, 0)   # (Hi,Lo)            → reverse
CLAMP = lambda v, lo=0, hi=0xFFFF: max(lo, min(hi, v))

# ────────────────────────────────────────────────── the ROS node
class MotorDriver(Node):

    def __init__(self):
        super().__init__("motor_driver")

        # 1. declare parameters (appear in YAML)
        #    motor_map is a *flat* integer array of channel numbers
        self.declare_parameter(
            "motor_map",
            [],                                   # default value
            ParameterDescriptor(
                type=Parameter.Type.INTEGER_ARRAY))   # ← enforce type

        self.declare_parameter("pwm_frequency", 100)  # Hz

        # 2. read parameters
        flat_map = self.get_parameter("motor_map") \
                         .get_parameter_value().integer_array_value

        if len(flat_map) % 2:
            self.get_logger().fatal(
                "motor_map must contain an even number of integers "
                "(speed_ch, dir_ch pairs)")
            rclpy.shutdown()
            return

        # reshape into {motor_id: (speed_ch, dir_ch)}
        self.motor_map: Dict[int, Tuple[int, int]] = {
            i: (flat_map[2 * i], flat_map[2 * i + 1])
            for i in range(len(flat_map) // 2)
        }
        pwm_freq = int(self.get_parameter("pwm_frequency").value)

        # 3. init hardware (real or stub)
        i2c = busio.I2C(board.SCL, board.SDA)          # type: ignore
        self.pca = adafruit_pca9685.PCA9685(i2c)       # type: ignore
        self.pca.frequency = pwm_freq

        self.get_logger().info(
            f"🟢 Motor-driver ready — {len(self.motor_map)} motors, "
            f"PCA9685 freq {pwm_freq} Hz "
            f"({'real' if HARDWARE else 'dummy'} hardware)")

        # 4. ROS service
        self.create_service(SetMotor, "set_motor", self.handle_set_motor)

    # ----------------------------- service callback -----------------------
    def handle_set_motor(self,
                         req: SetMotor.Request,
                         res: SetMotor.Response) -> SetMotor.Response:
        mid, dirn, speed = req.motor_id, req.direction.lower(), CLAMP(req.speed)

        if mid not in self.motor_map:
            res.success = False
            res.message = f"motor {mid} not in motor_map"
            return res

        speed_ch, dir_ch = self.motor_map[mid]

        if dirn == "forward":
            self.pca.channels[dir_ch].duty_cycle = FORWARD_POLARITY[1]
        elif dirn == "backward":
            self.pca.channels[dir_ch].duty_cycle = BACKWARD_POLARITY[0]
        elif dirn == "brake":
            self.pca.channels[dir_ch].duty_cycle = 0
            speed = 0
        else:
            res.success = False
            res.message = "direction must be forward/backward/brake"
            return res

        self.pca.channels[speed_ch].duty_cycle = speed
        res.success = True
        res.message = (
            f"motor {mid} dir={dirn} speed={speed} "
            f"(CH{speed_ch},CH{dir_ch})")
        return res


# ───────────────────────────────────────────── main entry-point
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
