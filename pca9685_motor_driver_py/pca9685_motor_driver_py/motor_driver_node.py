#!/usr/bin/env python3
"""
motor_driver_node.py
--------------------
ROS 2 node that drives DC motors through an Adafruit PCA9685 PWM board.

Modified for dual-PWM motor drivers (like DBH-1A):
Each motor uses two PCA channels:
    • forward_ch – PWM for forward motion (0 when backward)
    • backward_ch – PWM for backward motion (0 when forward)

The mapping is supplied at run-time via the parameter *motor_map*
(encoded as an INTEGER_ARRAY):

    motor_map: [forward0, backward0,  forward1, backward1,  …]

Example for two motors:
    motor_map: [0, 1, 2, 3]   # motor 0 → CH0/1, motor 1 → CH2/3
"""

from __future__ import annotations
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from pca9685_interfaces.srv import SetMotor

# ────────────────────────────── hardware abstraction (stub fallback)
try:
    import board                # type: ignore
    import busio                # type: ignore
    import adafruit_pca9685     # type: ignore

    HARDWARE = True
except ImportError:             # running on a PC with no I²C bus
    HARDWARE = False

    class _DummyChannel:
        def __init__(self):
            self.duty_cycle = 0

    class _DummyPCA:
        def __init__(self):
            self.channels = [_DummyChannel() for _ in range(16)]
            self.frequency = 100

    class _DummyI2C:
        pass

    busio = type("busio", (), {"I2C": lambda *_: _DummyI2C()})
    board = type("board", (), {"SCL": None, "SDA": None})
    adafruit_pca9685 = type("adafruit_pca9685", (), {"PCA9685": _DummyPCA})

# ────────────────────────────── helpers
CLAMP = lambda v, lo=0, hi=0xFFFF: max(lo, min(hi, v))

# ────────────────────────────── main node
class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        # 1. Declare parameters (force motor_map to be INTEGER_ARRAY)
        self.declare_parameter(
            "motor_map",
            [0, 1, 2, 3],                                            # <- default: 2 motors
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter("pwm_frequency", 100)

        # 2. Read parameters
        flat_map = self.get_parameter("motor_map") \
                         .get_parameter_value().integer_array_value

        if len(flat_map) % 2:
            self.get_logger().fatal(
                "motor_map must contain an even number of integers "
                "(forward_ch, backward_ch pairs)")
            rclpy.shutdown()
            return

        # Convert flat list → {motor_id: (forward_ch, backward_ch)}
        self.motor_map: Dict[int, Tuple[int, int]] = {
            i: (flat_map[2 * i], flat_map[2 * i + 1])
            for i in range(len(flat_map) // 2)
        }
        pwm_freq = int(self.get_parameter("pwm_frequency").value)

        # 3. Init hardware (real or stub)
        i2c = busio.I2C(board.SCL, board.SDA)          # type: ignore
        self.pca = adafruit_pca9685.PCA9685(i2c)       # type: ignore
        self.pca.frequency = pwm_freq

        self.get_logger().info(
            f"🟢 Motor-driver ready — {len(self.motor_map)} motors, "
            f"PCA9685 freq {pwm_freq} Hz "
            f"({'real' if HARDWARE else 'dummy'} hardware)"
        )
        self.get_logger().info(f"Motor mapping: {self.motor_map}")

        # 4. ROS service
        self.create_service(SetMotor, "set_motor", self.handle_set_motor)

    # -------------------------- service callback -------------------------
    def handle_set_motor(
        self,
        req: SetMotor.Request,
        res: SetMotor.Response
    ) -> SetMotor.Response:

        mid, dirn, speed = req.motor_id, req.direction.lower(), CLAMP(req.speed)

        if mid not in self.motor_map:
            res.success = False
            res.message = f"motor {mid} not in motor_map"
            return res

        forward_ch, backward_ch = self.motor_map[mid]

        # Dual-PWM control logic
        if dirn == "forward":
            # PWM on forward channel, 0 on backward channel
            self.pca.channels[forward_ch].duty_cycle = speed
            self.pca.channels[backward_ch].duty_cycle = 0
            self.get_logger().debug(
                f"Motor {mid} forward: CH{forward_ch}={speed}, CH{backward_ch}=0"
            )
        elif dirn == "backward":
            # 0 on forward channel, PWM on backward channel
            self.pca.channels[forward_ch].duty_cycle = 0
            self.pca.channels[backward_ch].duty_cycle = speed
            self.get_logger().debug(
                f"Motor {mid} backward: CH{forward_ch}=0, CH{backward_ch}={speed}"
            )
        elif dirn == "brake":
            # Both channels to 0
            self.pca.channels[forward_ch].duty_cycle = 0
            self.pca.channels[backward_ch].duty_cycle = 0
            self.get_logger().debug(
                f"Motor {mid} brake: CH{forward_ch}=0, CH{backward_ch}=0"
            )
        else:
            res.success = False
            res.message = "direction must be forward/backward/brake"
            return res

        res.success = True
        res.message = (
            f"motor {mid} dir={dirn} speed={speed} "
            f"(CH{forward_ch},CH{backward_ch})"
        )
        return res

# ────────────────────────────── entry-point
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