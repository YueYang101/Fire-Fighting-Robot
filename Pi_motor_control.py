#!/usr/bin/env python3
import board
import busio
import adafruit_pca9685
import socket

# ------------------------------------------------------------------------------
# Motor channel mapping
# motor_map = {
#     motor_id: (speed_channel, direction_channel)
# }
# Adjust to match how you've wired each motor to the PCA9685 channels
# ------------------------------------------------------------------------------
motor_map = {
    1: (0, 1),
    2: (2, 3),
    3: (4, 5),
    4: (6, 7)
}

# ------------------------------------------------------------------------------
# Basic forward/backward polarity definitions
# (used in set_motor() below)
# ------------------------------------------------------------------------------
FORWARD_POLARITY = (0, 0xFFFF)
BACKWARD_POLARITY = (0xFFFF, 0)

# ------------------------------------------------------------------------------
# Initialize PCA9685 on the Pi's I2C
# ------------------------------------------------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 100  # Hz, adjust as needed for your motor driver

# ------------------------------------------------------------------------------
# Set motor based on direction/speed
# ------------------------------------------------------------------------------
def set_motor(motor_id, direction, speed):
    """
    Sets motor_id's direction and speed on the PCA9685 channels.
    
    direction can be "forward", "backward", or "brake".
    speed is an integer 0..65535 for the duty cycle.
    """
    speed_channel, dir_channel = motor_map[motor_id]
    
    if direction == "forward":
        # Forward => direction pin is FORWARD_POLARITY[1], speed is the PWM value
        pca.channels[dir_channel].duty_cycle = FORWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed

    elif direction == "backward":
        # Backward => direction pin is BACKWARD_POLARITY[1], speed is the PWM value
        pca.channels[dir_channel].duty_cycle = BACKWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed

    elif direction == "brake":
        # "Brake" approach: set the speed channel to 0, direction channel to 0
        # For some drivers, you might want to drive both pins high or both low
        # depending on "short brake" vs. "coast." Adjust as needed.
        pca.channels[speed_channel].duty_cycle = 0
        pca.channels[dir_channel].duty_cycle = 0

# ------------------------------------------------------------------------------
# Parse commands from the socket and execute
# ------------------------------------------------------------------------------
def parse_and_execute(command_str):
    """
    Supported commands:
      1) "ping" -> returns "pong"
      2) "<motor_id>,<direction>,<speed>"
         where direction in {"forward", "backward", "brake"}
               speed in 0..65535 (PWM)
               motor_id in {1,2,3,4} (based on motor_map)
    """
    command_str = command_str.strip()

    # Special "ping" command
    if command_str == "ping":
        return "pong"

    # Otherwise assume it's "motor_id,direction,speed"
    parts = command_str.split(",")
    if len(parts) != 3:
        return "ERROR: invalid command format (expected motor_id,direction,speed or 'ping')"

    try:
        motor_id = int(parts[0])
        direction = parts[1].lower()
        speed = int(parts[2])

        # Validate motor ID
        if motor_id not in motor_map:
            return f"ERROR: motor_id must be 1..4, got {motor_id}"

        # Clamp speed to 0..65535
        if speed < 0:
            speed = 0
        if speed > 65535:
            speed = 65535

        # Validate direction
        if direction not in ["forward", "backward", "brake"]:
            return "ERROR: direction must be forward, backward, or brake"

        # Set motor
        set_motor(motor_id, direction, speed)
        return f"OK: motor={motor_id}, dir={direction}, speed={speed}"
    except ValueError:
        return "ERROR: could not parse motor_id or speed"

# ------------------------------------------------------------------------------
# Main server loop
# ------------------------------------------------------------------------------
def main():
    HOST = "0.0.0.0"  # Listen on all interfaces
    PORT = 12345
    print(f"Starting motor server on {HOST}:{PORT}")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)

    try:
        while True:
            client, address = server_socket.accept()
            data = client.recv(1024).decode("utf-8")
            if data:
                response = parse_and_execute(data)
                client.sendall(response.encode("utf-8"))
            client.close()
    except KeyboardInterrupt:
        print("Shutting down server...")
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()
