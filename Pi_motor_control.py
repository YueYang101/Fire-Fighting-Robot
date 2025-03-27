#!/usr/bin/env python3
import board
import busio
import adafruit_pca9685
import socket

motor_map = {
    1: (0, 1),
    2: (2, 3),
    3: (4, 5),
    4: (6, 7)
}

FORWARD_POLARITY = (0, 0xFFFF)
BACKWARD_POLARITY = (0xFFFF, 0)

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 100

def set_motor(motor_id, direction, speed):
    speed_channel, dir_channel = motor_map[motor_id]
    if direction == "forward":
        pca.channels[dir_channel].duty_cycle = FORWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed
    else:
        pca.channels[dir_channel].duty_cycle = BACKWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed

def parse_and_execute(command_str):
    """
    command_str examples:
      "2,forward,30000"  -> set motor2 forward speed=30000
      "ping"             -> respond "pong"
    """
    command_str = command_str.strip()
    
    # 1) Special ping command
    if command_str == "ping":
        return "pong"
    
    # 2) Otherwise, assume it's motor commands: "motor_id,direction,speed"
    parts = command_str.split(",")
    if len(parts) != 3:
        return "ERROR: invalid command format (expected motor_id,direction,speed or 'ping')"

    try:
        motor_id = int(parts[0])
        direction = parts[1].lower()
        speed = int(parts[2])

        if direction not in ["forward", "backward"]:
            return "ERROR: direction must be forward or backward"
        if motor_id not in motor_map:
            return f"ERROR: motor_id must be 1..4, got {motor_id}"
        if speed < 0: 
            speed = 0
        if speed > 65535: 
            speed = 65535

        set_motor(motor_id, direction, speed)
        return f"OK: motor={motor_id}, dir={direction}, speed={speed}"
    except ValueError:
        return "ERROR: could not parse numbers"

def main():
    HOST = "0.0.0.0"
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
