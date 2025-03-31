#!/usr/bin/env python3
import board
import busio
import adafruit_pca9685
import socket
import signal
import sys

# Global server socket variable for cleanup
server_socket = None

# Default motor channel mapping (This will be dynamic)
motor_map = {}

# ------------------------------------------------------------------------------
# Polarity definitions
# ------------------------------------------------------------------------------
FORWARD_POLARITY = (0, 0xFFFF)
BACKWARD_POLARITY = (0xFFFF, 0)

# ------------------------------------------------------------------------------
# Initialize PCA9685 on the Pi's I2C
# ------------------------------------------------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 100  # Hz

# ------------------------------------------------------------------------------
# Set motor function
# ------------------------------------------------------------------------------
def set_motor(motor_id, direction, speed):
    speed_channel, dir_channel = motor_map.get(motor_id, (None, None))
    if not speed_channel or not dir_channel:
        return "ERROR: Motor channel not mapped"
    
    if direction == "forward":
        pca.channels[dir_channel].duty_cycle = FORWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed
    elif direction == "backward":
        pca.channels[dir_channel].duty_cycle = BACKWARD_POLARITY[1]
        pca.channels[speed_channel].duty_cycle = speed
    elif direction == "brake":
        # Example brake: set both channels to 0 (adjust per your hardware requirements)
        pca.channels[speed_channel].duty_cycle = 0
        pca.channels[dir_channel].duty_cycle = 0

# ------------------------------------------------------------------------------
# Parse and execute command function
# ------------------------------------------------------------------------------
def parse_and_execute(command_str):
    command_str = command_str.strip()
    if command_str == "ping":
        return "pong"
    
    parts = command_str.split(",")
    if len(parts) != 3:
        return "ERROR: invalid command format (expected motor_id,direction,speed)"
    
    try:
        motor_id = int(parts[0])
        direction = parts[1].lower()
        speed = int(parts[2])
        
        if motor_id not in motor_map:
            return f"ERROR: motor_id must be mapped, got {motor_id}"
        if direction not in ["forward", "backward", "brake"]:
            return "ERROR: direction must be forward, backward, or brake"
        
        # Clamp speed
        speed = max(0, min(speed, 65535))
        
        set_motor(motor_id, direction, speed)
        return f"OK: motor={motor_id}, dir={direction}, speed={speed}"
    except ValueError:
        return "ERROR: could not parse motor_id or speed"

# ------------------------------------------------------------------------------
# Signal handler for graceful exit
# ------------------------------------------------------------------------------
def clean_exit(signum, frame):
    global server_socket
    print("\nReceived signal to terminate. Cleaning up...")
    if server_socket:
        server_socket.close()
        print("Socket closed.")
    sys.exit(0)

# Register signal handlers for SIGINT and SIGTERM
signal.signal(signal.SIGINT, clean_exit)
signal.signal(signal.SIGTERM, clean_exit)

# ------------------------------------------------------------------------------
# Main server loop
# ------------------------------------------------------------------------------
def main():
    global server_socket
    HOST = "0.0.0.0"
    PORT = 12345
    print(f"Starting motor server on {HOST}:{PORT}")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Allow immediate reuse of the address after the server stops.
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(5)
        
        while True:
            client, address = server_socket.accept()
            data = client.recv(1024).decode("utf-8")
            if data:
                response = parse_and_execute(data)
                client.sendall(response.encode("utf-8"))
            client.close()
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        if server_socket:
            server_socket.close()
            print("Socket closed in finally block.")

if __name__ == "__main__":
    main()
