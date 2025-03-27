#!/usr/bin/env python3
import socket
from flask import Flask, request, render_template_string

app = Flask(__name__)

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html>
<head><title>Robot Motor Control (Hosted on Mac)</title></head>
<body>
<h1>Robot Motor Control</h1>
<p>This web page is served by your Mac, but commands are sent to the Pi.</p>
<hr/>
{% for m in motors %}
<h2>Motor {{m}}</h2>
<form action="/control" method="GET">
  <input type="hidden" name="motor_id" value="{{m}}"/>
  Direction:
  <select name="direction">
    <option value="forward">Forward</option>
    <option value="backward">Backward</option>
  </select>
  Speed (0..65535):
  <input type="number" name="speed" value="30000" min="0" max="65535"/>
  <input type="submit" value="Update"/>
</form>
<br/>
{% endfor %}
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE, motors=[1,2,3,4])

@app.route("/control", methods=["GET"])
def control():
    # Get parameters from the form
    motor_id = request.args.get("motor_id", "1")
    direction = request.args.get("direction", "forward")
    speed = request.args.get("speed", "30000")

    # Build the command string
    command_str = f"{motor_id},{direction},{speed}"

    # Send to Pi
    pi_ip = "192.168.1.100"  # <--- CHANGE THIS to your actual Pi's IP
    pi_port = 12345
    response = send_to_pi(pi_ip, pi_port, command_str)

    return f"Sent '{command_str}' -> Pi responded: {response}"

def send_to_pi(ip, port, command_str):
    """
    Open a TCP socket to the Pi's motor server, send the command, return response
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ip, port))
            s.sendall(command_str.encode("utf-8"))
            data = s.recv(1024).decode("utf-8")
        return data
    except Exception as e:
        return f"ERROR contacting Pi: {e}"

if __name__ == "__main__":
    # Run Flask on your Mac. Access it on http://localhost:5000/ 
    # or if you want it accessible on your local network, use host="0.0.0.0"
    app.run(host="127.0.0.1", port=5000, debug=True)
