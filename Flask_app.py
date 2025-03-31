#!/usr/bin/env python3
import socket
from flask import Flask, request, render_template_string

app = Flask(__name__)

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html>
<head>
  <title>Robot Motor Control (Hosted on Mac)</title>
</head>
<body>
  <h1>Robot Motor Control</h1>
  <p>This web page is served by your Mac, but commands are forwarded to the Pi.</p>

  <hr/>
  <!-- Connection status display -->
  <p><strong>Pi Connection Status:</strong> {{connection_status}}</p>
  
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
    # We'll do a quick "ping" to see if Pi is reachable
    pi_ip = "192.168.1.100"  # <-- CHANGE to your Pi's IP
    pi_port = 12345

    # Attempt to send "ping"; if we get "pong", it's connected
    status_msg = check_pi_connection(pi_ip, pi_port)
    
    return render_template_string(HTML_TEMPLATE,
                                  motors=[1, 2, 3, 4],
                                  connection_status=status_msg)


@app.route("/control", methods=["GET"])
def control():
    motor_id = request.args.get("motor_id", "1")
    direction = request.args.get("direction", "forward")
    speed = request.args.get("speed", "30000")

    command_str = f"{motor_id},{direction},{speed}"

    pi_ip = "192.168.1.100"  # <-- same IP as above
    pi_port = 12345
    response = send_to_pi(pi_ip, pi_port, command_str)

    return f"Sent '{command_str}' -> Pi responded: {response}"

def check_pi_connection(ip, port):
    """
    Send a "ping" to the Pi. If the response is "pong", return "Connected".
    Otherwise, return the error or the unexpected response.
    """
    resp = send_to_pi(ip, port, "ping")
    if resp == "pong":
        return "Connected"
    else:
        return f"Not connected (response='{resp}')"

def send_to_pi(ip, port, command_str):
    """
    Open a TCP socket to the Pi's motor server, send 'command_str', return the Pi's response.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)  # 2-second timeout; adjust as needed
            s.connect((ip, port))
            s.sendall(command_str.encode("utf-8"))
            data = s.recv(1024).decode("utf-8")
        return data
    except Exception as e:
        return f"ERROR contacting Pi: {e}"

if __name__ == "__main__":
    # Run Flask on your Mac. 
    # Access at http://127.0.0.1:5000/
    app.run(host="127.0.0.1", port=5000, debug=True)
