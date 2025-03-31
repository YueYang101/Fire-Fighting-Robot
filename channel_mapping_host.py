#!/usr/bin/env python3
import socket
import json
from flask import Flask, request, jsonify, render_template_string

app = Flask(__name__)

# -------------------
# ADJUST THESE
# -------------------
PI_IP = "192.168.2.3"  # Replace with your Pi's IP
PI_PORT = 12345

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html>
<head>
  <title>Robot Motor Control (Dynamic)</title>
</head>
<body>
  <h1>Robot Motor Control (Hosted on Mac)</h1>
  <p>This page will stay visible, and commands are sent via JavaScript so it doesn't navigate away.</p>
  
  <hr/>
  <p><strong>Pi Connection Status:</strong> <span id="status_span">Checking...</span></p>
  
  <div id="motors_container">
    {% for m in motors %}
      <h2>Motor {{m}}</h2>
      <div>
        <label>Direction:</label>
        <select id="dir_{{m}}">
          <option value="forward">Forward</option>
          <option value="backward">Backward</option>
        </select>
        <br/><br/>
        <label>Speed (%):</label>
        <!-- Slider for percentage speed -->
        <input type="range" id="speed_{{m}}" value="50" min="0" max="100" oninput="updateSpeedLabel({{m}})">
        <span id="speed_label_{{m}}">50%</span>
        <br/><br/>
        <button onclick="sendCommand({{m}})">Update</button>
        <button onclick="brakeMotor({{m}})">Brake</button>
      </div>
      <p>Response: <span id="resp_{{m}}"></span></p>
      <hr/>
    {% endfor %}
  </div>
  
  <script>
    // Update the speed label to show the current percentage value
    function updateSpeedLabel(motorId) {
      let speedSlider = document.getElementById("speed_" + motorId);
      let speedLabel = document.getElementById("speed_label_" + motorId);
      speedLabel.textContent = speedSlider.value + "%";
    }
    
    // Periodically ping the Pi to update the status text
    function checkPiStatus() {
      fetch("/ping_status")
        .then(response => response.json())
        .then(data => {
          if (data.connected === true) {
            document.getElementById("status_span").textContent = "Connected";
          } else {
            document.getElementById("status_span").textContent = "Not connected";
          }
        })
        .catch(err => {
          document.getElementById("status_span").textContent = "Error checking status";
          console.error(err);
        });
    }
    
    // Call checkPiStatus every 2 seconds and immediately on load
    setInterval(checkPiStatus, 2000);
    checkPiStatus();
    
    // Send command to Pi (motor_id, direction, speed) without reloading the page
    function sendCommand(motorId) {
      let directionSel = document.getElementById("dir_" + motorId);
      // Get slider value as a percentage
      let percent = parseInt(document.getElementById("speed_" + motorId).value);
      // Convert percentage (0-100) to raw value (0-65535)
      let speed = Math.round((percent / 100) * 65535);
      
      let payload = {
        motor_id: motorId,
        direction: directionSel.value,
        speed: speed
      };
      
      fetch("/send_command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      })
      .then(response => response.json())
      .then(data => {
        document.getElementById("resp_" + motorId).textContent = data.result;
      })
      .catch(err => {
        document.getElementById("resp_" + motorId).textContent = "Error: " + err;
      });
    }
    
    // Brake motor by sending { direction:"brake", speed:0 }
    function brakeMotor(motorId) {
      let payload = {
        motor_id: motorId,
        direction: "brake",
        speed: 0
      };
      
      fetch("/send_command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      })
      .then(response => response.json())
      .then(data => {
        document.getElementById("resp_" + motorId).textContent = data.result;
      })
      .catch(err => {
        document.getElementById("resp_" + motorId).textContent = "Error: " + err;
      });
    }
  </script>
</body>
</html>
"""

@app.route("/")
def index():
    # Render the template with motor IDs 1 through 4
    return render_template_string(HTML_TEMPLATE, motors=[1, 2, 3, 4])

@app.route("/ping_status", methods=["GET"])
def ping_status():
    """
    Returns JSON indicating whether the Pi responded to 'ping' or not.
    """
    resp = send_to_pi(PI_IP, PI_PORT, "ping")
    if resp == "pong":
        return jsonify({"connected": True})
    else:
        return jsonify({"connected": False})

@app.route("/send_command", methods=["POST"])
def send_command():
    """
    Expects JSON: { "motor_id": int, "direction": str, "speed": int }
    Sends that to the Pi as "motor_id,direction,speed" and returns the Pi's response.
    """
    try:
        data = request.get_json()
        motor_id = data.get("motor_id", 1)
        direction = data.get("direction", "forward")
        speed = data.get("speed", 30000)
        
        command_str = f"{motor_id},{direction},{speed}"
        pi_resp = send_to_pi(PI_IP, PI_PORT, command_str)
        return jsonify({"result": pi_resp})
    except Exception as e:
        return jsonify({"result": f"ERROR: {e}"}), 400

def send_to_pi(ip, port, command_str):
    """
    Opens a TCP socket to the Pi, sends command_str, and returns the Pi's response.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((ip, port))
            s.sendall(command_str.encode("utf-8"))
            data = s.recv(1024).decode("utf-8")
        return data
    except Exception as e:
        return f"ERROR contacting Pi: {e}"

if __name__ == "__main__":
    # Launch Flask on Mac
    app.run(host="127.0.0.1", port=5000, debug=True)
