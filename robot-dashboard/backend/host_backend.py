#!/usr/bin/env python3
"""
Flask backend for ROS 2 motor control via rosbridge websocket
"""

import json
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
from websocket import create_connection
import logging

app = Flask(__name__)
CORS(app)  # Enable CORS for API requests

# Configuration
PI_IP = "192.168.2.4"  # Change to your Pi's address
PORT = 9090
WEBSOCKET_URL = f"ws://{PI_IP}:{PORT}"

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def percent_to_pwm(percent):
    """Convert percentage (0-100) to PWM value (0-65535)"""
    return int((percent / 100.0) * 65535)

def pwm_to_percent(pwm):
    """Convert PWM value (0-65535) to percentage (0-100)"""
    return int((pwm / 65535.0) * 100)

def send_motor_command(motor_id, direction, speed_percent):
    """
    Send motor command to ROS 2 via rosbridge
    
    Args:
        motor_id (int): Motor ID (0-3)
        direction (str): Direction ("forward", "backward", "brake")
        speed_percent (int): Speed in percentage (0-100)
    
    Returns:
        dict: Response from ROS service
    """
    try:
        ws = create_connection(WEBSOCKET_URL, timeout=5)
        
        # Convert percentage to PWM
        speed_pwm = percent_to_pwm(speed_percent) if direction != "brake" else 0
        
        # Prepare service call
        req = {
            "op": "call_service",
            "service": "/set_motor",
            "id": f"motor_cmd_{motor_id}",
            "args": {
                "motor_id": motor_id,
                "direction": direction,
                "speed": speed_pwm
            }
        }
        
        # Send request
        ws.send(json.dumps(req))
        reply = ws.recv()
        ws.close()
        
        # Parse response
        response = json.loads(reply)
        logger.info(f"Motor {motor_id} set to {direction} at {speed_percent}% (PWM: {speed_pwm})")
        
        return {
            "success": True,
            "motor_id": motor_id,
            "direction": direction,
            "speed_percent": speed_percent,
            "speed_pwm": speed_pwm,
            "response": response
        }
        
    except Exception as e:
        logger.error(f"Error sending motor command: {str(e)}")
        return {
            "success": False,
            "error": str(e),
            "motor_id": motor_id
        }

@app.route('/')
def index():
    """Serve the main control interface"""
    return render_template('index.html')

@app.route('/api/motor/<int:motor_id>', methods=['POST'])
def control_motor(motor_id):
    """
    Control a specific motor
    
    Expected JSON payload:
    {
        "direction": "forward" | "backward" | "brake",
        "speed": 0-100 (percentage)
    }
    """
    if motor_id < 0 or motor_id > 3:
        return jsonify({"error": "Invalid motor ID. Must be 0-3"}), 400
    
    data = request.get_json()
    
    if not data:
        return jsonify({"error": "No data provided"}), 400
    
    direction = data.get('direction', 'brake')
    speed = data.get('speed', 0)
    
    # Validate direction
    if direction not in ['forward', 'backward', 'brake']:
        return jsonify({"error": "Invalid direction. Must be 'forward', 'backward', or 'brake'"}), 400
    
    # Validate speed
    if not isinstance(speed, (int, float)) or speed < 0 or speed > 100:
        return jsonify({"error": "Invalid speed. Must be 0-100"}), 400
    
    # Send command
    result = send_motor_command(motor_id, direction, int(speed))
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 500

@app.route('/api/motors/stop', methods=['POST'])
def stop_all_motors():
    """Emergency stop - brake all motors"""
    results = []
    for motor_id in range(4):
        result = send_motor_command(motor_id, 'brake', 0)
        results.append(result)
    
    all_success = all(r['success'] for r in results)
    
    return jsonify({
        "success": all_success,
        "results": results
    }), 200 if all_success else 500

@app.route('/api/status', methods=['GET'])
def get_status():
    """Check connection status to ROS bridge"""
    try:
        ws = create_connection(WEBSOCKET_URL, timeout=2)
        ws.close()
        return jsonify({
            "connected": True,
            "rosbridge_url": WEBSOCKET_URL
        })
    except Exception as e:
        return jsonify({
            "connected": False,
            "rosbridge_url": WEBSOCKET_URL,
            "error": str(e)
        }), 503

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)