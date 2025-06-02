#!/usr/bin/env python3
"""
Flask backend server for ROS 2 motor control
Main application entry point
"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import logging

# Import ROS bridge components
from backend.ros_bridge import get_ros_bridge, get_motor_controller

# Create Flask app
app = Flask(__name__, 
            template_folder='../frontend/templates',
            static_folder='../frontend/static')
CORS(app)  # Enable CORS for API requests

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration (can be moved to config file)
CONFIG = {
    "PI_IP": os.environ.get("ROS_ROBOT_IP", "192.168.2.4"),
    "ROS_BRIDGE_PORT": int(os.environ.get("ROS_BRIDGE_PORT", "9090")),
    "FLASK_PORT": int(os.environ.get("FLASK_PORT", "5000")),
    "FLASK_HOST": os.environ.get("FLASK_HOST", "0.0.0.0")
}

# Initialize ROS components
ros_bridge = get_ros_bridge(CONFIG["PI_IP"], CONFIG["ROS_BRIDGE_PORT"])
motor_controller = get_motor_controller()

# Routes
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
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"error": "No data provided"}), 400
        
        direction = data.get('direction', 'brake')
        speed = data.get('speed', 0)
        
        # Use motor controller
        result = motor_controller.set_motor(motor_id, direction, speed)
        
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
            
    except Exception as e:
        logger.error(f"Error in control_motor: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/motor/<int:motor_id>', methods=['GET'])
def get_motor_status(motor_id):
    """Get current status of a specific motor"""
    state = motor_controller.get_motor_state(motor_id)
    
    if state:
        return jsonify({
            "success": True,
            "motor_id": motor_id,
            "state": state
        })
    else:
        return jsonify({
            "success": False,
            "error": "Invalid motor ID"
        }), 404

@app.route('/api/motors', methods=['GET'])
def get_all_motors_status():
    """Get current status of all motors"""
    states = motor_controller.get_all_motor_states()
    
    return jsonify({
        "success": True,
        "motors": states
    })

@app.route('/api/motors/stop', methods=['POST'])
def stop_all_motors():
    """Emergency stop - brake all motors"""
    result = motor_controller.stop_all_motors()
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 500

@app.route('/api/status', methods=['GET'])
def get_system_status():
    """Check connection status to ROS bridge and system info"""
    ros_connected = ros_bridge.test_connection()
    
    return jsonify({
        "connected": ros_connected,
        "rosbridge_url": ros_bridge.url,
        "config": {
            "robot_ip": CONFIG["PI_IP"],
            "rosbridge_port": CONFIG["ROS_BRIDGE_PORT"]
        }
    }), 200 if ros_connected else 503

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration"""
    return jsonify({
        "robot_ip": CONFIG["PI_IP"],
        "rosbridge_port": CONFIG["ROS_BRIDGE_PORT"],
        "flask_port": CONFIG["FLASK_PORT"],
        "flask_host": CONFIG["FLASK_HOST"]
    })

@app.route('/api/config', methods=['POST'])
def update_config():
    """Update configuration (requires restart to take effect)"""
    data = request.get_json()
    
    if not data:
        return jsonify({"error": "No data provided"}), 400
    
    # Update configuration
    if "robot_ip" in data:
        CONFIG["PI_IP"] = data["robot_ip"]
        # Note: You'll need to recreate ros_bridge connection
        
    if "rosbridge_port" in data:
        CONFIG["ROS_BRIDGE_PORT"] = int(data["rosbridge_port"])
    
    return jsonify({
        "success": True,
        "message": "Configuration updated. Restart required for changes to take effect.",
        "config": CONFIG
    })

# Error handlers
@app.errorhandler(404)
def not_found(error):
    return jsonify({"error": "Not found"}), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({"error": "Internal server error"}), 500

# Health check endpoint
@app.route('/health', methods=['GET'])
def health_check():
    """Simple health check endpoint"""
    return jsonify({
        "status": "healthy",
        "service": "motor-control-backend"
    })

def main():
    """Main entry point"""
    logger.info("=" * 50)
    logger.info("ROS 2 Motor Control Backend")
    logger.info(f"Robot IP: {CONFIG['PI_IP']}")
    logger.info(f"ROS Bridge Port: {CONFIG['ROS_BRIDGE_PORT']}")
    logger.info(f"Flask Server: {CONFIG['FLASK_HOST']}:{CONFIG['FLASK_PORT']}")
    logger.info("=" * 50)
    
    # Run Flask app
    debug_mode = os.environ.get('FLASK_ENV', 'development') == 'development'
    
    app.run(
        debug=debug_mode,
        host=CONFIG['FLASK_HOST'],
        port=CONFIG['FLASK_PORT']
    )

if __name__ == '__main__':
    main()