#!/usr/bin/env python3
"""
Flask backend server for ROS 2 robot control
Main application entry point with multi-component support
"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import logging

# Import ROS bridge components
from backend.ros_bridge import get_ros_bridge, get_motor_controller
from backend.sensors.lidar import get_lidar_sensor

# Create Flask app
app = Flask(__name__, 
            template_folder='../frontend/templates',
            static_folder='../frontend/static')
CORS(app)  # Enable CORS for API requests
socketio = SocketIO(app, cors_allowed_origins="*")  # For real-time lidar data

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
lidar_sensor = get_lidar_sensor()

# Lidar data streaming state
lidar_streaming = False

# =============================================================================
# MAIN DASHBOARD ROUTES
# =============================================================================

@app.route('/')
def index():
    """Serve the main dashboard home page"""
    return render_template('dashboard_home.html')

@app.route('/motors')
def motors_page():
    """Serve the motor control interface"""
    return render_template('motor_control.html')

@app.route('/lidar')
def lidar_page():
    """Serve the lidar visualization interface"""
    return render_template('lidar_visualization.html')

# =============================================================================
# MOTOR CONTROL API ROUTES
# =============================================================================

@app.route('/api/motor/<int:motor_id>', methods=['POST'])
def control_motor(motor_id):
    """Control a specific motor"""
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

# =============================================================================
# LIDAR API ROUTES
# =============================================================================

@app.route('/api/lidar/status', methods=['GET'])
def get_lidar_status():
    """Get lidar sensor status"""
    latest_scan = lidar_sensor.get_latest_scan()
    
    return jsonify({
        "connected": lidar_sensor.subscription_active,
        "has_data": latest_scan is not None,
        "timestamp": latest_scan["timestamp"] if latest_scan else None
    })

@app.route('/api/lidar/latest', methods=['GET'])
def get_latest_lidar_scan():
    """Get the most recent lidar scan data"""
    scan_data = lidar_sensor.get_latest_scan()
    
    if scan_data:
        return jsonify({
            "success": True,
            "data": scan_data
        })
    else:
        return jsonify({
            "success": False,
            "error": "No scan data available"
        }), 404

@app.route('/api/lidar/subscribe', methods=['POST'])
def subscribe_lidar():
    """Start lidar data subscription"""
    global lidar_streaming
    
    def lidar_callback(scan_data):
        """Emit lidar data through WebSocket"""
        if lidar_streaming:
            socketio.emit('lidar_data', scan_data)
    
    success = lidar_sensor.subscribe(callback=lidar_callback, processed_data=True)
    
    if success:
        lidar_streaming = True
        return jsonify({"success": True, "message": "Lidar subscription started"})
    else:
        return jsonify({"success": False, "error": "Failed to subscribe to lidar"}), 500

@app.route('/api/lidar/unsubscribe', methods=['POST'])
def unsubscribe_lidar():
    """Stop lidar data subscription"""
    global lidar_streaming
    
    lidar_streaming = False
    lidar_sensor.unsubscribe()
    
    return jsonify({"success": True, "message": "Lidar subscription stopped"})

# =============================================================================
# SYSTEM STATUS ROUTES
# =============================================================================

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
        },
        "components": {
            "motors": True,
            "lidar": lidar_sensor.subscription_active
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

# =============================================================================
# WEBSOCKET EVENTS
# =============================================================================

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    logger.info("Client connected to WebSocket")
    emit('connected', {'data': 'Connected to robot dashboard'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    logger.info("Client disconnected from WebSocket")

# =============================================================================
# ERROR HANDLERS
# =============================================================================

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
        "service": "robot-dashboard-backend"
    })

def main():
    """Main entry point"""
    logger.info("=" * 50)
    logger.info("ROS 2 Robot Dashboard Backend")
    logger.info(f"Robot IP: {CONFIG['PI_IP']}")
    logger.info(f"ROS Bridge Port: {CONFIG['ROS_BRIDGE_PORT']}")
    logger.info(f"Flask Server: {CONFIG['FLASK_HOST']}:{CONFIG['FLASK_PORT']}")
    logger.info("=" * 50)
    
    # Run Flask app with SocketIO
    debug_mode = os.environ.get('FLASK_ENV', 'development') == 'development'
    
    socketio.run(
        app,
        debug=debug_mode,
        host=CONFIG['FLASK_HOST'],
        port=CONFIG['FLASK_PORT']
    )

if __name__ == '__main__':
    main()