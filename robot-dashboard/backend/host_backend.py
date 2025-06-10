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

# Import configuration manager
from backend.config_manager import get_config_manager

# Import ROS bridge components
from backend.ros_bridge import get_ros_bridge, get_motor_controller
from backend.sensors.lidar import get_lidar_sensor
from backend.sensors.thermal_camera import get_thermal_camera_sensor
from backend.sensors.system_monitor import get_system_monitor
from backend.motors.servo_control import ServoController
from backend.motors.actuator_control import get_actuator_controller

# Create Flask app
app = Flask(__name__, 
            template_folder='../frontend/templates',
            static_folder='../frontend/static')
CORS(app)  # Enable CORS for API requests
socketio = SocketIO(app, cors_allowed_origins="*")  # For real-time data

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load configuration from persistent storage
config_manager = get_config_manager()
CONFIG = config_manager.get_all()

# Initialize ROS components with saved configuration
ros_bridge = get_ros_bridge(CONFIG["PI_IP"], CONFIG["ROS_BRIDGE_PORT"])
motor_controller = get_motor_controller()
lidar_sensor = get_lidar_sensor()
thermal_camera_sensor = get_thermal_camera_sensor()
system_monitor = get_system_monitor()
servo_controller = ServoController()
actuator_controller = get_actuator_controller()

# Set ROS bridge for servo controller
servo_controller.set_ros_bridge(ros_bridge)
actuator_controller.set_ros_bridge(ros_bridge)

# Data streaming states
lidar_streaming = False
servo_streaming = False
system_streaming = False

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

@app.route('/thermal')
def thermal_page():
    """Serve the thermal camera interface"""
    return render_template('thermal_camera.html')

@app.route('/aiming')
def aiming_system():
    """Serve the aiming system control interface"""
    return render_template('aiming_system.html')

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
# SERVO/AIMING SYSTEM API ROUTES
# =============================================================================

@app.route('/api/servo/position', methods=['POST'])
def set_servo_position():
    """Set servo position for pan/tilt"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"error": "No data provided"}), 400
        
        pan_angle = data.get('pan_angle', servo_controller.pan_angle)
        tilt_angle = data.get('tilt_angle', servo_controller.tilt_angle)
        
        # Move servos
        result = servo_controller.move_to_position(pan_angle, tilt_angle)
        
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
            
    except Exception as e:
        logger.error(f"Error in set_servo_position: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/servo/position', methods=['GET'])
def get_servo_position():
    """Get current servo position"""
    state = servo_controller.get_state()
    
    return jsonify({
        "success": True,
        "state": state
    })

@app.route('/api/servo/center', methods=['POST'])
def center_servos():
    """Move servos to center position"""
    result = servo_controller.center_position()
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 400

@app.route('/api/servo/preset/<string:position>', methods=['POST'])
def move_to_preset(position):
    """Move servos to preset position (left, right, up, down)"""
    preset_functions = {
        'left': servo_controller.move_left,
        'right': servo_controller.move_right,
        'up': servo_controller.move_up,
        'down': servo_controller.move_down,
        'center': servo_controller.center_position
    }
    
    if position not in preset_functions:
        return jsonify({
            "success": False,
            "error": f"Invalid preset position: {position}"
        }), 400
    
    result = preset_functions[position]()
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 400

# =============================================================================
# ACTUATOR CONTROL API ROUTES
# =============================================================================

@app.route('/api/actuator/control', methods=['POST'])
def control_actuator():
    """Control the actuator for fire extinguisher trigger"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"error": "No data provided"}), 400
        
        action = data.get('action', 'stop')
        speed = data.get('speed', 0)
        duration = data.get('duration', 0.0)
        
        result = actuator_controller.control_actuator(action, speed, duration)
        
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
            
    except Exception as e:
        logger.error(f"Error in control_actuator: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/actuator/status', methods=['GET'])
def get_actuator_status():
    """Get current actuator status"""
    state = actuator_controller.get_state()
    
    return jsonify({
        "success": True,
        "state": state
    })

@app.route('/api/actuator/test', methods=['GET'])
def test_actuator_connection():
    """Test actuator connection"""
    connected = actuator_controller.test_connection()
    
    return jsonify({
        "success": True,
        "connected": connected,
        "message": "Actuator service is available" if connected else "Actuator service not responding"
    })

@app.route('/api/actuator/stop', methods=['POST'])
def stop_actuator():
    """Emergency stop for actuator"""
    result = actuator_controller.emergency_stop()
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 400

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
# THERMAL CAMERA API ROUTES
# =============================================================================

@app.route('/api/thermal/status', methods=['GET'])
def get_thermal_status():
    """Get thermal camera sensor status"""
    latest_frame = thermal_camera_sensor.get_latest_frame()
    
    return jsonify({
        "connected": thermal_camera_sensor.subscription_active,
        "has_data": latest_frame is not None,
        "timestamp": latest_frame["timestamp"] if latest_frame else None
    })

@app.route('/api/thermal/latest', methods=['GET'])
def get_latest_thermal_frame():
    """Get the most recent thermal frame data"""
    frame_data = thermal_camera_sensor.get_latest_frame()
    
    if frame_data:
        return jsonify({
            "success": True,
            "data": frame_data
        })
    else:
        return jsonify({
            "success": False,
            "error": "No thermal data available"
        }), 404

@app.route('/api/thermal/frame', methods=['GET'])
def get_thermal_frame():
    """Get a single thermal frame via service call"""
    frame_data = thermal_camera_sensor.get_thermal_frame_once()
    
    if frame_data:
        return jsonify({
            "success": True,
            "data": frame_data
        })
    else:
        return jsonify({
            "success": False,
            "error": "Failed to capture thermal frame"
        }), 500

@app.route('/api/thermal/subscribe', methods=['POST'])
def subscribe_thermal():
    """Start thermal camera data subscription"""
    def thermal_callback(frame_data):
        """Emit thermal data through WebSocket"""
        socketio.emit('thermal_data', frame_data)
    
    success = thermal_camera_sensor.subscribe(callback=thermal_callback)
    
    if success:
        return jsonify({"success": True, "message": "Thermal camera subscription started"})
    else:
        return jsonify({"success": False, "error": "Failed to subscribe to thermal camera"}), 500

@app.route('/api/thermal/unsubscribe', methods=['POST'])
def unsubscribe_thermal():
    """Stop thermal camera data subscription"""
    thermal_camera_sensor.unsubscribe()
    
    return jsonify({"success": True, "message": "Thermal camera subscription stopped"})

# =============================================================================
# SYSTEM MONITOR API ROUTES
# =============================================================================

@app.route('/api/system/status', methods=['GET'])
def get_system_status_monitor():
    """Get system monitor status"""
    latest_status = system_monitor.get_latest_status()
    
    return jsonify({
        "connected": system_monitor.subscription_active,
        "has_data": latest_status is not None,
        "data": latest_status,
        "timestamp": latest_status["timestamp"] if latest_status else None
    })

@app.route('/api/system/subscribe', methods=['POST'])
def subscribe_system_monitor():
    """Start system monitor subscription"""
    global system_streaming
    
    def system_callback(status_data):
        """Emit system data through WebSocket"""
        if system_streaming:
            socketio.emit('system_status', status_data)
    
    success = system_monitor.subscribe(callback=system_callback)
    
    if success:
        system_streaming = True
        return jsonify({"success": True, "message": "System monitor subscription started"})
    else:
        return jsonify({"success": False, "error": "Failed to subscribe to system monitor"}), 500

@app.route('/api/system/unsubscribe', methods=['POST'])
def unsubscribe_system_monitor():
    """Stop system monitor subscription"""
    global system_streaming
    
    system_streaming = False
    system_monitor.unsubscribe()
    
    return jsonify({"success": True, "message": "System monitor subscription stopped"})

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
            "lidar": lidar_sensor.subscription_active,
            "thermal": thermal_camera_sensor.subscription_active,
            "servo": servo_controller.connected,
            "system_monitor": system_monitor.subscription_active,
            "actuator": actuator_controller.connected
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
    """Update configuration with new connection settings"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"error": "No data provided"}), 400
        
        # Get new settings
        new_ip = data.get('robot_ip')
        new_port = data.get('rosbridge_port')
        
        if not new_ip or not new_port:
            return jsonify({"error": "Missing robot_ip or rosbridge_port"}), 400
        
        # Update configuration in memory
        CONFIG["PI_IP"] = new_ip
        CONFIG["ROS_BRIDGE_PORT"] = int(new_port)
        
        # Save to persistent storage
        config_updates = {
            "PI_IP": new_ip,
            "ROS_BRIDGE_PORT": int(new_port)
        }
        config_manager.update_config(config_updates)
        
        # Update ROS bridge connection
        global ros_bridge, motor_controller, lidar_sensor, thermal_camera_sensor, servo_controller, actuator_controller
        ros_bridge = get_ros_bridge(CONFIG["PI_IP"], CONFIG["ROS_BRIDGE_PORT"])
        motor_controller = get_motor_controller()
        lidar_sensor = get_lidar_sensor()
        thermal_camera_sensor = get_thermal_camera_sensor()
        
        # Update servo and actuator controllers with new ROS bridge
        servo_controller.set_ros_bridge(ros_bridge)
        actuator_controller.set_ros_bridge(ros_bridge)
        
        logger.info(f"Configuration updated and saved: IP={new_ip}, Port={new_port}")
        
        return jsonify({
            "success": True,
            "message": "Configuration updated successfully",
            "config": {
                "robot_ip": CONFIG["PI_IP"],
                "rosbridge_port": CONFIG["ROS_BRIDGE_PORT"]
            }
        })
        
    except Exception as e:
        logger.error(f"Error updating configuration: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

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

# Servo WebSocket events
@socketio.on('servo_command')
def handle_servo_command(data):
    """Handle servo commands via WebSocket"""
    global servo_streaming
    
    try:
        command_type = data.get('type', 'servo_command')
        
        if command_type == 'servo_command':
            result = servo_controller.handle_websocket_command(data)
            
            # Send response
            emit('servo_response', {
                'type': 'command_result',
                'result': result
            })
            
            # Send updated state to all clients
            state = servo_controller.get_state()
            socketio.emit('servo_state', {
                'type': 'servo_state',
                **state
            })
            
    except Exception as e:
        logger.error(f"Error handling servo command: {e}")
        emit('servo_error', {
            'type': 'error',
            'error': str(e)
        })

@socketio.on('request_servo_state')
def handle_servo_state_request():
    """Send current servo state via WebSocket"""
    state = servo_controller.get_state()
    emit('servo_state', {
        'type': 'servo_state',
        **state
    })

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

# =============================================================================
# INITIALIZATION
# =============================================================================

def initialize_system():
    """Initialize all system components"""
    logger.info("Initializing system components...")
    
    # Set WebSocket handler for servo updates
    def servo_websocket_handler(data):
        """Handle servo state updates"""
        socketio.emit('servo_state', data)
    
    servo_controller.set_websocket_handler(servo_websocket_handler)
    
    # Initialize servo to center position
    logger.info("Moving servos to center position...")
    servo_controller.center_position()
    
    logger.info("System initialization complete")

def main():
    """Main entry point"""
    logger.info("=" * 50)
    logger.info("ROS 2 Robot Dashboard Backend")
    logger.info(f"Robot IP: {CONFIG['PI_IP']}")
    logger.info(f"ROS Bridge Port: {CONFIG['ROS_BRIDGE_PORT']}")
    logger.info(f"Flask Server: {CONFIG['FLASK_HOST']}:{CONFIG['FLASK_PORT']}")
    logger.info(f"Config file: {config_manager.config_file}")
    logger.info("=" * 50)
    
    # Initialize system
    initialize_system()
    
    # Run Flask app with SocketIO
    debug_mode = os.environ.get('FLASK_ENV', 'development') == 'development'
    
    socketio.run(
        app,
        debug=debug_mode,
        host=CONFIG['FLASK_HOST'],
        port=CONFIG['FLASK_PORT'],
        use_reloader=False  # Disable auto-reloader to prevent numpy reload issues
    )

if __name__ == '__main__':
    main()