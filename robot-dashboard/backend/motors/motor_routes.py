"""
Motor Control Routes Module
Handles all motor-related API endpoints
"""

from flask import Blueprint, jsonify, request, render_template
import logging
from backend.motor.motor_controller import get_motor_controller

# Create Blueprint
motor_bp = Blueprint('motors', __name__)

# Setup logging
logger = logging.getLogger(__name__)

# =============================================================================
# MOTOR PAGE ROUTE
# =============================================================================

@motor_bp.route('/motors')
def motors_page():
    """Serve the motor control interface"""
    return render_template('motor_control.html')

# =============================================================================
# MOTOR CONTROL API ROUTES
# =============================================================================

@motor_bp.route('/api/motor/<int:motor_id>', methods=['POST'])
def control_motor(motor_id):
    """Control a specific motor"""
    try:
        motor_controller = get_motor_controller()
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

@motor_bp.route('/api/motor/<int:motor_id>', methods=['GET'])
def get_motor_status(motor_id):
    """Get current status of a specific motor"""
    motor_controller = get_motor_controller()
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

@motor_bp.route('/api/motors', methods=['GET'])
def get_all_motors_status():
    """Get current status of all motors"""
    motor_controller = get_motor_controller()
    states = motor_controller.get_all_motor_states()
    
    return jsonify({
        "success": True,
        "motors": states
    })

@motor_bp.route('/api/motors/stop', methods=['POST'])
def stop_all_motors():
    """Emergency stop - brake all motors"""
    motor_controller = get_motor_controller()
    result = motor_controller.stop_all_motors()
    
    if result['success']:
        return jsonify(result)
    else:
        return jsonify(result), 500

# Optional: WebSocket support for real-time motor control
def init_motor_websocket(socketio):
    """Initialize WebSocket handlers for motor control"""
    
    @socketio.on('motor_command')
    def handle_motor_command(data):
        """Handle real-time motor commands via WebSocket"""
        motor_controller = get_motor_controller()
        motor_id = data.get('motor_id')
        direction = data.get('direction', 'brake')
        speed = data.get('speed', 0)
        
        result = motor_controller.set_motor(motor_id, direction, speed)
        
        socketio.emit('motor_response', {
            'motor_id': motor_id,
            'result': result
        })
    
    @socketio.on('request_motor_states')
    def handle_motor_states_request():
        """Send current motor states via WebSocket"""
        motor_controller = get_motor_controller()
        states = motor_controller.get_all_motor_states()
        socketio.emit('motor_states', states)