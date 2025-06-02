#!/usr/bin/env python3
"""
Main entry point for the ROS 2 Motor Control application
Run this file to start the Flask backend server
"""

import sys
from pathlib import Path

# Add the current directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from backend.host_backend import main

if __name__ == '__main__':
    main()