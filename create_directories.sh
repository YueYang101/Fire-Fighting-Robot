#!/bin/bash
# Create directory structure for actuator packages

echo "Creating directory structure..."

# Create directories
mkdir -p actuator_controller/actuator_controller
mkdir -p actuator_controller/launch
mkdir -p actuator_controller/config
mkdir -p actuator_controller/resource
mkdir -p actuator_interfaces/srv

# Create empty __init__.py
touch actuator_controller/actuator_controller/__init__.py

# Create resource file
touch actuator_controller/resource/actuator_controller

echo "Directory structure created!"
