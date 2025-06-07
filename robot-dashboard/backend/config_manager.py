#!/usr/bin/env python3
"""
Configuration manager for persistent settings
Saves and loads configuration from a JSON file
"""

import json
import os
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

class ConfigManager:
    """Manages persistent configuration settings"""
    
    def __init__(self, config_file: str = None):
        """
        Initialize configuration manager
        
        Args:
            config_file: Path to configuration file (default: ~/.robot_dashboard_config.json)
        """
        if config_file is None:
            # Use home directory for config file
            self.config_file = Path.home() / ".robot_dashboard_config.json"
        else:
            self.config_file = Path(config_file)
        
        # Default configuration
        self.default_config = {
            "PI_IP": "192.168.2.1",
            "ROS_BRIDGE_PORT": 9090,
            "FLASK_PORT": 5001,
            "FLASK_HOST": "0.0.0.0"
        }
        
        # Load configuration
        self.config = self.load_config()
    
    def load_config(self) -> dict:
        """Load configuration from file or return defaults"""
        try:
            if self.config_file.exists():
                with open(self.config_file, 'r') as f:
                    loaded_config = json.load(f)
                    # Merge with defaults (in case new fields are added)
                    config = self.default_config.copy()
                    config.update(loaded_config)
                    logger.info(f"Loaded configuration from {self.config_file}")
                    return config
            else:
                logger.info("No config file found, using defaults")
                return self.default_config.copy()
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            return self.default_config.copy()
    
    def save_config(self):
        """Save current configuration to file"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=2)
            logger.info(f"Saved configuration to {self.config_file}")
            return True
        except Exception as e:
            logger.error(f"Error saving config: {e}")
            return False
    
    def update_config(self, updates: dict) -> bool:
        """
        Update configuration values
        
        Args:
            updates: Dictionary of configuration updates
            
        Returns:
            bool: True if successful
        """
        try:
            # Update configuration
            self.config.update(updates)
            # Save to file
            return self.save_config()
        except Exception as e:
            logger.error(f"Error updating config: {e}")
            return False
    
    def get(self, key: str, default=None):
        """Get configuration value"""
        return self.config.get(key, default)
    
    def get_all(self) -> dict:
        """Get all configuration values"""
        return self.config.copy()

# Singleton instance
_config_manager = None

def get_config_manager(config_file: str = None) -> ConfigManager:
    """Get or create ConfigManager instance"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager(config_file)
    return _config_manager