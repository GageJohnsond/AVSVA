"""
Config Module
Manages configurable settings and parameters
"""

import json
import os


class Config:
    """Configuration management for the application"""
    
    # Default anomaly types
    anomaly_types = [
        "zero_vel",      # Zero out velocity commands
        "noise",         # Add Gaussian noise
        "spoof_gps",     # Spoof GPS coordinates
        "delay",         # Introduce message delays
        "drop_packets",  # Drop packets
        "replay",        # Replay attack
        "corruption"     # Data corruption
    ]
    
    def __init__(self, config_file=None):
        self.config_dict = {
            'anomaly_types': self.anomaly_types,
            'anomaly_detection_threshold': 3.0,
            'max_playback_speed': 5.0,
            'default_playback_speed': 1.0,
            'collision_threshold': 0.5,
            'csv_export_dir': '/home/claude/csv_output',
            'report_export_dir': '/home/claude/reports',
            'log_level': 'INFO'
        }
        
        if config_file and os.path.exists(config_file):
            self.load_config(config_file)
            
    def get_config(self, key):
        """
        Retrieve a configuration value
        
        Args:
            key (str): Configuration key
            
        Returns:
            any: Configuration value
        """
        return self.config_dict.get(key)
        
    def set_config(self, key, value):
        """
        Set a configuration value
        
        Args:
            key (str): Configuration key
            value (any): Configuration value
        """
        self.config_dict[key] = value
        
    def load_config(self, filename):
        """
        Load configuration from JSON file
        
        Args:
            filename (str): Path to JSON config file
        """
        try:
            with open(filename, 'r') as f:
                loaded_config = json.load(f)
                self.config_dict.update(loaded_config)
        except Exception as e:
            print(f"Error loading config: {e}")
            
    def save_config(self, filename):
        """
        Save configuration to JSON file
        
        Args:
            filename (str): Path to output JSON file
        """
        try:
            with open(filename, 'w') as f:
                json.dump(self.config_dict, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving config: {e}")
            return False
            
    def get_anomaly_types(self):
        """
        Get list of supported anomaly types
        
        Returns:
            list: Anomaly type strings
        """
        return self.config_dict.get('anomaly_types', self.anomaly_types)
        
    def get_detection_threshold(self):
        """
        Get anomaly detection threshold
        
        Returns:
            float: Z-score threshold
        """
        return self.config_dict.get('anomaly_detection_threshold', 3.0)
        
    def get_export_dirs(self):
        """
        Get export directory paths
        
        Returns:
            dict: Export directory paths
        """
        return {
            'csv': self.config_dict.get('csv_export_dir'),
            'reports': self.config_dict.get('report_export_dir')
        }
