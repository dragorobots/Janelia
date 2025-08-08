"""
Validation utilities for the Janelia Robotics Simulation Platform.

This module provides validation functions for model loading, configuration
parameters, and simulation state to ensure robust operation.
"""

import logging
from typing import List, Tuple, Optional
import mujoco
import numpy as np

from config import SimulationConfig


class SimulationValidator:
    """Validation utilities for simulation components."""
    
    @staticmethod
    def validate_model(model_path: str) -> Tuple[bool, Optional[str]]:
        """
        Validate MuJoCo model file.
        
        Args:
            model_path: Path to the model XML file
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        try:
            model = mujoco.MjModel.from_xml_path(model_path)
            
            # Check for required components
            required_actuators = ["front_left_motor", "front_right_motor", 
                                "back_left_motor", "back_right_motor"]
            for actuator_name in required_actuators:
                try:
                    model.actuator(actuator_name)
                except Exception:
                    return False, f"Missing required actuator: {actuator_name}"
            
            # Check for required sensors
            required_sensors = ["lidar_0", "lidar_45", "lidar_90", "lidar_135",
                              "lidar_180", "lidar_225", "lidar_270", "lidar_315"]
            for sensor_name in required_sensors:
                try:
                    model.sensor(sensor_name)
                except Exception:
                    return False, f"Missing required sensor: {sensor_name}"
            
            return True, None
            
        except Exception as e:
            return False, f"Failed to load model: {str(e)}"
    
    @staticmethod
    def validate_config(config: SimulationConfig) -> List[str]:
        """
        Validate simulation configuration parameters.
        
        Args:
            config: Simulation configuration to validate
            
        Returns:
            List of validation error messages
        """
        errors = []
        
        # Physics validation
        if config.physics.timestep <= 0:
            errors.append("Physics timestep must be positive")
        if config.physics.wheel_max_torque <= 0:
            errors.append("Wheel max torque must be positive")
        if config.physics.wheel_joint_damping < 0:
            errors.append("Wheel joint damping must be non-negative")
        
        # Navigation validation
        if config.navigation.safe_front_dist <= 0:
            errors.append("Safe front distance must be positive")
        if config.navigation.safe_side_dist <= 0:
            errors.append("Safe side distance must be positive")
        if config.navigation.arrival_tolerance <= 0:
            errors.append("Arrival tolerance must be positive")
        if config.navigation.heading_tolerance <= 0:
            errors.append("Heading tolerance must be positive")
        if config.navigation.reverse_steps < 0:
            errors.append("Reverse steps must be non-negative")
        if config.navigation.turn_speed <= 0:
            errors.append("Turn speed must be positive")
        
        # Control validation
        if config.control.drive_speed <= 0:
            errors.append("Drive speed must be positive")
        if config.control.turn_ratio <= 0:
            errors.append("Turn ratio must be positive")
        if config.control.camera_step <= 0:
            errors.append("Camera step must be positive")
        
        # Debug validation
        if config.debug.debug_period <= 0:
            errors.append("Debug period must be positive")
        if config.debug.dbg_skip < 0:
            errors.append("Debug skip must be non-negative")
        
        return errors
    
    @staticmethod
    def validate_sensor_data(sensor_data: np.ndarray) -> Tuple[bool, Optional[str]]:
        """
        Validate sensor data for reasonable values.
        
        Args:
            sensor_data: Array of sensor readings
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        if sensor_data is None:
            return False, "Sensor data is None"
        
        if not isinstance(sensor_data, np.ndarray):
            return False, "Sensor data must be numpy array"
        
        if len(sensor_data) < 16:  # Minimum expected sensors
            return False, f"Insufficient sensor data: {len(sensor_data)} < 16"
        
        # Check for NaN or infinite values
        if np.any(np.isnan(sensor_data)):
            return False, "Sensor data contains NaN values"
        
        if np.any(np.isinf(sensor_data)):
            return False, "Sensor data contains infinite values"
        
        # Check LiDAR readings for reasonable range (0.01 to 10 meters)
        lidar_data = sensor_data[0:8]
        if np.any(lidar_data < 0.01) or np.any(lidar_data > 10.0):
            return False, "LiDAR readings outside reasonable range [0.01, 10.0] meters"
        
        return True, None


class SimulationLogger:
    """Structured logging for simulation events."""
    
    def __init__(self, name: str = "simulation"):
        """
        Initialize simulation logger.
        
        Args:
            name: Logger name
        """
        self.logger = logging.getLogger(name)
        self._setup_logging()
    
    def _setup_logging(self):
        """Setup logging configuration."""
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)
    
    def log_navigation_event(self, event: str, data: dict):
        """
        Log navigation-related events.
        
        Args:
            event: Event description
            data: Associated data dictionary
        """
        self.logger.info(f"Navigation: {event}", extra=data)
    
    def log_obstacle_event(self, event: str, data: dict):
        """
        Log obstacle detection events.
        
        Args:
            event: Event description
            data: Associated data dictionary
        """
        self.logger.info(f"Obstacle: {event}", extra=data)
    
    def log_error(self, error: str, context: dict = None):
        """
        Log error events.
        
        Args:
            error: Error description
            context: Additional context data
        """
        if context:
            self.logger.error(f"Error: {error}", extra=context)
        else:
            self.logger.error(f"Error: {error}")
    
    def log_config_change(self, parameter: str, old_value, new_value):
        """
        Log configuration parameter changes.
        
        Args:
            parameter: Parameter name
            old_value: Previous value
            new_value: New value
        """
        self.logger.info(f"Config change: {parameter} = {old_value} -> {new_value}")


def validate_simulation_setup(model_path: str, config: SimulationConfig) -> Tuple[bool, List[str]]:
    """
    Comprehensive validation of simulation setup.
    
    Args:
        model_path: Path to model file
        config: Simulation configuration
        
    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    errors = []
    
    # Validate model
    model_valid, model_error = SimulationValidator.validate_model(model_path)
    if not model_valid:
        errors.append(f"Model validation failed: {model_error}")
    
    # Validate configuration
    config_errors = SimulationValidator.validate_config(config)
    errors.extend(config_errors)
    
    return len(errors) == 0, errors
