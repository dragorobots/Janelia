"""
Centralized configuration for the Janelia Robotics Simulation Platform.

This module contains all simulation parameters, organized by category.
Modify these values to adjust simulation behavior without editing core code.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass
class PhysicsConfig:
    """Physics simulation parameters."""
    timestep: float = 0.001
    wheel_max_torque: float = 0.15
    wheel_joint_damping: float = 0.009
    nominal_friction: np.ndarray = None
    ice_friction: np.ndarray = None
    
    def __post_init__(self):
        if self.nominal_friction is None:
            self.nominal_friction = np.array([0.4, 0.002, 0.2], dtype=float)
        if self.ice_friction is None:
            self.ice_friction = np.array([1e-5, 1e-5, 1e-5], dtype=float)


@dataclass
class NavigationConfig:
    """Autonomous navigation parameters."""
    safe_front_dist: float = 0.2
    safe_side_dist: float = 0.2
    arrival_tolerance: float = 0.2
    heading_tolerance: float = 0.05
    reverse_steps: int = 200
    reverse_speed: float = -0.4
    turn_speed: float = 1.0


@dataclass
class ControlConfig:
    """Robot control parameters."""
    drive_speed: float = 0.5
    turn_ratio: float = 1.0
    camera_step: float = 0.05
    target_speed: float = 0.3
    kp_speed: float = 1.0
    heading_kp: float = 2.0


@dataclass
class DebugConfig:
    """Debugging and logging parameters."""
    debug_period: float = 1.0
    dbg_skip: int = 10
    beacon_debounce: float = 0.3


@dataclass
class SimulationConfig:
    """Complete simulation configuration."""
    physics: PhysicsConfig = None
    navigation: NavigationConfig = None
    control: ControlConfig = None
    debug: DebugConfig = None
    
    def __post_init__(self):
        if self.physics is None:
            self.physics = PhysicsConfig()
        if self.navigation is None:
            self.navigation = NavigationConfig()
        if self.control is None:
            self.control = ControlConfig()
        if self.debug is None:
            self.debug = DebugConfig()


# Default configuration instance
DEFAULT_CONFIG = SimulationConfig()
