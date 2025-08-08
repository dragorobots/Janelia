"""
Autonomous robot controller for the Janelia Robotics Simulation Platform.

This module provides a class-based interface for robot control, encapsulating
navigation, obstacle avoidance, and sensor fusion functionality.
"""

import time
import numpy as np
from typing import Tuple, Optional, NamedTuple
from enum import Enum
from dataclasses import dataclass
import mujoco

from config import SimulationConfig


class RobotState(Enum):
    """Robot operational states."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    REVERSING = "reversing"
    ARRIVED = "arrived"


@dataclass
class StateContext:
    """Context data for robot state machine."""
    target_beacon: Optional[Tuple[float, float]] = None
    obstacle_detected: bool = False
    reverse_counter: int = 0
    axis_mode: Optional[str] = None
    last_debug_time: float = 0.0


class ControlCommand(NamedTuple):
    """Robot control command."""
    left_speed: float
    right_speed: float


class ObstacleDetector:
    """LiDAR-based obstacle detection system."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
    
    def detect_obstacle(self, lidar_data: np.ndarray) -> bool:
        """
        Detect obstacles using LiDAR data.
        
        Args:
            lidar_data: Array of 8 LiDAR readings in meters
            
        Returns:
            True if obstacle detected, False otherwise
        """
        # Check front arc (beams 0, 1, 7)
        front_arc = min(lidar_data[[0, 1, 7]])
        if front_arc < self.config.navigation.safe_front_dist:
            return True
        
        # Check any beam for side obstacles
        if (lidar_data < self.config.navigation.safe_side_dist).any():
            return True
        
        return False
    
    def get_front_distance(self, lidar_data: np.ndarray) -> float:
        """Get minimum distance in front arc."""
        return min(lidar_data[[0, 1, 7]])


class NavigationController:
    """Axis-aligned navigation controller."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
    
    def quat_to_yaw(self, quat: np.ndarray) -> float:
        """Convert quaternion to yaw angle."""
        return np.arctan2(
            2*(quat[0]*quat[3] + quat[1]*quat[2]),
            1 - 2*(quat[2]**2 + quat[3]**2)
        )
    
    def get_axis_heading(self, dx: float, dy: float) -> Tuple[float, str]:
        """
        Determine axis-aligned heading and preferred axis.
        
        Args:
            dx: X distance to target
            dy: Y distance to target
            
        Returns:
            Tuple of (desired_yaw, axis)
        """
        if abs(dx) >= abs(dy):  # Go East/West first
            return (0 if dx >= 0 else np.pi), 'X'
        else:  # Go North/South first
            return (np.pi/2 if dy >= 0 else -np.pi/2), 'Y'
    
    def get_turn_command(self, heading_error: float) -> ControlCommand:
        """Generate turn command based on heading error."""
        if heading_error > 0:  # Turn left (CCW)
            return ControlCommand(
                self.config.navigation.turn_speed,
                -self.config.navigation.turn_speed
            )
        else:  # Turn right (CW)
            return ControlCommand(
                -self.config.navigation.turn_speed,
                self.config.navigation.turn_speed
            )


class AutonomousRobot:
    """
    Autonomous robot controller with obstacle avoidance and navigation.
    
    This class encapsulates the robot's behavior including:
    - LiDAR-based obstacle detection
    - Axis-aligned navigation to targets
    - State machine for obstacle avoidance
    - Motor control and sensor fusion
    """
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, 
                 config: SimulationConfig):
        """
        Initialize autonomous robot controller.
        
        Args:
            model: MuJoCo model instance
            data: MuJoCo data instance
            config: Simulation configuration
        """
        self.model = model
        self.data = data
        self.config = config
        self.state = RobotState.IDLE
        self.context = StateContext()
        
        # Initialize subsystems
        self.navigation = NavigationController(config)
        self.obstacle_detector = ObstacleDetector(config)
        
        # Setup wheel geoms for friction control
        self.wheel_geoms = ["zq_collision", "yq_collision", 
                           "yh_collision", "zh_collision"]
        
        # Apply initial configuration
        self._apply_physics_config()
    
    def _apply_physics_config(self):
        """Apply physics configuration to the model."""
        # Set motor torque limits
        for name in ["front_left_motor", "front_right_motor", 
                    "back_left_motor", "back_right_motor"]:
            aid = self.model.actuator(name).id
            self.model.actuator_forcerange[aid] = np.array([
                -self.config.physics.wheel_max_torque, 
                self.config.physics.wheel_max_torque
            ])
        
        # Set wheel joint damping
        for j in ["zq_Joint", "yq_Joint", "yh_Joint", "zh_Joint"]:
            adr = self.model.jnt_dofadr[self.model.joint(j).id]
            self.model.dof_damping[adr] = self.config.physics.wheel_joint_damping
        
        # Apply nominal friction
        self.apply_friction(self.config.physics.nominal_friction)
    
    def apply_friction(self, friction_triplet: np.ndarray):
        """Apply friction to wheels and floor."""
        for geom_name in self.wheel_geoms:
            self.model.geom_friction[self.model.geom(geom_name).id] = friction_triplet
        self.model.geom_friction[self.model.geom('floor').id] = friction_triplet
        mujoco.mj_forward(self.model, self.data)
    
    def set_target(self, target: Tuple[float, float]):
        """
        Set navigation target.
        
        Args:
            target: Tuple of (x, y) coordinates in meters
        """
        self.context.target_beacon = target
        self.context.axis_mode = None  # Reset axis mode for new target
        self.state = RobotState.NAVIGATING
    
    def has_arrived(self) -> bool:
        """
        Check if robot has reached the current target.
        
        Returns:
            True if within arrival tolerance of target
        """
        if self.context.target_beacon is None:
            return False
        
        pos = self.data.sensordata[9:12]  # Position from IMU
        dx = self.context.target_beacon[0] - pos[0]
        dy = self.context.target_beacon[1] - pos[1]
        dist = np.hypot(dx, dy)
        
        return dist < self.config.navigation.arrival_tolerance
    
    def get_sensor_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current sensor readings.
        
        Returns:
            Tuple of (lidar_data, position, orientation)
        """
        lidar_data = self.data.sensordata[0:8].copy()
        position = self.data.sensordata[9:12]
        orientation = self.data.sensordata[12:16]
        return lidar_data, position, orientation
    
    def update(self) -> ControlCommand:
        """
        Update robot state and return control command.
        
        This method performs one simulation step including:
        1. Sensor data acquisition
        2. Obstacle detection
        3. Navigation planning
        4. State machine updates
        
        Returns:
            ControlCommand: Left and right wheel speeds
            
        Raises:
            RuntimeError: If sensor data is invalid
        """
        # Get sensor data
        lidar_data, position, orientation = self.get_sensor_data()
        
        # Check for arrival
        if self.has_arrived():
            self.state = RobotState.ARRIVED
            return ControlCommand(0.0, 0.0)
        
        # No target set
        if self.context.target_beacon is None:
            return ControlCommand(0.0, 0.0)
        
        # Calculate navigation parameters
        yaw = self.navigation.quat_to_yaw(orientation)
        dx = self.context.target_beacon[0] - position[0]
        dy = self.context.target_beacon[1] - position[1]
        dist = np.hypot(dx, dy)
        
        # Determine desired heading and axis
        if self.context.axis_mode is None:
            desired_yaw, self.context.axis_mode = self.navigation.get_axis_heading(dx, dy)
        else:
            # Check if current axis is complete
            if (self.context.axis_mode == 'X' and abs(dx) < self.config.navigation.arrival_tolerance) or \
               (self.context.axis_mode == 'Y' and abs(dy) < self.config.navigation.arrival_tolerance):
                # Switch to other axis if needed
                if self.context.axis_mode == 'X' and abs(dy) >= self.config.navigation.arrival_tolerance:
                    desired_yaw, self.context.axis_mode = self.navigation.get_axis_heading(0, dy)
                elif self.context.axis_mode == 'Y' and abs(dx) >= self.config.navigation.arrival_tolerance:
                    desired_yaw, self.context.axis_mode = self.navigation.get_axis_heading(dx, 0)
                else:
                    # Both axes complete
                    return ControlCommand(0.0, 0.0)
            else:
                # Continue on current axis
                desired_yaw = 0 if self.context.axis_mode == 'X' else np.pi/2
                if self.context.axis_mode == 'X' and dx < 0:
                    desired_yaw = np.pi
                if self.context.axis_mode == 'Y' and dy < 0:
                    desired_yaw = -np.pi/2
        
        # Calculate heading error
        heading_error = (desired_yaw - yaw + np.pi) % (2*np.pi) - np.pi
        
        # Handle obstacle avoidance
        if self.context.reverse_counter > 0:
            self.context.reverse_counter -= 1
            return ControlCommand(
                self.config.navigation.reverse_speed,
                self.config.navigation.reverse_speed
            )
        
        # Check for obstacles
        obstacle_detected = self.obstacle_detector.detect_obstacle(lidar_data)
        
        if obstacle_detected:
            if not self.context.obstacle_detected:
                # Just detected obstacle - start reversing
                self.context.reverse_counter = self.config.navigation.reverse_steps
                self.context.obstacle_detected = True
                return ControlCommand(
                    self.config.navigation.reverse_speed,
                    self.config.navigation.reverse_speed
                )
            else:
                # Still avoiding - turn to clear
                return self.navigation.get_turn_command(heading_error)
        else:
            self.context.obstacle_detected = False
        
        # Normal navigation
        if abs(heading_error) > self.config.navigation.heading_tolerance:
            # Turn to align
            return self.navigation.get_turn_command(heading_error)
        else:
            # Drive forward
            return ControlCommand(
                self.config.control.drive_speed,
                self.config.control.drive_speed
            )
    
    def apply_control(self, command: ControlCommand):
        """Apply control command to robot actuators."""
        self.data.ctrl[0] = -command.left_speed   # front_left_motor
        self.data.ctrl[1] = command.right_speed   # front_right_motor
        self.data.ctrl[2] = -command.left_speed   # back_left_motor
        self.data.ctrl[3] = command.right_speed   # back_right_motor
