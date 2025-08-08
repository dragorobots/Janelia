"""
Unit tests for the autonomous robot controller.

This module tests the robot's navigation, obstacle detection, and control systems.
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch
import mujoco
import sys
import os

# Add the parent directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import AutonomousRobot, ObstacleDetector, NavigationController, ControlCommand
from config import SimulationConfig


class TestObstacleDetector:
    """Test obstacle detection functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.config = SimulationConfig()
        self.detector = ObstacleDetector(self.config)
    
    def test_no_obstacle_detection(self):
        """Test that no obstacle is detected with clear readings."""
        lidar_data = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        assert not self.detector.detect_obstacle(lidar_data)
    
    def test_front_obstacle_detection(self):
        """Test detection of front obstacles."""
        # Front arc beams (0, 1, 7) too close
        lidar_data = np.array([0.1, 0.15, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1])
        assert self.detector.detect_obstacle(lidar_data)
    
    def test_side_obstacle_detection(self):
        """Test detection of side obstacles."""
        # Side beam too close
        lidar_data = np.array([1.0, 1.0, 1.0, 0.15, 1.0, 1.0, 1.0, 1.0])
        assert self.detector.detect_obstacle(lidar_data)
    
    def test_front_distance_calculation(self):
        """Test front distance calculation."""
        lidar_data = np.array([0.5, 0.3, 1.0, 1.0, 1.0, 1.0, 1.0, 0.4])
        front_dist = self.detector.get_front_distance(lidar_data)
        assert front_dist == 0.3  # Minimum of front arc


class TestNavigationController:
    """Test navigation controller functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.config = SimulationConfig()
        self.nav = NavigationController(self.config)
    
    def test_quat_to_yaw_conversion(self):
        """Test quaternion to yaw conversion."""
        # Identity quaternion (no rotation)
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        yaw = self.nav.quat_to_yaw(quat)
        assert abs(yaw) < 1e-6
        
        # 90 degree rotation around Z
        quat = np.array([0.7071, 0.0, 0.0, 0.7071])
        yaw = self.nav.quat_to_yaw(quat)
        assert abs(yaw - np.pi/2) < 0.1
    
    def test_axis_heading_east_west(self):
        """Test axis heading for east/west movement."""
        # Go east (positive X)
        desired_yaw, axis = self.nav.get_axis_heading(2.0, 1.0)
        assert axis == 'X'
        assert abs(desired_yaw) < 1e-6
        
        # Go west (negative X)
        desired_yaw, axis = self.nav.get_axis_heading(-2.0, 1.0)
        assert axis == 'X'
        assert abs(desired_yaw - np.pi) < 1e-6
    
    def test_axis_heading_north_south(self):
        """Test axis heading for north/south movement."""
        # Go north (positive Y)
        desired_yaw, axis = self.nav.get_axis_heading(1.0, 2.0)
        assert axis == 'Y'
        assert abs(desired_yaw - np.pi/2) < 1e-6
        
        # Go south (negative Y)
        desired_yaw, axis = self.nav.get_axis_heading(1.0, -2.0)
        assert axis == 'Y'
        assert abs(desired_yaw + np.pi/2) < 1e-6
    
    def test_turn_command_generation(self):
        """Test turn command generation."""
        # Positive heading error (turn left)
        cmd = self.nav.get_turn_command(0.5)
        assert cmd.left_speed > 0
        assert cmd.right_speed < 0
        
        # Negative heading error (turn right)
        cmd = self.nav.get_turn_command(-0.5)
        assert cmd.left_speed < 0
        assert cmd.right_speed > 0


class TestAutonomousRobot:
    """Test autonomous robot functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.config = SimulationConfig()
        
        # Mock MuJoCo model and data
        self.mock_model = Mock()
        self.mock_data = Mock()
        
        # Mock sensor data
        self.mock_data.sensordata = np.zeros(16)
        self.mock_data.ctrl = np.zeros(6)
        
        # Mock actuators and joints
        self.mock_model.actuator = Mock()
        self.mock_model.joint = Mock()
        self.mock_model.geom = Mock()
        self.mock_model.actuator_forcerange = np.zeros((4, 2))
        self.mock_model.dof_damping = np.zeros(4)
        self.mock_model.geom_friction = np.zeros((10, 3))
        
        # Mock actuator and joint IDs
        mock_actuator = Mock()
        mock_actuator.id = 0
        self.mock_model.actuator.return_value = mock_actuator
        
        mock_joint = Mock()
        mock_joint.id = 0
        self.mock_model.joint.return_value = mock_joint
        
        mock_geom = Mock()
        mock_geom.id = 0
        self.mock_model.geom.return_value = mock_geom
        
        # Mock jnt_dofadr
        self.mock_model.jnt_dofadr = np.array([0, 0, 0, 0])
        
        # Mock mujoco.mj_forward to avoid calling the real function
        self.mj_forward_patcher = patch('robot.mujoco.mj_forward')
        self.mock_mj_forward = self.mj_forward_patcher.start()
    
    def teardown_method(self):
        """Clean up after each test."""
        if hasattr(self, 'mj_forward_patcher'):
            self.mj_forward_patcher.stop()
    
    def test_robot_initialization(self):
        """Test robot initialization."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        assert robot.state.value == "idle"
        assert robot.context.target_beacon is None
    
    def test_set_target(self):
        """Test setting navigation target."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        target = (1.0, 2.0)
        robot.set_target(target)
        assert robot.context.target_beacon == target
        assert robot.state.value == "navigating"
    
    def test_has_arrived_no_target(self):
        """Test arrival check with no target."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        assert not robot.has_arrived()
    
    def test_has_arrived_with_target(self):
        """Test arrival check with target."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        robot.set_target((0.0, 0.0))
        
        # Set position close to target
        self.mock_data.sensordata[9:12] = [0.1, 0.1, 0.0]
        assert robot.has_arrived()
        
        # Set position far from target
        self.mock_data.sensordata[9:12] = [1.0, 1.0, 0.0]
        assert not robot.has_arrived()
    
    def test_update_no_target(self):
        """Test update with no target set."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        cmd = robot.update()
        assert cmd.left_speed == 0.0
        assert cmd.right_speed == 0.0
    
    def test_update_with_target(self):
        """Test update with target set."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        robot.set_target((1.0, 0.0))
        
        # Set sensor data for navigation
        self.mock_data.sensordata[0:8] = [1.0] * 8  # Clear LiDAR
        self.mock_data.sensordata[9:12] = [0.0, 0.0, 0.0]  # Position
        self.mock_data.sensordata[12:16] = [1.0, 0.0, 0.0, 0.0]  # Orientation
        
        cmd = robot.update()
        # Should drive forward toward target
        assert cmd.left_speed > 0
        assert cmd.right_speed > 0
    
    def test_apply_control(self):
        """Test applying control commands."""
        robot = AutonomousRobot(self.mock_model, self.mock_data, self.config)
        cmd = ControlCommand(0.5, -0.3)
        robot.apply_control(cmd)
        
        # Check that control values are applied correctly
        assert self.mock_data.ctrl[0] == -0.5  # front_left_motor
        assert self.mock_data.ctrl[1] == -0.3  # front_right_motor
        assert self.mock_data.ctrl[2] == -0.5  # back_left_motor
        assert self.mock_data.ctrl[3] == -0.3  # back_right_motor


class TestControlCommand:
    """Test control command data structure."""
    
    def test_control_command_creation(self):
        """Test creating control commands."""
        cmd = ControlCommand(0.5, -0.3)
        assert cmd.left_speed == 0.5
        assert cmd.right_speed == -0.3
    
    def test_control_command_immutability(self):
        """Test that control commands are immutable."""
        cmd = ControlCommand(0.5, -0.3)
        with pytest.raises(AttributeError):
            cmd.left_speed = 1.0


if __name__ == "__main__":
    pytest.main([__file__])
