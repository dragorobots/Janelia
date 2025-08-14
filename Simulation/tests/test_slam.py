"""
Unit tests for the SLAM system.

This module tests the SLAM system components including occupancy grid,
particle filter, and sensor fusion.
"""

import pytest
import numpy as np
import time
from unittest.mock import Mock, patch
import sys
import os

# Add the parent directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from slam_system import (
    SLAMSystem, OccupancyGrid, ParticleFilter, LaserScan, 
    OdometryData, IMUData, MapCell
)
from config import SimulationConfig


class TestOccupancyGrid:
    """Test occupancy grid functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.grid = OccupancyGrid(width=100, height=100, resolution=0.05, origin=(-2.5, -2.5))
    
    def test_world_to_grid_conversion(self):
        """Test world to grid coordinate conversion."""
        # Test origin
        grid_x, grid_y = self.grid.world_to_grid(-2.5, -2.5)
        assert grid_x == 0
        assert grid_y == 0
        
        # Test positive coordinates
        grid_x, grid_y = self.grid.world_to_grid(0.0, 0.0)
        assert grid_x == 50
        assert grid_y == 50
        
        # Test negative coordinates
        grid_x, grid_y = self.grid.world_to_grid(-1.0, -1.0)
        assert grid_x == 30
        assert grid_y == 30
    
    def test_grid_to_world_conversion(self):
        """Test grid to world coordinate conversion."""
        # Test origin
        world_x, world_y = self.grid.grid_to_world(0, 0)
        assert world_x == -2.5
        assert world_y == -2.5
        
        # Test center
        world_x, world_y = self.grid.grid_to_world(50, 50)
        assert world_x == 0.0
        assert world_y == 0.0
    
    def test_cell_validation(self):
        """Test cell validation."""
        # Valid cells
        assert self.grid.is_valid_cell(0, 0)
        assert self.grid.is_valid_cell(99, 99)
        
        # Invalid cells
        assert not self.grid.is_valid_cell(-1, 0)
        assert not self.grid.is_valid_cell(0, -1)
        assert not self.grid.is_valid_cell(100, 0)
        assert not self.grid.is_valid_cell(0, 100)
    
    def test_cell_update(self):
        """Test cell update with log odds."""
        # Update a cell with hit
        self.grid.update_cell(50, 50, 0.7)  # Hit
        assert self.grid.grid[50, 50] == MapCell.OCCUPIED.value
        
        # Reset log odds and update with miss
        self.grid.log_odds[50, 50] = 0.0
        self.grid.update_cell(50, 50, -0.6)  # Miss (stronger than -0.5 threshold)
        assert self.grid.grid[50, 50] == MapCell.FREE.value
        
        # Reset and update with neutral
        self.grid.log_odds[50, 50] = 0.0
        self.grid.update_cell(50, 50, 0.0)  # Neutral
        assert self.grid.grid[50, 50] == MapCell.UNKNOWN.value


class TestParticleFilter:
    """Test particle filter functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.pf = ParticleFilter(num_particles=100)
        self.grid = OccupancyGrid(width=100, height=100, resolution=0.05, origin=(-2.5, -2.5))
    
    def test_initialization(self):
        """Test particle filter initialization."""
        assert not self.pf.initialized
        assert self.pf.num_particles == 100
        assert self.pf.particles.shape == (100, 3)
        assert self.pf.weights.shape == (100,)
    
    def test_initialization_with_pose(self):
        """Test particle filter initialization with pose."""
        self.pf.initialize(1.0, 2.0, 0.5)
        assert self.pf.initialized
        
        # Check that particles are around the initial pose
        mean_pose = np.mean(self.pf.particles, axis=0)
        assert abs(mean_pose[0] - 1.0) < 0.2
        assert abs(mean_pose[1] - 2.0) < 0.2
        assert abs(mean_pose[2] - 0.5) < 0.2
    
    def test_prediction(self):
        """Test particle prediction."""
        self.pf.initialize(0.0, 0.0, 0.0)
        initial_particles = self.pf.particles.copy()
        
        # Predict with odometry delta
        odom_delta = np.array([0.1, 0.0, 0.0])  # Move forward
        self.pf.predict(odom_delta)
        
        # Check that particles moved
        assert np.any(self.pf.particles != initial_particles)
    
    def test_update_and_resample(self):
        """Test particle update and resampling."""
        self.pf.initialize(0.0, 0.0, 0.0)
        
        # Create mock scan and map
        scan = LaserScan(
            ranges=np.array([1.0, 1.0, 1.0]),
            angles=np.array([0.0, np.pi/2, np.pi]),
            max_range=3.0,
            min_range=0.1,
            timestamp=0.0
        )
        
        # Update particles
        self.pf.update(scan, self.grid)
        
        # Check that weights are normalized
        assert abs(np.sum(self.pf.weights) - 1.0) < 1e-6
        
        # Test resampling
        initial_particles = self.pf.particles.copy()
        self.pf.resample()
        
        # Check that particles changed (due to resampling)
        assert np.any(self.pf.particles != initial_particles)
    
    def test_get_pose(self):
        """Test pose estimation."""
        self.pf.initialize(1.0, 2.0, 0.5)
        
        # Set all weights equal
        self.pf.weights = np.ones(100) / 100
        
        pose = self.pf.get_pose()
        assert abs(pose[0] - 1.0) < 0.2
        assert abs(pose[1] - 2.0) < 0.2
        assert abs(pose[2] - 0.5) < 0.2


class TestSLAMSystem:
    """Test SLAM system functionality."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.config = SimulationConfig()
        self.slam = SLAMSystem(self.config)
    
    def test_initialization(self):
        """Test SLAM system initialization."""
        assert self.slam.map_grid is not None
        assert self.slam.particle_filter is not None
        assert self.slam.robot_pose.shape == (3,)
        assert self.slam.robot_velocity.shape == (3,)
    
    def test_sensor_data_creation(self):
        """Test sensor data object creation."""
        # Test laser scan creation
        lidar_data = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
        scan = self.slam.create_laser_scan(lidar_data)
        assert isinstance(scan, LaserScan)
        assert len(scan.ranges) == 360
        assert len(scan.angles) == 360
        
        # Test IMU data creation
        imu_data = np.random.normal(0, 1, 10)
        imu = self.slam.create_imu_data(imu_data)
        assert isinstance(imu, IMUData)
        assert len(imu.orientation) == 4
        assert len(imu.angular_velocity) == 3
        assert len(imu.linear_acceleration) == 3
        
        # Test odometry data creation
        odom_data = np.array([1.0, 2.0, 0.5, 0.1, 0.0, 0.05])
        odom = self.slam.create_odometry_data(odom_data)
        assert isinstance(odom, OdometryData)
        assert odom.x == 1.0
        assert odom.y == 2.0
        assert odom.theta == 0.5
    
    def test_robot_state_update(self):
        """Test robot state update."""
        initial_pose = self.slam.robot_pose.copy()
        
        # Create odometry data
        odom = OdometryData(
            x=1.0, y=2.0, theta=0.5,
            vx=0.1, vy=0.0, vtheta=0.05,
            timestamp=0.0
        )
        
        # Update robot state
        self.slam.update_robot_state(odom, 0.01)
        
        # Check that pose changed
        assert np.any(self.slam.robot_pose != initial_pose)
    
    def test_odometry_delta_computation(self):
        """Test odometry delta computation."""
        # Add some odometry history
        odom1 = OdometryData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        odom2 = OdometryData(0.1, 0.0, 0.05, 0.1, 0.0, 0.05, 0.01)
        
        self.slam.odom_history.append(odom1)
        self.slam.odom_history.append(odom2)
        
        delta = self.slam.compute_odometry_delta()
        assert delta[0] == 0.1  # dx
        assert delta[1] == 0.0  # dy
        assert delta[2] == 0.05  # dtheta
    
    def test_navigation_goal_setting(self):
        """Test navigation goal setting."""
        goal_x, goal_y = 2.0, 3.0
        self.slam.set_navigation_goal(goal_x, goal_y)
        
        assert self.slam.current_goal == (goal_x, goal_y)
        assert self.slam.navigation_active
    
    def test_map_data_access(self):
        """Test map data access."""
        map_data = self.slam.get_map_data()
        assert map_data.shape == (200, 200)
        assert map_data.dtype == np.int8
    
    def test_robot_pose_access(self):
        """Test robot pose access."""
        pose = self.slam.get_robot_pose()
        assert len(pose) == 3
        assert all(isinstance(x, (int, float)) for x in pose)


class TestSLAMIntegration:
    """Test SLAM system integration."""
    
    def setup_method(self):
        """Setup test fixtures."""
        self.config = SimulationConfig()
        self.slam = SLAMSystem(self.config)
    
    def test_sensor_update_integration(self):
        """Test complete sensor update integration."""
        # Create sensor data
        lidar_data = np.random.uniform(0.5, 3.0, 8)
        imu_data = np.random.normal(0, 0.1, 10)
        odom_data = np.array([0.1, 0.0, 0.05, 0.1, 0.0, 0.05])
        
        # Update SLAM
        self.slam.update_sensors(lidar_data, imu_data, odom_data, 0.01)
        
        # Check that data was processed
        assert len(self.slam.scan_history) > 0
        assert len(self.slam.imu_history) > 0
        assert len(self.slam.odom_history) > 0
    
    def test_navigation_command_generation(self):
        """Test navigation command generation."""
        # Set a goal
        self.slam.set_navigation_goal(2.0, 0.0)
        
        # Get navigation command
        command = self.slam.get_navigation_command()
        assert command is not None
        assert len(command) == 3  # [vx, vy, vtheta]
    
    def test_map_update_with_scan(self):
        """Test map update with laser scan."""
        # Create a scan that should hit a wall
        lidar_data = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        imu_data = np.zeros(10)
        odom_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Update SLAM
        self.slam.update_sensors(lidar_data, imu_data, odom_data, 0.01)
        
        # Check that map was updated
        map_data = self.slam.get_map_data()
        # Should have some occupied cells after processing scan
        assert np.any(map_data == MapCell.OCCUPIED.value)


def test_slam_system_performance():
    """Test SLAM system performance."""
    config = SimulationConfig()
    slam = SLAMSystem(config)
    
    # Simulate 10 sensor updates (reduced for faster testing)
    start_time = time.time()
    
    for i in range(10):
        lidar_data = np.random.uniform(0.5, 3.0, 8)
        imu_data = np.random.normal(0, 0.1, 10)
        odom_data = np.array([i * 0.01, 0, 0, 0.1, 0, 0])
        
        slam.update_sensors(lidar_data, imu_data, odom_data, 0.01)
    
    end_time = time.time()
    processing_time = end_time - start_time
    
    # Should process 10 updates in reasonable time (< 5 seconds)
    assert processing_time < 5.0
    print(f"Processed 10 sensor updates in {processing_time:.3f} seconds")


if __name__ == "__main__":
    # Run performance test
    test_slam_system_performance()
