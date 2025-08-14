"""
Realistic SLAM System for MuJoCo Simulation.

This module implements a simplified but realistic SLAM system that mimics
the behavior of GMAPPING + Nav2 stack used on the real Yahboom robot.
"""

import numpy as np
import time
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass
from enum import Enum
import math
from collections import deque

from config import SimulationConfig


class MapCell(Enum):
    """Map cell states."""
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


@dataclass
class LaserScan:
    """Simulated laser scan data."""
    ranges: np.ndarray
    angles: np.ndarray
    max_range: float
    min_range: float
    timestamp: float


@dataclass
class OdometryData:
    """Simulated odometry data."""
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    vtheta: float
    timestamp: float


@dataclass
class IMUData:
    """Simulated IMU data."""
    orientation: np.ndarray  # [x, y, z, w] quaternion
    angular_velocity: np.ndarray  # [x, y, z]
    linear_acceleration: np.ndarray  # [x, y, z]
    timestamp: float


class OccupancyGrid:
    """2D occupancy grid map."""
    
    def __init__(self, width: int, height: int, resolution: float, origin: Tuple[float, float]):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x, self.origin_y = origin
        self.grid = np.full((height, width), MapCell.UNKNOWN.value, dtype=np.int8)
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y
    
    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid."""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def update_cell(self, grid_x: int, grid_y: int, log_odds_update: float):
        """Update cell with log odds."""
        if self.is_valid_cell(grid_x, grid_y):
            self.log_odds[grid_y, grid_x] += log_odds_update
            # Convert log odds to occupancy
            if self.log_odds[grid_y, grid_x] > 0.5:
                self.grid[grid_y, grid_x] = MapCell.OCCUPIED.value
            elif self.log_odds[grid_y, grid_x] < -0.5:
                self.grid[grid_y, grid_x] = MapCell.FREE.value
            else:
                self.grid[grid_y, grid_x] = MapCell.UNKNOWN.value


class ParticleFilter:
    """Particle filter for robot localization."""
    
    def __init__(self, num_particles: int = 1000):
        self.num_particles = num_particles
        self.particles = np.zeros((num_particles, 3))  # [x, y, theta]
        self.weights = np.ones(num_particles) / num_particles
        self.initialized = False
        
    def initialize(self, x: float, y: float, theta: float, std_dev: float = 0.1):
        """Initialize particles around given pose."""
        self.particles[:, 0] = np.random.normal(x, std_dev, self.num_particles)
        self.particles[:, 1] = np.random.normal(y, std_dev, self.num_particles)
        self.particles[:, 2] = np.random.normal(theta, std_dev, self.num_particles)
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.initialized = True
        
    def predict(self, odom_delta: np.ndarray, noise_std: float = 0.05):
        """Predict particle positions based on odometry."""
        if not self.initialized:
            return
            
        # Add noise to odometry
        noise = np.random.normal(0, noise_std, (self.num_particles, 3))
        self.particles += odom_delta + noise
        
        # Normalize angles
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2]))
        
    def update(self, scan: LaserScan, map_grid: OccupancyGrid):
        """Update particle weights based on laser scan."""
        if not self.initialized:
            return
            
        for i in range(self.num_particles):
            weight = self.compute_scan_weight(scan, map_grid, self.particles[i])
            self.weights[i] *= weight
            
        # Normalize weights
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles
            
    def resample(self):
        """Resample particles based on weights."""
        if not self.initialized:
            return
            
        # Systematic resampling
        cumulative_weights = np.cumsum(self.weights)
        new_particles = np.zeros_like(self.particles)
        
        for i in range(self.num_particles):
            u = np.random.uniform(0, 1)
            j = np.searchsorted(cumulative_weights, u)
            new_particles[i] = self.particles[j]
            
        self.particles = new_particles
        self.weights = np.ones(self.num_particles) / self.num_particles
        
    def get_pose(self) -> Tuple[float, float, float]:
        """Get estimated pose (weighted average)."""
        if not self.initialized:
            return 0.0, 0.0, 0.0
            
        weighted_pose = np.average(self.particles, weights=self.weights, axis=0)
        return weighted_pose[0], weighted_pose[1], weighted_pose[2]
        
    def compute_scan_weight(self, scan: LaserScan, map_grid: OccupancyGrid, pose: np.ndarray) -> float:
        """Compute weight for a particle based on scan match."""
        x, y, theta = pose
        weight = 1.0
        
        for i, (range_meas, angle) in enumerate(zip(scan.ranges, scan.angles)):
            if range_meas < scan.min_range or range_meas > scan.max_range:
                continue
                
            # Transform scan point to world coordinates
            scan_x = x + range_meas * np.cos(theta + angle)
            scan_y = y + range_meas * np.sin(theta + angle)
            
            # Check if point matches map
            grid_x, grid_y = map_grid.world_to_grid(scan_x, scan_y)
            if map_grid.is_valid_cell(grid_x, grid_y):
                if map_grid.grid[grid_y, grid_x] == MapCell.OCCUPIED.value:
                    weight *= 1.2  # Good match
                elif map_grid.grid[grid_y, grid_x] == MapCell.FREE.value:
                    weight *= 0.8  # Bad match
                    
        return weight


class SLAMSystem:
    """Complete SLAM system simulating GMAPPING + Nav2."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.map_grid = OccupancyGrid(
            width=200, height=200, 
            resolution=0.05, 
            origin=(-5.0, -5.0)
        )
        self.particle_filter = ParticleFilter(num_particles=500)
        
        # Robot state
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.robot_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vtheta]
        
        # Sensor data history
        self.scan_history = deque(maxlen=10)
        self.odom_history = deque(maxlen=10)
        self.imu_history = deque(maxlen=10)
        
        # Navigation state
        self.current_goal = None
        self.path_to_goal = []
        self.navigation_active = False
        
        # SLAM parameters
        self.log_odds_hit = 0.7
        self.log_odds_miss = -0.4
        self.max_range = 3.0
        self.min_range = 0.1
        
    def update_sensors(self, lidar_data: np.ndarray, imu_data: np.ndarray, 
                      odom_data: np.ndarray, dt: float):
        """Update sensor data and process SLAM."""
        # Create sensor data objects
        scan = self.create_laser_scan(lidar_data)
        imu = self.create_imu_data(imu_data)
        odom = self.create_odometry_data(odom_data)
        
        # Store history
        self.scan_history.append(scan)
        self.imu_history.append(imu)
        self.odom_history.append(odom)
        
        # Update robot state
        self.update_robot_state(odom, dt)
        
        # Initialize particle filter if needed
        if not self.particle_filter.initialized:
            self.particle_filter.initialize(
                self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]
            )
        
        # Update localization
        if len(self.odom_history) >= 2:
            odom_delta = self.compute_odometry_delta()
            self.particle_filter.predict(odom_delta)
            self.particle_filter.update(scan, self.map_grid)
            self.particle_filter.resample()
            
            # Update robot pose estimate
            estimated_pose = self.particle_filter.get_pose()
            self.robot_pose = np.array(estimated_pose)
        
        # Update map
        self.update_map(scan)
        
        # Update navigation
        if self.navigation_active:
            self.update_navigation()
    
    def create_laser_scan(self, lidar_data: np.ndarray) -> LaserScan:
        """Create laser scan from LiDAR data."""
        # Convert 8-beam LiDAR to full scan
        angles = np.array([0, 45, 90, 135, 180, 225, 270, 315]) * np.pi / 180
        ranges = lidar_data[:8]  # First 8 values are LiDAR
        
        # Interpolate to create more beams (simulate higher resolution)
        full_angles = np.linspace(0, 2*np.pi, 360, endpoint=False)
        full_ranges = np.interp(full_angles, angles, ranges)
        
        return LaserScan(
            ranges=full_ranges,
            angles=full_angles,
            max_range=self.max_range,
            min_range=self.min_range,
            timestamp=time.time()
        )
    
    def create_imu_data(self, imu_data: np.ndarray) -> IMUData:
        """Create IMU data from sensor readings."""
        return IMUData(
            orientation=imu_data[:4],  # Quaternion
            angular_velocity=imu_data[4:7],  # Angular velocity
            linear_acceleration=imu_data[7:10],  # Linear acceleration
            timestamp=time.time()
        )
    
    def create_odometry_data(self, odom_data: np.ndarray) -> OdometryData:
        """Create odometry data from wheel encoders."""
        return OdometryData(
            x=odom_data[0],
            y=odom_data[1],
            theta=odom_data[2],
            vx=odom_data[3],
            vy=odom_data[4],
            vtheta=odom_data[5],
            timestamp=time.time()
        )
    
    def update_robot_state(self, odom: OdometryData, dt: float):
        """Update robot state from odometry."""
        self.robot_velocity = np.array([odom.vx, odom.vy, odom.vtheta])
        # Integrate velocity to get position
        self.robot_pose += self.robot_velocity * dt
    
    def compute_odometry_delta(self) -> np.ndarray:
        """Compute odometry delta between last two readings."""
        if len(self.odom_history) < 2:
            return np.zeros(3)
        
        current = self.odom_history[-1]
        previous = self.odom_history[-2]
        
        dx = current.x - previous.x
        dy = current.y - previous.y
        dtheta = current.theta - previous.theta
        
        return np.array([dx, dy, dtheta])
    
    def update_map(self, scan: LaserScan):
        """Update occupancy grid with laser scan."""
        x, y, theta = self.robot_pose
        
        for range_meas, angle in zip(scan.ranges, scan.angles):
            if range_meas < scan.min_range or range_meas > scan.max_range:
                continue
            
            # Endpoint of laser ray
            end_x = x + range_meas * np.cos(theta + angle)
            end_y = y + range_meas * np.sin(theta + angle)
            
            # Bresenham's line algorithm to trace ray
            points = self.bresenham_line(x, y, end_x, end_y)
            
            for i, (px, py) in enumerate(points):
                grid_x, grid_y = self.map_grid.world_to_grid(px, py)
                
                if i == len(points) - 1:  # Endpoint
                    self.map_grid.update_cell(grid_x, grid_y, self.log_odds_hit)
                else:  # Ray points
                    self.map_grid.update_cell(grid_x, grid_y, self.log_odds_miss)
    
    def bresenham_line(self, x0: float, y0: float, x1: float, y1: float) -> List[Tuple[float, float]]:
        """Bresenham's line algorithm for ray tracing."""
        points = []
        
        # Convert to grid coordinates
        grid_x0, grid_y0 = self.map_grid.world_to_grid(x0, y0)
        grid_x1, grid_y1 = self.map_grid.world_to_grid(x1, y1)
        
        dx = abs(grid_x1 - grid_x0)
        dy = abs(grid_y1 - grid_y0)
        
        sx = 1 if grid_x0 < grid_x1 else -1
        sy = 1 if grid_y0 < grid_y1 else -1
        
        err = dx - dy
        
        x, y = grid_x0, grid_y0
        
        while True:
            # Convert back to world coordinates
            world_x, world_y = self.map_grid.grid_to_world(x, y)
            points.append((world_x, world_y))
            
            if x == grid_x1 and y == grid_y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return points
    
    def set_navigation_goal(self, goal_x: float, goal_y: float):
        """Set a navigation goal."""
        self.current_goal = (goal_x, goal_y)
        self.navigation_active = True
        self.path_to_goal = self.plan_path(goal_x, goal_y)
    
    def plan_path(self, goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Simple A* path planning."""
        # Simplified path planning - just direct line for now
        # In a real implementation, this would use A* on the occupancy grid
        return [(goal_x, goal_y)]
    
    def update_navigation(self):
        """Update navigation behavior."""
        if not self.navigation_active or not self.current_goal:
            return
        
        goal_x, goal_y = self.current_goal
        robot_x, robot_y, robot_theta = self.robot_pose
        
        # Check if goal reached
        distance_to_goal = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        if distance_to_goal < 0.1:
            self.navigation_active = False
            self.current_goal = None
            return
        
        # Simple navigation: turn towards goal, then move forward
        angle_to_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
        angle_diff = angle_to_goal - robot_theta
        
        # Normalize angle difference
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Navigation control
        if abs(angle_diff) > 0.1:
            # Turn towards goal
            return np.array([0.0, 0.0, np.sign(angle_diff) * 0.5])
        else:
            # Move forward
            return np.array([0.3, 0.0, 0.0])
    
    def get_navigation_command(self) -> np.ndarray:
        """Get navigation control command."""
        if not self.navigation_active:
            return np.zeros(3)
        
        return self.update_navigation()
    
    def get_map_data(self) -> np.ndarray:
        """Get current map data for visualization."""
        return self.map_grid.grid.copy()
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose estimate."""
        return tuple(self.robot_pose)
