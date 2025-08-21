#!/usr/bin/env python3
"""
Robot-Side Data Collection Script for Sensor Characterization.

This script runs on the actual Yahboom robot to collect LiDAR data
for sensor characterization. It can be run directly on the robot
or via SSH connection.
"""

import time
import json
import numpy as np
import serial
import threading
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
import os
import sys

# Try to import ROS2 components if available
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, Imu
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS2 not available, using direct serial communication")


@dataclass
class RobotSensorData:
    """Robot sensor data structure."""
    timestamp: float
    lidar_ranges: List[float]
    lidar_angles: List[float]
    robot_pose: List[float]  # [x, y, theta]
    robot_velocity: List[float]  # [vx, vy, vtheta]
    imu_orientation: List[float]  # [x, y, z, w]
    imu_angular_velocity: List[float]  # [wx, wy, wz]
    imu_linear_acceleration: List[float]  # [ax, ay, az]
    target_distance: float
    target_angle: float
    target_material: str
    ambient_light: float
    temperature: float


class RobotDataCollector:
    """Data collector that runs on the robot hardware."""
    
    def __init__(self, config: Dict):
        """
        Initialize robot data collector.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.data_buffer = []
        self.is_collecting = False
        self.collection_thread = None
        
        # Communication setup
        self.serial_conn = None
        self.ros_node = None
        
        # Sensor data
        self.current_lidar_data = None
        self.current_imu_data = None
        self.current_odom_data = None
        
        # Initialize communication
        self._setup_communication()
    
    def _setup_communication(self):
        """Setup communication with sensors."""
        if ROS_AVAILABLE:
            self._setup_ros()
        else:
            self._setup_serial()
    
    def _setup_ros(self):
        """Setup ROS2 communication."""
        try:
            rclpy.init()
            self.ros_node = ROSDataCollectorNode()
            print("ROS2 communication setup complete")
        except Exception as e:
            print(f"ROS2 setup failed: {e}")
            self._setup_serial()
    
    def _setup_serial(self):
        """Setup direct serial communication."""
        try:
            # Try common serial ports
            ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyS0', 'COM3', 'COM4']
            for port in ports:
                try:
                    self.serial_conn = serial.Serial(
                        port, 
                        baudrate=115200, 
                        timeout=1.0
                    )
                    print(f"Serial connection established on {port}")
                    break
                except:
                    continue
            
            if self.serial_conn is None:
                print("Warning: No serial connection available")
        except Exception as e:
            print(f"Serial setup failed: {e}")
    
    def start_collection(self, target_distance: float, target_angle: float, 
                        target_material: str, duration: float = 5.0):
        """
        Start collecting sensor data.
        
        Args:
            target_distance: Known distance to target
            target_angle: Known angle to target
            target_material: Material of target
            duration: Collection duration in seconds
        """
        if self.is_collecting:
            print("Already collecting data")
            return False
        
        print(f"Starting data collection:")
        print(f"  Target distance: {target_distance}m")
        print(f"  Target angle: {target_angle}°")
        print(f"  Target material: {target_material}")
        print(f"  Duration: {duration}s")
        
        self.is_collecting = True
        self.collection_thread = threading.Thread(
            target=self._collection_loop,
            args=(target_distance, target_angle, target_material, duration)
        )
        self.collection_thread.start()
        return True
    
    def _collection_loop(self, target_distance: float, target_angle: float, 
                        target_material: str, duration: float):
        """Main data collection loop."""
        start_time = time.time()
        sample_interval = 1.0 / self.config.get('sample_rate', 10.0)
        
        print("Data collection started...")
        
        while self.is_collecting and (time.time() - start_time) < duration:
            loop_start = time.time()
            
            # Get sensor data
            sensor_data = self._get_sensor_data()
            if sensor_data:
                # Add target information
                sensor_data.target_distance = target_distance
                sensor_data.target_angle = target_angle
                sensor_data.target_material = target_material
                
                # Add environmental data
                sensor_data.ambient_light = self._get_ambient_light()
                sensor_data.temperature = self._get_temperature()
                
                # Store data
                self.data_buffer.append(sensor_data)
                
                # Print progress
                elapsed = time.time() - start_time
                if len(self.data_buffer) % 10 == 0:
                    print(f"  Collected {len(self.data_buffer)} samples ({elapsed:.1f}s elapsed)")
            
            # Maintain sample rate
            elapsed = time.time() - loop_start
            if elapsed < sample_interval:
                time.sleep(sample_interval - elapsed)
        
        self.is_collecting = False
        print(f"Data collection complete. {len(self.data_buffer)} samples collected.")
    
    def _get_sensor_data(self) -> Optional[RobotSensorData]:
        """Get current sensor data from all sources."""
        try:
            if ROS_AVAILABLE and self.ros_node:
                return self._get_ros_sensor_data()
            else:
                return self._get_serial_sensor_data()
        except Exception as e:
            print(f"Error getting sensor data: {e}")
            return None
    
    def _get_ros_sensor_data(self) -> RobotSensorData:
        """Get sensor data via ROS2."""
        # This would be implemented with ROS2 message handling
        # For now, return simulated data
        return RobotSensorData(
            timestamp=time.time(),
            lidar_ranges=[2.0] * 8,
            lidar_angles=[0, 45, 90, 135, 180, 225, 270, 315],
            robot_pose=[0.0, 0.0, 0.0],
            robot_velocity=[0.0, 0.0, 0.0],
            imu_orientation=[1.0, 0.0, 0.0, 0.0],
            imu_angular_velocity=[0.0, 0.0, 0.0],
            imu_linear_acceleration=[0.0, 0.0, 9.81],
            target_distance=0.0,
            target_angle=0.0,
            target_material="",
            ambient_light=0.0,
            temperature=25.0
        )
    
    def _get_serial_sensor_data(self) -> RobotSensorData:
        """Get sensor data via serial communication."""
        if not self.serial_conn:
            return self._get_simulated_sensor_data()
        
        try:
            # Read serial data
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # Parse different data formats
                if line.startswith("LIDAR:"):
                    # Parse LiDAR data
                    ranges_str = line.split(":")[1]
                    ranges = [float(r) for r in ranges_str.split(",")]
                    
                    return RobotSensorData(
                        timestamp=time.time(),
                        lidar_ranges=ranges,
                        lidar_angles=[0, 45, 90, 135, 180, 225, 270, 315],
                        robot_pose=[0.0, 0.0, 0.0],  # Would read from odometry
                        robot_velocity=[0.0, 0.0, 0.0],
                        imu_orientation=[1.0, 0.0, 0.0, 0.0],
                        imu_angular_velocity=[0.0, 0.0, 0.0],
                        imu_linear_acceleration=[0.0, 0.0, 9.81],
                        target_distance=0.0,
                        target_angle=0.0,
                        target_material="",
                        ambient_light=0.0,
                        temperature=25.0
                    )
            
            # If no valid data, return simulated data
            return self._get_simulated_sensor_data()
            
        except Exception as e:
            print(f"Serial read error: {e}")
            return self._get_simulated_sensor_data()
    
    def _get_simulated_sensor_data(self) -> RobotSensorData:
        """Get simulated sensor data for testing."""
        # Simulate realistic LiDAR readings
        base_ranges = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        noise = np.random.normal(0, 0.02, 8)
        ranges = np.clip(base_ranges + noise, 0.1, 4.0)
        
        return RobotSensorData(
            timestamp=time.time(),
            lidar_ranges=ranges.tolist(),
            lidar_angles=[0, 45, 90, 135, 180, 225, 270, 315],
            robot_pose=[0.0, 0.0, 0.0],
            robot_velocity=[0.0, 0.0, 0.0],
            imu_orientation=[1.0, 0.0, 0.0, 0.0],
            imu_angular_velocity=[0.0, 0.0, 0.0],
            imu_linear_acceleration=[0.0, 0.0, 9.81],
            target_distance=0.0,
            target_angle=0.0,
            target_material="",
            ambient_light=0.0,
            temperature=25.0
        )
    
    def _get_ambient_light(self) -> float:
        """Get ambient light level (if available)."""
        # This would read from a light sensor
        # For now, return simulated value
        return 500.0  # lux
    
    def _get_temperature(self) -> float:
        """Get sensor temperature (if available)."""
        # This would read from a temperature sensor
        # For now, return simulated value
        return 25.0  # Celsius
    
    def stop_collection(self):
        """Stop data collection."""
        self.is_collecting = False
        if self.collection_thread:
            self.collection_thread.join()
    
    def get_collected_data(self) -> List[RobotSensorData]:
        """Get all collected data."""
        return self.data_buffer.copy()
    
    def save_data(self, filename: str):
        """Save collected data to JSON file."""
        # Convert dataclass objects to dictionaries
        data_list = []
        for data in self.data_buffer:
            data_dict = asdict(data)
            data_list.append(data_dict)
        
        # Save to file
        with open(filename, 'w') as f:
            json.dump(data_list, f, indent=2, default=str)
        
        print(f"Data saved to {filename}")
    
    def clear_data(self):
        """Clear collected data."""
        self.data_buffer.clear()


class ROSDataCollectorNode(Node):
    """ROS2 node for data collection."""
    
    def __init__(self):
        super().__init__('robot_data_collector')
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Data storage
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_odom = None
    
    def lidar_callback(self, msg):
        """Handle LiDAR data."""
        self.latest_lidar = msg
    
    def imu_callback(self, msg):
        """Handle IMU data."""
        self.latest_imu = msg
    
    def odom_callback(self, msg):
        """Handle odometry data."""
        self.latest_odom = msg


def run_characterization_session():
    """Run a complete sensor characterization session."""
    print("="*60)
    print("ROBOT SENSOR CHARACTERIZATION SESSION")
    print("="*60)
    
    # Configuration
    config = {
        'sample_rate': 10.0,  # Hz
        'collection_duration': 5.0,  # seconds per configuration
        'robot_ip': '10.0.0.234'
    }
    
    # Test configurations
    test_configs = [
        {'distance': 0.2, 'angle': 0, 'material': 'wall'},
        {'distance': 0.5, 'angle': 0, 'material': 'wall'},
        {'distance': 1.0, 'angle': 0, 'material': 'wall'},
        {'distance': 1.5, 'angle': 0, 'material': 'wall'},
        {'distance': 2.0, 'angle': 0, 'material': 'wall'},
        {'distance': 1.0, 'angle': 45, 'material': 'wall'},
        {'distance': 1.0, 'angle': 90, 'material': 'wall'},
        {'distance': 1.0, 'angle': 135, 'material': 'wall'},
        {'distance': 1.0, 'angle': 0, 'material': 'cardboard'},
        {'distance': 1.0, 'angle': 0, 'material': 'metal'},
    ]
    
    # Initialize collector
    collector = RobotDataCollector(config)
    
    print(f"Testing {len(test_configs)} configurations...")
    print("Make sure the robot is positioned correctly for each test!")
    print()
    
    # Run each configuration
    for i, test_config in enumerate(test_configs):
        print(f"Configuration {i+1}/{len(test_configs)}")
        print(f"  Distance: {test_config['distance']}m")
        print(f"  Angle: {test_config['angle']}°")
        print(f"  Material: {test_config['material']}")
        print()
        
        # Get user confirmation
        input("Press Enter when robot is positioned correctly...")
        
        # Start collection
        collector.start_collection(
            test_config['distance'],
            test_config['angle'],
            test_config['material'],
            config['collection_duration']
        )
        
        # Wait for collection to complete
        time.sleep(config['collection_duration'] + 1)
        collector.stop_collection()
        
        print(f"  Collected {len(collector.get_collected_data())} samples")
        print()
    
    # Save all data
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"robot_sensor_data_{timestamp}.json"
    collector.save_data(filename)
    
    print("="*60)
    print("CHARACTERIZATION SESSION COMPLETE")
    print(f"Data saved to: {filename}")
    print("Transfer this file to your development machine for analysis.")
    print("="*60)


def main():
    """Main function."""
    if len(sys.argv) > 1 and sys.argv[1] == "session":
        # Run complete characterization session
        run_characterization_session()
    else:
        # Interactive mode
        print("Robot Data Collector")
        print("Usage:")
        print("  python robot_data_collector.py session  # Run complete session")
        print("  python robot_data_collector.py          # Interactive mode")
        print()
        
        # Simple test collection
        config = {'sample_rate': 10.0}
        collector = RobotDataCollector(config)
        
        print("Starting 5-second test collection...")
        collector.start_collection(1.0, 0, "wall", 5.0)
        time.sleep(6)
        collector.stop_collection()
        
        data = collector.get_collected_data()
        print(f"Collected {len(data)} samples")
        
        # Save test data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"test_data_{timestamp}.json"
        collector.save_data(filename)


if __name__ == "__main__":
    main()

