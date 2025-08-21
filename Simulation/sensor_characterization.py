"""
Sensor Characterization System for Realistic SLAM Modeling.

This module provides tools to collect real sensor data from the Yahboom robot
and analyze it to create realistic sensor models for simulation.
"""

import time
import numpy as np
import json
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
import serial
import threading
from collections import deque
import statistics

from config import SimulationConfig


@dataclass
class SensorReading:
    """Single sensor reading with metadata."""
    timestamp: float
    lidar_ranges: np.ndarray  # 8 LiDAR readings
    lidar_angles: np.ndarray  # 8 corresponding angles
    robot_pose: np.ndarray    # [x, y, theta]
    robot_velocity: np.ndarray  # [vx, vy, vtheta]
    imu_orientation: np.ndarray  # [x, y, z, w] quaternion
    imu_angular_velocity: np.ndarray  # [wx, wy, wz]
    imu_linear_acceleration: np.ndarray  # [ax, ay, az]
    target_distance: float    # Known distance to target
    target_angle: float       # Known angle to target
    target_material: str      # Material of target (wall, cardboard, etc.)
    ambient_light: float      # Ambient light level (if available)
    temperature: float        # Sensor temperature (if available)


@dataclass
class CharacterizationConfig:
    """Configuration for sensor characterization."""
    collection_duration: float = 5.0  # seconds per configuration
    sample_rate: float = 10.0         # Hz
    robot_ip: str = "10.0.0.234"      # Default robot IP
    serial_port: str = "COM3"         # Serial port for direct LiDAR access
    baud_rate: int = 115200
    num_configurations: int = 10      # Number of different test configurations
    target_distances: List[float] = None  # Known distances to test
    target_angles: List[float] = None     # Known angles to test
    target_materials: List[str] = None    # Materials to test


class SensorDataCollector:
    """Collects real sensor data from the robot hardware."""
    
    def __init__(self, config: CharacterizationConfig):
        self.config = config
        self.serial_conn = None
        self.data_buffer = deque(maxlen=1000)
        self.is_collecting = False
        self.collection_thread = None
        
        # Initialize target configurations if not provided
        if self.config.target_distances is None:
            self.config.target_distances = [0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        if self.config.target_angles is None:
            self.config.target_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        if self.config.target_materials is None:
            self.config.target_materials = ["wall", "cardboard", "metal", "fabric"]
    
    def connect_to_robot(self) -> bool:
        """Connect to the robot via serial or network."""
        try:
            # Try serial connection first (direct LiDAR access)
            self.serial_conn = serial.Serial(
                self.config.serial_port,
                self.config.baud_rate,
                timeout=1.0
            )
            print(f"Connected to robot via {self.config.serial_port}")
            return True
        except Exception as e:
            print(f"Serial connection failed: {e}")
            print("Falling back to network connection...")
            return self.connect_via_network()
    
    def connect_via_network(self) -> bool:
        """Connect to robot via network (ROS2 or custom protocol)."""
        # This would implement network-based communication
        # For now, we'll simulate the connection
        print(f"Network connection to {self.config.robot_ip} (simulated)")
        return True
    
    def read_lidar_data(self) -> Optional[np.ndarray]:
        """Read LiDAR data from the robot."""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Read LiDAR data from serial
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.startswith("LIDAR:"):
                    # Parse LiDAR data (format: "LIDAR:r1,r2,r3,r4,r5,r6,r7,r8")
                    ranges_str = line.split(":")[1]
                    ranges = [float(r) for r in ranges_str.split(",")]
                    return np.array(ranges)
            except Exception as e:
                print(f"Error reading LiDAR: {e}")
        
        # Fallback: simulate LiDAR data for testing
        return self.simulate_lidar_data()
    
    def simulate_lidar_data(self) -> np.ndarray:
        """Simulate LiDAR data for testing when hardware is not available."""
        # Simulate realistic LiDAR readings with noise
        base_ranges = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        noise = np.random.normal(0, 0.02, 8)  # 2cm standard deviation
        return np.clip(base_ranges + noise, 0.1, 4.0)
    
    def get_robot_pose(self) -> np.ndarray:
        """Get current robot pose (simulated for now)."""
        # In real implementation, this would read from odometry/IMU
        return np.array([0.0, 0.0, 0.0])  # [x, y, theta]
    
    def get_robot_velocity(self) -> np.ndarray:
        """Get current robot velocity (simulated for now)."""
        # In real implementation, this would read from wheel encoders
        return np.array([0.0, 0.0, 0.0])  # [vx, vy, vtheta]
    
    def get_imu_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get IMU data (simulated for now)."""
        # In real implementation, this would read from IMU sensor
        orientation = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w] quaternion
        angular_velocity = np.array([0.0, 0.0, 0.0])  # [wx, wy, wz]
        linear_acceleration = np.array([0.0, 0.0, 9.81])  # [ax, ay, az] (gravity)
        return orientation, angular_velocity, linear_acceleration
    
    def start_collection(self, target_distance: float, target_angle: float, 
                        target_material: str) -> bool:
        """Start collecting sensor data for a specific configuration."""
        if self.is_collecting:
            return False
        
        self.is_collecting = True
        self.collection_thread = threading.Thread(
            target=self._collection_loop,
            args=(target_distance, target_angle, target_material)
        )
        self.collection_thread.start()
        return True
    
    def _collection_loop(self, target_distance: float, target_angle: float, 
                        target_material: str):
        """Main data collection loop."""
        start_time = time.time()
        sample_interval = 1.0 / self.config.sample_rate
        
        print(f"Collecting data for: {target_distance}m at {target_angle}° ({target_material})")
        
        while self.is_collecting and (time.time() - start_time) < self.config.collection_duration:
            loop_start = time.time()
            
            # Read sensor data
            lidar_ranges = self.read_lidar_data()
            if lidar_ranges is not None:
                robot_pose = self.get_robot_pose()
                robot_velocity = self.get_robot_velocity()
                imu_orientation, imu_angular_velocity, imu_linear_acceleration = self.get_imu_data()
                
                # Create sensor reading
                reading = SensorReading(
                    timestamp=time.time(),
                    lidar_ranges=lidar_ranges,
                    lidar_angles=np.array([0, 45, 90, 135, 180, 225, 270, 315]),
                    robot_pose=robot_pose,
                    robot_velocity=robot_velocity,
                    imu_orientation=imu_orientation,
                    imu_angular_velocity=imu_angular_velocity,
                    imu_linear_acceleration=imu_linear_acceleration,
                    target_distance=target_distance,
                    target_angle=target_angle,
                    target_material=target_material,
                    ambient_light=0.0,  # Would read from light sensor
                    temperature=25.0    # Would read from temperature sensor
                )
                
                self.data_buffer.append(reading)
            
            # Maintain sample rate
            elapsed = time.time() - loop_start
            if elapsed < sample_interval:
                time.sleep(sample_interval - elapsed)
        
        self.is_collecting = False
        print(f"Collection complete. {len(self.data_buffer)} samples collected.")
    
    def stop_collection(self):
        """Stop data collection."""
        self.is_collecting = False
        if self.collection_thread:
            self.collection_thread.join()
    
    def get_collected_data(self) -> List[SensorReading]:
        """Get all collected data."""
        return list(self.data_buffer)
    
    def clear_data(self):
        """Clear collected data."""
        self.data_buffer.clear()


class SensorModelAnalyzer:
    """Analyzes collected sensor data to create realistic sensor models."""
    
    def __init__(self):
        self.analysis_results = {}
    
    def analyze_configuration(self, readings: List[SensorReading]) -> Dict:
        """Analyze sensor data for a specific configuration."""
        if not readings:
            return {}
        
        # Extract data
        ranges = np.array([r.lidar_ranges for r in readings])
        timestamps = np.array([r.timestamp for r in readings])
        
        # Get target information from first reading
        target_distance = readings[0].target_distance
        target_angle = readings[0].target_angle
        target_material = readings[0].target_material
        
        # Find the LiDAR beam closest to the target angle
        beam_angles = readings[0].lidar_angles
        target_beam_idx = np.argmin(np.abs(beam_angles - target_angle))
        
        # Analyze the target beam
        target_beam_ranges = ranges[:, target_beam_idx]
        
        analysis = {
            'target_distance': target_distance,
            'target_angle': target_angle,
            'target_material': target_material,
            'target_beam_idx': target_beam_idx,
            'mean_range': float(np.mean(target_beam_ranges)),
            'std_range': float(np.std(target_beam_ranges)),
            'min_range': float(np.min(target_beam_ranges)),
            'max_range': float(np.max(target_beam_ranges)),
            'range_error': float(np.mean(target_beam_ranges) - target_distance),
            'range_error_std': float(np.std(target_beam_ranges - target_distance)),
            'sample_count': len(readings),
            'collection_duration': timestamps[-1] - timestamps[0],
            'all_ranges': target_beam_ranges.tolist(),
            'timestamps': timestamps.tolist()
        }
        
        # Analyze IMU data if available
        if readings and hasattr(readings[0], 'imu_orientation'):
            imu_orientations = np.array([r.imu_orientation for r in readings])
            imu_angular_velocities = np.array([r.imu_angular_velocity for r in readings])
            imu_linear_accelerations = np.array([r.imu_linear_acceleration for r in readings])
            
            analysis['imu_analysis'] = {
                'orientation_std': float(np.std(imu_orientations, axis=0).tolist()),
                'angular_velocity_std': float(np.std(imu_angular_velocities, axis=0).tolist()),
                'linear_acceleration_std': float(np.std(imu_linear_accelerations, axis=0).tolist()),
                'gyro_drift_rate': self._estimate_gyro_drift(imu_angular_velocities, timestamps),
                'accel_bias': float(np.mean(imu_linear_accelerations, axis=0).tolist())
            }
        
        # Analyze odometry data if available
        if readings and hasattr(readings[0], 'robot_velocity'):
            robot_poses = np.array([r.robot_pose for r in readings])
            robot_velocities = np.array([r.robot_velocity for r in readings])
            
            analysis['odometry_analysis'] = {
                'pose_std': float(np.std(robot_poses, axis=0).tolist()),
                'velocity_std': float(np.std(robot_velocities, axis=0).tolist()),
                'systematic_bias': float(np.mean(robot_poses, axis=0).tolist()),
                'drift_rate': self._estimate_odometry_drift(robot_poses, timestamps)
            }
        
        return analysis
    
    def analyze_all_configurations(self, all_readings: Dict[str, List[SensorReading]]) -> Dict:
        """Analyze all collected configurations."""
        results = {}
        
        for config_name, readings in all_readings.items():
            results[config_name] = self.analyze_configuration(readings)
        
        self.analysis_results = results
        return results
    
    def generate_sensor_model(self) -> Dict:
        """Generate a realistic sensor model from analysis results."""
        if not self.analysis_results:
            return {}
        
        # Analyze systematic errors
        range_errors = []
        range_stds = []
        materials = set()
        
        for config_name, analysis in self.analysis_results.items():
            range_errors.append(analysis['range_error'])
            range_stds.append(analysis['range_error_std'])
            materials.add(analysis['target_material'])
        
        # Create sensor model parameters
        sensor_model = {
            'systematic_bias': float(np.mean(range_errors)),
            'systematic_bias_std': float(np.std(range_errors)),
            'measurement_noise_std': float(np.mean(range_stds)),
            'materials': list(materials),
            'material_effects': self._analyze_material_effects(),
            'distance_effects': self._analyze_distance_effects(),
            'angle_effects': self._analyze_angle_effects(),
            'configurations': self.analysis_results
        }
        
        return sensor_model
    
    def _analyze_material_effects(self) -> Dict:
        """Analyze how different materials affect sensor readings."""
        material_data = {}
        
        for config_name, analysis in self.analysis_results.items():
            material = analysis['target_material']
            if material not in material_data:
                material_data[material] = []
            material_data[material].append(analysis['range_error'])
        
        material_effects = {}
        for material, errors in material_data.items():
            material_effects[material] = {
                'mean_error': float(np.mean(errors)),
                'std_error': float(np.std(errors))
            }
        
        return material_effects
    
    def _analyze_distance_effects(self) -> Dict:
        """Analyze how distance affects sensor accuracy."""
        distances = []
        errors = []
        
        for config_name, analysis in self.analysis_results.items():
            distances.append(analysis['target_distance'])
            errors.append(analysis['range_error'])
        
        # Fit polynomial to model distance-dependent errors
        if len(distances) > 2:
            coeffs = np.polyfit(distances, errors, 2)
            distance_effects = {
                'polynomial_coefficients': coeffs.tolist(),
                'distance_error_correlation': float(np.corrcoef(distances, errors)[0, 1])
            }
        else:
            distance_effects = {
                'polynomial_coefficients': [0, 0, 0],
                'distance_error_correlation': 0.0
            }
        
        return distance_effects
    
    def _analyze_angle_effects(self) -> Dict:
        """Analyze how angle affects sensor accuracy."""
        angles = []
        errors = []
        
        for config_name, analysis in self.analysis_results.items():
            angles.append(analysis['target_angle'])
            errors.append(analysis['range_error'])
        
        angle_effects = {
            'angle_error_correlation': float(np.corrcoef(angles, errors)[0, 1]),
            'angle_error_std': float(np.std(errors))
        }
        
        return angle_effects
    
    def save_analysis(self, filename: str):
        """Save analysis results to JSON file."""
        with open(filename, 'w') as f:
            json.dump(self.analysis_results, f, indent=2, default=str)
    
    def save_sensor_model(self, filename: str):
        """Save generated sensor model to JSON file."""
        sensor_model = self.generate_sensor_model()
        with open(filename, 'w') as f:
            json.dump(sensor_model, f, indent=2, default=str)


class CharacterizationRunner:
    """Main class to run the complete sensor characterization process."""
    
    def __init__(self, config: CharacterizationConfig):
        self.config = config
        self.collector = SensorDataCollector(config)
        self.analyzer = SensorModelAnalyzer()
        self.all_readings = {}
    
    def run_characterization(self) -> Dict:
        """Run the complete sensor characterization process."""
        print("Starting sensor characterization...")
        
        # Connect to robot
        if not self.collector.connect_to_robot():
            print("Failed to connect to robot. Exiting.")
            return {}
        
        # Generate test configurations
        configurations = self._generate_test_configurations()
        
        # Collect data for each configuration
        for i, config in enumerate(configurations):
            print(f"\nConfiguration {i+1}/{len(configurations)}")
            print(f"Distance: {config['distance']}m, Angle: {config['angle']}°, Material: {config['material']}")
            
            # Clear previous data
            self.collector.clear_data()
            
            # Start collection
            if self.collector.start_collection(config['distance'], config['angle'], config['material']):
                # Wait for collection to complete
                time.sleep(self.config.collection_duration + 1)
                self.collector.stop_collection()
                
                # Store collected data
                config_name = f"config_{i+1}_{config['distance']}m_{config['angle']}deg_{config['material']}"
                self.all_readings[config_name] = self.collector.get_collected_data()
                
                print(f"Collected {len(self.all_readings[config_name])} samples")
            else:
                print("Failed to start collection")
        
        # Analyze collected data
        print("\nAnalyzing collected data...")
        analysis_results = self.analyzer.analyze_all_configurations(self.all_readings)
        
        # Generate sensor model
        print("Generating sensor model...")
        sensor_model = self.analyzer.generate_sensor_model()
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.analyzer.save_analysis(f"sensor_analysis_{timestamp}.json")
        self.analyzer.save_sensor_model(f"sensor_model_{timestamp}.json")
        
        print(f"\nCharacterization complete!")
        print(f"Results saved to sensor_analysis_{timestamp}.json")
        print(f"Sensor model saved to sensor_model_{timestamp}.json")
        
        return sensor_model
    
    def _generate_test_configurations(self) -> List[Dict]:
        """Generate test configurations based on the config."""
        configurations = []
        
        # Create combinations of distances, angles, and materials
        for distance in self.config.target_distances:
            for angle in self.config.target_angles:
                for material in self.config.target_materials:
                    configurations.append({
                        'distance': distance,
                        'angle': angle,
                        'material': material
                    })
        
        # Limit to the specified number of configurations
        if len(configurations) > self.config.num_configurations:
            # Randomly sample configurations
            np.random.shuffle(configurations)
            configurations = configurations[:self.config.num_configurations]
        
        return configurations
    
    def _estimate_gyro_drift(self, angular_velocities: np.ndarray, timestamps: np.ndarray) -> float:
        """Estimate gyroscope drift rate from angular velocity data."""
        if len(angular_velocities) < 2:
            return 0.0
        
        # Calculate drift as change in angular velocity over time
        time_delta = timestamps[-1] - timestamps[0]
        if time_delta <= 0:
            return 0.0
        
        # Use z-axis angular velocity (yaw) for drift estimation
        z_velocities = angular_velocities[:, 2]
        drift_rate = (z_velocities[-1] - z_velocities[0]) / time_delta
        return float(drift_rate)
    
    def _estimate_odometry_drift(self, poses: np.ndarray, timestamps: np.ndarray) -> float:
        """Estimate odometry drift rate from pose data."""
        if len(poses) < 2:
            return 0.0
        
        # Calculate drift as change in position over time
        time_delta = timestamps[-1] - timestamps[0]
        if time_delta <= 0:
            return 0.0
        
        # Use x and y position for drift estimation
        x_positions = poses[:, 0]
        y_positions = poses[:, 1]
        
        # Calculate total displacement
        displacement = np.sqrt((x_positions[-1] - x_positions[0])**2 + 
                             (y_positions[-1] - y_positions[0])**2)
        
        drift_rate = displacement / time_delta
        return float(drift_rate)


def main():
    """Main function to run sensor characterization."""
    # Create configuration
    config = CharacterizationConfig(
        collection_duration=5.0,
        sample_rate=10.0,
        num_configurations=10,
        target_distances=[0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
        target_angles=[0, 45, 90, 135, 180, 225, 270, 315],
        target_materials=["wall", "cardboard", "metal"]
    )
    
    # Run characterization
    runner = CharacterizationRunner(config)
    sensor_model = runner.run_characterization()
    
    # Print summary
    if sensor_model:
        print("\n" + "="*50)
        print("SENSOR MODEL SUMMARY")
        print("="*50)
        print(f"Systematic bias: {sensor_model['systematic_bias']:.3f} ± {sensor_model['systematic_bias_std']:.3f} m")
        print(f"Measurement noise: {sensor_model['measurement_noise_std']:.3f} m")
        print(f"Materials tested: {sensor_model['materials']}")
        print(f"Configurations analyzed: {len(sensor_model['configurations'])}")


if __name__ == "__main__":
    main()
