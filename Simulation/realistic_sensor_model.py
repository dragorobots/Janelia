"""
Realistic Sensor Model for SLAM Simulation.

This module implements a realistic LiDAR sensor model based on actual hardware
characterization data, including systematic biases, measurement noise, and
material-dependent effects.
"""

import numpy as np
import json
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import math

from config import SimulationConfig


@dataclass
class SensorModelParams:
    """Parameters for realistic sensor modeling."""
    systematic_bias: float = 0.0
    systematic_bias_std: float = 0.0
    measurement_noise_std: float = 0.02
    material_effects: Dict[str, Dict] = None
    distance_effects: Dict = None
    angle_effects: Dict = None
    
    def __post_init__(self):
        if self.material_effects is None:
            self.material_effects = {
                "wall": {"mean_error": 0.0, "std_error": 0.02},
                "cardboard": {"mean_error": 0.01, "std_error": 0.03},
                "metal": {"mean_error": -0.005, "std_error": 0.015},
                "fabric": {"mean_error": 0.02, "std_error": 0.05}
            }
        if self.distance_effects is None:
            self.distance_effects = {
                "polynomial_coefficients": [0.001, -0.002, 0.0],
                "distance_error_correlation": 0.3
            }
        if self.angle_effects is None:
            self.angle_effects = {
                "angle_error_correlation": 0.1,
                "angle_error_std": 0.01
            }


class RealisticLidarModel:
    """
    Realistic LiDAR sensor model based on hardware characterization.
    
    This model simulates real LiDAR behavior including:
    - Systematic biases
    - Measurement noise
    - Material-dependent effects
    - Distance-dependent errors
    - Angle-dependent errors
    - Multi-path effects
    - Saturation and minimum range effects
    """
    
    def __init__(self, sensor_model_file: Optional[str] = None, 
                 config: Optional[SensorModelParams] = None):
        """
        Initialize realistic LiDAR model.
        
        Args:
            sensor_model_file: Path to JSON file with characterization data
            config: Sensor model parameters (if file not provided)
        """
        if sensor_model_file:
            self.params = self._load_sensor_model(sensor_model_file)
        else:
            self.params = config or SensorModelParams()
        
        # LiDAR beam configuration
        self.beam_angles = np.array([0, 45, 90, 135, 180, 225, 270, 315]) * np.pi / 180
        self.num_beams = len(self.beam_angles)
        
        # Sensor limits
        self.min_range = 0.1  # 10cm minimum range
        self.max_range = 4.0  # 4m maximum range
        self.range_resolution = 0.01  # 1cm resolution
        
        # Multi-path effect parameters
        self.multipath_probability = 0.05  # 5% chance of multi-path
        self.multipath_factor = 1.5  # Multi-path readings are 1.5x longer
        
        # Saturation effects
        self.saturation_distance = 3.5  # Distance where saturation starts
        self.saturation_factor = 0.95  # Saturation reduces readings to 95%
    
    def _load_sensor_model(self, filename: str) -> SensorModelParams:
        """Load sensor model from JSON file."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            return SensorModelParams(
                systematic_bias=data.get('systematic_bias', 0.0),
                systematic_bias_std=data.get('systematic_bias_std', 0.0),
                measurement_noise_std=data.get('measurement_noise_std', 0.02),
                material_effects=data.get('material_effects', {}),
                distance_effects=data.get('distance_effects', {}),
                angle_effects=data.get('angle_effects', {})
            )
        except Exception as e:
            print(f"Warning: Could not load sensor model from {filename}: {e}")
            return SensorModelParams()
    
    def simulate_reading(self, true_distance: float, beam_angle: float, 
                        material: str = "wall", temperature: float = 25.0) -> float:
        """
        Simulate a single LiDAR reading with realistic errors.
        
        Args:
            true_distance: True distance to target in meters
            beam_angle: Angle of the LiDAR beam in radians
            material: Material of the target surface
            temperature: Sensor temperature in Celsius
            
        Returns:
            Simulated distance reading in meters
        """
        # Start with true distance
        reading = true_distance
        
        # Apply systematic bias
        bias = np.random.normal(self.params.systematic_bias, self.params.systematic_bias_std)
        reading += bias
        
        # Apply material-dependent effects
        if material in self.params.material_effects:
            material_error = np.random.normal(
                self.params.material_effects[material]["mean_error"],
                self.params.material_effects[material]["std_error"]
            )
            reading += material_error
        
        # Apply distance-dependent effects
        if self.params.distance_effects:
            coeffs = self.params.distance_effects.get("polynomial_coefficients", [0, 0, 0])
            distance_error = np.polyval(coeffs, true_distance)
            reading += distance_error
        
        # Apply angle-dependent effects
        if self.params.angle_effects:
            angle_error = np.random.normal(0, self.params.angle_effects["angle_error_std"])
            reading += angle_error
        
        # Apply measurement noise
        noise = np.random.normal(0, self.params.measurement_noise_std)
        reading += noise
        
        # Apply multi-path effects (simulate reflections)
        if np.random.random() < self.multipath_probability:
            reading *= self.multipath_factor
        
        # Apply saturation effects for very bright targets
        if reading > self.saturation_distance:
            reading = self.saturation_distance + (reading - self.saturation_distance) * self.saturation_factor
        
        # Apply sensor limits
        reading = np.clip(reading, self.min_range, self.max_range)
        
        # Apply range resolution (quantization)
        reading = round(reading / self.range_resolution) * self.range_resolution
        
        return reading
    
    def simulate_scan(self, true_distances: np.ndarray, materials: List[str] = None,
                     temperature: float = 25.0) -> np.ndarray:
        """
        Simulate a complete LiDAR scan with realistic errors.
        
        Args:
            true_distances: Array of true distances for each beam
            materials: List of materials for each beam (optional)
            temperature: Sensor temperature in Celsius
            
        Returns:
            Array of simulated distance readings
        """
        if materials is None:
            materials = ["wall"] * len(true_distances)
        
        readings = np.zeros(len(true_distances))
        
        for i, (distance, angle) in enumerate(zip(true_distances, self.beam_angles)):
            material = materials[i] if i < len(materials) else "wall"
            readings[i] = self.simulate_reading(distance, angle, material, temperature)
        
        return readings
    
    def simulate_environment_scan(self, robot_pose: np.ndarray, 
                                environment_map: np.ndarray,
                                map_resolution: float = 0.05,
                                map_origin: Tuple[float, float] = (-5.0, -5.0)) -> np.ndarray:
        """
        Simulate LiDAR scan in a 2D environment.
        
        Args:
            robot_pose: Robot pose [x, y, theta]
            environment_map: 2D occupancy grid (0=free, 1=occupied)
            map_resolution: Resolution of the map in meters per pixel
            map_origin: Origin of the map in world coordinates
            
        Returns:
            Array of simulated LiDAR readings
        """
        readings = np.zeros(self.num_beams)
        
        for i, beam_angle in enumerate(self.beam_angles):
            # Calculate beam direction in world coordinates
            world_angle = robot_pose[2] + beam_angle
            
            # Ray trace to find intersection with environment
            true_distance = self._ray_trace(robot_pose[:2], world_angle, 
                                          environment_map, map_resolution, map_origin)
            
            # Simulate reading
            readings[i] = self.simulate_reading(true_distance, beam_angle)
        
        return readings
    
    def _ray_trace(self, start_pos: np.ndarray, angle: float, 
                   environment_map: np.ndarray, map_resolution: float,
                   map_origin: Tuple[float, float]) -> float:
        """
        Ray trace to find distance to nearest obstacle.
        
        Args:
            start_pos: Starting position [x, y]
            angle: Ray angle in radians
            environment_map: 2D occupancy grid
            map_resolution: Map resolution in meters per pixel
            map_origin: Map origin in world coordinates
            
        Returns:
            Distance to nearest obstacle
        """
        # Convert world coordinates to map coordinates
        map_x = int((start_pos[0] - map_origin[0]) / map_resolution)
        map_y = int((start_pos[1] - map_origin[1]) / map_resolution)
        
        # Ray direction
        dx = np.cos(angle)
        dy = np.sin(angle)
        
        # Step size for ray tracing
        step_size = map_resolution / 2.0
        
        # Maximum ray length
        max_distance = self.max_range
        
        # Ray trace
        current_x = start_pos[0]
        current_y = start_pos[1]
        distance = 0.0
        
        while distance < max_distance:
            # Step along ray
            current_x += dx * step_size
            current_y += dy * step_size
            distance += step_size
            
            # Convert to map coordinates
            map_x = int((current_x - map_origin[0]) / map_resolution)
            map_y = int((current_y - map_origin[1]) / map_resolution)
            
            # Check bounds
            if (map_x < 0 or map_x >= environment_map.shape[1] or 
                map_y < 0 or map_y >= environment_map.shape[0]):
                return self.max_range
            
            # Check for obstacle
            if environment_map[map_y, map_x] > 0.5:  # Occupied
                return distance
        
        return self.max_range
    
    def get_sensor_characteristics(self) -> Dict:
        """Get sensor characteristics for analysis."""
        return {
            'systematic_bias': self.params.systematic_bias,
            'systematic_bias_std': self.params.systematic_bias_std,
            'measurement_noise_std': self.params.measurement_noise_std,
            'min_range': self.min_range,
            'max_range': self.max_range,
            'range_resolution': self.range_resolution,
            'beam_angles': self.beam_angles.tolist(),
            'material_effects': self.params.material_effects,
            'distance_effects': self.params.distance_effects,
            'angle_effects': self.params.angle_effects
        }


class RealisticIMUModel:
    """
    Realistic IMU sensor model.
    
    Simulates realistic IMU behavior including:
    - Gyroscope drift
    - Accelerometer bias
    - Temperature effects
    - Noise characteristics
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """Initialize realistic IMU model."""
        self.config = config or {
            'gyro_drift_rate': 0.001,  # rad/s^2
            'gyro_noise_std': 0.01,    # rad/s
            'accel_bias': np.array([0.1, 0.1, 0.1]),  # m/s^2
            'accel_noise_std': 0.05,   # m/s^2
            'temperature_coefficient': 0.001  # per degree C
        }
        
        # Initialize bias states
        self.gyro_bias = np.zeros(3)
        self.accel_bias = self.config['accel_bias'].copy()
        
    def simulate_reading(self, true_angular_vel: np.ndarray, 
                        true_linear_accel: np.ndarray,
                        temperature: float = 25.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulate IMU readings with realistic errors.
        
        Args:
            true_angular_vel: True angular velocity [wx, wy, wz]
            true_linear_accel: True linear acceleration [ax, ay, az]
            temperature: Sensor temperature in Celsius
            
        Returns:
            Tuple of (angular_velocity, linear_acceleration) with noise
        """
        # Update gyroscope bias (drift)
        self.gyro_bias += np.random.normal(0, self.config['gyro_drift_rate'], 3)
        
        # Apply temperature effects
        temp_factor = 1.0 + self.config['temperature_coefficient'] * (temperature - 25.0)
        
        # Simulate gyroscope reading
        gyro_reading = true_angular_vel + self.gyro_bias * temp_factor
        gyro_reading += np.random.normal(0, self.config['gyro_noise_std'], 3)
        
        # Simulate accelerometer reading
        accel_reading = true_linear_accel + self.accel_bias * temp_factor
        accel_reading += np.random.normal(0, self.config['accel_noise_std'], 3)
        
        return gyro_reading, accel_reading


class RealisticOdometryModel:
    """
    Realistic Odometry sensor model.
    
    Simulates realistic odometry behavior including:
    - Wheel slip and skidding
    - Encoder resolution and quantization
    - Systematic bias (uneven wheels, misalignment)
    - Drift over time
    - Temperature effects on wheel diameter
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """Initialize realistic odometry model."""
        self.config = config or {
            'wheel_slip_probability': 0.02,  # 2% chance of slip
            'wheel_slip_factor': 0.1,        # Slip reduces movement by 10%
            'encoder_resolution': 0.001,     # 1mm encoder resolution
            'systematic_bias': np.array([0.02, 0.02, 0.01]),  # [x, y, theta] bias
            'drift_rate': 0.001,             # Drift per second
            'temperature_coefficient': 0.0001,  # Wheel diameter change per degree C
            'noise_std': 0.005               # 5mm standard deviation
        }
        
        # Initialize bias states
        self.accumulated_bias = np.zeros(3)
        self.drift_offset = np.zeros(3)
        self.last_temperature = 25.0
        
    def simulate_reading(self, true_pose: np.ndarray, true_velocity: np.ndarray,
                        dt: float, temperature: float = 25.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulate odometry readings with realistic errors.
        
        Args:
            true_pose: True robot pose [x, y, theta]
            true_velocity: True robot velocity [vx, vy, vtheta]
            dt: Time step
            temperature: Sensor temperature in Celsius
            
        Returns:
            Tuple of (pose, velocity) with realistic errors
        """
        # Update drift over time
        self.drift_offset += np.random.normal(0, self.config['drift_rate'] * dt, 3)
        
        # Temperature effects on wheel diameter
        temp_factor = 1.0 + self.config['temperature_coefficient'] * (temperature - self.last_temperature)
        self.last_temperature = temperature
        
        # Apply systematic bias
        biased_pose = true_pose + self.config['systematic_bias'] * temp_factor
        
        # Add accumulated bias and drift
        biased_pose += self.accumulated_bias + self.drift_offset
        
        # Simulate wheel slip
        if np.random.random() < self.config['wheel_slip_probability']:
            # Reduce movement due to slip
            velocity_reduction = 1.0 - self.config['wheel_slip_factor']
            biased_velocity = true_velocity * velocity_reduction
        else:
            biased_velocity = true_velocity.copy()
        
        # Add measurement noise
        pose_noise = np.random.normal(0, self.config['noise_std'], 3)
        velocity_noise = np.random.normal(0, self.config['noise_std'] / dt, 3)
        
        # Quantize to encoder resolution
        pose_quantized = np.round(biased_pose / self.config['encoder_resolution']) * self.config['encoder_resolution']
        velocity_quantized = np.round(biased_velocity / self.config['encoder_resolution']) * self.config['encoder_resolution']
        
        # Final readings with noise
        final_pose = pose_quantized + pose_noise
        final_velocity = velocity_quantized + velocity_noise
        
        # Update accumulated bias (simulate calibration drift)
        self.accumulated_bias += np.random.normal(0, 0.0001 * dt, 3)
        
        return final_pose, final_velocity
    
    def reset_bias(self):
        """Reset accumulated bias (useful for calibration)."""
        self.accumulated_bias = np.zeros(3)
        self.drift_offset = np.zeros(3)


def create_realistic_sensor_system(sensor_model_file: Optional[str] = None) -> Tuple[RealisticLidarModel, RealisticIMUModel, RealisticOdometryModel]:
    """
    Create a complete realistic sensor system.
    
    Args:
        sensor_model_file: Path to sensor characterization file
        
    Returns:
        Tuple of (LiDAR model, IMU model, Odometry model)
    """
    lidar_model = RealisticLidarModel(sensor_model_file)
    imu_model = RealisticIMUModel()
    odometry_model = RealisticOdometryModel()
    
    return lidar_model, imu_model, odometry_model


# Example usage and testing
def test_sensor_model():
    """Test the realistic sensor model."""
    # Create sensor model
    lidar_model = RealisticLidarModel()
    
    # Test single reading
    true_distance = 1.5
    reading = lidar_model.simulate_reading(true_distance, 0.0, "wall")
    print(f"True distance: {true_distance}m, Reading: {reading:.3f}m")
    
    # Test complete scan
    true_distances = np.array([1.0, 1.2, 1.5, 1.8, 2.0, 1.8, 1.5, 1.2])
    readings = lidar_model.simulate_scan(true_distances)
    print(f"True distances: {true_distances}")
    print(f"Simulated readings: {readings}")
    
    # Test sensor characteristics
    characteristics = lidar_model.get_sensor_characteristics()
    print(f"Sensor characteristics: {characteristics}")


if __name__ == "__main__":
    test_sensor_model()
