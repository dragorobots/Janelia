"""
Realistic SLAM-based Autonomous Navigation Simulation.

This simulation integrates the realistic sensor model with MuJoCo to provide
highly accurate autonomous navigation that closely matches real robot behavior.
"""

import time
import keyboard
import numpy as np
import glfw
import mujoco
import mujoco.viewer
import argparse
from typing import Optional, Tuple
import json

from config import SimulationConfig
from slam_system import SLAMSystem
from validation import SimulationValidator, SimulationLogger
from realistic_sensor_model import RealisticLidarModel, RealisticIMUModel, create_realistic_sensor_system


class RealisticSLAMSimulation:
    """
    Realistic SLAM-based autonomous navigation simulation.
    
    This class integrates realistic sensor models with MuJoCo simulation to provide
    highly accurate autonomous navigation capabilities that closely match real robot behavior.
    """
    
    def __init__(self, model_path: str = 'main_scene.xml', 
                 config: Optional[SimulationConfig] = None,
                 sensor_model_file: Optional[str] = None):
        self.config = config or SimulationConfig()
        self.validator = SimulationValidator()
        self.logger = SimulationLogger()
        
        # Validate model
        is_valid, error_msg = self.validator.validate_model(model_path)
        if not is_valid:
            raise ValueError(f"Invalid model: {error_msg}")
        
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Initialize realistic sensor models
        self.lidar_model, self.imu_model, self.odometry_model = create_realistic_sensor_system(sensor_model_file)
        
        # Initialize SLAM system
        self.slam = SLAMSystem(self.config)
        
        # Simulation state
        self.running = True
        self.paused = False
        self.last_time = time.time()
        
        # Navigation state
        self.navigation_mode = False
        self.goal_points = []
        self.current_goal_index = 0
        
        # Sensor data
        self.lidar_data = np.zeros(8)
        self.imu_data = np.zeros(10)
        self.odom_data = np.zeros(6)
        
        # Control commands
        self.control_command = np.zeros(4)
        
        # Initialize robot pose
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        
        # Environment map for realistic sensor simulation
        self.environment_map = self._create_environment_map()
        
        # Sensor characteristics logging
        self.sensor_log = []
        
        self.logger.info("Realistic SLAM Simulation initialized successfully")
        self.logger.info(f"LiDAR model characteristics: {self.lidar_model.get_sensor_characteristics()}")
    
    def _create_environment_map(self) -> np.ndarray:
        """Create a 2D occupancy grid map from the MuJoCo environment."""
        # Create a 200x200 grid covering the simulation area
        map_size = 200
        map_resolution = 0.05  # 5cm per pixel
        map_origin = (-5.0, -5.0)
        
        # Initialize empty map
        env_map = np.zeros((map_size, map_size))
        
        # Add walls from the maze (this would be extracted from main_scene.xml)
        # For now, create a simple test environment
        wall_positions = [
            # Outer walls
            (slice(0, 200), slice(0, 5)),      # Bottom wall
            (slice(0, 200), slice(195, 200)),  # Top wall
            (slice(0, 5), slice(0, 200)),      # Left wall
            (slice(195, 200), slice(0, 200)),  # Right wall
            
            # Inner maze walls (simplified)
            (slice(80, 120), slice(80, 85)),   # Horizontal wall
            (slice(120, 125), slice(60, 120)), # Vertical wall
        ]
        
        for wall_slice in wall_positions:
            env_map[wall_slice] = 1.0
        
        return env_map
    
    def setup_viewer(self):
        """Setup MuJoCo viewer with custom callbacks."""
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )
        
        # Set camera position for better view
        self.viewer.cam.distance = 8.0
        self.viewer.cam.azimuth = 45.0
        self.viewer.cam.elevation = -20.0
    
    def update_sensors_realistic(self):
        """Update sensor data using realistic sensor models."""
        # Get true robot pose from MuJoCo
        true_pos = self.data.sensordata[9:12]  # Position from IMU
        true_quat = self.data.sensordata[12:16]  # Orientation from IMU
        
        # Update robot pose estimate
        self.robot_pose = np.array([true_pos[0], true_pos[1], 0.0])  # 2D pose
        
        # Get true angular and linear velocities
        true_angular_vel = self.data.sensordata[15:18]  # Angular velocity
        true_linear_accel = self.data.sensordata[18:21]  # Linear acceleration
        
        # Simulate realistic IMU readings
        imu_angular_vel, imu_linear_accel = self.imu_model.simulate_reading(
            true_angular_vel, true_linear_accel
        )
        
        # Update IMU data
        self.imu_data = np.concatenate([
            true_quat,  # Orientation (quaternion)
            imu_angular_vel,  # Angular velocity
            imu_linear_accel  # Linear acceleration
        ])
        
        # Simulate realistic LiDAR readings
        self.lidar_data = self.lidar_model.simulate_environment_scan(
            self.robot_pose,
            self.environment_map,
            map_resolution=0.05,
            map_origin=(-5.0, -5.0)
        )
        
        # Compute true odometry from wheel positions
        wheel_positions = self.data.qpos[2:6]
        wheel_velocities = self.data.qvel[2:6]
        
        # Simple odometry computation
        wheel_radius = 0.05
        wheelbase = 0.2
        
        left_wheel_avg = (wheel_positions[0] + wheel_positions[1]) / 2
        right_wheel_avg = (wheel_positions[2] + wheel_positions[3]) / 2
        
        v_left = wheel_velocities[0] * wheel_radius
        v_right = wheel_velocities[2] * wheel_radius
        
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / wheelbase
        
        # True pose and velocity
        true_pose = np.array([self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]])
        true_velocity = np.array([
            v_linear * np.cos(self.robot_pose[2]),
            v_linear * np.sin(self.robot_pose[2]),
            v_angular
        ])
        
        # Apply realistic odometry model
        dt = self.model.opt.timestep
        realistic_pose, realistic_velocity = self.odometry_model.simulate_reading(
            true_pose, true_velocity, dt, temperature=25.0
        )
        
        # Update odometry data with realistic errors
        self.odom_data = np.concatenate([realistic_pose, realistic_velocity])
        
        # Log sensor characteristics for analysis
        self._log_sensor_characteristics()
    
    def _log_sensor_characteristics(self):
        """Log sensor characteristics for analysis."""
        if len(self.sensor_log) % 100 == 0:  # Log every 100 steps
            characteristics = {
                'timestamp': time.time(),
                'robot_pose': self.robot_pose.tolist(),
                'lidar_ranges': self.lidar_data.tolist(),
                'lidar_std': float(np.std(self.lidar_data)),
                'imu_angular_vel_std': float(np.std(self.imu_data[4:7])),
                'imu_accel_std': float(np.std(self.imu_data[7:10])),
                'odometry_velocity': float(np.linalg.norm(self.odom_data[3:5])),
                'odometry_pose_std': float(np.std(self.odom_data[0:3])),
                'odometry_velocity_std': float(np.std(self.odom_data[3:6]))
            }
            self.sensor_log.append(characteristics)
    
    def apply_control(self, command: np.ndarray):
        """Apply control command to robot actuators."""
        # Set wheel torques
        self.data.ctrl[0:4] = command[0:4]  # 4 wheel motors
        
        # Set camera pan/tilt (if available)
        if len(command) > 4 and self.model.nu > 4:
            self.data.ctrl[4:6] = command[4:6]
    
    def handle_keyboard_input(self):
        """Handle keyboard input for manual control."""
        # Manual control keys
        if keyboard.is_pressed('w'):
            self.control_command = np.array([0.1, 0.1, 0.0, 0.0])  # Forward
        elif keyboard.is_pressed('s'):
            self.control_command = np.array([-0.1, -0.1, 0.0, 0.0])  # Backward
        elif keyboard.is_pressed('a'):
            self.control_command = np.array([-0.05, 0.05, 0.0, 0.0])  # Turn left
        elif keyboard.is_pressed('d'):
            self.control_command = np.array([0.05, -0.05, 0.0, 0.0])  # Turn right
        elif keyboard.is_pressed('space'):
            self.control_command = np.zeros(4)  # Stop
        elif keyboard.is_pressed('n'):
            self.toggle_navigation_mode()
        elif keyboard.is_pressed('g'):
            self.set_random_goal()
        elif keyboard.is_pressed('m'):
            self.toggle_map_view()
        elif keyboard.is_pressed('q'):
            self.running = False
        elif keyboard.is_pressed('p'):
            self.paused = not self.paused
        elif keyboard.is_pressed('l'):
            self.save_sensor_log()
    
    def toggle_navigation_mode(self):
        """Toggle between manual and autonomous navigation."""
        self.navigation_mode = not self.navigation_mode
        if self.navigation_mode:
            self.logger.info("Switched to realistic autonomous navigation mode")
            self.set_random_goal()
        else:
            self.logger.info("Switched to manual control mode")
            self.control_command = np.zeros(4)
    
    def set_random_goal(self):
        """Set a random navigation goal."""
        # Generate random goal within maze bounds
        goal_x = np.random.uniform(-3.0, 3.0)
        goal_y = np.random.uniform(-3.0, 3.0)
        
        self.slam.set_navigation_goal(goal_x, goal_y)
        self.logger.info(f"Set navigation goal: ({goal_x:.2f}, {goal_y:.2f})")
    
    def toggle_map_view(self):
        """Toggle map visualization."""
        self.logger.info("Map view toggled (not implemented in this version)")
    
    def update_slam(self, dt: float):
        """Update SLAM system with realistic sensor data."""
        self.slam.update_sensors(
            lidar_data=self.lidar_data,
            imu_data=self.imu_data,
            odom_data=self.odom_data,
            dt=dt
        )
        
        # Get navigation command if in autonomous mode
        if self.navigation_mode:
            nav_command = self.slam.get_navigation_command()
            if nav_command is not None:
                # Convert navigation command to wheel torques
                v_linear, v_angular = nav_command[0], nav_command[2]
                
                # Differential drive control
                wheel_radius = 0.05
                wheelbase = 0.2
                
                v_left = v_linear - (v_angular * wheelbase) / 2
                v_right = v_linear + (v_angular * wheelbase) / 2
                
                # Convert to torques (simplified)
                torque_left = v_left * 0.5
                torque_right = v_right * 0.5
                
                self.control_command = np.array([torque_left, torque_left, torque_right, torque_right])
    
    def save_sensor_log(self):
        """Save sensor characteristics log to file."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_log_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.sensor_log, f, indent=2, default=str)
        
        self.logger.info(f"Sensor log saved to {filename}")
    
    def run(self):
        """Main simulation loop."""
        self.setup_viewer()
        
        self.logger.info("Starting Realistic SLAM simulation...")
        self.logger.info("Controls:")
        self.logger.info("  WASD - Manual control")
        self.logger.info("  N - Toggle autonomous navigation")
        self.logger.info("  G - Set random goal")
        self.logger.info("  M - Toggle map view")
        self.logger.info("  P - Pause/Resume")
        self.logger.info("  L - Save sensor log")
        self.logger.info("  Q - Quit")
        
        step_count = 0
        
        while self.running and self.viewer.is_running():
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            if not self.paused:
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Update sensors with realistic models
                self.update_sensors_realistic()
                
                # Update SLAM
                self.update_slam(dt)
                
                # Handle input
                self.handle_keyboard_input()
                
                # Apply control
                self.apply_control(self.control_command)
                
                # Update viewer
                self.viewer.sync()
                
                step_count += 1
                
                # Print sensor statistics every 1000 steps
                if step_count % 1000 == 0:
                    self._print_sensor_statistics()
            
            # Handle viewer events
            glfw.poll_events()
        
        # Save final sensor log
        self.save_sensor_log()
        
        self.viewer.close()
        self.logger.info("Realistic SLAM simulation ended")
    
    def _print_sensor_statistics(self):
        """Print sensor statistics for monitoring."""
        lidar_std = np.std(self.lidar_data)
        imu_angular_std = np.std(self.imu_data[4:7])
        imu_accel_std = np.std(self.imu_data[7:10])
        
        print(f"LiDAR std: {lidar_std:.3f}m, IMU angular std: {imu_angular_std:.3f}rad/s, "
              f"IMU accel std: {imu_accel_std:.3f}m/s²")


def main():
    """Main function to run realistic SLAM simulation."""
    parser = argparse.ArgumentParser(description="Realistic SLAM-based Autonomous Navigation Simulation")
    parser.add_argument("--model", default="main_scene.xml", help="MuJoCo model file")
    parser.add_argument("--config", help="Configuration file (optional)")
    parser.add_argument("--sensor-model", help="Sensor model file from characterization")
    
    args = parser.parse_args()
    
    try:
        # Load configuration
        config = SimulationConfig()
        if args.config:
            # Load from file if provided
            pass
        
        # Create and run simulation
        simulation = RealisticSLAMSimulation(
            args.model, 
            config, 
            args.sensor_model
        )
        simulation.run()
        
    except Exception as e:
        print(f"Error running realistic simulation: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
