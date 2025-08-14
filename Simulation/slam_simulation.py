"""
SLAM-based Autonomous Navigation Simulation.

This simulation integrates the SLAM system with MuJoCo to provide
realistic autonomous navigation similar to the real Yahboom robot.
"""

import time
import keyboard
import numpy as np
import glfw
import mujoco
import mujoco.viewer
import argparse
from typing import Optional, Tuple

from config import SimulationConfig
from slam_system import SLAMSystem
from validation import SimulationValidator, SimulationLogger


class SLAMSimulation:
    """
    SLAM-based autonomous navigation simulation.
    
    This class integrates the SLAM system with MuJoCo simulation to provide
    realistic autonomous navigation capabilities similar to ROS2 Nav2.
    """
    
    def __init__(self, model_path: str = 'main_scene.xml', config: Optional[SimulationConfig] = None):
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
        self.imu_data = np.zeros(10)  # [quat_x, quat_y, quat_z, quat_w, ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z]
        self.odom_data = np.zeros(6)  # [x, y, theta, vx, vy, vtheta]
        
        # Control commands
        self.control_command = np.zeros(4)  # [left_wheel, right_wheel, camera_pan, camera_tilt]
        
        # Initialize robot pose
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        
        self.logger.info("SLAM Simulation initialized successfully")
    
    def setup_viewer(self):
        """Setup MuJoCo viewer with custom callbacks."""
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )
        
        # Set camera position for better view
        self.viewer.cam.distance = 8.0
        self.viewer.cam.azimuth = 45.0
        self.viewer.cam.elevation = -20.0
    
    def update_sensors(self):
        """Update sensor data from MuJoCo simulation."""
        # Extract LiDAR data (first 8 sensor values)
        self.lidar_data = self.data.sensordata[:8].copy()
        
        # Extract IMU data (next 10 sensor values)
        self.imu_data = self.data.sensordata[8:18].copy()
        
        # Compute odometry from wheel positions
        wheel_positions = self.data.qpos[2:6]  # 4 wheel positions
        wheel_velocities = self.data.qvel[2:6]  # 4 wheel velocities
        
        # Simple odometry computation
        # Assuming differential drive with wheel radius and wheelbase
        wheel_radius = 0.05  # 5cm wheel radius
        wheelbase = 0.2      # 20cm wheelbase
        
        # Convert wheel positions to robot pose
        left_wheel_avg = (wheel_positions[0] + wheel_positions[1]) / 2
        right_wheel_avg = (wheel_positions[2] + wheel_positions[3]) / 2
        
        # Compute linear and angular velocity
        v_left = wheel_velocities[0] * wheel_radius
        v_right = wheel_velocities[2] * wheel_radius
        
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / wheelbase
        
        # Integrate to get position (simplified)
        dt = self.model.opt.timestep
        self.robot_pose[0] += v_linear * np.cos(self.robot_pose[2]) * dt
        self.robot_pose[1] += v_linear * np.sin(self.robot_pose[2]) * dt
        self.robot_pose[2] += v_angular * dt
        
        # Update odometry data
        self.odom_data = np.array([
            self.robot_pose[0], self.robot_pose[1], self.robot_pose[2],
            v_linear * np.cos(self.robot_pose[2]),
            v_linear * np.sin(self.robot_pose[2]),
            v_angular
        ])
    
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
    
    def toggle_navigation_mode(self):
        """Toggle between manual and autonomous navigation."""
        self.navigation_mode = not self.navigation_mode
        if self.navigation_mode:
            self.logger.info("Switched to autonomous navigation mode")
            # Set initial goal
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
        # This would be implemented with a separate visualization window
        self.logger.info("Map view toggled (not implemented in this version)")
    
    def update_slam(self, dt: float):
        """Update SLAM system with current sensor data."""
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
    
    def run(self):
        """Main simulation loop."""
        self.setup_viewer()
        
        self.logger.info("Starting SLAM simulation...")
        self.logger.info("Controls:")
        self.logger.info("  WASD - Manual control")
        self.logger.info("  N - Toggle autonomous navigation")
        self.logger.info("  G - Set random goal")
        self.logger.info("  M - Toggle map view")
        self.logger.info("  P - Pause/Resume")
        self.logger.info("  Q - Quit")
        
        while self.running and self.viewer.is_running():
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            if not self.paused:
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Update sensors
                self.update_sensors()
                
                # Update SLAM
                self.update_slam(dt)
                
                # Handle input
                self.handle_keyboard_input()
                
                # Apply control
                self.apply_control(self.control_command)
                
                # Update viewer
                self.viewer.sync()
            
            # Handle viewer events
            glfw.poll_events()
        
        self.viewer.close()
        self.logger.info("Simulation ended")


def main():
    """Main function to run SLAM simulation."""
    parser = argparse.ArgumentParser(description="SLAM-based Autonomous Navigation Simulation")
    parser.add_argument("--model", default="main_scene.xml", help="MuJoCo model file")
    parser.add_argument("--config", help="Configuration file (optional)")
    
    args = parser.parse_args()
    
    try:
        # Load configuration
        config = SimulationConfig()
        if args.config:
            # Load from file if provided
            pass
        
        # Create and run simulation
        simulation = SLAMSimulation(args.model, config)
        simulation.run()
        
    except Exception as e:
        print(f"Error running simulation: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
