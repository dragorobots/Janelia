"""
Improved main simulation using structured architecture.

This version demonstrates the new class-based design with proper error handling,
logging, and configuration management.
"""

import time
import keyboard
import numpy as np
import glfw
import mujoco
import mujoco.viewer
import itertools

from config import DEFAULT_CONFIG, SimulationConfig
from robot import AutonomousRobot
from validation import SimulationValidator, SimulationLogger, validate_simulation_setup


class ImprovedSimulation:
    """
    Improved simulation controller with structured architecture.
    
    This class demonstrates proper error handling, logging, and configuration
    management while maintaining the same functionality as the original.
    """
    
    def __init__(self, model_path: str = 'main_scene.xml', config: SimulationConfig = None):
        """
        Initialize improved simulation.
        
        Args:
            model_path: Path to MuJoCo model file
            config: Simulation configuration (uses default if None)
        """
        self.model_path = model_path
        self.config = config or DEFAULT_CONFIG
        self.logger = SimulationLogger("simulation")
        
        # Validate setup
        self._validate_setup()
        
        # Load model
        self._load_model()
        
        # Initialize robot controller
        self.robot = AutonomousRobot(self.model, self.data, self.config)
        
        # Initialize state
        self.autonomous_mode = False
        self.last_key_press_time = 0.0
        self.friction_enabled = True
        
        # Camera control
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self._setup_camera_limits()
        
        # Beacon mapping
        self._setup_beacons()
        
        self.logger.logger.info("Simulation initialized successfully")
    
    def _validate_setup(self):
        """Validate simulation setup."""
        is_valid, errors = validate_simulation_setup(self.model_path, self.config)
        if not is_valid:
            error_msg = "Simulation setup validation failed:\n" + "\n".join(errors)
            self.logger.log_error(error_msg)
            raise RuntimeError(error_msg)
    
    def _load_model(self):
        """Load MuJoCo model with error handling."""
        try:
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)
            self.logger.logger.info(f"Model loaded successfully: {self.model_path}")
        except Exception as e:
            self.logger.log_error(f"Failed to load model: {e}")
            raise
    
    def _setup_camera_limits(self):
        """Setup camera joint limits."""
        try:
            yaw_range = self.model.jnt_range[self.model.joint('camera_yaw_joint').id]
            pitch_range = self.model.jnt_range[self.model.joint('camera_pitch_joint').id]
            self.yaw_range = yaw_range
            self.pitch_range = pitch_range
        except Exception as e:
            self.logger.logger.warning(f"Could not setup camera limits: {e}")
            self.yaw_range = [-0.785, 0.785]
            self.pitch_range = [-0.785, 0.785]
    
    def _setup_beacons(self):
        """Setup navigation beacons."""
        try:
            self.beacons = {
                'a': self.model.site('pylon_A').id,
                'b': self.model.site('pylon_B').id,
                'c': self.model.site('pylon_C').id,
                'd': self.model.site('pylon_D').id,
            }
        except Exception as e:
            self.logger.logger.warning(f"Could not setup beacons: {e}")
            self.beacons = {}
    
    def handle_keyboard_input(self):
        """Handle keyboard input with debouncing."""
        current_time = time.time()
        
        # Beacon selection
        for key in self.beacons.keys():
            if keyboard.is_pressed(key) and current_time - self.last_key_press_time > self.config.debug.beacon_debounce:
                site_id = self.beacons[key]
                beacon_pos = tuple(self.data.site_xpos[site_id][:2])
                self.robot.set_target(beacon_pos)
                self.autonomous_mode = True
                self.logger.log_navigation_event(f"Target set to {key.upper()}", {"target": beacon_pos})
                self.last_key_press_time = current_time
        
        # Toggle autonomous mode
        if keyboard.is_pressed('space') and current_time - self.last_key_press_time > self.config.debug.beacon_debounce:
            self.autonomous_mode = not self.autonomous_mode
            mode_str = "ON" if self.autonomous_mode else "OFF"
            self.logger.logger.info(f"Autonomous mode {mode_str}")
            self.last_key_press_time = current_time
        
        # Toggle friction
        if keyboard.is_pressed('f') and current_time - self.last_key_press_time > self.config.debug.beacon_debounce:
            self.friction_enabled = not self.friction_enabled
            friction_type = "NOMINAL" if self.friction_enabled else "ICE"
            self.robot.apply_friction(
                self.config.physics.nominal_friction if self.friction_enabled 
                else self.config.physics.ice_friction
            )
            self.logger.log_config_change("friction", not self.friction_enabled, self.friction_enabled)
            self.last_key_press_time = current_time
    
    def get_manual_control(self):
        """Get manual control commands."""
        if keyboard.is_pressed('up'):
            return (self.config.control.drive_speed, self.config.control.drive_speed)
        elif keyboard.is_pressed('down'):
            return (-self.config.control.drive_speed, -self.config.control.drive_speed)
        elif keyboard.is_pressed('left'):
            return (-self.config.control.drive_speed * self.config.control.turn_ratio,
                   self.config.control.drive_speed * self.config.control.turn_ratio)
        elif keyboard.is_pressed('right'):
            return (self.config.control.drive_speed * self.config.control.turn_ratio,
                   -self.config.control.drive_speed * self.config.control.turn_ratio)
        else:
            return (0.0, 0.0)
    
    def handle_camera_control(self):
        """Handle camera pan/tilt control."""
        if keyboard.is_pressed('j'):
            self.target_yaw += self.config.control.camera_step
        if keyboard.is_pressed('l'):
            self.target_yaw -= self.config.control.camera_step
        if keyboard.is_pressed('i'):
            self.target_pitch += self.config.control.camera_step
        if keyboard.is_pressed('k'):
            self.target_pitch -= self.config.control.camera_step
        
        # Clamp to limits
        self.target_yaw = np.clip(self.target_yaw, self.yaw_range[0], self.yaw_range[1])
        self.target_pitch = np.clip(self.target_pitch, self.pitch_range[0], self.pitch_range[1])
    
    def apply_control(self, drive_cmd, camera_cmd):
        """Apply control commands to actuators."""
        # Apply drive commands
        self.robot.apply_control(drive_cmd)
        
        # Apply camera commands
        self.data.ctrl[4] = camera_cmd['yaw']
        self.data.ctrl[5] = camera_cmd['pitch']
    
    def run(self):
        """Run the simulation main loop."""
        self.logger.logger.info("Starting simulation...")
        print("Drive: ↑↓←→ | Camera: I,J,K,L | Space: auto | F: friction | B: beacon | Q: quit")
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                t0 = time.time()
                glfw.poll_events()
                
                # Handle input
                self.handle_keyboard_input()
                self.handle_camera_control()
                
                # Determine control commands
                if self.autonomous_mode:
                    drive_cmd = self.robot.update()
                else:
                    left, right = self.get_manual_control()
                    drive_cmd = type('ControlCommand', (), {'left_speed': left, 'right_speed': right})()
                
                camera_cmd = {'yaw': self.target_yaw, 'pitch': self.target_pitch}
                
                # Apply control
                self.apply_control(drive_cmd, camera_cmd)
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                
                # Pacing
                dt = self.model.opt.timestep - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
                
                # Quit
                if keyboard.is_pressed('q'):
                    break
        
        self.logger.logger.info("Simulation ended.")


def main():
    """Main entry point."""
    try:
        simulation = ImprovedSimulation()
        simulation.run()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    except Exception as e:
        print(f"Simulation failed: {e}")
        raise


if __name__ == "__main__":
    main()
