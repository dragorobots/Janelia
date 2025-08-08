"""
Demo script showing the improved architecture.

This script demonstrates the new structured development practices
including configuration management, validation, and logging.
"""

import numpy as np
from config import SimulationConfig, DEFAULT_CONFIG
from validation import SimulationValidator, SimulationLogger
from robot import AutonomousRobot, ControlCommand


def demo_configuration():
    """Demonstrate configuration management."""
    print("=== Configuration Management Demo ===")
    
    # Create custom configuration
    config = SimulationConfig()
    config.navigation.safe_front_dist = 0.3  # More conservative
    config.control.drive_speed = 0.7         # Faster movement
    
    print(f"Custom safe front distance: {config.navigation.safe_front_dist}m")
    print(f"Custom drive speed: {config.control.drive_speed}")
    print()


def demo_validation():
    """Demonstrate validation utilities."""
    print("=== Validation Demo ===")
    
    # Test configuration validation
    config = SimulationConfig()
    errors = SimulationValidator.validate_config(config)
    
    if errors:
        print("Configuration errors found:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("Configuration validation passed!")
    
    # Test invalid configuration
    bad_config = SimulationConfig()
    bad_config.navigation.safe_front_dist = -0.1  # Invalid negative value
    errors = SimulationValidator.validate_config(bad_config)
    
    if errors:
        print("Invalid configuration caught:")
        for error in errors:
            print(f"  - {error}")
    print()


def demo_logging():
    """Demonstrate structured logging."""
    print("=== Logging Demo ===")
    
    logger = SimulationLogger("demo")
    
    # Log different types of events
    logger.log_navigation_event("Target set", {"target": (1.0, 2.0)})
    logger.log_obstacle_event("Obstacle detected", {"distance": 0.15})
    logger.log_config_change("drive_speed", 0.5, 0.7)
    logger.log_error("Test error", {"context": "demo"})
    print()


def demo_robot_components():
    """Demonstrate robot component functionality."""
    print("=== Robot Components Demo ===")
    
    config = SimulationConfig()
    
    # Test obstacle detector
    from robot import ObstacleDetector
    detector = ObstacleDetector(config)
    
    # Test with clear readings
    clear_lidar = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    obstacle_detected = detector.detect_obstacle(clear_lidar)
    print(f"Clear readings - Obstacle detected: {obstacle_detected}")
    
    # Test with obstacle readings
    obstacle_lidar = np.array([0.1, 0.15, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1])
    obstacle_detected = detector.detect_obstacle(obstacle_lidar)
    print(f"Obstacle readings - Obstacle detected: {obstacle_detected}")
    
    # Test navigation controller
    from robot import NavigationController
    nav = NavigationController(config)
    
    # Test quaternion to yaw conversion
    quat = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    yaw = nav.quat_to_yaw(quat)
    print(f"Identity quaternion yaw: {yaw:.6f}")
    
    # Test axis heading
    desired_yaw, axis = nav.get_axis_heading(2.0, 1.0)
    print(f"Desired heading: {desired_yaw:.2f} rad, Axis: {axis}")
    
    # Test turn command
    cmd = nav.get_turn_command(0.5)  # Positive heading error
    print(f"Turn command: left={cmd.left_speed:.2f}, right={cmd.right_speed:.2f}")
    print()


def demo_control_command():
    """Demonstrate control command functionality."""
    print("=== Control Command Demo ===")
    
    # Create control commands
    forward_cmd = ControlCommand(0.5, 0.5)
    turn_cmd = ControlCommand(0.3, -0.3)
    stop_cmd = ControlCommand(0.0, 0.0)
    
    print(f"Forward command: {forward_cmd}")
    print(f"Turn command: {turn_cmd}")
    print(f"Stop command: {stop_cmd}")
    
    # Demonstrate immutability
    try:
        forward_cmd.left_speed = 1.0
        print("ERROR: Should not be able to modify control command")
    except AttributeError:
        print("✓ Control commands are immutable (as expected)")
    print()


def main():
    """Run all demos."""
    print("Janelia Robotics Simulation Platform - Architecture Demo")
    print("=" * 60)
    print()
    
    demo_configuration()
    demo_validation()
    demo_logging()
    demo_robot_components()
    demo_control_command()
    
    print("Demo completed successfully!")
    print("\nKey improvements demonstrated:")
    print("✓ Centralized configuration management")
    print("✓ Comprehensive validation")
    print("✓ Structured logging")
    print("✓ Class-based architecture")
    print("✓ Immutable data structures")
    print("✓ Unit testing framework")


if __name__ == "__main__":
    main()
