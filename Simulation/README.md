# Janelia Robotics Simulation Platform

A MuJoCo-based robotics simulation for autonomous navigation research, featuring a 4-wheeled robot with LiDAR, IMU, and camera sensors navigating through procedurally generated mazes.

## Features

- **Autonomous Navigation**: Axis-aligned pathfinding with obstacle avoidance
- **Multi-Sensor Robot**: 8-beam LiDAR, IMU, pan-tilt camera
- **Interactive Environment**: Configurable friction, multiple navigation targets
- **Development Tools**: Visual maze builder, coordinate generation utilities
- **Structured Architecture**: Class-based design with comprehensive testing

## Quick Start

### Prerequisites

```bash
# Install dependencies
pip install mujoco keyboard numpy glfw pillow pytest
```

### Running Simulations

```bash
# Run main simulation with autonomous navigation
python simulate.py

# Run simplified version for testing
python run_simple.py

# Build custom maze using GUI
python gui_test.py
```

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_robot.py

# Run with verbose output
python -m pytest -v tests/
```

## Architecture

### Core Components

- **`main_scene.xml`**: Primary simulation environment with robot and maze
- **`simulate.py`**: Main simulation with autonomous navigation
- **`robot.py`**: Class-based robot controller with obstacle avoidance
- **`config.py`**: Centralized configuration management
- **`validation.py`**: Model and configuration validation utilities

### Robot Model

The robot features:
- **4 Independent Wheels**: Differential drive with torque limits
- **8-Beam LiDAR**: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° coverage
- **IMU**: Position, orientation, linear and angular velocity
- **Pan-Tilt Camera**: Controllable viewing angle
- **Realistic Physics**: Mass properties, friction modeling, collision detection

### Navigation System

- **Axis-Aligned Navigation**: Moves along X-axis first, then Y-axis
- **Obstacle Avoidance**: LiDAR-based detection with reverse and turn behaviors
- **State Machine**: Normal → Obstacle → Reverse → Turn → Resume
- **Safety Thresholds**: Configurable distances for front and side detection

## Configuration

All simulation parameters are centralized in `config.py`:

```python
from config import SimulationConfig

# Create custom configuration
config = SimulationConfig()
config.navigation.safe_front_dist = 0.3  # Increase safety distance
config.control.drive_speed = 0.7         # Increase speed
```

### Key Parameters

#### Physics
- `timestep`: Simulation timestep (default: 0.001s)
- `wheel_max_torque`: Maximum wheel torque (default: 0.15 N⋅m)
- `wheel_joint_damping`: Wheel damping (default: 0.009)

#### Navigation
- `safe_front_dist`: Front obstacle threshold (default: 0.2m)
- `safe_side_dist`: Side obstacle threshold (default: 0.2m)
- `arrival_tolerance`: Target arrival distance (default: 0.2m)
- `heading_tolerance`: Heading alignment tolerance (default: 0.05 rad)

#### Control
- `drive_speed`: Nominal forward speed (default: 0.5)
- `turn_ratio`: Turning speed multiplier (default: 1.0)
- `camera_step`: Camera pan/tilt step size (default: 0.05 rad)

## API Documentation

### AutonomousRobot Class

#### Constructor
```python
AutonomousRobot(model: mujoco.MjModel, data: mujoco.MjData, config: SimulationConfig)
```

#### Methods

##### `set_target(target: Tuple[float, float])`
Sets the navigation target in world coordinates (x, y).

**Parameters:**
- `target`: Tuple of (x, y) coordinates in meters

**Example:**
```python
robot.set_target((3.0, 2.0))  # Navigate to beacon A
```

##### `update() -> ControlCommand`
Performs one simulation step and returns control commands.

**Returns:**
- `ControlCommand`: Named tuple with `left_speed` and `right_speed` attributes

**Example:**
```python
cmd = robot.update()
data.ctrl[0] = cmd.left_speed
data.ctrl[1] = cmd.right_speed
```

##### `has_arrived() -> bool`
Checks if robot has reached the current target.

**Returns:**
- `bool`: True if within arrival tolerance of target

##### `apply_friction(friction_triplet: np.ndarray)`
Applies friction coefficients to wheels and floor.

**Parameters:**
- `friction_triplet`: Array of [sliding, rolling, spinning] friction coefficients

## Development Tools

### Maze Generation

#### `generate_maze.py`
Converts coordinate pairs to MuJoCo wall geometries:

```bash
python generate_maze.py
```

Outputs XML-compatible wall segments for inclusion in simulation files.

#### `gui_test.py`
Visual maze builder with real-world coordinate system:

```bash
python gui_test.py
```

Features:
- Click-to-draw wall segments
- Meter-based coordinate system
- Image overlay support
- Coordinate export functionality

## Testing

The project includes comprehensive unit tests:

```bash
# Run all tests
python -m pytest tests/

# Run specific test categories
python -m pytest tests/test_robot.py::TestObstacleDetector
python -m pytest tests/test_robot.py::TestNavigationController
python -m pytest tests/test_robot.py::TestAutonomousRobot
```

### Test Coverage

- **Obstacle Detection**: LiDAR data processing and threshold checking
- **Navigation**: Quaternion conversion, axis-aligned pathfinding
- **Robot Control**: State machine, sensor fusion, motor commands
- **Configuration**: Parameter validation and bounds checking

## Troubleshooting

### Common Issues

#### Robot Gets Stuck
**Symptoms:** Robot stops moving or spins in place

**Causes:** 
- Obstacle detection too sensitive
- Navigation algorithm failure
- Sensor data corruption

**Solutions:**
1. Check `SAFE_FRONT_DIST` and `SAFE_SIDE_DIST` values
2. Verify LiDAR sensor readings
3. Enable debug output to see state machine

#### Simulation Crashes
**Symptoms:** Program exits with error

**Causes:**
- Invalid XML model
- Memory issues
- Invalid sensor data

**Solutions:**
1. Validate model with `SimulationValidator.validate_model()`
2. Check system memory usage
3. Enable detailed logging

#### Poor Navigation Performance
**Symptoms:** Robot takes inefficient paths or misses targets

**Causes:**
- Incorrect control gains
- Poor obstacle detection
- Navigation algorithm issues

**Solutions:**
1. Tune `HEADING_KP` and `KP_SPEED` parameters
2. Adjust safety distance thresholds
3. Review navigation state machine logic

## Contributing

### Development Workflow

1. **Fork the repository**
2. **Create feature branch**: `git checkout -b feature/new-navigation-algorithm`
3. **Write tests first** (TDD approach)
4. **Implement feature** with proper error handling
5. **Update documentation**
6. **Run full test suite**: `python -m pytest tests/`
7. **Submit pull request**

### Code Review Checklist

- [ ] All tests pass
- [ ] Documentation updated
- [ ] Error handling implemented
- [ ] Logging added for debugging
- [ ] Configuration parameters documented
- [ ] Performance impact assessed

## License

This project is developed at Janelia Research Campus for research purposes.

## Acknowledgments

- MuJoCo physics engine for realistic simulation
- Janelia Research Campus for research support
- Open source robotics community for inspiration and tools 
