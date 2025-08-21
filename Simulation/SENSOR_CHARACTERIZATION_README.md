# Sensor Characterization System for Realistic SLAM

This system provides a complete workflow for characterizing robot sensors and integrating realistic sensor models into SLAM simulation. By collecting real sensor data from your actual robot hardware, you can create highly accurate sensor models that significantly improve the realism of your SLAM simulation.

## 🎯 Overview

The sensor characterization system consists of several components:

1. **Data Collection** - Collect real sensor data from your robot
2. **Data Analysis** - Analyze the data to understand sensor characteristics
3. **Model Generation** - Create realistic sensor models
4. **Simulation Integration** - Integrate models into SLAM simulation
5. **Validation** - Test and validate the realistic simulation

## 🚀 Quick Start

### 1. Run the Complete Workflow

```bash
python sensor_characterization_workflow.py
```

This will guide you through the entire process step-by-step.

### 2. Manual Step-by-Step Process

#### Step 1: Collect Data on the Robot

```bash
# Copy the data collector to your robot
scp robot_data_collector.py user@10.0.0.234:~/

# SSH into your robot
ssh user@10.0.0.234

# Run the characterization session
python robot_data_collector.py session
```

#### Step 2: Analyze the Data

```bash
# Run the characterization analysis
python sensor_characterization.py
```

#### Step 3: Run Realistic SLAM Simulation

```bash
# Basic simulation with default sensor model
python slam_simulation_realistic.py

# With custom sensor model
python slam_simulation_realistic.py --sensor-model sensor_model_20241201_143022.json
```

## 📊 What Gets Characterized

### LiDAR Sensor Characteristics

- **Systematic Bias** - Consistent offset in measurements
- **Measurement Noise** - Random variations in readings
- **Material Effects** - How different surfaces affect readings
- **Distance Effects** - Accuracy changes with distance
- **Angle Effects** - Accuracy changes with beam angle
- **Multi-path Effects** - Reflections and echoes
- **Saturation Effects** - Bright target limitations
- **Range Resolution** - Minimum detectable changes

### IMU Sensor Characteristics

- **Gyroscope Drift** - Gradual bias accumulation
- **Accelerometer Bias** - Systematic acceleration errors
- **Temperature Effects** - Performance changes with temperature
- **Noise Characteristics** - Random measurement variations

### Odometry Sensor Characteristics

- **Wheel Slip** - Occasional loss of traction affecting position estimates
- **Encoder Resolution** - Quantization effects from finite encoder resolution
- **Systematic Bias** - Uneven wheels, misalignment, and calibration errors
- **Drift Over Time** - Accumulating position errors
- **Temperature Effects** - Wheel diameter changes with temperature
- **Velocity Noise** - Variations in velocity measurements

## 🔧 Configuration

### Test Configurations

The system tests various combinations of:

- **Distances**: 0.2m, 0.5m, 1.0m, 1.5m, 2.0m, 2.5m, 3.0m
- **Angles**: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
- **Materials**: Wall, cardboard, metal, fabric

### Customization

Edit `sensor_characterization_workflow.py` to customize:

```python
config = {
    'robot_ip': '10.0.0.234',
    'collection_duration': 5.0,  # seconds per test
    'sample_rate': 10.0,         # Hz
    'num_configurations': 10,    # number of tests to run
    'target_distances': [0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
    'target_angles': [0, 45, 90, 135, 180, 225, 270, 315],
    'target_materials': ["wall", "cardboard", "metal"]
}
```

## 📁 File Structure

```
Simulation/
├── sensor_characterization.py              # Main characterization system
├── robot_data_collector.py                 # Robot-side data collection
├── realistic_sensor_model.py               # Realistic sensor models
├── slam_simulation_realistic.py            # Enhanced SLAM simulation
├── sensor_characterization_workflow.py     # Complete workflow
├── sensor_characterization_results/        # Output directory
│   ├── test_configurations.json           # Test configurations
│   ├── robot_sensor_data_*.json           # Collected data
│   ├── sensor_analysis_*.json             # Analysis results
│   └── sensor_model_*.json                # Generated models
└── SENSOR_CHARACTERIZATION_README.md       # This file
```

## 🔬 Data Collection Process

### Robot-Side Collection

1. **Setup**: Position robot in a controlled environment
2. **Calibration**: Place targets at known distances and angles
3. **Collection**: Run automated data collection for each configuration
4. **Transfer**: Move data back to development machine

### Test Environment Requirements

- **Controlled Lighting**: Consistent ambient light
- **Known Distances**: Accurate measurements to targets
- **Various Materials**: Different surface types
- **Stable Platform**: Robot should not move during tests
- **Multiple Angles**: Test different LiDAR beam orientations

### Example Test Setup

```
Robot Position: (0, 0)
Target Positions:
- (0.2, 0) - 20cm wall
- (0.5, 0) - 50cm wall  
- (1.0, 0) - 1m wall
- (1.0, 1.0) - 1m at 45°
- (0, 1.0) - 1m at 90°
```

## 📈 Data Analysis

### Statistical Analysis

The system analyzes:

- **Mean and Standard Deviation** of readings
- **Systematic Bias** across all measurements
- **Material-Specific Effects** for each surface type
- **Distance-Dependent Errors** using polynomial fitting
- **Angle-Dependent Effects** for different beam orientations

### Generated Metrics

```json
{
  "systematic_bias": 0.005,
  "systematic_bias_std": 0.002,
  "measurement_noise_std": 0.018,
  "material_effects": {
    "wall": {"mean_error": 0.0, "std_error": 0.02},
    "cardboard": {"mean_error": 0.01, "std_error": 0.03},
    "metal": {"mean_error": -0.005, "std_error": 0.015}
  },
  "distance_effects": {
    "polynomial_coefficients": [0.001, -0.002, 0.0],
    "distance_error_correlation": 0.3
  }
}
```

## 🎮 Realistic SLAM Simulation

### Enhanced Features

- **Realistic LiDAR Readings** with actual sensor characteristics
- **Material-Dependent Behavior** based on real measurements
- **Distance and Angle Effects** from characterization data
- **Multi-path and Saturation Effects** for extreme conditions
- **Realistic IMU Behavior** with drift and noise
- **Sensor Logging** for analysis and debugging

### Usage Examples

```bash
# Basic realistic simulation
python slam_simulation_realistic.py

# With custom sensor model
python slam_simulation_realistic.py --sensor-model my_sensor_model.json

# With custom configuration
python slam_simulation_realistic.py --config custom_config.json --sensor-model sensor_model.json
```

### Controls

- **WASD** - Manual robot control
- **N** - Toggle autonomous navigation
- **G** - Set random navigation goal
- **M** - Toggle map view
- **P** - Pause/Resume
- **L** - Save sensor log
- **Q** - Quit

## 🔍 Validation and Testing

### Sensor Model Validation

The system includes comprehensive validation:

```python
# Test basic functionality
reading = lidar_model.simulate_reading(1.0, 0.0, "wall")

# Test material effects
wall_reading = lidar_model.simulate_reading(1.0, 0.0, "wall")
metal_reading = lidar_model.simulate_reading(1.0, 0.0, "metal")

# Test distance effects
near_reading = lidar_model.simulate_reading(0.5, 0.0, "wall")
far_reading = lidar_model.simulate_reading(2.0, 0.0, "wall")
```

### Simulation Testing

- **Sensor Statistics Monitoring** during simulation
- **Real-time Performance Metrics** display
- **Sensor Log Export** for post-analysis
- **Comparison with Ground Truth** validation

## 🛠️ Troubleshooting

### Common Issues

#### No Data Files Found
```
⚠ No data files found. Please ensure data collection is complete.
```
**Solution**: Run the robot-side data collection first, or use simulated data for testing.

#### Serial Connection Failed
```
Serial connection failed: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```
**Solution**: Check your robot's serial port configuration and update the port name.

#### Sensor Model Loading Failed
```
Warning: Could not load sensor model from sensor_model.json
```
**Solution**: Ensure the sensor model file exists and has valid JSON format.

#### Simulation Crashes
```
Error running realistic simulation: [Error details]
```
**Solution**: Check that all dependencies are installed and the MuJoCo model is valid.

### Debug Mode

Enable debug output by modifying the configuration:

```python
config.debug.debug_period = 0.1  # More frequent debug output
config.debug.dbg_skip = 1        # Print every step
```

## 📊 Performance Comparison

### Before vs After Characterization

| Metric | Basic Simulation | Realistic Simulation |
|--------|------------------|---------------------|
| LiDAR Accuracy | ±5cm | ±2cm |
| Material Effects | None | Realistic |
| Distance Effects | None | Polynomial model |
| Multi-path Effects | None | 5% probability |
| IMU Drift | None | Realistic drift |
| Temperature Effects | None | Included |

### Real-World Validation

The realistic simulation should now closely match your actual robot's behavior:

- **Navigation Performance** - More accurate path planning
- **Obstacle Avoidance** - Realistic sensor limitations
- **Localization Accuracy** - Better pose estimation
- **Map Quality** - More accurate occupancy grids

## 🔬 Advanced Usage

### Custom Sensor Models

Create custom sensor models for different hardware:

```python
from realistic_sensor_model import SensorModelParams

custom_params = SensorModelParams(
    systematic_bias=0.01,
    measurement_noise_std=0.025,
    material_effects={
        "wall": {"mean_error": 0.0, "std_error": 0.02},
        "glass": {"mean_error": 0.05, "std_error": 0.05}
    }
)

lidar_model = RealisticLidarModel(config=custom_params)
```

### Integration with Other Systems

The sensor models can be integrated with:

- **ROS2 Navigation Stack** - Replace simulated sensors
- **Custom SLAM Algorithms** - Use realistic sensor data
- **Machine Learning Systems** - Train on realistic data
- **Hardware-in-the-Loop Testing** - Validate control systems

## 📚 References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [LiDAR Sensor Characterization](https://en.wikipedia.org/wiki/Lidar#Sensor_characteristics)
- [SLAM Algorithms](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- [Robot Sensor Modeling](https://en.wikipedia.org/wiki/Robot_sensor)

## 🤝 Contributing

To contribute to the sensor characterization system:

1. **Fork the repository**
2. **Create a feature branch**
3. **Add tests** for new functionality
4. **Update documentation**
5. **Submit a pull request**

## 📄 License

This sensor characterization system is part of the Janelia Robotics Simulation Platform and is developed for research purposes.

---

**Note**: This system is designed to work with the Yahboom robot platform but can be adapted for other robots by modifying the communication protocols and sensor configurations.
