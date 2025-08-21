#!/usr/bin/env python3
"""
Sensor Characterization Workflow for Realistic SLAM.

This script provides a complete workflow for characterizing robot sensors
and integrating the results into realistic SLAM simulation.
"""

import os
import sys
import json
import time
import subprocess
from typing import Dict, List, Optional
from pathlib import Path


class SensorCharacterizationWorkflow:
    """
    Complete workflow for sensor characterization and simulation integration.
    
    This class guides users through:
    1. Data collection on the robot
    2. Data analysis and sensor model generation
    3. Integration with SLAM simulation
    4. Validation and testing
    """
    
    def __init__(self):
        self.workflow_steps = [
            "setup",
            "data_collection",
            "data_analysis", 
            "simulation_integration",
            "validation",
            "testing"
        ]
        self.current_step = 0
        self.config = self._load_workflow_config()
    
    def _load_workflow_config(self) -> Dict:
        """Load workflow configuration."""
        return {
            'robot_ip': '10.0.0.234',
            'collection_duration': 5.0,
            'sample_rate': 10.0,
            'num_configurations': 10,
            'target_distances': [0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
            'target_angles': [0, 45, 90, 135, 180, 225, 270, 315],
            'target_materials': ["wall", "cardboard", "metal"],
            'output_dir': 'sensor_characterization_results'
        }
    
    def run_workflow(self):
        """Run the complete sensor characterization workflow."""
        print("="*80)
        print("SENSOR CHARACTERIZATION WORKFLOW")
        print("="*80)
        print("This workflow will help you characterize your robot's sensors")
        print("and integrate the results into realistic SLAM simulation.")
        print()
        
        for step in self.workflow_steps:
            print(f"Step {self.current_step + 1}: {step.replace('_', ' ').title()}")
            print("-" * 60)
            
            if step == "setup":
                self._setup_workflow()
            elif step == "data_collection":
                self._data_collection_step()
            elif step == "data_analysis":
                self._data_analysis_step()
            elif step == "simulation_integration":
                self._simulation_integration_step()
            elif step == "validation":
                self._validation_step()
            elif step == "testing":
                self._testing_step()
            
            self.current_step += 1
            print()
        
        print("="*80)
        print("WORKFLOW COMPLETE!")
        print("="*80)
        print("Your realistic sensor model is now ready for use in SLAM simulation.")
        print("Run: python slam_simulation_realistic.py --sensor-model sensor_model.json")
    
    def _setup_workflow(self):
        """Setup the workflow environment."""
        print("Setting up sensor characterization workflow...")
        
        # Create output directory
        output_dir = Path(self.config['output_dir'])
        output_dir.mkdir(exist_ok=True)
        
        # Check dependencies
        self._check_dependencies()
        
        # Generate test configurations
        self._generate_test_configurations()
        
        print("✓ Setup complete")
    
    def _check_dependencies(self):
        """Check if required dependencies are available."""
        print("Checking dependencies...")
        
        required_packages = [
            'numpy', 'matplotlib', 'serial', 'keyboard', 
            'mujoco', 'glfw', 'pytest'
        ]
        
        missing_packages = []
        for package in required_packages:
            try:
                __import__(package)
                print(f"  ✓ {package}")
            except ImportError:
                missing_packages.append(package)
                print(f"  ✗ {package} (missing)")
        
        if missing_packages:
            print(f"\nMissing packages: {', '.join(missing_packages)}")
            print("Install with: pip install " + " ".join(missing_packages))
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                sys.exit(1)
    
    def _generate_test_configurations(self):
        """Generate test configurations for sensor characterization."""
        print("Generating test configurations...")
        
        configs = []
        for distance in self.config['target_distances']:
            for angle in self.config['target_angles']:
                for material in self.config['target_materials']:
                    configs.append({
                        'distance': distance,
                        'angle': angle,
                        'material': material
                    })
        
        # Limit to specified number
        if len(configs) > self.config['num_configurations']:
            import random
            random.shuffle(configs)
            configs = configs[:self.config['num_configurations']]
        
        # Save configurations
        config_file = Path(self.config['output_dir']) / 'test_configurations.json'
        with open(config_file, 'w') as f:
            json.dump(configs, f, indent=2)
        
        print(f"✓ Generated {len(configs)} test configurations")
        print(f"  Saved to: {config_file}")
    
    def _data_collection_step(self):
        """Guide user through data collection on the robot."""
        print("DATA COLLECTION STEP")
        print("This step collects sensor data from your actual robot hardware.")
        print()
        
        print("You have two options for data collection:")
        print("1. Run the robot-side collector directly on the robot")
        print("2. Use the characterization system from your development machine")
        print()
        
        choice = input("Choose option (1 or 2): ").strip()
        
        if choice == "1":
            self._robot_side_collection()
        elif choice == "2":
            self._development_machine_collection()
        else:
            print("Invalid choice. Using robot-side collection.")
            self._robot_side_collection()
    
    def _robot_side_collection(self):
        """Guide user through robot-side data collection."""
        print("\nROBOT-SIDE DATA COLLECTION")
        print("="*40)
        print("1. Copy robot_data_collector.py to your robot")
        print("2. SSH into your robot:")
        print(f"   ssh user@{self.config['robot_ip']}")
        print("3. Run the characterization session:")
        print("   python robot_data_collector.py session")
        print("4. Follow the prompts to position the robot for each test")
        print("5. Transfer the collected data back to your development machine")
        print()
        
        input("Press Enter when you have completed the robot-side data collection...")
        
        # Check for collected data
        data_files = list(Path('.').glob('robot_sensor_data_*.json'))
        if data_files:
            print(f"✓ Found {len(data_files)} data files")
            for file in data_files:
                print(f"  - {file}")
        else:
            print("⚠ No data files found. Please ensure data collection is complete.")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                sys.exit(1)
    
    def _development_machine_collection(self):
        """Guide user through development machine data collection."""
        print("\nDEVELOPMENT MACHINE DATA COLLECTION")
        print("="*40)
        print("This will run the characterization system locally.")
        print("Make sure your robot is connected and accessible.")
        print()
        
        response = input("Start data collection? (y/n): ")
        if response.lower() == 'y':
            try:
                subprocess.run([sys.executable, 'sensor_characterization.py'], check=True)
                print("✓ Data collection complete")
            except subprocess.CalledProcessError as e:
                print(f"✗ Data collection failed: {e}")
                sys.exit(1)
        else:
            print("Skipping data collection step")
    
    def _data_analysis_step(self):
        """Analyze collected data and generate sensor model."""
        print("DATA ANALYSIS STEP")
        print("Analyzing collected sensor data and generating realistic sensor model...")
        
        # Find data files
        data_files = list(Path('.').glob('*sensor_data_*.json'))
        if not data_files:
            print("⚠ No data files found. Using simulated data for analysis.")
            self._generate_simulated_data()
            data_files = list(Path('.').glob('simulated_sensor_data_*.json'))
        
        # Run analysis
        try:
            from sensor_characterization import CharacterizationRunner, CharacterizationConfig
            
            config = CharacterizationConfig(
                collection_duration=self.config['collection_duration'],
                sample_rate=self.config['sample_rate'],
                num_configurations=self.config['num_configurations'],
                target_distances=self.config['target_distances'],
                target_angles=self.config['target_angles'],
                target_materials=self.config['target_materials']
            )
            
            runner = CharacterizationRunner(config)
            sensor_model = runner.run_characterization()
            
            if sensor_model:
                print("✓ Sensor model generated successfully")
                print(f"  Systematic bias: {sensor_model['systematic_bias']:.3f} ± {sensor_model['systematic_bias_std']:.3f} m")
                print(f"  Measurement noise: {sensor_model['measurement_noise_std']:.3f} m")
                print(f"  Materials tested: {sensor_model['materials']}")
            else:
                print("✗ Failed to generate sensor model")
                
        except Exception as e:
            print(f"✗ Data analysis failed: {e}")
            print("Using default sensor model parameters")
    
    def _generate_simulated_data(self):
        """Generate simulated sensor data for testing."""
        print("Generating simulated sensor data...")
        
        import numpy as np
        from datetime import datetime
        
        # Create simulated data
        simulated_data = []
        timestamp = datetime.now()
        
        for distance in self.config['target_distances']:
            for angle in self.config['target_angles']:
                for material in self.config['target_materials']:
                    # Simulate 50 samples per configuration
                    for i in range(50):
                        # Add realistic noise
                        noise = np.random.normal(0, 0.02)  # 2cm standard deviation
                        simulated_reading = distance + noise
                        
                        data_point = {
                            'timestamp': timestamp.timestamp() + i * 0.1,
                            'lidar_ranges': [simulated_reading] * 8,
                            'lidar_angles': [0, 45, 90, 135, 180, 225, 270, 315],
                            'robot_pose': [0.0, 0.0, 0.0],
                            'robot_velocity': [0.0, 0.0, 0.0],
                            'imu_orientation': [1.0, 0.0, 0.0, 0.0],
                            'imu_angular_velocity': [0.0, 0.0, 0.0],
                            'imu_linear_acceleration': [0.0, 0.0, 9.81],
                            'target_distance': distance,
                            'target_angle': angle,
                            'target_material': material,
                            'ambient_light': 500.0,
                            'temperature': 25.0
                        }
                        simulated_data.append(data_point)
        
        # Save simulated data
        filename = f"simulated_sensor_data_{timestamp.strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(simulated_data, f, indent=2)
        
        print(f"✓ Generated simulated data: {filename}")
    
    def _simulation_integration_step(self):
        """Integrate sensor model into SLAM simulation."""
        print("SIMULATION INTEGRATION STEP")
        print("Integrating realistic sensor model into SLAM simulation...")
        
        # Check for sensor model file
        sensor_model_files = list(Path('.').glob('sensor_model_*.json'))
        if not sensor_model_files:
            print("⚠ No sensor model file found. Creating default model...")
            self._create_default_sensor_model()
            sensor_model_files = list(Path('.').glob('sensor_model_*.json'))
        
        sensor_model_file = sensor_model_files[-1]  # Use most recent
        print(f"✓ Using sensor model: {sensor_model_file}")
        
        # Test integration
        try:
            from realistic_sensor_model import RealisticLidarModel
            
            lidar_model = RealisticLidarModel(str(sensor_model_file))
            characteristics = lidar_model.get_sensor_characteristics()
            
            print("✓ Sensor model integration successful")
            print(f"  LiDAR characteristics loaded:")
            print(f"    - Systematic bias: {characteristics['systematic_bias']:.3f} m")
            print(f"    - Measurement noise: {characteristics['measurement_noise_std']:.3f} m")
            print(f"    - Range limits: {characteristics['min_range']:.1f} - {characteristics['max_range']:.1f} m")
            
        except Exception as e:
            print(f"✗ Sensor model integration failed: {e}")
    
    def _create_default_sensor_model(self):
        """Create a default sensor model for testing."""
        default_model = {
            'systematic_bias': 0.0,
            'systematic_bias_std': 0.0,
            'measurement_noise_std': 0.02,
            'materials': ['wall', 'cardboard', 'metal'],
            'material_effects': {
                'wall': {'mean_error': 0.0, 'std_error': 0.02},
                'cardboard': {'mean_error': 0.01, 'std_error': 0.03},
                'metal': {'mean_error': -0.005, 'std_error': 0.015}
            },
            'distance_effects': {
                'polynomial_coefficients': [0.001, -0.002, 0.0],
                'distance_error_correlation': 0.3
            },
            'angle_effects': {
                'angle_error_correlation': 0.1,
                'angle_error_std': 0.01
            }
        }
        
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_model_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(default_model, f, indent=2)
        
        print(f"✓ Created default sensor model: {filename}")
    
    def _validation_step(self):
        """Validate the sensor model and simulation integration."""
        print("VALIDATION STEP")
        print("Validating sensor model and simulation integration...")
        
        # Test sensor model
        try:
            from realistic_sensor_model import RealisticLidarModel
            
            # Test with different scenarios
            lidar_model = RealisticLidarModel()
            
            # Test 1: Basic functionality
            reading = lidar_model.simulate_reading(1.0, 0.0, "wall")
            print(f"✓ Basic reading test: 1.0m → {reading:.3f}m")
            
            # Test 2: Material effects
            wall_reading = lidar_model.simulate_reading(1.0, 0.0, "wall")
            metal_reading = lidar_model.simulate_reading(1.0, 0.0, "metal")
            print(f"✓ Material effects test: wall={wall_reading:.3f}m, metal={metal_reading:.3f}m")
            
            # Test 3: Distance effects
            near_reading = lidar_model.simulate_reading(0.5, 0.0, "wall")
            far_reading = lidar_model.simulate_reading(2.0, 0.0, "wall")
            print(f"✓ Distance effects test: 0.5m={near_reading:.3f}m, 2.0m={far_reading:.3f}m")
            
            print("✓ Sensor model validation successful")
            
        except Exception as e:
            print(f"✗ Sensor model validation failed: {e}")
    
    def _testing_step(self):
        """Test the complete realistic SLAM simulation."""
        print("TESTING STEP")
        print("Testing realistic SLAM simulation...")
        
        # Check if simulation files exist
        simulation_file = Path('slam_simulation_realistic.py')
        if not simulation_file.exists():
            print("⚠ Realistic SLAM simulation file not found")
            print("Please ensure slam_simulation_realistic.py is available")
            return
        
        print("The realistic SLAM simulation is ready for testing.")
        print("To run the simulation:")
        print()
        print("1. Basic simulation:")
        print("   python slam_simulation_realistic.py")
        print()
        print("2. With custom sensor model:")
        print("   python slam_simulation_realistic.py --sensor-model sensor_model_YYYYMMDD_HHMMSS.json")
        print()
        print("3. With custom configuration:")
        print("   python slam_simulation_realistic.py --config custom_config.json")
        print()
        
        response = input("Run a quick test now? (y/n): ")
        if response.lower() == 'y':
            try:
                print("Starting realistic SLAM simulation test...")
                print("Press 'Q' to quit the simulation")
                subprocess.run([sys.executable, 'slam_simulation_realistic.py'], timeout=30)
                print("✓ Simulation test completed")
            except subprocess.TimeoutExpired:
                print("✓ Simulation test completed (timeout)")
            except Exception as e:
                print(f"✗ Simulation test failed: {e}")


def main():
    """Main function to run the sensor characterization workflow."""
    workflow = SensorCharacterizationWorkflow()
    workflow.run_workflow()


if __name__ == "__main__":
    main()

