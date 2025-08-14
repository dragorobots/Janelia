"""
Map Visualization for SLAM System.

This module provides real-time visualization of the SLAM occupancy grid map
and robot trajectory, similar to RViz in ROS.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import threading
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass

from slam_system import SLAMSystem, MapCell


@dataclass
class MapVisualizationConfig:
    """Configuration for map visualization."""
    map_size: Tuple[int, int] = (200, 200)
    map_resolution: float = 0.05
    map_origin: Tuple[float, float] = (-5.0, -5.0)
    update_rate: float = 5.0  # Hz
    show_trajectory: bool = True
    trajectory_length: int = 1000
    robot_radius: float = 0.22


class MapVisualizer:
    """
    Real-time map visualization for SLAM system.
    
    This class provides a matplotlib-based visualization of the occupancy grid
    map, robot pose, and trajectory, similar to RViz in ROS.
    """
    
    def __init__(self, slam_system: SLAMSystem, config: Optional[MapVisualizationConfig] = None):
        self.slam = slam_system
        self.config = config or MapVisualizationConfig()
        
        # Visualization state
        self.fig = None
        self.ax = None
        self.robot_plot = None
        self.trajectory_plot = None
        self.map_image = None
        
        # Trajectory storage
        self.trajectory_x = []
        self.trajectory_y = []
        
        # Threading
        self.running = False
        self.update_thread = None
        
        # Color map for occupancy grid
        self.colormap = {
            MapCell.UNKNOWN.value: 0.5,  # Gray
            MapCell.FREE.value: 1.0,     # White
            MapCell.OCCUPIED.value: 0.0  # Black
        }
    
    def start(self):
        """Start the map visualization."""
        self.running = True
        self.setup_plot()
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def stop(self):
        """Stop the map visualization."""
        self.running = False
        if self.update_thread:
            self.update_thread.join()
        if self.fig:
            plt.close(self.fig)
    
    def setup_plot(self):
        """Setup the matplotlib plot."""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_title("SLAM Map Visualization")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.set_aspect('equal')
        
        # Set plot limits
        map_width = self.config.map_size[0] * self.config.map_resolution
        map_height = self.config.map_size[1] * self.config.map_resolution
        self.ax.set_xlim(self.config.map_origin[0], self.config.map_origin[0] + map_width)
        self.ax.set_ylim(self.config.map_origin[1], self.config.map_origin[1] + map_height)
        
        # Initialize map image
        self.map_image = self.ax.imshow(
            np.zeros(self.config.map_size),
            extent=[
                self.config.map_origin[0],
                self.config.map_origin[0] + map_width,
                self.config.map_origin[1],
                self.config.map_origin[1] + map_height
            ],
            cmap='gray',
            vmin=0,
            vmax=1
        )
        
        # Initialize robot plot
        self.robot_plot, = self.ax.plot([], [], 'ro', markersize=10, label='Robot')
        
        # Initialize trajectory plot
        if self.config.show_trajectory:
            self.trajectory_plot, = self.ax.plot([], [], 'b-', alpha=0.7, linewidth=2, label='Trajectory')
        
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    def _update_loop(self):
        """Main update loop for the visualization."""
        while self.running:
            try:
                self.update_visualization()
                time.sleep(1.0 / self.config.update_rate)
            except Exception as e:
                print(f"Error in map visualization: {e}")
                break
    
    def update_visualization(self):
        """Update the map visualization with current data."""
        if not self.fig or not plt.fignum_exists(self.fig.number):
            return
        
        # Get current map data
        map_data = self.slam.get_map_data()
        
        # Convert to visualization format
        vis_data = np.zeros_like(map_data, dtype=np.float32)
        for cell_type, value in self.colormap.items():
            vis_data[map_data == cell_type] = value
        
        # Update map image
        if self.map_image:
            self.map_image.set_array(vis_data)
        
        # Update robot position
        robot_x, robot_y, robot_theta = self.slam.get_robot_pose()
        self.robot_plot.set_data([robot_x], [robot_y])
        
        # Update trajectory
        if self.config.show_trajectory:
            self.trajectory_x.append(robot_x)
            self.trajectory_y.append(robot_y)
            
            # Limit trajectory length
            if len(self.trajectory_x) > self.config.trajectory_length:
                self.trajectory_x = self.trajectory_x[-self.config.trajectory_length:]
                self.trajectory_y = self.trajectory_y[-self.config.trajectory_length:]
            
            self.trajectory_plot.set_data(self.trajectory_x, self.trajectory_y)
        
        # Redraw
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def add_goal_marker(self, goal_x: float, goal_y: float):
        """Add a goal marker to the map."""
        if self.ax:
            goal_marker = self.ax.plot([goal_x], [goal_y], 'g*', markersize=15, label='Goal')
            self.ax.legend()
            return goal_marker
    
    def clear_goals(self):
        """Clear all goal markers."""
        if self.ax:
            for artist in self.ax.get_children():
                if hasattr(artist, 'get_label') and artist.get_label() == 'Goal':
                    artist.remove()
    
    def save_map(self, filename: str):
        """Save the current map to a file."""
        map_data = self.slam.get_map_data()
        
        # Convert to image format
        vis_data = np.zeros_like(map_data, dtype=np.uint8)
        for cell_type, value in self.colormap.items():
            vis_data[map_data == cell_type] = int(value * 255)
        
        # Save using matplotlib
        plt.imsave(filename, vis_data, cmap='gray')
        print(f"Map saved to {filename}")


class SimpleMapViewer:
    """
    Simple map viewer for non-interactive display.
    
    This class provides a simpler interface for viewing the SLAM map
    without real-time updates.
    """
    
    def __init__(self, slam_system: SLAMSystem):
        self.slam = slam_system
    
    def show_map(self, title: str = "SLAM Map"):
        """Display the current map."""
        map_data = self.slam.get_map_data()
        robot_x, robot_y, robot_theta = self.slam.get_robot_pose()
        
        # Create visualization
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Convert map data to visualization format
        vis_data = np.zeros_like(map_data, dtype=np.float32)
        vis_data[map_data == MapCell.UNKNOWN.value] = 0.5  # Gray
        vis_data[map_data == MapCell.FREE.value] = 1.0     # White
        vis_data[map_data == MapCell.OCCUPIED.value] = 0.0 # Black
        
        # Display map
        map_width = map_data.shape[1] * 0.05  # 5cm resolution
        map_height = map_data.shape[0] * 0.05
        ax.imshow(vis_data, cmap='gray', extent=[-5, -5 + map_width, -5, -5 + map_height])
        
        # Add robot position
        ax.plot(robot_x, robot_y, 'ro', markersize=10, label='Robot')
        
        # Add robot orientation
        arrow_length = 0.3
        arrow_dx = arrow_length * np.cos(robot_theta)
        arrow_dy = arrow_length * np.sin(robot_theta)
        ax.arrow(robot_x, robot_y, arrow_dx, arrow_dy, 
                head_width=0.1, head_length=0.1, fc='red', ec='red')
        
        ax.set_title(title)
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_map_image(self, filename: str):
        """Save the current map as an image."""
        map_data = self.slam.get_map_data()
        
        # Convert to image format
        vis_data = np.zeros_like(map_data, dtype=np.uint8)
        vis_data[map_data == MapCell.UNKNOWN.value] = 128  # Gray
        vis_data[map_data == MapCell.FREE.value] = 255     # White
        vis_data[map_data == MapCell.OCCUPIED.value] = 0   # Black
        
        plt.imsave(filename, vis_data, cmap='gray')
        print(f"Map image saved to {filename}")


def demo_map_visualization():
    """Demo function to test map visualization."""
    from config import SimulationConfig
    
    # Create SLAM system
    config = SimulationConfig()
    slam = SLAMSystem(config)
    
    # Create visualizer
    viewer = SimpleMapViewer(slam)
    
    # Simulate some sensor updates
    for i in range(100):
        # Simulate sensor data
        lidar_data = np.random.uniform(0.5, 3.0, 8)
        imu_data = np.random.normal(0, 0.1, 10)
        odom_data = np.array([i * 0.01, 0, 0, 0.1, 0, 0])  # Moving forward
        
        # Update SLAM
        slam.update_sensors(lidar_data, imu_data, odom_data, 0.01)
    
    # Show the map
    viewer.show_map("Demo SLAM Map")
    viewer.save_map_image("demo_map.png")


if __name__ == "__main__":
    demo_map_visualization()
