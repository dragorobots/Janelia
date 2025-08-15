#!/usr/bin/env python3
"""
Color Measurement Tool for Hide and Seek Robot
Allows users to select a region of interest on the robot's camera feed
and get RGB values for line following calibration.
"""

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import socket
import subprocess
import requests
import sys

class ColorMeasurer:
    def __init__(self, robot_ip="10.0.0.234", camera_port=8080):
        self.robot_ip = robot_ip
        self.camera_port = camera_port
        self.cap = None
        self.is_running = False
        self.roi_selected = False
        self.roi_coords = None
        self.camera_server_process = None
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the GUI for color measurement"""
        self.root = tk.Tk()
        self.root.title("Robot Color Measurer")
        self.root.geometry("800x600")
        
        # Control frame
        control_frame = ttk.LabelFrame(self.root, text="Controls", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # Robot IP input
        ttk.Label(control_frame, text="Robot IP:").pack(side='left', padx=(0,5))
        self.ip_var = tk.StringVar(value=self.robot_ip)
        self.ip_entry = ttk.Entry(control_frame, textvariable=self.ip_var, width=15)
        self.ip_entry.pack(side='left', padx=(0,10))
        
        # Camera port input
        ttk.Label(control_frame, text="Camera Port:").pack(side='left', padx=(0,5))
        self.port_var = tk.StringVar(value=str(self.camera_port))
        self.port_entry = ttk.Entry(control_frame, textvariable=self.port_var, width=8)
        self.port_entry.pack(side='left', padx=(0,10))
        
        # Connect button
        self.connect_btn = ttk.Button(control_frame, text="Connect to Camera", 
                                     command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=(0,10))
        
        # Instructions
        instruction_frame = ttk.LabelFrame(self.root, text="Instructions", padding=10)
        instruction_frame.pack(fill='x', padx=10, pady=5)
        
        instructions = """
1. Enter robot IP and camera port (default: 8080)
2. Click "Connect to Camera" to start video feed (will prompt you to start camera server on robot)
3. Click and drag on the video to select a region of interest (ROI)
4. The RGB values of the selected region will be displayed below
5. Use these values for line color configuration in the main GUI
6. Click "✅ Finished - Close All" when done (will prompt you to stop camera server)
        """
        ttk.Label(instruction_frame, text=instructions, justify='left').pack(anchor='w')
        
        # Video frame
        video_frame = ttk.LabelFrame(self.root, text="Camera Feed", padding=10)
        video_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Video display (placeholder)
        self.video_label = ttk.Label(video_frame, text="Click 'Connect to Camera' to start")
        self.video_label.pack(expand=True)
        
        # Results frame
        results_frame = ttk.LabelFrame(self.root, text="Color Results", padding=10)
        results_frame.pack(fill='x', padx=10, pady=5)
        
        # RGB values display
        self.rgb_label = ttk.Label(results_frame, text="RGB: No region selected", 
                                  font=('Arial', 12, 'bold'))
        self.rgb_label.pack()
        
        # HSV values display
        self.hsv_label = ttk.Label(results_frame, text="HSV: No region selected", 
                                  font=('Arial', 12, 'bold'))
        self.hsv_label.pack()
        
        # Copy button
        self.copy_btn = ttk.Button(results_frame, text="Copy RGB Values", 
                                  command=self.copy_rgb_values, state='disabled')
        self.copy_btn.pack(pady=5)
        
        # Finished button
        self.finished_btn = ttk.Button(results_frame, text="✅ Finished - Close All", 
                                      command=self.finish_and_close, style='Accent.TButton')
        self.finished_btn.pack(pady=10)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(self.root, textvariable=self.status_var, 
                                     relief='sunken', anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=2)
        
    def toggle_connection(self):
        """Toggle camera connection"""
        if not self.is_running:
            self.connect_to_camera()
        else:
            self.disconnect_camera()
            
    def check_camera_server_status(self):
        """Check if camera server is running on robot"""
        try:
            response = requests.get(f"http://{self.robot_ip}:{self.camera_port}/status", timeout=2)
            return response.status_code == 200
        except:
            return False
            
    def start_camera_server_on_robot(self):
        """Start camera server on robot via SSH"""
        try:
            self.status_var.set("Starting camera server on robot...")
            self.root.update()
            
            # Show instructions to user
            result = messagebox.askyesno(
                "Start Camera Server", 
                f"Camera server needs to be started on the robot.\n\n"
                f"Please run this command on the robot:\n\n"
                f"python3 camera_server.py --port {self.camera_port}\n\n"
                f"Then click 'Yes' to continue, or 'No' to cancel."
            )
            
            if not result:
                self.status_var.set("Camera server start cancelled")
                return False
            
            # Wait a bit for server to start
            time.sleep(2)
            
            # Check if server started successfully
            if self.check_camera_server_status():
                self.status_var.set("Camera server started successfully")
                return True
            else:
                self.status_var.set("Camera server not detected. Please make sure it's running.")
                return False
                
        except Exception as e:
            self.status_var.set(f"Error starting camera server: {str(e)}")
            return False
            
    def stop_camera_server_on_robot(self):
        """Stop camera server on robot"""
        try:
            # Show instructions to user
            result = messagebox.askyesno(
                "Stop Camera Server", 
                f"Camera server should be stopped on the robot.\n\n"
                f"Please press Ctrl+C in the camera server terminal on the robot.\n\n"
                f"Click 'Yes' to continue, or 'No' to skip."
            )
            
            if result:
                self.status_var.set("Camera server stopped")
            else:
                self.status_var.set("Camera server stop skipped")
            
            return True
            
        except Exception as e:
            self.status_var.set(f"Error stopping camera server: {str(e)}")
            return False
            
    def connect_to_camera(self):
        """Connect to robot's camera feed"""
        try:
            self.robot_ip = self.ip_var.get()
            self.camera_port = int(self.port_var.get())
            
            # Test connection
            self.status_var.set(f"Connecting to {self.robot_ip}:{self.camera_port}...")
            self.root.update()
            
            # First check if camera server is running
            if not self.check_camera_server_status():
                self.status_var.set("Camera server not running. Starting it automatically...")
                self.root.update()
                
                if not self.start_camera_server_on_robot():
                    raise Exception("Failed to start camera server on robot")
            
            # Try to connect to camera stream
            camera_url = f"http://{self.robot_ip}:{self.camera_port}/stream.mjpg"
            self.cap = cv2.VideoCapture(camera_url)
            
            if not self.cap.isOpened():
                # Try alternative URL format
                camera_url = f"http://{self.robot_ip}:{self.camera_port}/video_feed"
                self.cap = cv2.VideoCapture(camera_url)
                
            if not self.cap.isOpened():
                raise Exception("Could not connect to camera stream")
            
            self.is_running = True
            self.connect_btn.config(text="Disconnect Camera")
            self.status_var.set("Connected - Click and drag to select ROI")
            
            # Start video thread
            self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
            self.video_thread.start()
            
        except Exception as e:
            self.status_var.set(f"Connection failed: {str(e)}")
            messagebox.showerror("Connection Error", 
                               f"Failed to connect to camera:\n{str(e)}\n\n"
                               f"Make sure the robot's camera server is running on port {self.camera_port}")
            
    def disconnect_camera(self):
        """Disconnect from camera"""
        self.is_running = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.connect_btn.config(text="Connect to Camera")
        self.video_label.config(text="Click 'Connect to Camera' to start")
        self.status_var.set("Disconnected")
        
    def finish_and_close(self):
        """Finish color measurement and close everything"""
        try:
            # Disconnect camera
            self.disconnect_camera()
            
            # Stop camera server on robot
            self.stop_camera_server_on_robot()
            
            # Show completion message
            messagebox.showinfo("Color Measurement Complete", 
                              "Color measurement finished!\n\n"
                              "Please stop the camera server on the robot (Ctrl+C).\n"
                              "You can now start the robot system.")
            
            # Close GUI
            self.root.quit()
            
        except Exception as e:
            messagebox.showerror("Error", f"Error during cleanup: {str(e)}")
            self.root.quit()
        
    def video_loop(self):
        """Main video processing loop"""
        while self.is_running:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        # Process frame for display
                        display_frame = self.process_frame(frame)
                        
                        # Update GUI in main thread
                        self.root.after(0, lambda: self.update_video_display(display_frame))
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Video loop error: {e}")
                time.sleep(0.1)
                
    def process_frame(self, frame):
        """Process frame and handle ROI selection"""
        # Create a copy for display
        display_frame = frame.copy()
        
        # Draw ROI if selected
        if self.roi_selected and self.roi_coords:
            x1, y1, x2, y2 = self.roi_coords
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calculate average color in ROI
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                avg_color = np.mean(roi, axis=(0, 1))
                rgb_values = tuple(map(int, avg_color))
                
                # Convert to HSV
                hsv_color = cv2.cvtColor(np.uint8([[rgb_values]]), cv2.COLOR_RGB2HSV)[0][0]
                hsv_values = tuple(map(int, hsv_color))
                
                # Update results in main thread
                self.root.after(0, lambda: self.update_color_results(rgb_values, hsv_values))
        
        return display_frame
        
    def update_video_display(self, frame):
        """Update video display in GUI"""
        try:
            # Resize frame to fit in GUI
            height, width = frame.shape[:2]
            max_width = 640
            max_height = 480
            
            if width > max_width or height > max_height:
                scale = min(max_width/width, max_height/height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                frame = cv2.resize(frame, (new_width, new_height))
            
            # Convert to RGB for tkinter
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to PhotoImage
            try:
                # Try PPM format first
                frame_pil = tk.PhotoImage(data=cv2.imencode('.ppm', frame_rgb)[1].tobytes())
            except Exception:
                # Fallback to PNG format if PPM fails
                frame_pil = tk.PhotoImage(data=cv2.imencode('.png', frame_rgb)[1].tobytes())
            
            # Update label
            self.video_label.config(image=frame_pil, text="")
            self.video_label.image = frame_pil  # Keep a reference
            
            # Bind mouse events for ROI selection
            self.video_label.bind('<Button-1>', self.on_mouse_down)
            self.video_label.bind('<B1-Motion>', self.on_mouse_drag)
            self.video_label.bind('<ButtonRelease-1>', self.on_mouse_up)
            
        except Exception as e:
            print(f"Display update error: {e}")
            
    def on_mouse_down(self, event):
        """Handle mouse button down for ROI selection"""
        self.roi_start = (event.x, event.y)
        self.roi_selected = False
        
    def on_mouse_drag(self, event):
        """Handle mouse drag for ROI selection"""
        if hasattr(self, 'roi_start'):
            x1, y1 = self.roi_start
            x2, y2 = event.x, event.y
            self.roi_coords = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
            
    def on_mouse_up(self, event):
        """Handle mouse button up for ROI selection"""
        if hasattr(self, 'roi_start'):
            x1, y1 = self.roi_start
            x2, y2 = event.x, event.y
            
            # Only select if ROI is large enough
            if abs(x2 - x1) > 5 and abs(y2 - y1) > 5:
                self.roi_coords = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
                self.roi_selected = True
                self.status_var.set("ROI selected - Color values updated")
            else:
                self.roi_selected = False
                self.roi_coords = None
                
    def update_color_results(self, rgb_values, hsv_values):
        """Update color results display"""
        r, g, b = rgb_values
        h, s, v = hsv_values
        
        # Store the RGB values for copying
        self.current_rgb_values = rgb_values
        
        self.rgb_label.config(text=f"RGB: ({r}, {g}, {b})")
        self.hsv_label.config(text=f"HSV: ({h}, {s}, {v})")
        self.copy_btn.config(state='normal')
        
    def copy_rgb_values(self):
        """Copy RGB values to clipboard"""
        if self.roi_selected and self.roi_coords and hasattr(self, 'current_rgb_values'):
            r, g, b = self.current_rgb_values
            rgb_str = f"{r},{g},{b}"
            self.root.clipboard_clear()
            self.root.clipboard_append(rgb_str)
            self.status_var.set(f"RGB values copied to clipboard: {rgb_str}")
            
    def run(self):
        """Start the color measurer GUI"""
        self.root.mainloop()
        
    def __del__(self):
        """Cleanup on exit"""
        if self.cap:
            self.cap.release()

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Robot Color Measurer")
    parser.add_argument("--ip", default="10.0.0.234", help="Robot IP address")
    parser.add_argument("--port", type=int, default=8080, help="Camera port")
    
    args = parser.parse_args()
    
    print("🎨 Robot Color Measurer")
    print("=" * 30)
    print("This tool helps you measure line colors for the hide and seek robot.")
    print("Connect to the robot's camera and select regions to get RGB values.")
    print()
    print(f"Target: {args.ip}:{args.port}")
    print()
    
    # Create and run the color measurer
    measurer = ColorMeasurer(robot_ip=args.ip, camera_port=args.port)
    measurer.run()

if __name__ == "__main__":
    main()
