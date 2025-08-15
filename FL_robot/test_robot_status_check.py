#!/usr/bin/env python3
"""
Test script to verify that the robot status check doesn't crash the GUI
when the robot is not connected.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

class MockRobotStatusTest:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Status Check Test")
        self.root.geometry("400x300")
        
        # Mock robot link (not connected)
        self.robot_link = None
        
        # Create GUI
        self.setup_gui()
        
    def setup_gui(self):
        # Title
        title_label = ttk.Label(self.root, text="Robot Status Check Test", font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # IP input
        ip_frame = ttk.Frame(self.root)
        ip_frame.pack(fill='x', padx=20, pady=10)
        
        ttk.Label(ip_frame, text="Robot IP:").pack(anchor='w')
        self.robot_ip_var = tk.StringVar(value="10.0.0.234")
        self.ip_entry = ttk.Entry(ip_frame, textvariable=self.robot_ip_var)
        self.ip_entry.pack(fill='x', pady=2)
        
        # Test button
        self.test_button = ttk.Button(self.root, text="🔍 Test Robot Status Check", 
                                     command=self.test_status_check)
        self.test_button.pack(pady=10)
        
        # Status display
        status_frame = ttk.LabelFrame(self.root, text="Test Results", padding=10)
        status_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        self.status_text = tk.Text(status_frame, height=10, wrap='word')
        status_scrollbar = ttk.Scrollbar(status_frame, orient='vertical', command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=status_scrollbar.set)
        
        self.status_text.pack(side='left', fill='both', expand=True)
        status_scrollbar.pack(side='right', fill='y')
        
    def log_status(self, message):
        """Add a timestamped message to the status display"""
        timestamp = time.strftime('%H:%M:%S')
        status_line = f"[{timestamp}] {message}\n"
        self.status_text.insert(tk.END, status_line)
        self.status_text.see(tk.END)
        
    def test_status_check(self):
        """Test the robot status check function"""
        self.log_status("Starting robot status check test...")
        self.log_status("This simulates clicking 'Check Robot Status' when robot is not connected")
        
        try:
            # This is the same logic as the fixed on_check_robot_status function
            import subprocess
            import socket
            import threading
            
            robot_ip = self.robot_ip_var.get()
            
            # Validate IP address format
            if not robot_ip or robot_ip.strip() == "":
                self.log_status("ERROR: Please enter a valid robot IP address first.")
                messagebox.showerror("Error", "Please enter a valid robot IP address first.")
                return
            
            # Check if we're already connected via ROS bridge
            bridge_connected = (self.robot_link and hasattr(self.robot_link, 'connected') and self.robot_link.connected)
            
            # Show initial status
            status_message = "🤖 Robot Status Check Results:\n\n"
            
            # Check 1: Can we reach the robot via SSH?
            ssh_reachable = False
            try:
                socket.create_connection((robot_ip, 22), timeout=5)
                ssh_reachable = True
                status_message += "✅ Robot is reachable via SSH\n"
                self.log_status("SSH connection test: SUCCESS")
            except (socket.timeout, socket.error, OSError) as e:
                status_message += "❌ Cannot reach robot via SSH (port 22)\n"
                status_message += "   - Check if robot is powered on\n"
                status_message += "   - Check network connection\n"
                status_message += "   - Verify IP address is correct\n"
                self.log_status(f"SSH connection test: FAILED - {str(e)}")
            
            # Check 2: Is ROS bridge active?
            if bridge_connected:
                status_message += "✅ ROS bridge is connected\n"
                self.log_status("ROS bridge test: CONNECTED")
            else:
                status_message += "❌ ROS bridge is NOT connected\n"
                status_message += "   - Click 'Connect to Robot' first\n"
                self.log_status("ROS bridge test: NOT CONNECTED")
            
            # Check 3: Is hide_and_seek.sh running? (only if SSH is reachable)
            if ssh_reachable:
                try:
                    # Run SSH command in a separate thread to prevent GUI blocking
                    def check_hide_and_seek():
                        try:
                            ssh_command = f"ssh root@{robot_ip} 'pgrep -f hide_and_seek'"
                            result = subprocess.run(ssh_command, shell=True, capture_output=True, text=True, timeout=10)
                            hide_and_seek_running = result.returncode == 0 and result.stdout.strip()
                            
                            # Update status message based on result
                            if hide_and_seek_running:
                                status_message += "✅ hide_and_seek.sh is running on robot\n"
                                self.root.after(0, lambda: self.log_status("hide_and_seek.sh test: RUNNING"))
                            else:
                                status_message += "❌ hide_and_seek.sh is NOT running on robot\n"
                                status_message += "   - Start it with: ssh root@" + robot_ip + "\n"
                                status_message += "   - Then run: cd ~/yahboomcar_ws/src/Janelia/FL_robot\n"
                                status_message += "   - Then run: ./start_hide_and_seek.sh\n"
                                self.root.after(0, lambda: self.log_status("hide_and_seek.sh test: NOT RUNNING"))
                            
                            # Show final status
                            if hide_and_seek_running and bridge_connected:
                                status_message += "\n🎉 Robot is ready for trials!"
                                self.root.after(0, lambda: messagebox.showinfo("Robot Status", status_message))
                            else:
                                status_message += "\n⚠️ Robot needs attention before trials."
                                self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                                
                        except subprocess.TimeoutExpired:
                            status_message += "⚠️ SSH command timed out\n"
                            status_message += "\n⚠️ Robot needs attention before trials."
                            self.root.after(0, lambda: self.log_status("hide_and_seek.sh test: TIMEOUT"))
                            self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                        except Exception as e:
                            status_message += f"⚠️ Error checking hide_and_seek: {str(e)}\n"
                            status_message += "\n⚠️ Robot needs attention before trials."
                            self.root.after(0, lambda: self.log_status(f"hide_and_seek.sh test: ERROR - {str(e)}"))
                            self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                    
                    # Start the SSH check thread
                    ssh_thread = threading.Thread(target=check_hide_and_seek, daemon=True)
                    ssh_thread.start()
                    
                    # Show immediate status while SSH check is running
                    status_message += "⏳ Checking hide_and_seek.sh status...\n"
                    self.log_status("Starting hide_and_seek.sh check in background thread...")
                    messagebox.showinfo("Robot Status", status_message)
                    
                except Exception as e:
                    status_message += f"⚠️ Error starting SSH check: {str(e)}\n"
                    status_message += "\n⚠️ Robot needs attention before trials."
                    self.log_status(f"Error starting SSH check: {str(e)}")
                    messagebox.showwarning("Robot Status", status_message)
            else:
                # If SSH is not reachable, show status without hide_and_seek check
                status_message += "\n⚠️ Cannot check hide_and_seek.sh status (SSH unreachable)"
                self.log_status("Skipping hide_and_seek.sh check (SSH unreachable)")
                messagebox.showwarning("Robot Status", status_message)
                
            self.log_status("Robot status check test completed successfully!")
            
        except Exception as e:
            error_msg = f"Error during robot status check: {str(e)}"
            self.log_status(f"ERROR: {error_msg}")
            messagebox.showerror("Error", error_msg)

def main():
    root = tk.Tk()
    app = MockRobotStatusTest(root)
    root.mainloop()

if __name__ == '__main__':
    main()
