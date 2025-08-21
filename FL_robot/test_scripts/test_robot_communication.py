#!/usr/bin/env python3
"""
Test script for robot communication functionality.
This script tests the RosLink class without requiring a real robot connection.
"""

import sys
import time
from ktom_experimenter import RosLink

def test_roslink_creation():
    """Test creating a RosLink instance."""
    print("Testing RosLink creation...")
    try:
        link = RosLink(host="192.168.1.100", port=10090)
        print("✅ RosLink created successfully")
        return link
    except Exception as e:
        print(f"❌ Failed to create RosLink: {e}")
        return None

def test_connection_failure():
    """Test that connection fails gracefully when robot is not available."""
    print("\nTesting connection failure (expected)...")
    link = RosLink(host="192.168.1.100", port=10090)
    
    # This should fail gracefully since no robot is running
    success = link.connect()
    if not success:
        print("✅ Connection failed gracefully as expected")
    else:
        print("❌ Connection succeeded unexpectedly")
    
    link.close()
    print("✅ RosLink closed successfully")

def test_message_creation():
    """Test creating ROS messages."""
    print("\nTesting message creation...")
    link = RosLink(host="192.168.1.100", port=10090)
    
    try:
        # Test target message
        link.send_target(2)  # Should create Int32 message with data=2
        print("✅ Target message creation successful")
        
        # Test toggle message
        link.send_toggle("drive_mode=auto_line")  # Should create String message
        print("✅ Toggle message creation successful")
        
        # Test velocity message
        link.send_cmdvel(0.5, 1.0)  # Should create Twist message
        print("✅ Velocity message creation successful")
        
    except Exception as e:
        print(f"❌ Message creation failed: {e}")
    
    link.close()

def test_gui_import():
    """Test that the GUI can be imported and created."""
    print("\nTesting GUI import...")
    try:
        import tkinter as tk
        from ktom_experimenter import KToMExperimenterGUI
        
        # Create a minimal GUI (don't show it)
        root = tk.Tk()
        root.withdraw()  # Hide the window
        app = KToMExperimenterGUI(root)
        print("✅ GUI created successfully")
        
        # Test robot control tab exists
        if hasattr(app, 'create_robot_control_tab'):
            print("✅ Robot control tab method exists")
        else:
            print("❌ Robot control tab method missing")
        
        # Test RosLink integration
        if hasattr(app, 'robot_link'):
            print("✅ RosLink integration exists")
        else:
            print("❌ RosLink integration missing")
        
        root.destroy()
        
    except Exception as e:
        print(f"❌ GUI import failed: {e}")

def main():
    print("🧪 Testing Robot Communication Components")
    print("=" * 50)
    
    # Test basic RosLink functionality
    test_roslink_creation()
    test_connection_failure()
    test_message_creation()
    
    # Test GUI integration
    test_gui_import()
    
    print("\n" + "=" * 50)
    print("✅ All tests completed!")
    print("\nTo test with a real robot:")
    print("1. Start the robot's rosbridge server")
    print("2. Run: python ktom_experimenter.py")
    print("3. Go to 'Robot Control' tab")
    print("4. Enter robot IP and click 'Connect to Robot'")

if __name__ == "__main__":
    main()
