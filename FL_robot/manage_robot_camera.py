#!/usr/bin/env python3
"""
Robot Camera Management Script
Helps manage camera access between camera server and main robot system
"""

import cv2
import subprocess
import time
import sys
import os

def check_camera_availability():
    """Check which camera indices are available"""
    print("🔍 Checking camera availability...")
    available_cameras = []
    
    for i in range(4):  # Check cameras 0-3
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    height, width = frame.shape[:2]
                    print(f"✅ Camera {i}: Available ({width}x{height})")
                    available_cameras.append(i)
                else:
                    print(f"❌ Camera {i}: Opened but can't read frames")
                cap.release()
            else:
                print(f"❌ Camera {i}: Not available")
        except Exception as e:
            print(f"❌ Camera {i}: Error - {e}")
    
    return available_cameras

def test_camera_conflict():
    """Test if multiple processes can access the same camera"""
    print("\n🧪 Testing camera access...")
    
    # Try to open camera 0
    cap1 = cv2.VideoCapture(0)
    if not cap1.isOpened():
        print("❌ Cannot open camera 0")
        return False
    
    print("✅ First process opened camera 0")
    
    # Try to open camera 0 again (simulating conflict)
    cap2 = cv2.VideoCapture(0)
    if cap2.isOpened():
        print("✅ Second process can also open camera 0")
        ret, frame = cap2.read()
        if ret:
            print("✅ Second process can read from camera 0")
        else:
            print("❌ Second process cannot read from camera 0")
        cap2.release()
    else:
        print("❌ Second process cannot open camera 0 (conflict detected)")
    
    cap1.release()
    return True

def show_usage_guide():
    """Show usage guide for managing camera access"""
    print("\n📋 Camera Management Guide")
    print("=" * 40)
    print()
    print("🎯 Problem: Camera conflict between camera server and robot system")
    print()
    print("🔧 Solutions:")
    print()
    print("1. 🎨 COLOR MEASUREMENT MODE:")
    print("   - Start camera server: python3 camera_server.py")
    print("   - Use color measurer from PC")
    print("   - Stop camera server when done (Ctrl+C)")
    print()
    print("2. 🤖 ROBOT OPERATION MODE:")
    print("   - Stop camera server (Ctrl+C)")
    print("   - Start robot system: ./start_hide_and_seek.sh")
    print()
    print("3. 🔄 QUICK SWITCH:")
    print("   - Use this script to manage camera access")
    print("   - Run: python3 manage_robot_camera.py --mode color")
    print("   - Run: python3 manage_robot_camera.py --mode robot")
    print()
    print("💡 Tip: The camera can only be accessed by one process at a time!")

def start_color_mode():
    """Start color measurement mode"""
    print("🎨 Starting Color Measurement Mode...")
    print("This will start the camera server for color measurement.")
    print("Press Ctrl+C to stop and return to robot mode.")
    print()
    
    try:
        # Start camera server
        subprocess.run(["python3", "camera_server.py", "--port", "8080"])
    except KeyboardInterrupt:
        print("\n🛑 Color measurement mode stopped")
        print("You can now start the robot system")

def start_robot_mode():
    """Start robot operation mode"""
    print("🤖 Starting Robot Operation Mode...")
    print("This will start the hide and seek robot system.")
    print("Make sure the camera server is stopped first.")
    print()
    
    # Check if camera server is running
    try:
        import requests
        response = requests.get("http://10.0.0.234:8080/status", timeout=2)
        if response.status_code == 200:
            print("⚠️  Warning: Camera server appears to be running!")
            print("   Please stop it first (Ctrl+C in camera server terminal)")
            print("   Then run this command again.")
            return
    except:
        pass  # Camera server not running, which is good
    
    try:
        # Start hide and seek system
        subprocess.run(["./start_hide_and_seek.sh"])
    except KeyboardInterrupt:
        print("\n🛑 Robot operation mode stopped")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Manage robot camera access")
    parser.add_argument("--mode", choices=["check", "color", "robot", "guide"], 
                       default="check", help="Operation mode")
    
    args = parser.parse_args()
    
    print("🤖 Robot Camera Management")
    print("=" * 30)
    
    if args.mode == "check":
        available_cameras = check_camera_availability()
        test_camera_conflict()
        show_usage_guide()
        
    elif args.mode == "color":
        start_color_mode()
        
    elif args.mode == "robot":
        start_robot_mode()
        
    elif args.mode == "guide":
        show_usage_guide()

if __name__ == "__main__":
    main()
