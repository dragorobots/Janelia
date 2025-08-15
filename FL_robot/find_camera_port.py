#!/usr/bin/env python3
"""
Camera Port Finder for Robot
Scans common ports to find the camera stream
"""

import cv2
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

class CameraPortFinder:
    def __init__(self, robot_ip="10.0.0.234"):
        self.robot_ip = robot_ip
        self.common_ports = [
            80,      # HTTP
            8080,    # Common camera port
            8081,    # Alternative camera port
            8082,    # Another alternative
            5000,    # Flask default
            5001,    # Alternative Flask
            3000,    # Node.js default
            8000,    # Django default
            8888,    # Jupyter/alternative
            9000,    # Alternative
            9090,    # Alternative
            10000,   # Alternative
        ]
        
        self.common_paths = [
            "/stream.mjpg",
            "/video_feed",
            "/camera/stream",
            "/camera/feed",
            "/stream",
            "/video",
            "/camera",
            "/mjpeg",
            "/webcam",
            "/",
        ]
        
    def test_port_connection(self, port):
        """Test if a port is open"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((self.robot_ip, port))
            sock.close()
            return result == 0
        except:
            return False
    
    def test_camera_url(self, port, path):
        """Test if a camera URL works"""
        url = f"http://{self.robot_ip}:{port}{path}"
        try:
            cap = cv2.VideoCapture(url)
            if cap.isOpened():
                # Try to read a frame
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    return True, url, frame.shape
            return False, url, None
        except Exception as e:
            return False, url, None
    
    def scan_ports(self):
        """Scan for open ports"""
        print(f"🔍 Scanning for open ports on {self.robot_ip}...")
        open_ports = []
        
        with ThreadPoolExecutor(max_workers=10) as executor:
            future_to_port = {executor.submit(self.test_port_connection, port): port 
                            for port in self.common_ports}
            
            for future in as_completed(future_to_port):
                port = future_to_port[future]
                try:
                    if future.result():
                        open_ports.append(port)
                        print(f"✅ Port {port} is open")
                    else:
                        print(f"❌ Port {port} is closed")
                except Exception as e:
                    print(f"❌ Error testing port {port}: {e}")
        
        return open_ports
    
    def test_camera_streams(self, open_ports):
        """Test camera streams on open ports"""
        print(f"\n🎥 Testing camera streams on open ports...")
        working_streams = []
        
        for port in open_ports:
            print(f"\nTesting port {port}:")
            for path in self.common_paths:
                try:
                    works, url, shape = self.test_camera_url(port, path)
                    if works:
                        print(f"  ✅ {url} - Shape: {shape}")
                        working_streams.append({
                            'port': port,
                            'path': path,
                            'url': url,
                            'shape': shape
                        })
                    else:
                        print(f"  ❌ {url}")
                except Exception as e:
                    print(f"  ❌ {url} - Error: {e}")
        
        return working_streams
    
    def find_camera(self):
        """Main method to find camera"""
        print("🎯 Robot Camera Port Finder")
        print("=" * 40)
        print(f"Target IP: {self.robot_ip}")
        print()
        
        # Step 1: Scan for open ports
        open_ports = self.scan_ports()
        
        if not open_ports:
            print("\n❌ No open ports found!")
            print("Possible issues:")
            print("1. Robot is not running")
            print("2. Robot IP is incorrect")
            print("3. Firewall is blocking connections")
            print("4. Robot is not on the same network")
            return None
        
        # Step 2: Test camera streams
        working_streams = self.test_camera_streams(open_ports)
        
        if not working_streams:
            print("\n❌ No camera streams found!")
            print("Possible issues:")
            print("1. Camera server is not running on the robot")
            print("2. Camera server uses a different URL pattern")
            print("3. Camera requires authentication")
            return None
        
        # Step 3: Display results
        print(f"\n🎉 Found {len(working_streams)} working camera stream(s)!")
        print("\nRecommended configuration:")
        
        for i, stream in enumerate(working_streams):
            print(f"\n{i+1}. Port: {stream['port']}")
            print(f"   URL: {stream['url']}")
            print(f"   Path: {stream['path']}")
            print(f"   Resolution: {stream['shape'][1]}x{stream['shape'][0]}")
            
            # Generate color measurer command
            print(f"   Color Measurer Command:")
            print(f"   python color_measurer.py --ip {self.robot_ip} --port {stream['port']}")
        
        return working_streams

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Find camera port for robot")
    parser.add_argument("--ip", default="10.0.0.234", help="Robot IP address")
    parser.add_argument("--ports", nargs="+", type=int, help="Specific ports to test")
    
    args = parser.parse_args()
    
    finder = CameraPortFinder(args.ip)
    
    if args.ports:
        finder.common_ports = args.ports
    
    results = finder.find_camera()
    
    if results:
        print(f"\n💡 To use the first working stream in color_measurer.py:")
        print(f"   Change the default camera_port to {results[0]['port']}")
        print(f"   Or modify the URL patterns to include '{results[0]['path']}'")

if __name__ == "__main__":
    main()
