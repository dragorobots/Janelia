#!/usr/bin/env python3
"""
Simple Camera Server for Robot
Run this on the robot to expose the camera over HTTP
"""

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
import threading
import time
import argparse

app = Flask(__name__)

class CameraServer:
    def __init__(self, camera_index=0, port=8080, quality=80):
        self.camera_index = camera_index
        self.port = port
        self.quality = quality
        self.cap = None
        self.is_running = False
        self.frame = None
        self.lock = threading.Lock()
        
    def start_camera(self):
        """Start the camera capture"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print(f"❌ Failed to open camera at index {self.camera_index}")
                return False
                
            print(f"✅ Camera opened successfully at index {self.camera_index}")
            self.is_running = True
            
            # Start camera thread
            camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            camera_thread.start()
            
            return True
        except Exception as e:
            print(f"❌ Error starting camera: {e}")
            return False
    
    def camera_loop(self):
        """Camera capture loop"""
        while self.is_running:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        with self.lock:
                            self.frame = frame.copy()
                    else:
                        print("Failed to read frame from camera")
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Camera loop error: {e}")
                time.sleep(0.1)
    
    def get_frame(self):
        """Get the current frame"""
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
        return None
    
    def stop(self):
        """Stop the camera server"""
        self.is_running = False
        if self.cap:
            self.cap.release()

# Global camera server instance
camera_server = None

@app.route('/')
def index():
    """Main page with camera feed"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Camera Feed</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { text-align: center; }
            .camera-feed { border: 2px solid #333; margin: 20px 0; }
            .info { background: #f0f0f0; padding: 10px; border-radius: 5px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 Robot Camera Feed</h1>
            <div class="info">
                <p><strong>Camera Index:</strong> {{ camera_index }}</p>
                <p><strong>Port:</strong> {{ port }}</p>
                <p><strong>Status:</strong> {{ status }}</p>
            </div>
            <div class="camera-feed">
                <img src="/stream.mjpg" alt="Camera Feed" style="max-width: 100%; height: auto;">
            </div>
            <p><a href="/stream.mjpg" target="_blank">Direct Stream Link</a></p>
            <p><a href="/video_feed" target="_blank">Alternative Stream Link</a></p>
        </div>
    </body>
    </html>
    """
    return render_template_string(html, 
                                camera_index=camera_server.camera_index if camera_server else "Unknown",
                                port=camera_server.port if camera_server else "Unknown",
                                status="Running" if camera_server and camera_server.is_running else "Stopped")

@app.route('/stream.mjpg')
def stream_mjpg():
    """MJPEG stream endpoint"""
    def generate():
        while True:
            if camera_server:
                frame = camera_server.get_frame()
                if frame is not None:
                    # Encode frame as JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), camera_server.quality]
                    _, buffer = cv2.imencode('.jpg', frame, encode_param)
                    frame_bytes = buffer.tobytes()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    return Response(generate(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed')
def video_feed():
    """Alternative video feed endpoint"""
    return stream_mjpg()

@app.route('/status')
def status():
    """Status endpoint"""
    if camera_server:
        return {
            'running': camera_server.is_running,
            'camera_index': camera_server.camera_index,
            'port': camera_server.port,
            'frame_available': camera_server.get_frame() is not None
        }
    return {'running': False, 'error': 'Camera server not initialized'}

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Robot Camera Server")
    parser.add_argument("--camera", "-c", type=int, default=0, 
                       help="Camera index (default: 0)")
    parser.add_argument("--port", "-p", type=int, default=8080, 
                       help="Server port (default: 8080)")
    parser.add_argument("--quality", "-q", type=int, default=80, 
                       help="JPEG quality 1-100 (default: 80)")
    
    args = parser.parse_args()
    
    print("🎥 Robot Camera Server")
    print("=" * 30)
    print(f"Camera Index: {args.camera}")
    print(f"Port: {args.port}")
    print(f"Quality: {args.quality}")
    print()
    
    global camera_server
    camera_server = CameraServer(args.camera, args.port, args.quality)
    
    if not camera_server.start_camera():
        print("❌ Failed to start camera server")
        return
    
    print(f"🌐 Starting web server on port {args.port}...")
    print(f"📺 Camera feed available at:")
    print(f"   http://localhost:{args.port}/")
    print(f"   http://localhost:{args.port}/stream.mjpg")
    print(f"   http://localhost:{args.port}/video_feed")
    print()
    print("Press Ctrl+C to stop")
    
    try:
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Stopping camera server...")
    finally:
        if camera_server:
            camera_server.stop()
        print("✅ Camera server stopped")

if __name__ == "__main__":
    main()
