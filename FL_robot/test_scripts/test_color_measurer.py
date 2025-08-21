#!/usr/bin/env python3
"""
Simple test script for color measurer functionality
"""

import cv2
import numpy as np

def test_color_measurer_imports():
    """Test that all required modules can be imported"""
    try:
        import cv2
        import numpy as np
        import tkinter as tk
        from tkinter import ttk, messagebox
        import threading
        import time
        import socket
        print("✅ All imports successful")
        return True
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False

def test_color_processing():
    """Test basic color processing functionality"""
    try:
        # Create a test image
        test_image = np.zeros((100, 100, 3), dtype=np.uint8)
        test_image[:, :, 0] = 255  # Red channel
        
        # Test color conversion
        hsv_image = cv2.cvtColor(test_image, cv2.COLOR_RGB2HSV)
        
        # Test ROI processing
        roi = test_image[25:75, 25:75]
        avg_color = np.mean(roi, axis=(0, 1))
        rgb_values = tuple(map(int, avg_color))
        
        print(f"✅ Color processing test passed - RGB: {rgb_values}")
        return True
    except Exception as e:
        print(f"❌ Color processing error: {e}")
        return False

def test_photoimage_creation():
    """Test PhotoImage creation for GUI"""
    try:
        import tkinter as tk
        
        # Create a test image
        test_image = np.zeros((100, 100, 3), dtype=np.uint8)
        test_image[:, :, 1] = 255  # Green channel
        
        # Convert to RGB
        frame_rgb = cv2.cvtColor(test_image, cv2.COLOR_BGR2RGB)
        
        # Try PPM format
        try:
            frame_pil = tk.PhotoImage(data=cv2.imencode('.ppm', frame_rgb)[1].tobytes())
            print("✅ PPM PhotoImage creation successful")
        except Exception:
            # Try PNG format
            frame_pil = tk.PhotoImage(data=cv2.imencode('.png', frame_rgb)[1].tobytes())
            print("✅ PNG PhotoImage creation successful")
        
        return True
    except Exception as e:
        print(f"❌ PhotoImage creation error: {e}")
        return False

def main():
    """Run all tests"""
    print("🧪 Testing Color Measurer Components")
    print("=" * 40)
    
    tests = [
        test_color_measurer_imports,
        test_color_processing,
        test_photoimage_creation
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Color measurer should work properly.")
    else:
        print("⚠️  Some tests failed. Check the errors above.")

if __name__ == "__main__":
    main()
