#!/usr/bin/env python3
"""
Test script for abort mission functionality
"""

import tkinter as tk
from ktom_experimenter import KToMExperimenterGUI

def test_abort_functionality():
    """Test the abort mission functionality"""
    print("🧪 Testing Abort Mission Functionality")
    print("=" * 40)
    
    # Create GUI (don't show it)
    root = tk.Tk()
    root.withdraw()
    app = KToMExperimenterGUI(root)
    
    # Test abort mission method exists
    if hasattr(app, 'on_abort_mission'):
        print("✅ on_abort_mission method exists")
    else:
        print("❌ on_abort_mission method missing")
        return False
    
    # Test kill_robot_processes method exists
    if hasattr(app, 'kill_robot_processes'):
        print("✅ kill_robot_processes method exists")
    else:
        print("❌ kill_robot_processes method missing")
        return False
    
    # Test abort button exists
    if hasattr(app, 'abort_button'):
        print("✅ abort_button exists")
    else:
        print("❌ abort_button missing")
        return False
    
    # Test abort status label exists
    if hasattr(app, 'abort_status_label'):
        print("✅ abort_status_label exists")
    else:
        print("❌ abort_status_label missing")
        return False
    
    # Test keyboard shortcuts are bound
    try:
        # Check if Escape key is bound
        bindings = root.bind_all()
        if '<Escape>' in str(bindings):
            print("✅ Escape key shortcut bound")
        else:
            print("⚠️ Escape key shortcut not found")
    except:
        print("⚠️ Could not verify keyboard shortcuts")
    
    root.destroy()
    print("\n✅ Abort mission functionality test completed!")
    return True

if __name__ == "__main__":
    test_abort_functionality()
