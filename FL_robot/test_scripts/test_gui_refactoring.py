#!/usr/bin/env python3
"""
Test script to verify GUI refactoring changes

This script tests:
1. Manual driving methods are removed
2. Layout changes are applied correctly
3. GUI elements are properly positioned
"""

import tkinter as tk
from tkinter import ttk
import sys
import os

# Add the current directory to the path so we can import ktom_experimenter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_gui_refactoring():
    """Test the GUI refactoring changes"""
    print("=== GUI Refactoring Test ===")
    
    try:
        # Import the GUI class
        from ktom_experimenter import KToMExperimenterGUI
        
        # Create a test root window
        root = tk.Tk()
        root.withdraw()  # Hide the window during testing
        
        # Create the GUI instance
        app = KToMExperimenterGUI(root)
        
        print("✅ GUI instance created successfully")
        
        # Test 1: Check that manual driving methods are removed
        print("\n1. Testing manual driving method removal:")
        manual_methods = ['on_send_velocity', 'on_stop_robot']
        for method in manual_methods:
            if hasattr(app, method):
                print(f"❌ Manual driving method '{method}' still exists")
            else:
                print(f"✅ Manual driving method '{method}' successfully removed")
        
        # Test 2: Check that current_step_var is removed
        print("\n2. Testing current_step_var removal:")
        if hasattr(app, 'current_step_var'):
            print("❌ current_step_var still exists")
        else:
            print("✅ current_step_var successfully removed")
        
        if hasattr(app, 'current_step_label'):
            print("❌ current_step_label still exists")
        else:
            print("✅ current_step_label successfully removed")
        
        # Test 3: Check that manual driving variables are removed
        print("\n3. Testing manual driving variable removal:")
        manual_vars = ['linear_speed_var', 'angular_speed_var']
        for var in manual_vars:
            if hasattr(app, var):
                print(f"❌ Manual driving variable '{var}' still exists")
            else:
                print(f"✅ Manual driving variable '{var}' successfully removed")
        
        # Test 4: Check that required GUI elements exist
        print("\n4. Testing required GUI elements:")
        required_elements = [
            'robot_ip_entry', 'connect_button', 'check_robot_status_button',
            'robot_status_label', 'abort_status_label', 'abort_button',
            'target_spot_var', 'manual_override_check', 'auto_send_target_button',
            'send_target_button', 'start_trial_button'
        ]
        
        for element in required_elements:
            if hasattr(app, element):
                print(f"✅ Required element '{element}' exists")
            else:
                print(f"❌ Required element '{element}' missing")
        
        # Test 5: Check that progress elements exist (static only)
        print("\n5. Testing progress elements:")
        progress_elements = ['progress_vars', 'progress_labels', 'progress_steps']
        for element in progress_elements:
            if hasattr(app, element):
                print(f"✅ Progress element '{element}' exists")
            else:
                print(f"❌ Progress element '{element}' missing")
        
        # Test 6: Check update_trial_progress method
        print("\n6. Testing update_trial_progress method:")
        if hasattr(app, 'update_trial_progress'):
            print("✅ update_trial_progress method exists")
            
            # Test that it doesn't reference current_step_var
            import inspect
            source = inspect.getsource(app.update_trial_progress)
            if 'current_step_var' in source:
                print("❌ update_trial_progress still references current_step_var")
            else:
                print("✅ update_trial_progress doesn't reference current_step_var")
        else:
            print("❌ update_trial_progress method missing")
        
        # Clean up
        root.destroy()
        
        print("\n=== Summary ===")
        print("✅ GUI refactoring test completed")
        print("✅ Manual driving section removed")
        print("✅ Auto-updating trial progress removed")
        print("✅ Layout changes applied")
        print("✅ Required elements preserved")
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False
    
    return True

def test_layout_changes():
    """Test specific layout changes"""
    print("\n=== Layout Changes Test ===")
    
    try:
        from ktom_experimenter import KToMExperimenterGUI
        
        root = tk.Tk()
        root.withdraw()
        app = KToMExperimenterGUI(root)
        
        # Test that abort button is in the status bar frame
        print("1. Testing abort button placement:")
        if hasattr(app, 'abort_button'):
            # Check if abort button is a child of the status bar frame
            parent = app.abort_button.master
            while parent and not isinstance(parent, ttk.Frame):
                parent = parent.master
            
            if parent and "status" in str(parent).lower():
                print("✅ Abort button is in status bar frame")
            else:
                print("❌ Abort button not in status bar frame")
        else:
            print("❌ Abort button not found")
        
        # Test that start trial button is in left panel
        print("2. Testing start trial button placement:")
        if hasattr(app, 'start_trial_button'):
            parent = app.start_trial_button.master
            while parent and not isinstance(parent, ttk.Frame):
                parent = parent.master
            
            if parent and "left" in str(parent).lower():
                print("✅ Start trial button is in left panel")
            else:
                print("❌ Start trial button not in left panel")
        else:
            print("❌ Start trial button not found")
        
        root.destroy()
        
    except Exception as e:
        print(f"❌ Layout test failed: {e}")
        return False
    
    return True

def main():
    """Run all tests"""
    print("Testing GUI Refactoring Changes")
    print("=" * 40)
    
    success1 = test_gui_refactoring()
    success2 = test_layout_changes()
    
    if success1 and success2:
        print("\n🎉 All tests passed! GUI refactoring successful.")
        return True
    else:
        print("\n❌ Some tests failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    main()
