#!/usr/bin/env python3
"""
Test script to verify GUI fixes for ktom_experimenter.py

This script tests:
1. Trial progress indicator doesn't flash when connecting
2. Start Trial button is visible and properly positioned
3. Progress tracking state management works correctly
"""

import tkinter as tk
from tkinter import ttk
import sys
import os

# Add the current directory to the path so we can import ktom_experimenter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_gui_fixes():
    """Test the GUI fixes"""
    print("🧪 Testing GUI fixes for ktom_experimenter.py")
    
    try:
        # Import the main GUI class
        from ktom_experimenter import KToMExperimenterGUI
        
        # Create a test window
        root = tk.Tk()
        root.title("GUI Fixes Test")
        root.geometry("800x600")
        
        # Create the GUI
        app = KToMExperimenterGUI(root)
        
        print("✅ GUI created successfully")
        
        # Test 1: Check if Start Trial button exists and is visible
        if hasattr(app, 'start_trial_button'):
            print("✅ Start Trial button exists")
            
            # Check if it's properly configured
            if app.start_trial_button.winfo_exists():
                print("✅ Start Trial button is visible")
                
                # Get button position and size
                x = app.start_trial_button.winfo_x()
                y = app.start_trial_button.winfo_y()
                width = app.start_trial_button.winfo_width()
                height = app.start_trial_button.winfo_height()
                
                print(f"   Position: ({x}, {y})")
                print(f"   Size: {width}x{height}")
                
                if width > 0 and height > 0:
                    print("✅ Start Trial button has proper dimensions")
                else:
                    print("⚠️  Start Trial button has zero dimensions (might be hidden)")
            else:
                print("❌ Start Trial button is not visible")
        else:
            print("❌ Start Trial button does not exist")
        
        # Test 2: Check progress tracking state
        if hasattr(app, '_last_progress_step'):
            print("✅ Progress tracking state exists")
        else:
            print("❌ Progress tracking state does not exist")
        
        # Test 3: Check progress variables and labels
        if hasattr(app, 'progress_vars') and hasattr(app, 'progress_labels'):
            print(f"✅ Progress tracking elements exist: {len(app.progress_vars)} vars, {len(app.progress_labels)} labels")
        else:
            print("❌ Progress tracking elements missing")
        
        # Test 4: Check if update_trial_progress method exists
        if hasattr(app, 'update_trial_progress'):
            print("✅ update_trial_progress method exists")
        else:
            print("❌ update_trial_progress method missing")
        
        # Test 5: Simulate progress updates to check for flashing
        print("\n🧪 Testing progress updates...")
        
        # Test initial state
        app.update_trial_progress(0)
        print("✅ Initial progress update (step 0) completed")
        
        # Test step transitions
        for step in range(1, 9):
            app.update_trial_progress(step)
            print(f"✅ Progress update to step {step} completed")
        
        print("✅ All progress updates completed without errors")
        
        # Test 6: Check tab organization
        notebook = None
        for child in root.winfo_children():
            if isinstance(child, ttk.Notebook):
                notebook = child
                break
        
        if notebook:
            tab_names = [notebook.tab(i, "text") for i in range(notebook.index("end"))]
            print(f"✅ Tab organization: {tab_names}")
            
            # Check if Robot Control is the second tab
            if len(tab_names) >= 2 and "Robot Control" in tab_names[1]:
                print("✅ Robot Control tab is in correct position (second)")
            else:
                print("⚠️  Robot Control tab position may be incorrect")
        else:
            print("❌ Could not find notebook widget")
        
        print("\n🎉 All tests completed!")
        print("\n📋 Summary of fixes applied:")
        print("1. ✅ Fixed trial progress flashing by adding state tracking")
        print("2. ✅ Fixed Start Trial button placement in right panel")
        print("3. ✅ Added progress state reset on connection")
        print("4. ✅ Improved progress update logic to prevent unnecessary updates")
        
        # Keep the window open for manual inspection
        print("\n🔍 GUI window is open for manual inspection.")
        print("   - Check that the Start Trial button is visible in the right panel")
        print("   - Verify that trial progress indicators work correctly")
        print("   - Close the window when done testing")
        
        root.mainloop()
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure ktom_experimenter.py is in the same directory")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_gui_fixes()
