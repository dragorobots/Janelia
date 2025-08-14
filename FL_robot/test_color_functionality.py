#!/usr/bin/env python3
"""
Test script for color measurement and line color configuration functionality
"""

import tkinter as tk
from ktom_experimenter import KToMExperimenterGUI

def test_color_functionality():
    """Test the color measurement and line color configuration functionality"""
    print("🎨 Testing Color Functionality")
    print("=" * 40)
    
    # Create GUI (don't show it)
    root = tk.Tk()
    root.withdraw()
    app = KToMExperimenterGUI(root)
    
    # Test color validation method exists
    if hasattr(app, 'validate_rgb_color'):
        print("✅ validate_rgb_color method exists")
    else:
        print("❌ validate_rgb_color method missing")
        return False
    
    # Test get_line_colors method exists
    if hasattr(app, 'get_line_colors'):
        print("✅ get_line_colors method exists")
    else:
        print("❌ get_line_colors method missing")
        return False
    
    # Test open_color_measurer method exists
    if hasattr(app, 'open_color_measurer'):
        print("✅ open_color_measurer method exists")
    else:
        print("❌ open_color_measurer method missing")
        return False
    
    # Test predefined colors toggle method exists
    if hasattr(app, 'on_predefined_colors_toggle'):
        print("✅ on_predefined_colors_toggle method exists")
    else:
        print("❌ on_predefined_colors_toggle method missing")
        return False
    
    # Test color validation with valid colors
    print("\nTesting color validation:")
    valid_colors = ["255,0,0", "0,255,0", "0,0,255", "255,255,255", "0,0,0"]
    for color in valid_colors:
        try:
            app.validate_rgb_color(color)
            print(f"✅ Valid: '{color}'")
        except ValueError as e:
            print(f"❌ Invalid: '{color}' - {e}")
    
    # Test color validation with invalid colors
    invalid_colors = ["255,0", "255,0,0,0", "256,0,0", "-1,0,0", "abc,def,ghi"]
    print("\nTesting invalid colors:")
    for color in invalid_colors:
        try:
            app.validate_rgb_color(color)
            print(f"❌ Should be invalid: '{color}'")
        except ValueError as e:
            print(f"✅ Correctly rejected: '{color}' - {e}")
    
    # Test line color variables exist
    color_vars = [
        'use_predefined_colors_var',
        'start_color_var',
        'line_a_color_var',
        'line_b_color_var',
        'line_c_color_var',
        'line_d_color_var'
    ]
    
    print("\nTesting line color variables:")
    for var_name in color_vars:
        if hasattr(app, var_name):
            print(f"✅ {var_name} exists")
        else:
            print(f"❌ {var_name} missing")
    
    # Test color measurer button exists
    if hasattr(app, 'color_measurer_button'):
        print("✅ color_measurer_button exists")
    else:
        print("❌ color_measurer_button missing")
    
    root.destroy()
    print("\n✅ Color functionality test completed!")
    return True

if __name__ == "__main__":
    test_color_functionality()
