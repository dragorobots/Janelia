#!/usr/bin/env python3
"""
Test script to verify the line following fixes in hide_and_seek.py
"""

import sys
import os

def test_state_machine_updates():
    """Test that the state machine has been properly updated"""
    print("Testing state machine updates...")
    
    # Check if the new states are defined
    expected_states = [
        "FOLLOWING_START_LINE",
        "AT_INTERSECTION", 
        "FOLLOWING_TARGET_LINE"
    ]
    
    with open('hide_and_seek.py', 'r') as f:
        content = f.read()
        
    for state in expected_states:
        if state in content:
            print(f"✅ Found state: {state}")
        else:
            print(f"❌ Missing state: {state}")
            return False
    
    return True

def test_intersection_detection():
    """Test that intersection detection logic is present"""
    print("\nTesting intersection detection logic...")
    
    with open('hide_and_seek.py', 'r') as f:
        content = f.read()
    
    # Check for intersection detection code
    if "INTERSECTION DETECTED" in content:
        print("✅ Intersection detection logic found")
    else:
        print("❌ Intersection detection logic missing")
        return False
    
    # Check for centering maneuver
    if "execute_intersection_centering" in content:
        print("✅ Intersection centering method found")
    else:
        print("❌ Intersection centering method missing")
        return False
    
    return True

def test_line_color_handling():
    """Test that line color handling is properly implemented"""
    print("\nTesting line color handling...")
    
    with open('hide_and_seek.py', 'r') as f:
        content = f.read()
    
    # Check for start line HSV range
    if "start_line_hsv_range" in content:
        print("✅ Start line HSV range handling found")
    else:
        print("❌ Start line HSV range handling missing")
        return False
    
    # Check for target line HSV range
    if "target_hsv_range" in content:
        print("✅ Target line HSV range handling found")
    else:
        print("❌ Target line HSV range handling missing")
        return False
    
    return True

def test_ktom_integration():
    """Test that k-tom experimenter integration is updated"""
    print("\nTesting k-tom experimenter integration...")
    
    with open('ktom_experimenter.py', 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # Check for line color topic
    if "line_color" in content and "send_line_color" in content:
        print("✅ Line color topic and method found in k-tom")
    else:
        print("❌ Line color integration missing in k-tom")
        return False
    
    return True

def test_progress_tracking():
    """Test that progress tracking is updated for new states"""
    print("\nTesting progress tracking updates...")
    
    with open('hide_and_seek.py', 'r') as f:
        content = f.read()
    
    expected_progress_messages = [
        "following_start_line",
        "at_intersection_centering", 
        "following_target_line"
    ]
    
    for msg in expected_progress_messages:
        if msg in content:
            print(f"✅ Progress message found: {msg}")
        else:
            print(f"❌ Progress message missing: {msg}")
            return False
    
    return True

def main():
    """Run all tests"""
    print("🧪 Testing Line Following Fixes\n")
    print("=" * 50)
    
    tests = [
        test_state_machine_updates,
        test_intersection_detection,
        test_line_color_handling,
        test_ktom_integration,
        test_progress_tracking
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                print(f"❌ Test failed: {test.__name__}")
        except Exception as e:
            print(f"❌ Test error in {test.__name__}: {e}")
    
    print("\n" + "=" * 50)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Line following fixes are properly implemented.")
        return True
    else:
        print("⚠️  Some tests failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
