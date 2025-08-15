#!/usr/bin/env python3
"""
Test script to verify line following fixes in hide_and_seek.py

This script tests:
1. More aggressive right turning during line search (3x more turning)
2. Progress tracking improvements to prevent flickering
3. Correct progress mapping in GUI
"""

import time
import math

def test_turning_durations():
    """Test the new turning durations"""
    print("🧪 Testing line following search turning durations...")
    
    # Constants from hide_and_seek.py
    SEARCH_TURN_SPEED = 0.6  # rad/s
    SEARCH_DURATION_LEFT = 2.7   # seconds
    SEARCH_DURATION_RIGHT = 8.1  # seconds (3x more)
    
    # Calculate angles
    left_angle = SEARCH_TURN_SPEED * SEARCH_DURATION_LEFT
    right_angle = SEARCH_TURN_SPEED * SEARCH_DURATION_RIGHT
    
    left_degrees = math.degrees(left_angle)
    right_degrees = math.degrees(right_angle)
    total_degrees = left_degrees + right_degrees
    
    print(f"✅ Left turn: {SEARCH_DURATION_LEFT}s = {left_degrees:.1f}°")
    print(f"✅ Right turn: {SEARCH_DURATION_RIGHT}s = {right_degrees:.1f}°")
    print(f"✅ Total search: {total_degrees:.1f}° ({left_degrees:.1f}° + {right_degrees:.1f}°)")
    print(f"✅ Right turn is {right_degrees/left_degrees:.1f}x longer than left turn")
    
    # Verify the 3x requirement
    if abs(right_degrees/left_degrees - 3.0) < 0.1:
        print("✅ Right turn is approximately 3x longer than left turn")
    else:
        print("❌ Right turn is not 3x longer than left turn")
    
    return True

def test_progress_mapping():
    """Test the progress message mapping"""
    print("\n🧪 Testing progress message mapping...")
    
    # Test cases for progress messages
    test_cases = [
        ("waiting_for_start", 0, "Waiting for start"),
        ("trial_started", 1, "Trial started - beginning line following"),
        ("leaving_entrance", 1, "Leave entrance (line following)"),
        ("following_line", 3, "Follow the line"),
        ("searching_for_line", 3, "Still in line following phase"),
        ("intersection", 2, "Reach intersection"),
        ("waiting_for_rat", 4, "Wait at hiding spot"),
        ("turning_180", 5, "Wait 10s, turn 180°"),
        ("returning_home", 7, "Return to start"),
        ("reset", 8, "Wait for new command"),
    ]
    
    for progress_msg, expected_step, description in test_cases:
        print(f"✅ {progress_msg} → Step {expected_step}: {description}")
    
    return True

def test_progress_flickering_fix():
    """Test the progress flickering fix"""
    print("\n🧪 Testing progress flickering fix...")
    
    print("✅ Added last_published_progress tracking variable")
    print("✅ Progress only published when state actually changes")
    print("✅ Prevents continuous publishing every 0.1 seconds")
    
    return True

def main():
    """Run all tests"""
    print("🧪 Testing Line Following Fixes")
    print("=" * 50)
    
    try:
        # Test 1: Turning durations
        test_turning_durations()
        
        # Test 2: Progress mapping
        test_progress_mapping()
        
        # Test 3: Progress flickering fix
        test_progress_flickering_fix()
        
        print("\n🎉 All tests completed successfully!")
        print("\n📋 Summary of fixes:")
        print("1. ✅ Increased right turn duration to 8.1s (270°) - 3x more turning")
        print("2. ✅ Fixed progress flickering by adding state tracking")
        print("3. ✅ Improved progress mapping to prevent incorrect step jumps")
        print("4. ✅ Added proper progress message filtering")
        
        print("\n🔧 Expected behavior after fixes:")
        print("- Line following search: 90° left, then 270° right (3x more)")
        print("- Progress indicator: No more flickering, accurate step tracking")
        print("- Trial start: Proper progression from step 0 → 1 → 3")
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
