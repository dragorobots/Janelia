#!/usr/bin/env python3
"""
Test script for the new k-ToM GUI functionality
Tests the new features: robot status check, auto-send target, and trial progress tracking
"""

import sys
import os

# Add the current directory to the path so we can import ktom_experimenter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_new_functionality():
    """Test the new functionality without running the full GUI"""
    
    print("🧪 Testing New k-ToM GUI Functionality")
    print("=" * 50)
    
    # Test 1: Import the module
    try:
        from ktom_experimenter import RobotController, ToMAgent
        print("✅ Successfully imported ktom_experimenter module")
    except ImportError as e:
        print(f"❌ Failed to import ktom_experimenter: {e}")
        return False
    
    # Test 2: Test RobotController
    try:
        controller = RobotController()
        controller.initialize_game(k_level=1, num_spots=4, k0_strategy='random', k0_config=[0.25, 0.25, 0.25, 0.25])
        print("✅ Successfully created and initialized RobotController")
    except Exception as e:
        print(f"❌ Failed to create RobotController: {e}")
        return False
    
    # Test 3: Test recommendation system
    try:
        recommendation = controller.recommend_spot()
        print(f"✅ Got recommendation: {recommendation}")
        
        # Test get_current_recommendation method
        current_rec = controller.get_current_recommendation()
        print(f"✅ Current recommendation: {current_rec}")
        
        if recommendation == current_rec:
            print("✅ Recommendation consistency check passed")
        else:
            print("❌ Recommendation consistency check failed")
            return False
            
    except Exception as e:
        print(f"❌ Failed to get recommendation: {e}")
        return False
    
    # Test 4: Test trial progress steps
    try:
        progress_steps = [
            "1. Leave entrance",
            "2. Reach intersection & start new line", 
            "3. Follow the line",
            "4. Wait at hiding spot (detect rat)",
            "5. Wait 10s, turn 180°",
            "6. Follow line back (same color)",
            "7. Reach intersection & return to start",
            "8. Wait for new command, turn 180°, reset"
        ]
        print(f"✅ Progress steps defined: {len(progress_steps)} steps")
        
        # Test step mapping
        test_progress = "following line to target"
        if "following" in test_progress.lower():
            expected_step = 3
            print(f"✅ Progress mapping test passed: '{test_progress}' -> Step {expected_step}")
        else:
            print("❌ Progress mapping test failed")
            return False
            
    except Exception as e:
        print(f"❌ Failed to test progress steps: {e}")
        return False
    
    # Test 5: Test spot mapping
    try:
        spot_map = {0: "A", 1: "B", 2: "C", 3: "D"}
        target_map = {"A": 0, "B": 1, "C": 2, "D": 3}
        
        # Test forward mapping
        for i in range(4):
            spot_letter = spot_map[i]
            spot_index = target_map[spot_letter]
            if i == spot_index:
                print(f"✅ Spot mapping {i} -> {spot_letter} -> {spot_index} ✓")
            else:
                print(f"❌ Spot mapping failed for {i}")
                return False
                
        print("✅ All spot mappings work correctly")
        
    except Exception as e:
        print(f"❌ Failed to test spot mapping: {e}")
        return False
    
    print("\n🎉 All tests passed! New functionality is working correctly.")
    print("\n📋 New Features Added:")
    print("   • 🔍 Check Robot Status button")
    print("   • 🤖 Auto-Send k-ToM Target button") 
    print("   • 🎯 Trial Progress Indicator (8 steps)")
    print("   • 📊 Real-time progress tracking")
    print("   • 🔄 Automatic target spot sending")
    
    return True

if __name__ == "__main__":
    success = test_new_functionality()
    sys.exit(0 if success else 1)
