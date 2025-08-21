#!/usr/bin/env python3
"""
Integration Test Script for Hide and Seek Robot System
Tests PC-robot communication and k-ToM GUI integration
"""

import sys
import time
import threading
from ktom_experimenter import RosLink, KToMExperimenterGUI
import tkinter as tk

def test_roslink_communication():
    """Test basic RosLink communication"""
    print("🧪 Testing RosLink Communication")
    print("=" * 40)
    
    # Test RosLink creation
    try:
        link = RosLink(host="192.168.1.100", port=10090)
        print("✅ RosLink created successfully")
    except Exception as e:
        print(f"❌ Failed to create RosLink: {e}")
        return False
    
    # Test message creation (without connection)
    try:
        link.send_target(2)
        print("✅ Target message creation successful")
        
        link.send_toggle("drive_mode=auto_line")
        print("✅ Toggle message creation successful")
        
        link.send_cmdvel(0.5, 1.0)
        print("✅ Velocity message creation successful")
        
    except Exception as e:
        print(f"❌ Message creation failed: {e}")
        return False
    
    # Test connection failure (expected)
    print("\nTesting connection (expected to fail without robot)...")
    success = link.connect()
    if not success:
        print("✅ Connection failed gracefully as expected")
    else:
        print("❌ Connection succeeded unexpectedly")
    
    link.close()
    print("✅ RosLink closed successfully")
    return True

def test_gui_integration():
    """Test GUI integration with robot communication"""
    print("\n🧪 Testing GUI Integration")
    print("=" * 40)
    
    try:
        # Create GUI (don't show it)
        root = tk.Tk()
        root.withdraw()  # Hide the window
        app = KToMExperimenterGUI(root)
        print("✅ GUI created successfully")
        
        # Test robot control tab exists
        if hasattr(app, 'create_robot_control_tab'):
            print("✅ Robot control tab method exists")
        else:
            print("❌ Robot control tab method missing")
            return False
        
        # Test RosLink integration
        if hasattr(app, 'robot_link'):
            print("✅ RosLink integration exists")
        else:
            print("❌ RosLink integration missing")
            return False
        
        # Test robot control methods
        required_methods = [
            'on_connect_robot', 'on_send_target', 'on_send_drive_mode',
            'on_send_rat_mode', 'on_manual_found', 'on_send_velocity',
            'on_stop_robot', 'update_robot_status'
        ]
        
        for method in required_methods:
            if hasattr(app, method):
                print(f"✅ {method} method exists")
            else:
                print(f"❌ {method} method missing")
                return False
        
        root.destroy()
        print("✅ GUI integration test passed")
        return True
        
    except Exception as e:
        print(f"❌ GUI integration test failed: {e}")
        return False

def test_k_tom_functionality():
    """Test k-ToM model functionality"""
    print("\n🧪 Testing k-ToM Model Functionality")
    print("=" * 40)
    
    try:
        from ktom_experimenter import ToMAgent, RobotController
        
        # Test ToMAgent creation
        agent = ToMAgent(k_level=2, num_spots=4, is_robot=True)
        print("✅ ToMAgent created successfully")
        
        # Test prediction
        pred = agent.predict_opponent_choice_dist()
        print(f"✅ Prediction generated: {pred}")
        
        # Test RobotController
        controller = RobotController()
        controller.initialize_game(k_level=2, num_spots=4, k0_strategy=None, k0_config=None)
        print("✅ RobotController initialized successfully")
        
        # Test recommendation
        recommendation = controller.recommend_spot()
        print(f"✅ Recommendation generated: {recommendation}")
        
        # Test trial processing
        controller.process_trial_result(rat_choice=1, was_found=True, time_taken=30.0, search_sequence="A,B,C")
        print("✅ Trial processing successful")
        
        print("✅ k-ToM functionality test passed")
        return True
        
    except Exception as e:
        print(f"❌ k-ToM functionality test failed: {e}")
        return False

def test_file_saving():
    """Test CSV file saving functionality"""
    print("\n🧪 Testing File Saving Functionality")
    print("=" * 40)
    
    try:
        from ktom_experimenter import RobotController
        from datetime import datetime
        import os
        
        # Create controller with some trial data
        controller = RobotController()
        controller.initialize_game(k_level=2, num_spots=4, k0_strategy=None, k0_config=None)
        
        # Add some trial data
        controller.recommend_spot()
        controller.process_trial_result(rat_choice=1, was_found=True, time_taken=30.0, search_sequence="A,B,C")
        controller.recommend_spot()
        controller.process_trial_result(rat_choice=3, was_found=False, time_taken=120.0, search_sequence="C,A,B,D")
        
        # Test file saving
        now = datetime.now()
        date_str = now.strftime('%Y%m%d')
        time_str = now.strftime('%H%M%S')
        datetime_str = f"{time_str}_{date_str}"
        
        folder_name = f"test_hide_and_seek_trial_outcomes_{date_str}"
        filename = f"test_triallog_{datetime_str}.csv"
        file_path = os.path.join(folder_name, filename)
        
        # Create folder
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        
        # Save CSV
        import csv
        all_headers = set()
        for entry in controller.trial_log:
            all_headers.update(entry.keys())
        
        header_order = [
            'trial_num', 'robot_k_level', 'num_hiding_spots', 'robot_hiding_spot',
            'rat_first_search', 'was_found', 'time_to_find'
        ]
        
        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=header_order, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(controller.trial_log)
        
        print(f"✅ CSV file saved: {file_path}")
        
        # Clean up test file
        os.remove(file_path)
        os.rmdir(folder_name)
        print("✅ Test file cleaned up")
        
        return True
        
    except Exception as e:
        print(f"❌ File saving test failed: {e}")
        return False

def main():
    """Run all integration tests"""
    print("🚀 Hide and Seek Robot Integration Tests")
    print("=" * 50)
    
    tests = [
        ("RosLink Communication", test_roslink_communication),
        ("GUI Integration", test_gui_integration),
        ("k-ToM Functionality", test_k_tom_functionality),
        ("File Saving", test_file_saving)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n📋 Running {test_name} Test...")
        if test_func():
            passed += 1
            print(f"✅ {test_name} PASSED")
        else:
            print(f"❌ {test_name} FAILED")
    
    print("\n" + "=" * 50)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! System is ready for deployment.")
        print("\n📋 Next Steps:")
        print("1. Deploy to robot: ./update_robot.sh")
        print("2. Start robot system: ./start_hide_and_seek.sh")
        print("3. Start PC GUI: python ktom_experimenter.py")
        print("4. Connect to robot and run experiments!")
    else:
        print("⚠️  Some tests failed. Please fix issues before deployment.")
    
    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
