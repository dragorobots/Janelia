#!/bin/bash

source ~/yahboomcar_ws/install/setup.bash

echo "---------------------------------------------"
echo "???  Yahboom Robot First-Time Calibration Tool"
echo "---------------------------------------------"

while true; do
  echo ""
  echo "Choose a test to run:"
  echo "  1) Check Camera"
  echo "  2) Check LiDAR"
  echo "  3) Check IMU"
  echo "  4) Motor Control (Keyboard)"
  echo "  5) Exit"
  read -p "Enter choice [1-5]: " choice

  case $choice in
    1)
      echo "Launching camera test..."
      python3 ~/yahboomcar_ws/startup/check_camera.py
      ;;
    2)
      echo "Launching LiDAR test..."
      python3 ~/yahboomcar_ws/startup/check_lidar.py
      ;;
    3)
      echo "Launching IMU test..."
      python3 ~/yahboomcar_ws/startup/check_imu.py
      ;;
    4)
      echo "Starting keyboard motor control..."
      ros2 run teleop_twist_keyboard teleop_twist_keyboard
      ;;
    5)
      echo "Exiting. Bye!"
      break
      ;;
    *)
      echo "Invalid choice. Please enter a number between 1 and 5."
      ;;
  esac
done
