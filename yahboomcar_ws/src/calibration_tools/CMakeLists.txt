cmake_minimum_required(VERSION 3.5)
project(calibration_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    calibration_tools/calibration_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
