# Yahboom ROS2 Robot Platform Setup

This repository contains the full calibration, testing, and custom tooling used for a Yahboom MicroROS-Pi5 robot.

It includes:
- Full ROS2 workspace with all required Yahboom packages
- Custom calibration and verification scripts
- A first-time setup script to walk new users through system checks

## What's Included

- `yahboomcar_ws/src/`: ROS2 packages (e.g. `yahboomcar_ctrl`, `robot_localization`, `calibration_tools`, etc.)
- `yahboomcar_ws/startup/`: Diagnostic scripts and `first_time_setup.sh`
- `.gitignore`: Prevents build and install files from being committed

## Getting Started

See `setup_instructions.md` for full instructions on building and testing this code on a new robot.
