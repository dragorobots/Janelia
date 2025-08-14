#!/bin/bash

echo "🔥 FORCE UPDATE - Overwriting all local changes..."

# Navigate to the robot directory
cd /root/yahboomcar_ws/src/Janelia/FL_robot

# Force reset any local changes
echo "Resetting local changes..."
git reset --hard HEAD

# Clean any untracked files (optional - be careful!)
echo "Cleaning untracked files..."
git clean -fd

# Pull latest changes
echo "Pulling latest changes..."
git pull

# Rebuild the workspace
echo "Rebuilding workspace..."
cd /root/yahboomcar_ws
colcon build --symlink-install

# Source the setup
echo "Sourcing setup..."
source install/setup.bash

echo "✅ Force update complete!"
echo "All local changes have been overwritten with the latest version."
