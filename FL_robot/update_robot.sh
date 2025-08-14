#!/usr/bin/env bash
set -e
cd /root/yahboomcar_ws/src/Janelia && git pull
cd /root/yahboomcar_ws
colcon build --symlink-install
source install/setup.bash
echo "[OK] Updated and built."
