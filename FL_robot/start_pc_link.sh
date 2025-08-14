#!/usr/bin/env bash
set -e
pkill -f rosbridge_websocket || true
pkill -f rosapi_node || true
pkill -f hide_and_seek_bridge.py || true
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=10090 &
sleep 1
python3 /root/yahboomcar_ws/src/Janelia/FL_robot/hide_and_seek_bridge.py &
echo "[OK] rosbridge on :10090 and hide_and_seek_bridge running."
