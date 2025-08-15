# FL_robot — Dev & Live Link Guide

This folder sets up two things for the Yahboom car(s):
1) **Git workflow** — edit on your PC, push to GitHub, pull/rebuild on any robot.
2) **Live link** — your PC app talks to the robot over Wi‑Fi via rosbridge (WebSocket).

---

## Repo layout (relevant)

```
Janelia/
  FL_robot/
    hide_and_seek.py              # your working script (symlinked as working dir)
    hide_and_seek_bridge.py       # robot-side ROS 2 bridge node (PC <-> robot topics)
    update_robot.sh               # pull from GitHub + colcon build + source
    start_pc_link.sh              # start rosbridge (:10090) + bridge node
    README.md
    CHANGELOG.md
```

Robot working path is symlinked to this repo folder:

```
/root/yahboomcar_ws/src/yahboomcar_astra/hide_and_seek  ->  /root/yahboomcar_ws/src/Janelia/FL_robot
```

---

## Daily workflow

### On your Windows PC (cmd.exe)
Edit files in:
```
C:\Users\andyc\Code\Janelia\Janelia\FL_robot\
```
Then:
```bat
cd C:\Users\andyc\Code\Janelia\Janelia
git add FL_robot\*
git commit -m "Your message"
git push
```

### On any robot (inside the Docker container)
```bash
/root/yahboomcar_ws/src/Janelia/FL_robot/update_robot.sh
```
This pulls latest, rebuilds (`colcon build --symlink-install`), and sources the workspace.

---

## Live PC ↔ Robot link (rosbridge on **port 10090**)

### Start it on the robot (container)
```bash
/root/yahboomcar_ws/src/Janelia/FL_robot/start_pc_link.sh
# Starts: rosbridge websocket on :10090 + hide_and_seek_bridge.py
```

### Minimal test from the PC
```bat
python - <<^PY
import roslibpy, time
ros = roslibpy.Ros(host='<ROBOT_IP>', port=10090)  # e.g., 10.0.0.234
ros.run(); print('connected?', ros.is_connected)
t = roslibpy.Topic(ros, '/hide_and_seek/target_spot', 'std_msgs/msg/Int32')
t.advertise(); t.publish(roslibpy.Message({'data': 3})); time.sleep(0.5)
t.unadvertise(); ros.terminate()
^PY
```

**Robot should log (from bridge):** `target_spot=3`

### Topics

**PC → Robot**
- `/hide_and_seek/target_spot` (`std_msgs/msg/Int32`)
- `/hide_and_seek/toggles` (`std_msgs/msg/String`) — e.g. `line_follow=on;rat_detect=off`
- `/hide_and_seek/cmd_vel` (`geometry_msgs/msg/Twist`)

**Robot → PC**
- `/line_follow/status` (`std_msgs/msg/String`)
- `/rat_detection/found` (`std_msgs/msg/Bool`)
- `/hide_and_seek/progress` (`std_msgs/msg/String`)

---

## First-time setup on a **new robot** (Robot B)

1) **SSH key for GitHub (unique per robot)**
```bash
ssh-keygen -t ed25519 -C "robot-b" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub
```
Add the printed key in GitHub: **Settings → SSH and GPG keys → New SSH key**.

2) **Clone & link**
```bash
cd /root/yahboomcar_ws/src
git clone git@github.com:dragorobots/Janelia.git
mv /root/yahboomcar_ws/src/yahboomcar_astra/hide_and_seek \
   /root/yahboomcar_ws/src/yahboomcar_astra/hide_and_seek.bak.$(date +%F-%H%M%S) 2>/dev/null || true
ln -s /root/yahboomcar_ws/src/Janelia/FL_robot \
      /root/yahboomcar_ws/src/yahboomcar_astra/hide_and_seek
```

3) **Build & start live link**
```bash
/root/yahboomcar_ws/src/Janelia/FL_robot/update_robot.sh
/root/yahboomcar_ws/src/Janelia/FL_robot/start_pc_link.sh
```

4) **PC connects to new robot IP**
Use `Ros(host='<ROBOT_B_IP>', port=10090)`.

> **Recommended:** start the container with `--network=host` to avoid port mapping/tunnels.
> ```
> docker run -d --name ros2_robot --network=host \
>   -v /root/yahboomcar_ws:/root/yahboomcar_ws \
>   <your-image> bash -lc "sleep infinity"
> ```

---

## Things that will differ per robot

- **Robot IP address** (PC must use the correct `host='<robot_ip>'`).
- **SSH key** (each robot uses a unique SSH key added to GitHub).
- **Container name** (we use `ros2_robot` in examples; yours may differ).
- **Networking mode** (prefer `--network=host`; if using bridge, ensure port 10090 is reachable or use an SSH tunnel).
- **DNS/Apt state** (some images need the ROS apt key refreshed; see troubleshooting).

---

## Troubleshooting

- **PC connects but robot logs nothing**
  - Use ROS 2 type strings (`std_msgs/msg/Int32`) and call `advertise()` before `publish()`.
  - Make sure **only one** rosbridge is running (we standardize on port **10090**).

- **Port already in use**
  ```bash
  pkill -f rosbridge_websocket || true
  pkill -f rosapi_node || true
  /root/yahboomcar_ws/src/Janelia/FL_robot/start_pc_link.sh
  ```

- **Build error “cannot create symlink … Is a directory”**
  ```
  rm -rf build/<pkg> install/<pkg> log
  colcon build --symlink-install
  ```

- **ROS apt key/index warnings**
  Refresh ROS key & list:
  ```bash
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list
  apt update
  ```

---

## CHANGELOG

Keep operational notes in `FL_robot/CHANGELOG.md`, e.g.:
- `2025-08-14 – Standardized rosbridge :10090, added start_pc_link.sh`
- `2025-08-14 – Robot A linked; PC publish/echo verified`
- `2025-08-15 – Robot B setup: new SSH key, IP=<...>, host networking`
