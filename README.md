# mercury

Official repository for ICMTC UGVC-2026

## Prerequisites

* Ubuntu 24.04
* ROS 2 Jazzy
* colcon
* rosdep
* Docker (optional)

---

## First-Time Setup (Fresh Clone)

This repository is already a ROS 2 workspace.

```bash
# Clone workspace
git clone <repo-url>
cd mercury

# Source ROS
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
pip install opencv-python numpy psutil --break-system-packages

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

---

## Python Virtual Environment Setup

Some packages (e.g. face recognition) require Python dependencies that must be installed in a virtual environment alongside ROS 2.

```bash
# Create venv — allow access to ROS 2 system packages
python3 -m venv ~/mercury_venv --system-site-packages

# Activate
source ~/mercury_venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

Add activation to your `~/.bashrc` so it persists across terminals:

```bash
echo "source ~/mercury_venv/bin/activate" >> ~/.bashrc
source ~/.bashrc
```

> **Note:** Always activate the venv before running any face task or perception nodes. The `--system-site-packages` flag ensures ROS 2 Python packages (`rclpy`, etc.) remain accessible inside the venv.

---

## Environment Setup

Add this to your `~/.bashrc` or `~/.zshrc`:

```bash
# ROS
source /opt/ros/jazzy/setup.bash

# Workspace
source ~/mercury/install/setup.bash

# Gazebo resource path
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH

# Gazebo system plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib

# Python venv
source ~/mercury_venv/bin/activate
```

Apply:

```bash
source ~/.bashrc
```

---

## Running with Docker

```bash
sudo docker compose build
sudo docker compose run ros
```

---

## Running Simulation

```bash
cd mercury
source install/setup.bash
colcon build
ros2 launch bringup bringup_sim.launch.py
```

---

## watchdog_monitor

A non-intrusive ROS 2 monitoring and observability package for the Mercury robot. Runs alongside the existing stack without modifying control logic.

### Nodes

| Node | Publishes | Rate | Description |
| --- | --- | --- | --- |
| `system_monitor_node` | `/system_status` | 2s | Tracks running vs expected nodes and publishes JSON health |
| `watchdog_node` | `/system_alerts` | 3s | Detects node crashes, topic silence, TF failures |
| `waypoint_detector_node` | `/waypoint_reached`, `/waypoint_status` | 10Hz / 1Hz | Detects arrival at predefined waypoints |
| `control_listener_node` | — | Event-driven | Passive observer logging monitoring events |
| `monitoring_dashboard` | — | 1Hz | Live terminal dashboard |

### Launching the Watchdog

To build and launch the watchdog monitoring system:

```bash
ros2 launch watchdog_monitor monitoring_all.launch.py
```

### Dashboard

To spin up the live terminal dashboard:

```bash
ros2 launch watchdog_monitor dashboard.launch.py
```

---

## Waypoint Configuration

Edit `config/waypoints.yaml`:

```yaml
waypoint_detector_node:
  ros__parameters:
    spawn_x: -21.0
    spawn_y: -47.0
    waypoints: [-19.0, -47.0, -19.0, -43.0, -21.0, -43.0]
    waypoint_names: ["WP-1", "WP-2", "WP-3"]
    arrival_radius: 0.5
```

> **Note:** `spawn_x` and `spawn_y` must match the `-x` / `-y` values passed to `ros_gz_sim create` in the launch file. Waypoints are specified in world coordinates — the node offsets odometry by the spawn position automatically.

---

## Face Detection Task

> **Prerequisite:** `monitoring_all` must be running before launching the face task. It provides the waypoint and system monitoring events that the face task node depends on.

**Terminal 1 — start monitoring stack:**

```bash
ros2 launch watchdog_monitor monitoring_all.launch.py
```

**Terminal 2 — launch face detection:**

```bash
ros2 launch face_task face_task.launch.py target_image:=/home/soap/probes/mercury/photo1.jpg
```

*Replace the `target_image` path with the absolute path to your target face image.*

---

## Topics

| Topic | Type | Publisher |
| --- | --- | --- |
| `/system_status` | `std_msgs/String` (JSON) | `system_monitor_node` |
| `/system_alerts` | `std_msgs/String` (JSON) | `watchdog_node` |
| `/waypoint_reached` | `std_msgs/String` (JSON) | `waypoint_detector_node` |
| `/waypoint_status` | `std_msgs/String` (JSON) | `waypoint_detector_node` |
| `/final_goal` | `geometry_msgs/PoseStamped` | External / operator |

---

## Sending Navigation Goal

Publish a goal directly to the navigation stack:

```bash
ros2 topic pub --once /final_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: <X>, y: <Y>}, orientation: {w: 1.0}}}"
```

Replace `<X>` and `<Y>` with target world coordinates (in metres, `map` frame).

**Example — drive to (24.5, −22.4):**

```bash
ros2 topic pub --once /final_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 24.5, y: -22.4}, orientation: {w: 1.0}}}"
```

> **`orientation: {w: 1.0}`** sets a neutral (zero-yaw) heading.  
> The planner determines the actual approach angle automatically.  
> To command a specific heading, use the quaternion formula:  
> `w = cos(θ/2)`, `z = sin(θ/2)` where θ is yaw in radians  
> (e.g. 90° → `z: 0.707, w: 0.707`).

---

## Turret Control

To manually move the turret, publish commands to the controller:

```bash
ros2 topic pub /turret_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"
```

*The array represents joint commands (e.g., yaw, pitch). Adjust values based on your turret configuration.*

---

## Manually Trigger a Waypoint Event

```bash
ros2 topic pub --once /waypoint_reached std_msgs/msg/String \
  '{"data": "{\"event\": \"waypoint_reached\", \"waypoint\": {\"name\": \"WP-2\", \"index\": 2}}"}'
```

---

## Clean Build

```bash
rm -rf build/ install/ log/
colcon build
```
