# umi_dex — ROS2 Jazzy Capture Packages

ROS2 Jazzy capture pipeline for synchronized data recording from:

- **Intel D455** — stereo IR (848x480 @ 30 fps) + IMU (gyro/accel @ 200 Hz)
- **Intel D405** — color stream + camera info
- **CAN controller** — raw CAN frames via SocketCAN (CAN ID 0x112)

All streams are recorded into a single **ros2 bag** (mcap format) with a shared ROS clock. CAN frame assembly, filtering, and calibration happen offline in the Python pipeline — the recorder captures raw frames only.

## Package Structure

Two colcon packages:

| Package | Type | Contents |
|---------|------|----------|
| `umi_dex_msgs` | ament_cmake | Custom message definitions (CanFrame, HandJointState) |
| `umi_dex_bringup` | ament_python | Nodes, launch files, config |

## Prerequisites

| Component | Install |
|-----------|---------|
| Ubuntu 24.04 | — |
| ROS2 Jazzy | `sudo apt install ros-jazzy-desktop` |
| RealSense ROS2 | `sudo apt install ros-jazzy-realsense2-camera` or build from source |
| SocketCAN | Kernel built-in; configure with `sudo ip link set can0 up type can bitrate 1000000` |

### Install RealSense ROS2 from source (if apt version is insufficient)

```bash
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b development
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description
source install/setup.bash
```

## Setup

```bash
# 1. Create (or reuse) a colcon workspace.
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Symlink both packages into the workspace.
ln -s /path/to/UMI-Dex/ros2/umi_dex_msgs umi_dex_msgs
ln -s /path/to/UMI-Dex/ros2/umi_dex_bringup umi_dex_bringup

# 3. Build.
cd ~/ros2_ws
colcon build --packages-select umi_dex_msgs umi_dex_bringup

# 4. Source.
source install/setup.bash
```

## Output directories

`ros2 bag record` creates the output directory automatically. Default `bag_dir` is `outputs` (relative to CWD). Create it before capture if desired:

```bash
cd /path/to/UMI-Dex && mkdir -p outputs
```

## Usage

### Record a capture session (interactive)

```bash
# Bring up CAN interface first.
sudo ip link set can0 up type can bitrate 1000000

# Set camera serials in:
#   ros2/umi_dex_bringup/config/camera_serials.conf
# (symlinked to ros/config/camera_serials.conf)

# Launch all streams + interactive recorder.
ros2 launch umi_dex_bringup capture.launch.py

# After launch, use interactive commands:
#   s : start new session (IMU warm-up + episode recording)
#   e : start/end episode (within a session)
#   c : end session (save bag with all episodes)
#   l : list recordings in bag_dir
#   r : delete last finished recording
#   q : quit
```

A `<bag_dir>.session.json` sidecar is written alongside the bag directory with provenance anchors (ROS time, wall clock, host info) and episode metadata.

### Launch individual components

```bash
# D455 camera only
ros2 launch umi_dex_bringup d455.launch.py

# D405 camera only
ros2 launch umi_dex_bringup d405.launch.py

# CAN raw frame publisher only
ros2 launch umi_dex_bringup controller.launch.py

# Override defaults
ros2 launch umi_dex_bringup capture.launch.py can_channel:=can1 warmup_duration_s:=20.0
```

### Play back a bag

```bash
ros2 launch umi_dex_bringup playback.launch.py bag:=/path/to/bag_dir
```

## ROS Topics

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | realsense2_camera |
| `/camera/infra1/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | realsense2_camera |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | realsense2_camera |
| `/camera/infra2/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | realsense2_camera |
| `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz | realsense2_camera |
| `/camera_d405/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | realsense2_camera |
| `/camera_d405/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | realsense2_camera |
| `/hand/can_raw` | `umi_dex_msgs/msg/CanFrame` | ~300 Hz | can_raw_node |
| `/session/episode` | `std_msgs/msg/String` | event | interactive_capture_node |

## Custom Messages

### CanFrame

```
std_msgs/Header header
uint32   arb_id
uint8    dlc
uint8[8] data
```

### HandJointState (legacy, kept for backward compatibility)

```
std_msgs/Header header
string[6]  names
float64[6] positions
bool[6]    valid
```

## Differences from ROS1 Version

| Aspect | ROS1 (`ros/`) | ROS2 (`ros2/`) |
|--------|---------------|----------------|
| Build system | catkin | ament_cmake + ament_python |
| Bag format | `.bag` (single file) | mcap directory |
| Launch files | XML `.launch` | Python `.launch.py` |
| Recording CLI | `rosbag record` | `ros2 bag record -s mcap` |
| Message package | `umi_dex` | `umi_dex_msgs` |
| Node package | `umi_dex` | `umi_dex_bringup` |
| Episode QoS | N/A (TCP) | RELIABLE + TRANSIENT_LOCAL |

The Python offline pipeline (`umi-process`, `umi-extract`, etc.) works with bags from either version — the `BagReader` auto-detects the format.

## License

Apache License 2.0 — see [LICENSE](../LICENSE).
