# umi_dex — ROS1 Capture Package

ROS Noetic (Python) package for synchronized data capture from:

- **Intel D455** — stereo IR (848x480 @ 30 fps) + IMU (gyro/accel @ 200 Hz)
- **Intel D405** — color stream + camera info
- **Hand controller** — raw frames via **CAN** (SocketCAN, ID 0x112) or **USART** (ttyUSB, 16-byte framed). Protocol is selected at launch.

All streams are recorded into a single **rosbag** with a shared ROS clock. CAN frame assembly, filtering, and calibration happen offline in the Python pipeline — the recorder captures raw frames only.

## Prerequisites

| Component | Install |
|-----------|---------|
| Ubuntu 20.04 | — |
| ROS Noetic | `sudo apt install ros-noetic-desktop-full` |
| RealSense ROS (D405 recommended) | Build `librealsense` + `realsense-ros` from source (see below) |
| SocketCAN (for `controller_protocol:=can`) | Kernel built-in; configure with `sudo ip link set can0 up type can bitrate 1000000` |
| USART / ttyUSB (for `controller_protocol:=usart`) | `sudo apt install python3-serial`; add user to `dialout` once: `sudo usermod -aG dialout $USER && newgrp dialout` |

## Install librealsense + realsense-ros from source (recommended for D405)

`ros-noetic-realsense2-camera` from apt can lag behind and may not include the D405 support/behavior you need.
For D405, install both `librealsense` and the ROS1 wrapper from source in this order:

```bash
# 0. Optional: remove apt ROS wrapper if already installed.
sudo apt remove -y ros-noetic-realsense2-camera

# 1. Build and install librealsense.
sudo apt update
sudo apt install -y \
  git cmake build-essential pkg-config \
  libssl-dev libusb-1.0-0-dev libgtk-3-dev \
  libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.55.1

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=false
make -j"$(nproc)"
sudo make install
sudo ldconfig

# 2. Build ROS1 wrapper inside your catkin workspace.
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros1-legacy

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# 3. Verify ROS can find the wrapper package.
rospack find realsense2_camera
```

## Setup

```bash
# 1. Create (or reuse) a catkin workspace.
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 2. Symlink this package into the workspace.
ln -s /path/to/UMI-Dex/ros umi_dex

# 3. Build.
cd ~/catkin_ws
catkin_make

# 4. Source.
source devel/setup.bash
```

## Output directories

`rosbag record` does **not** create missing parent directories. Default `bag_dir` is `$(find umi_dex)/../../outputs`. Create those directories before capture:

```bash
cd /path/to/UMI-Dex && mkdir -p outputs
```

## Usage

### Record a capture session (interactive)

```bash
# Bring up the controller link:
#   CAN:   sudo ip link set can0 up type can bitrate 1000000
#   USART: ensure the user is in `dialout` (one-time: sudo usermod -aG dialout $USER),
#          then plug the device (default /dev/ttyUSB0).

# Set camera serials once in:
#   ros/config/camera_serials.conf

# Terminal 1 — hardware streams (cameras + controller)
roslaunch umi_dex capture.launch                               # defaults to CAN
roslaunch umi_dex capture.launch controller_protocol:=usart \
  usart_port:=/dev/ttyUSB0 usart_baud:=115200

# Terminal 2 — interactive recorder (needs its own tty for stdin)
rosrun umi_dex record.sh --protocol can
rosrun umi_dex record.sh --protocol usart
# Override defaults:
rosrun umi_dex record.sh --protocol can --bag-dir outputs --warmup 15

# Interactive commands (shown context-sensitively in the recorder's prompt):
#   s : start new session (IMU warm-up + episode recording)
#   e : start/end episode (within a session)
#   c : end session (save bag with all episodes)
#   l : list recordings in bag_dir
#   r : delete last finished recording
#   q : quit
```

Two terminals are required because `roslaunch` closes each `<node>`'s stdin, which breaks the interactive hotkey prompt. The recorder must run via `rosrun` so it inherits a real tty.

A `<bag>.session.json` sidecar is written at recording start with provenance anchors (ROS time, wall clock, host info).

### Launch individual components

```bash
# D455 camera only
roslaunch umi_dex d455.launch

# Controller raw frame publisher only (CAN by default; USART via arg)
roslaunch umi_dex controller.launch
roslaunch umi_dex controller.launch controller_protocol:=usart usart_port:=/dev/ttyUSB0

# Override defaults
roslaunch umi_dex capture.launch can_channel:=can1 d405_serial:=123456
```

### Play back a bag

```bash
roslaunch umi_dex playback.launch bag:=/path/to/capture.bag
```

## ROS Topics

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 30 Hz | realsense2_camera |
| `/camera/infra1/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | realsense2_camera |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 30 Hz | realsense2_camera |
| `/camera/infra2/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | realsense2_camera |
| `/camera/imu` | `sensor_msgs/Imu` | 200 Hz | realsense2_camera |
| `/camera_d405/color/image_raw` | `sensor_msgs/Image` | 30 Hz | realsense2_camera |
| `/camera_d405/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | realsense2_camera |
| `/hand/can_raw` | `umi_dex/CanFrame` | ~300 Hz | `can_raw_node` (when `controller_protocol=can`) |
| `/hand/usart_raw` | `umi_dex/UsartFrame` | ~300 Hz | `usart_raw_node` (when `controller_protocol=usart`) |

Only one of `/hand/can_raw` or `/hand/usart_raw` appears in any given bag — the recorder subscribes to the topic matching the selected protocol.

## Custom Messages

### CanFrame

```
std_msgs/Header header
uint32   arb_id
uint8    dlc
uint8[8] data
```

Raw CAN bus frame. Assembly into 6-channel joint angles and calibration happen offline in the Python pipeline.

### UsartFrame

```
std_msgs/Header header
uint16[6] raw         # 12-bit encoder counts
uint8     valid_mask  # bit i => channel i valid
```

Raw 16-byte USART packet (`55 AA | mask | 6×(lo,hi12) | checksum`) emitted by the controller firmware. Already assembled; decoding to joint angles happens offline (or, eventually, inside firmware once calibration moves there).

### HandJointState (legacy, kept for backward compatibility)

```
std_msgs/Header header
string[6]  names
float64[6] positions
bool[6]    valid
```

## Package Structure

```
ros/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── calibration.csv
│   ├── camera_serials.conf
│   └── d455_params.yaml
├── launch/
│   ├── capture.launch
│   ├── d405.launch
│   ├── d455.launch
│   ├── controller.launch
│   └── playback.launch
├── msg/
│   ├── CanFrame.msg
│   ├── HandJointState.msg
│   └── UsartFrame.msg
├── nodes/
│   ├── can_raw_node
│   ├── usart_raw_node
│   └── interactive_capture_node
└── umi_dex/
    └── __init__.py
```

## License

Apache License 2.0 — see [LICENSE](../LICENSE).
