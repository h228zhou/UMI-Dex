# Recording SOP (Standard Operating Procedure)

This document defines the operator procedure for recording data collection sessions with UMI-Dex.

## Hardware Setup

1. **D455** — mounted rigidly, USB 3.0+ cable, stereo IR + IMU enabled.
2. **D405** — mounted on end-effector or tool, USB 3.0+ cable, color stream.
3. **Hand controller link** — pick one, matching the unit:
   - **CAN** units: `sudo ip link set can0 up type can bitrate 1000000`.
   - **USART / ttyUSB** units: one-time permission setup, `sudo usermod -aG dialout $USER` followed by a logout/login (or `newgrp dialout` in the current shell). Verify with `ls -l /dev/ttyUSB0` — the device should be owned by `root:dialout` with mode `660`. Do **not** use `chmod 666` — it is wiped every time the device re-enumerates.

Verify camera serials are set in `ros/config/camera_serials.conf` (shared by both ROS1 and ROS2).

## Pre-flight

### ROS1 Noetic

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd /path/to/UMI-Dex && mkdir -p outputs
```

### ROS2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd /path/to/UMI-Dex && mkdir -p outputs
```

## Recording Procedure

### 1. Launch the capture pipeline

Pick the flags matching the unit's controller link. CAN is the default.

**ROS1:**
```bash
# CAN (default)
roslaunch umi_dex capture.launch

# USART / ttyUSB
roslaunch umi_dex capture.launch \
  controller_protocol:=usart \
  usart_port:=/dev/ttyUSB0 \
  usart_baud:=115200
```

**ROS2:**
```bash
# CAN (default)
ros2 launch umi_dex_bringup capture.launch.py

# USART / ttyUSB
ros2 launch umi_dex_bringup capture.launch.py \
  controller_protocol:=usart \
  usart_port:=/dev/ttyUSB0 \
  usart_baud:=115200
```

The bag records exactly one hand topic — `/hand/can_raw` or `/hand/usart_raw` — matching the selected protocol.

The interactive capture node starts in **idle** state. Available commands are context-sensitive and shown in the prompt.

### 2. Start a session (`s`)

Press `s` to begin a new recording session. The system:

1. Starts a rosbag recording (continuous, one bag for the entire session).
2. Writes a session sidecar (`.session.json`) with clock anchors and provenance.
3. Enters **warm-up** mode automatically.

### 3. IMU warm-up (automatic, 15 seconds)

During warm-up, perform the **IMU excitation motion**:

- Point the D455 at a **textured, well-lit** area (shelves, desk with objects, posters — avoid blank walls).
- Perform **slow, smooth translational motion**: left-right, forward-backward, up-down.
- **Do not**: rotate only (insufficient linear acceleration), hold still, or move so fast that frames blur.
- The countdown timer shows elapsed/remaining time. You cannot skip warm-up.

This warm-up is critical because ORB-SLAM3 needs sufficient IMU excitation to complete its Visual-Inertial Bundle Adjustment (VIBA) initialization. Without it, the offline SLAM replay will produce a degraded or failed trajectory.

After the timer completes, the system enters **ready** state.

### 4. Record episodes

The session supports **multiple episodes** without restarting or re-warming the IMU:

| Action | Key | From State | To State |
|--------|-----|------------|----------|
| Start episode | `e` | ready | recording |
| End episode | `e` | recording | ready |
| End session | `c` | ready | idle |
| End session + discard current | `c` | recording | idle |
| Quit | `q` | any | exit |

**Typical workflow:**

1. Press `e` to start an episode → perform the task demonstration.
2. Press `e` to end the episode → system confirms episode kept.
3. Repeat for each demonstration in this environment.
4. Press `c` to end the session → bag and sidecar are saved.

If you press `c` during a recording episode, the current episode is **discarded** (marked in metadata) but all previously completed episodes are kept. The bag is saved intact — discarded episodes are filtered out during offline processing.

### 5. Multiple sessions

Press `s` again to start a new session (new bag, new warm-up). Press `q` to exit.

## What a Good Session Looks Like

- Warm-up: 15 seconds of smooth translation in a textured scene (automatic timer).
- Multiple clean episodes per session — minimises warm-up overhead.
- No prolonged blank-wall exposure.
- Consistent lighting (no sudden dark-to-bright transitions).
- Controller link active — CAN bus up, or ttyUSB device present and readable by the `dialout` group.

## Post-Recording

Process the bag offline. The Python pipeline accepts both ROS1 `.bag` files and ROS2 bag directories:

```bash
# ROS1 bag
uv run umi-process /path/to/capture.bag \
  --vocab ./config/ORBvoc.txt \
  --settings ./config/intel_d455.yaml \
  --split-episodes \
  --out sessions/<session_id>/

# ROS2 bag (pass the bag directory)
uv run umi-process /path/to/capture_2025-01-01-12-00-00/ \
  --vocab ./config/ORBvoc.txt \
  --settings ./config/intel_d455.yaml \
  --split-episodes \
  --out sessions/<session_id>/
```

See [processing.md](processing.md) for details.
