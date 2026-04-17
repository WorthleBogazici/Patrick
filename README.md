# Patrick Robot — ROS 2 Workspace

Patrick is a mobile manipulation robot built on a **Kobuki** base with a **TurtleBot-style hexagon stack**, a **custom 5-DOF robotic arm** (Dynamixel RX-64 / AX-18A servos driven via Arduino), and a **ZED2 stereo camera** (or Kinect) for vision-based object detection. The system uses **FastSAM** for real-time instance segmentation and color-based object recognition.

## Packages

| Package | Description |
|---|---|
| `patrick_main` | Top-level URDF, meshes, launch files, and controller/parameter configs |
| `patrick_manipulation` | Arm control lifecycle node, Arduino hardware interface (`ros2_control` plugin) |
| `patrick_navigation` | Base navigation lifecycle node (Kobuki diff-drive) |
| `patrick_vision` | FastSAM color detection (Python), 3D object frame publisher, RViz marker publisher |
| `patrick_coordination` | High-level task coordination (stub/in-progress) |
| `patrick_simulation` | Gazebo Harmonic simulation launch and world configs |
| `patrick_moveit_config` | MoveIt2 config: SRDF, kinematics (pick-ik), controller bridge, RViz MotionPlanning |
| `patrick_msgs` | Custom messages, services, and actions |
| `third_party_ros2/` | Vendored third-party packages (Kobuki, DynamixelSDK, ZED wrapper, RPLidar, etc.) |

## Prerequisites

- **Ubuntu 24.04** (Noble Numbat)
- **ROS 2 Jazzy** (desktop install)
- **Gazebo Harmonic** (for simulation)
- **Python 3.12+**

## Installation

### 1. Install ROS 2 Jazzy

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) if not already installed.

### 2. Clone the workspace

```bash
mkdir -p ~/patrick_ws/src
cd ~/patrick_ws/src
git clone <repository-url> .
```

### 3. Install ZED camera support

```bash
cd ~/patrick_ws/src/third_party_ros2
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
touch zed-ros2-wrapper/zed_components/COLCON_IGNORE
touch zed-ros2-wrapper/zed_debug/COLCON_IGNORE
```

> **Note:** `zed_components` and `zed_debug` require the ZED SDK to build. They are ignored unless you have the SDK installed. The URDF macros and meshes come from the `ros-jazzy-zed-description` apt package (installed in the next step).

### 4. Install APT dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-urdf \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-effort-controllers \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-realtime-tools \
  ros-jazzy-hardware-interface \
  ros-jazzy-ur-controllers \
  ros-jazzy-parallel-gripper-controller \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-planners-chomp \
  ros-jazzy-moveit-planners-stomp \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-moveit-ros-visualization \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-moveit-ros-control-interface \
  ros-jazzy-moveit-visual-tools \
  ros-jazzy-moveit-servo \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-pick-ik \
  ros-jazzy-srdfdom \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-lifecycle-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-pcl-conversions \
  ros-jazzy-visualization-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-zed-description \
  ros-jazzy-zed-msgs \
  ros-jazzy-nmea-msgs \
  ros-jazzy-rviz2 \
  libeigen3-dev \
  libpcl-all-dev \
  python3-pip \
  python3-colcon-common-extensions
```

### 5. Install rosdep dependencies

```bash
cd ~/patrick_ws
sudo rosdep init  # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Install Python dependencies

```bash
pip3 install numpy opencv-python torch ultralytics
```

### 7. Build the workspace

```bash
cd ~/patrick_ws
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## How to Run

### Simulation (Gazebo + RViz)

**With ZED2 camera (default):**

```bash
source ~/patrick_ws/install/setup.bash
ros2 launch patrick_main patrick_bringup.launch.py simulation:=true
```

**With Kinect camera:**

```bash
ros2 launch patrick_main patrick_bringup.launch.py simulation:=true kinect:=true
```

### Real Hardware

```bash
source ~/patrick_ws/install/setup.bash
ros2 launch patrick_main patrick_bringup.launch.py simulation:=false
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `simulation` | `false` | `true` for Gazebo sim, `false` for real hardware |
| `kinect` | `false` | `true` to use Kinect camera, `false` for ZED2 |
| `gui` | `true` | Enable Gazebo GUI |
| `rviz` | `true` | Launch RViz (auto-suppressed when MoveIt brings its own) |
| `moveit` | `true` | Launch the MoveIt2 stack (move_group + MotionPlanning RViz) |
| `vision` | `true` | Launch the vision pipeline (FastSAM) |
| `world` | `small_house.world` | Path to Gazebo world file |

## FastSAM Model File

> **Important:** The FastSAM model weights file (`FastSAM-x.pt`) is **not included** in this repository due to its size. You must download it manually and place it in the correct location before running the vision pipeline.

1. Download `FastSAM-x.pt` from the [FastSAM releases](https://github.com/CASIA-IVA-Lab/FastSAM) or [Ultralytics](https://docs.ultralytics.com/models/fast-sam/).

2. Place the file at:

   ```
   ~/patrick_ws/src/patrick_vision/segmentation/models/FastSAM-x.pt
   ```

3. Rebuild `patrick_vision` so the model is installed to the share directory:

   ```bash
   cd ~/patrick_ws
   colcon build --symlink-install --packages-select patrick_vision
   ```

Without this file, the vision pipeline (`fastsam_color_detector`) will fail to start.

## MoveIt2 Integration

`patrick_moveit_config` provides a full MoveIt2 setup for the 4-DOF arm + parallel-jaw gripper:

- **Planning group** `arm`: `<chain base_link="base_link" tip_link="arm_tool0"/>` — the TCP frame `arm_tool0` sits between the jaws at `Link5 + (0, 0, 0.22)`, so the interactive EE marker drags the real tool point.
- **Gripper group** `gripper`: `Joint5R` (driven); `Joint5L` mimics with `multiplier="-1.0"` for opposite motion.
- **Named states** (calibrated from the live robot): `home`, `stowed`, `ready`, `open`, `closed`.
- **IK solver**: `pick_ik/PickIkPlugin` in `mode: global` with `position_scale: 1.0`, `rotation_scale: 0.5`, and a loose approximate-solution fallback so position-only goals succeed when full orientation is unreachable.
- **Controller bridge**: `moveit_simple_controller_manager` — `FollowJointTrajectory` on `arm_controller` (sim) / `scaled_arm_controller` (hw, `ur_controllers/ScaledJointTrajectoryController`), `GripperCommand` on `gripper_controller/gripper_cmd`.
- **Collision matrix**: arm-vs-base and base-vs-base pairs are all disabled in the SRDF (the base/stack/sensor/camera links are rigid-joined to `base_link`, so they cannot dynamically collide).

MoveIt launches automatically as part of `patrick_bringup.launch.py` (`moveit:=true` by default, with an 8 s `TimerAction` delay so `/clock` and controllers are up first). When MoveIt runs, the bringup's own RViz is suppressed and MoveIt's `moveit.rviz` (with the MotionPlanning panel) takes over.

To disable MoveIt and use plain bringup RViz:

```bash
ros2 launch patrick_main patrick_bringup.launch.py simulation:=true moveit:=false
```

To launch the MoveIt demo standalone (no Gazebo, fake controllers):

```bash
ros2 launch patrick_moveit_config demo.launch.py
```

## Docker

A `Dockerfile`, `docker-compose.yaml`, and `entrypoint.sh` are included. The image is built on `ros:jazzy` and installs every dependency above (ROS 2 desktop, Gazebo Harmonic, ros2_control, the full MoveIt2 stack with `pick-ik`, ZED description, etc.), copies `src/` into `/patrick_ws/src/`, and runs `colcon build --symlink-install`.

### Build

Either Docker Compose or a plain `docker build` works:

```bash
# Option A — Compose
docker compose build

# Option B — plain docker
docker build -t patrick:latest .
```

### Run

```bash
sudo xhost +   # allow GUI from container
docker compose up patrick-simulation    # Gazebo + MoveIt + RViz
docker compose up patrick-hardware      # real hardware bringup
```

The compose file mounts `./src` into the container so source edits are picked up without rebuilding — just re-run `colcon build` inside the container.
