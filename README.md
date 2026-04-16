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
| `rviz` | `true` | Launch RViz |
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
