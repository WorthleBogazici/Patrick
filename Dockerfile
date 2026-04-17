FROM ros:jazzy
SHELL ["/bin/bash", "-c"]

# ── Switch APT sources to HTTPS (port 80 may be blocked) ──────────────────
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g; s|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list.d/ubuntu.sources

# ── System & ROS 2 APT dependencies ────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS 2 desktop (includes rviz2, rqt, etc.)
    ros-jazzy-desktop \
    # Robot description
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    # ros2_control framework
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-effort-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-realtime-tools \
    ros-jazzy-hardware-interface \
    ros-jazzy-ecl-build \
    ros-jazzy-ur-controllers \
    ros-jazzy-parallel-gripper-controller \
    # MoveIt2 stack
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
    # Gazebo Harmonic integration
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    # TF2 & navigation messages
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-lifecycle-msgs \
    # Vision & perception
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-conversions \
    ros-jazzy-visualization-msgs \
    ros-jazzy-image-transport \
    # ZED camera description (meshes & URDF macros — no SDK needed)
    ros-jazzy-zed-description \
    ros-jazzy-zed-msgs \
    ros-jazzy-nmea-msgs \
    # Visualization
    ros-jazzy-rviz2 \
    # System libraries
    libeigen3-dev \
    libpcl-dev \
    python3-pip \
    python3-colcon-common-extensions \
    # Utilities
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/*

# ── Create workspace ───────────────────────────────────────────────────────
RUN mkdir -p /patrick_ws/src

# ── Copy source into image ─────────────────────────────────────────────────
COPY ./src/ /patrick_ws/src/

# ── Mark packages that need unavailable HW/SDKs as COLCON_IGNORE ───────────
RUN touch /patrick_ws/src/third_party_ros2/zed-ros2-wrapper/zed_components/COLCON_IGNORE \
    && touch /patrick_ws/src/third_party_ros2/zed-ros2-wrapper/zed_debug/COLCON_IGNORE \
    # && touch /patrick_ws/src/third_party_ros2/zed-ros2-wrapper/zed_wrapper/COLCON_IGNORE \
    # && touch /patrick_ws/src/third_party_ros2/zed-ros2-wrapper/zed_ros2/COLCON_IGNORE \
    && touch /patrick_ws/src/third_party_ros2/ThirdParty/ros_astra_camera/COLCON_IGNORE \
    && touch /patrick_ws/src/third_party_ros2/ThirdParty/openni2_camera/COLCON_IGNORE \
    && touch /patrick_ws/src/third_party_ros2/ThirdParty/rplidar_ros/COLCON_IGNORE

# ── Install any remaining build deps declared in package.xml files ─────────
RUN apt-get update \
    && rosdep update \
    && source /opt/ros/jazzy/setup.bash \
    && rosdep install --from-paths /patrick_ws/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# ── Build the workspace ───────────────────────────────────────────────────
RUN source /opt/ros/jazzy/setup.bash \
    && cd /patrick_ws \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
RUN echo "source /entrypoint.sh" >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
