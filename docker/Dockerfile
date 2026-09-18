# LUCIA ROS2 container for the Raspberry Pi (arm64).
#
# Build (from the repo root, on the Pi; needs internet for the apt installs):
#   docker build -t lucia/ros2:latest docker/
#
# Run: see docs/setup/ros2-pi.md (Step 4), using lucia/ros2:latest as the image.
# The workspace (src/ros2) is mounted at /ros2_ws/src at run time, not baked in.

FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    python3-serial \
    python3-colcon-common-extensions \
    ros-jazzy-rplidar-ros \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=42

WORKDIR /ros2_ws

# Source ROS (and our built workspace, once it exists) in every new terminal,
# including ones opened with `docker exec -it lucia_ros2 bash`.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc
