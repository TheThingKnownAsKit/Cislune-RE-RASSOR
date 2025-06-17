# ──────────────────────────────────────────────────────
# Base: Official ROS 2 Jazzy with Gazebo
# ──────────────────────────────────────────────────────
FROM ros:jazzy-ros-base

# Locale config
ENV DEBIAN_FRONTEND=noninteractive

# ──────────────────────────────────────────────────────
# Install system & ROS tools
# ──────────────────────────────────────────────────────
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    git wget nano vim sudo lsb-release locales \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────────────
# Create workspace and copy source
# ──────────────────────────────────────────────────────
WORKDIR /workspace
COPY ../src/ src/

# ──────────────────────────────────────────────────────
# Build workspace
# ──────────────────────────────────────────────────────
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ──────────────────────────────────────────────────────
# Source environment
# ──────────────────────────────────────────────────────
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["bash"]
