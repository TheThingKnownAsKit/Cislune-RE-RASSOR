ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04

####################  BUILD STAGE  ####################

# NOTE: You have to run build_multiarch_container.bash beforehand

# ---------- amd64 ----------
FROM --platform=linux/amd64 ros:${ROS_DISTRO}-ros-base AS amd64

# ---------- jetson arm64 ----------
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base AS arm64

RUN apt-get update && rm -rf /var/lib/apt/lists/*

####################  RUNTIME STAGE  ####################
ARG TARGETARCH
FROM $TARGETARCH AS final

# Set user and permissions
RUN useradd -m ros2dev && usermod -aG dialout,video ros2dev
USER ros2dev
WORKDIR /workspaces/rover_ros2