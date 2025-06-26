ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04

####################  BUILD STAGE  ####################

# ---------- amd64 ----------
FROM --platform=linux/amd64 ros:${ROS_DISTRO}-ros-base AS amd64

# ---------- jetson arm64 ----------
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base AS arm64

RUN apt-get update && rm -rf /var/lib/apt/lists/*

####################  LOAD CORRECT IMAGE ####################
ARG TARGETARCH
FROM $TARGETARCH AS final

####################  RUNTIME STAGE ####################
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

ARG RUNTIME=notnvidia

USER ${USERNAME}

WORKDIR /workspaces/Cislune-RE-RASSOR

# ----- install dependencies -----
# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
        gnupg \
        gnupg2 \
        lsb-release \
        software-properties-common \
        wget \
        curl \
        python3-pip \
        libegl1-mesa-dev \
        libglu1-mesa-dev \
        libxv1 \
        libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# Install core python tools
RUN apt install -y build-essential \ 
    python3-colcon-common-extensions \
    python3-vcstool python3-rosdep

# Install Nvidia specific dependencies (non-nvidia just uses mesa)
RUN if [ "$RUNTIME" = "nvidia" ]; then \
        echo "Installing NVIDIA dependencies..." && \
        sudo apt-get update && \
        sudo apt-get install -y nvidia-container-toolkit && \
        sudo systemctl restart docker; \
    else \
        echo "Not on jetson, skipping NVIDIA installations"; \
    fi

# Install ROS 2 packagess
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 2 core packages
    ros-${ROS_DISTRO}-desktop \
    # ROS 2 control stack
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rplidar-ros \
    # ROS–Gazebo integration
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    # Navigation stack
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc