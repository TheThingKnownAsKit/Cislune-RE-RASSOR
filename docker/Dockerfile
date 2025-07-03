ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04

####################  BUILD STAGE  ####################

ARG TARGETARCH

FROM --platform=${TARGETARCH} ros:${ROS_DISTRO}-ros-base

####################  RUNTIME STAGE ####################
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

ARG RUNTIME=notnvidia

RUN apt-get update && rm -rf /var/lib/apt/lists/*

# ----- install dependencies -----
# Install Python and Python interpreter
RUN apt-get install python3

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
        libxtst6

# Install needed elements from Ubuntu Universe
RUN  add-apt-repository -y universe && \
     apt-get update && \
     apt-get install -y --no-install-recommends usbutils joystick evtest && \
     rm -rf /var/lib/apt/lists/*

# Install core python tools
RUN apt-get update && apt install -y build-essential \ 
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool python3-rosdep \
    && apt-get install python-is-python3

# Install PlatformIO
# RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
# RUN python3 get-platformio.py

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
    # ROS–Gazebo integration
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    # Navigation stack
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/profile.d/ros2.sh

ARG UID=1000
ARG GID=1000
RUN groupadd -g "${GID}" dev || true && \
    useradd -m -u "${UID}" -g "${GID}" dev || true
USER dev

WORKDIR /workspaces/Cislune-RE-RASSOR/