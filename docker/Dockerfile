ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04
ARG TARGETARCH
####################  REALSENSE SDK BUILD STAGE  ####################

FROM --platform=${TARGETARCH} ros:${ROS_DISTRO}-ros-base AS realsense_builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies for the Intel RealSense SDK (librealsense2)
RUN apt-get update && apt-get install -y --no-install-recommends \  
    build-essential cmake git libssl-dev libusb-1.0-0-dev pkg-config \  
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \  
    curl python3-dev ca-certificates && rm -rf /var/lib/apt/lists/*

# Download the latest Librealsense SDK source code and build it from source
WORKDIR /usr/src
RUN export LIBRS_GIT_TAG=$(git -c 'versionsort.suffix=-' \
        ls-remote --refs --sort="version:refname" --tags https://github.com/IntelRealSense/librealsense "*.*.*" | \
        tail -n1 | cut -d'/' -f3) && \
    export LIBRS_VERSION=${LIBRS_GIT_TAG#"v"} && \
    curl -L https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBRS_VERSION}.tar.gz -o librealsense.tar.gz && \
    tar -xzf librealsense.tar.gz && rm librealsense.tar.gz && \
    ln -s /usr/src/librealsense-${LIBRS_VERSION} /usr/src/librealsense

RUN cd /usr/src/librealsense && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
             -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
             -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
             -DBUILD_GRAPHICAL_EXAMPLES=OFF \
             -DBUILD_EXAMPLES=OFF \
             -DBUILD_PYTHON_BINDINGS=TRUE && \
    make -j$(nproc) && make install

ENV DEBIAN_FRONTEND=dialog

####################  RUNTIME STAGE ####################
FROM --platform=${TARGETARCH} ros:${ROS_DISTRO}-ros-base

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Environment settings
ENV PYTHONPATH="${PYTHONPATH}:/usr/local/lib"
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# ----- install dependencies -----
# Install runtime dependencies for librealsense and general tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 udev apt-transport-https ca-certificates curl

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
        gnupg \
        gnupg2 \
        lsb-release \
        wget \
        curl \
        python3-pip \
        libegl1-mesa-dev \
        libglu1-mesa-dev \
        libxv1 \
        libxtst6

# Install needed elements from Ubuntu Universe
# RUN apt-get update && add-apt-repository -y universe && \
#      apt-get update && \
#      apt-get install -y --no-install-recommends usbutils joystick evtest
RUN apt-get update && \
    apt-get install -y --no-install-recommends usbutils joystick evtest

# Install core python tools
RUN apt-get update && apt install -y build-essential \ 
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool python3-rosdep \
    && apt-get install python-is-python3

# Install PlatformIO
RUN apt-get update && apt-get install -y pipx \ 
    && pipx install platformio \
    && pipx ensurepath

# Install ROS 2 packagess
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 2 core packages
    ros-${ROS_DISTRO}-desktop \
    # ROS 2 control stack
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    libserial-dev \
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
    ros-${ROS_DISTRO}-twist-mux

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/profile.d/ros2.sh

# Copy the RealSense SDK binaries and assets from the builder stage
COPY --from=realsense_builder /opt/librealsense /usr/local/
COPY --from=realsense_builder /opt/librealsense /opt/librealsense
COPY --from=realsense_builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --chown=${USER_UID}:${USER_GID} realsense_ros /workspaces/Cislune-RE-RASSOR/realsense-ros

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update
RUN rosdep install -i --from-path /workspaces/Cislune-RE-RASSOR/realsense-ros \
    --rosdistro ${ROS_DISTRO} --skip-keys="launch_pytest" -y

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME
WORKDIR /workspaces/Cislune-RE-RASSOR/