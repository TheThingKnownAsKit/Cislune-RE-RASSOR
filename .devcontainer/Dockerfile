ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04

####################  BUILD STAGE  ####################

# ---------- amd64 ----------
FROM --platform=linux/amd64 ros:${ROS_DISTRO}-ros-base AS amd64

# ---------- jetson arm64 ----------
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base AS arm64

RUN apt-get update && rm -rf /var/lib/apt/lists/*

####################  RUNTIME STAGE  ####################
ARG TARGETARCH
FROM $TARGETARCH AS final

# ----- create user that owns host files -----
ARG HOST_UID=1000
ARG HOST_GID=${HOST_UID}
ARG USERNAME=ros2dev

RUN groupadd -g ${HOST_GID} ${USERNAME} \
 && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${USERNAME} \
 && apt-get update && apt-get install -y sudo \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && sudo usermod -aG video,dialout ${USERNAME}

USER ${USERNAME}

WORKDIR /workspaces/Cislune-RE-RASSOR
