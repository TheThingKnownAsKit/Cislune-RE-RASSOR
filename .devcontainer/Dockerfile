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
ARG USERNAME=ros2dev
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# ----- create user that owns host files -----
# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
USER ${USERNAME}

# Give user relevant permissions
RUN sudo usermod --append --groups video $USERNAME

WORKDIR /workspaces/Cislune-RE-RASSOR
