ARG ROS_DISTRO=jazzy
ARG UBUNTU_VER=24.04
ARG RUNTIME=
ARG TARGETARCH=
ARG CUDA_VARIANT=off

####################  SELECT BASE BY ARCH  ####################
FROM ${CUDA_VARIANT}-selector as choose

# ---------- amd64 ----------
FROM --platform=linux/amd64 ros:${ROS_DISTRO}-ros-base AS amd64

# ---------- jetson arm64 ----------
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base AS jetson

# Build and push both arch variants with the below command
# docker buildx build \
#   --push \
#   --platform linux/amd64,linux/arm64 \
#   -t ghcr.io/yourorg/ros2:jazzy-dev .

####################  RUNTIME STAGE  ####################
FROM ${TARGETARCH} AS final            # BuildKit selects amd64/arm64 stage
COPY --from=${TARGETARCH} / /          # copy rootfs
RUN useradd -m ros2dev && usermod -aG dialout,video vscode
USER ros2dev                           # good practice for devcontainer
WORKDIR /workspaces/rover_ros2 # might need to change this