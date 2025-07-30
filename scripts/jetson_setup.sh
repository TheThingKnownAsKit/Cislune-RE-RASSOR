#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         jetson_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-10-2025
# LAST EDITED:  07-30-2025
# DESCRIPTION:  This script performs all of the rover computer necessary setup to
#               enable full teleop and autonomy functionality. It assumes you're
#               going to be using the full groundstation + Jetson Orin Nano setup.
# USAGE:        ./scripts/jetson_setup.sh
# DEPENDS:      bash
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

log() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
error() { echo "[ERROR] $*" >&2; exit 1; }

if ! command -v sudo &>/dev/null; then
    error "sudo is required but not installed."
fi



# ----- DOCKER SETUP -----

has_docker_engine=-1
has_docker_buildx=-1
has_docker_compose=-1

# Check that Docker Engine is installed
if command -v docker &>/dev/null; then
    if docker --version &>/dev/null; then
        log "Docker Engine is installed: $(docker --version)"
        has_docker_engine=1
    else
        warn "Found 'docker' but failed version check."
    fi
else
    warn "Docker Engine not found."
    log "Setting install Docker Engine flag to true."
    has_docker_engine=0
fi

# Check that Docker Buildx is installed
if docker buildx version &>/dev/null; then
    log "Docker Buildx plugin is installed: $(docker buildx version)"
    has_docker_buildx=1
else
    warn "Docker Buildx not found."
    log "Setting install Docker Builx flag to true."
    has_docker_buildx=0
fi

# Check that Docker Compose is installed (v2)
if docker compose version &>/dev/null; then
    log "Docker Compose (v2 plugin) is installed: $(docker compose version)"
    has_docker_compose=1
else
    warn "Docker Compose V2 not found."
    log "Setting install Docker Compose V2 flag to true."
    has_docker_compose=0
fi

# Install any missing Docker services
if (( has_docker_engine != 1 || has_docker_buildx != 1 || has_docker_compose != 1)); then
    log "Missing Docker installation detected. Installing using apt..."
    
    # From official Docker docs: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository 
    # Add Docker's official GPG key:
    log "Adding Docker's official GPG key.."
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    log "Adding Docker repository to Apt sources..."
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update

    if (( has_docker_engine != 1)); then
        log "Installing Docker Engine and necessary dependencies..."
        sudo apt-get install docker-ce docker-ce-cli containerd.io

        if docker --version &>/dev/null; then
            log "Docker Engine and necessary dependencies installed successfully."
        else
            error "Installation failed, please attempt installing manually. Aborting."
        fi
    fi

    if (( has_docker_buildx != 1)); then
        log "Installing Docker Buildx..."
        sudo apt-get install docker-buildx-plugin

        if docker buildx version &>/dev/null; then
            log "Docker Buildx installed successfully."
        else
            error "Installation failed, please attempt installing manually. Aborting."
        fi
    fi

    if (( has_docker_compose != 1 )); then
        log "Installing Docker Compose V2..."
        sudo apt-get install docker-compose-plugin

        if docker compose version &>/dev/null; then
            log "Docker Compose V2 installed successfully."
        else
            error "Installation failed, please attempt installing manually. Aborting."
        fi
    fi
fi

# Buildx must have multiarch enabled
if docker buildx inspect multiarch &>/dev/null; then
    log "Docker Buildx multiarch enabled."
else
    warn "Docker does not have multiarch enabled. Enabling..."
    docker run --privileged --rm tonistiigi/binfmt --install all
    docker buildx create --name multiarch --driver docker-container --boostrap --use
fi


# Ensure that there is a Docker permission group and that the user is in it
if ! getent group docker >/dev/null; then
    warn "There is no docker permission group. Creating docker usergroup."
    sudo groupadd docker
fi

if ! id -nG "$USER" | grep -qw docker; then
    warn "User has not been added to the docker group."
    sudo usermod -aG docker "$USER"

    if id -nG "$USER" | grep -qw docker; then
        log "User successfully added to docker group."
        log "IMPORTANT: Please log out/in or run 'newgrp docker' to apply Docker group membership."
    else
        error "User could not be added to docker group. Aborting."
    fi

fi

log "Giving Docker local xhost access only for this user."
xhost +SI:localuser:"$(whoami)"



# ----- NVIDIA SETUP -----

# Make sure nvidia-smi is accessible (drivers) and container toolkit
if nvidia-smi &>/dev/null; then
    log "Nvidia GPU drivers are installed."
else
    error "Nvidia GPU drivers are not detected."
fi

if command -v nvidia-ctk &>/dev/null; then
    log "Nvidia Container Toolkit is installed: $(nvidia-ctk --version)"
else
    warn "Nvidia Container Toolkit is not installed. Installing..."
    sudo apt-get install nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker

    if command -v nvidia-ctk &>/dev/null; then
        log "Nvidia Container Toolkit successfully installed."
    else
        error "Cannot install Nvidia Container Toolkit. Aborting."
    fi
fi



# ----- CAMERA SETUP -----
has_turbojpeg=0

# Accept both runtime and -dev packages
if dpkg -s libturbojpeg >/dev/null 2>&1 || dpkg -s libjpeg-turbo* >/dev/null 2>&1; then
    # Look for the .so on any architecture directory (x86_64, aarch64, armhf ...)
    if find /usr/lib -maxdepth 2 -name 'libturbojpeg.so*' | grep -q . ; then
        log "libturbojpeg detected: $(dpkg -s libturbojpeg | grep Version)"
        has_turbojpeg=1
    fi
fi

if (( has_turbojpeg == 1 )); then
    log "libturbojpeg shared library found."
else
    warn "libturbojpeg shared library NOT found."
    sudo apt-get update
    sudo apt-get install libturbojpeg libturbojpeg0-dev

    if find /usr/lib -maxdepth 2 -name 'libturbojpeg.so*' | grep -q . ; then
        log "libturbojpeg successfully installed."
    else
        error "libturbojpeg could not be installed. Aborting."
    fi
fi



# ----- GIT SETUP -----

# Ensure that a .gitconfig file exists even if it's empty (for git container)
touch ~/.gitconfig



# ----- MOSH SETUP -----
./scripts/mosh_server_setup.sh



# ----- CLOSING -----
log "IMPORTANT: If your user was added to the docker group during this script, please log out/in or run 'newgrp docker' to apply Docker group membership."