#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         jetson_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-10-2025
# LAST EDITED:  07-31-2025
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

./scripts/docker_install.sh

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

# Note that this ONLY works for Intel Realsense D4xx series (i.e. D455, D435, D435i)

has_realsense_udev_rules=-1

for d in /etc/udev/rules.d /lib/udev/rules.d; do
    if [[ -e "$d/99-realsense-libusb.rules" ]]; then
        has_realsense_udev_rules=1
        break
    fi
done

if (( has_realsense_udev_rules != 1)); then
    log "Intel Realsense udev rules not installed. Installing..."

    wget -O 99-realsense-libusb.rules \
        https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    
    sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
    rm 99-realsense-libusb.rules

    sudo udevadm control --reload-rules
    sudo udevadm trigger

    has_realsense_udev_rules=-1
    for d in /etc/udev/rules.d /lib/udev/rules.d; do
        if [[ -e "$d/99-realsense-libusb.rules" ]]; then
            has_realsense_udev_rules=1
            break
        fi
    done

    if (( has_realsense_udev_rules != 1)); then
        error "Could not install Realsense udev rules. Aborting."
    else
        log "Realsense udev rules installed successfully."
    fi
fi



# ----- GIT SETUP -----

# Ensure that a .gitconfig file exists even if it's empty (for git container)
touch ~/.gitconfig



# ----- MOSH SETUP -----
./scripts/mosh_setup.sh -s