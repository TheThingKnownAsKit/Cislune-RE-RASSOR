#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         groundstation_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-10-2025
# LAST EDITED:  07-31-2025
# DESCRIPTION:  This script performs all of the groundstaion computer setup
#               necessary to enable full Docker functionality with the camera,
#               joystick, and mosh. It assumes you're going to be using the
#               full groundstation + Jetson Orin Nano setup.
# USAGE:        ./scripts/groundstation_setup.sh
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



# ----- MOSH SETUP -----

./scripts/mosh_setup -c