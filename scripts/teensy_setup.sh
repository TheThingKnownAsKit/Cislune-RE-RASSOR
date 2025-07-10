#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         teensy_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-10-2025
# LAST EDITED:  07-10-2025
# DESCRIPTION:  This script installs the Teensy 4.1 rules onto the system for
#               upload capabilities. It works on Docker or Host.
# USAGE:        ./scripts/teensy_setup.sh
# DEPENDS:      bash, curl
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

log() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
error() { echo "[ERROR] $*" >&2; exit 1; }

if ! command -v sudo &>/dev/null; then
    error "sudo is required but not installed."
fi


# ----- INSTALL RULES -----

log "Curling 00-teensy.rules."
curl -sSL https://www.pjrc.com/teensy/00-teensy.rules \
| sudo tee /etc/udev/rules.d/00-teensy.rules >/dev/null

log "Reloading triggers."
sudo udevadm control --reload-rules && sudo udevadm trigger



# ----- SETUP DIALOUT PERMMISSIONS -----

if ! id -nG "$USER" | grep -qw dialout; then
    warn "User has not been added to the dialout group."
    sudo usermod -aG dialout "$USER"

    if id -nG "$USER" | grep -qw dialout; then
        log "User successfully added to dialout group."
        log "IMPORTANT: Please log out/in or run 'newgrp dialout' to apply dialout group membership."
    else
        error "User could not be added to dialout group. Aborting."
    fi

fi
