#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         mosh_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-31-2025
# LAST EDITED:  07-31-2025
# DESCRIPTION:  This script sets up a mosh server that allows ssh connections from
#               another computer on the same network. You can connect via the command
#               mosh nostromo@ubuntu
#
# USAGE:        ./scripts/mosh_setup.sh
#
# OPTIONS:
#   -s Setup the mosh server
#   -c Setup the mosh client
# 
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

USER_OPTION=""

usage() {
    cat << EOF
Usage: $(basename "$0") [-s] [-c]
    -s  Setup the mosh server
    -c  Setup the mosh client
    -h  Show this help message
EOF
    exit 1
}

# Parse options
while getopts ":sch" opt; do
    case "$opt" in
        s) USER_OPTION="server" ;;
        c) USER_OPTION="client" ;;
        h) usage ;;
        \?) echo "Invalid option: -$OPTARG"; usage ;;
    esac
done
shift $((OPTIND - 1))



# ----- SERVER SETUP -----

if [ "$USER_OPTION" = "server" ]; then
    # Download required packages
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
        openssh-server mosh ufw avahi-daemon avahi-utils libnss-mdns

    # Enable and start services
    systemctl enable --now ssh
    systemctl enable --now avahi-daemon
    hostnamectl set-hostname nostromo

    # Set up UFW rules
    ufw --force enable
    ufw allow 22/tcp            # ssh
    ufw allow 5353/udp          # mDNS/Bonjour
    ufw allow 60000:61000/udp   # mosh default
fi



# ----- CLIENT SETUP -----

if [ "$USER_OPTION" = "client" ]; then
    # Download required packages
    sudo apt-get update
    sudo apt-get install -y libnss-mdns mosh

    ping nostromo@ubuntu
fi
