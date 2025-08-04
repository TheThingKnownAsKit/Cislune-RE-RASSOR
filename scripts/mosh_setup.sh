#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         mosh_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com> and Raegan Scheet <cscheet2@unl.edu>
# CREATED:      07-31-2025
# LAST EDITED:  07-31-2025
# DESCRIPTION:  This script sets up a mosh server that allows ssh connections from
#               another computer on the same network. You can connect via the command
#               mosh rerassor@ubuntu.local
#
# USAGE:        sudo ./scripts/mosh_setup.sh
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

log "Configuring ufw to allow UDP and DDS in/out..."
NET=$(ip -4 -o addr show scope global | awk '!/ lo / && !/ docker/ {print $4; exit}')
# 1. Multicast IP traffic
sudo ufw allow in proto udp from $NET to 224.0.0.0/4                comment 'ROS2 UDP multicast in'
sudo ufw allow out proto udp to 224.0.0.0/4                         comment 'ROS2 UDP multicast out'

# 2. RTPS discovery/data ports
sudo ufw allow in proto udp from $NET to any port 7400:7500         comment 'ROS2 DDS peer discovery/data in'
sudo ufw allow out proto udp to   $NET port 7400:7500               comment 'ROS2 DDS peer discovery/data out'

# 3. IGMP (so OS will reply to IGMP queries and properly join groups)
# No port; datagram is IGMP protocol (IP protocol 2)
# Note: 'proto igmp' works on Ubuntu Focal+, aligns with kernel modules
sudo ufw allow in proto igmp to 224.0.0.0/4                         comment 'Enable IGMP for multicast group membership'

log "Reloading ufw rules"
sudo ufw reload



# ----- SERVER SETUP -----

if [ "$USER_OPTION" = "server" ]; then
    log "Setting up mosh server..."

    # Download required packages
    apt-get update
    apt-get install -y --no-install-recommends \
        openssh-server mosh ufw avahi-daemon avahi-utils libnss-mdns

    # Enable and start services
    nmcli connection modify "Cislune" \
      connection.permissions "" \
      connection.autoconnect yes \
      connection.autoconnect-priority 10
    systemctl enable NetworkManager-wait-online.service
    hostnamectl set-hostname rerassor
    systemctl enable --now ssh
    systemctl enable --now avahi-daemon

    # Set up UFW rules
    ufw --force enable
    ufw allow 22/tcp            # ssh
    ufw allow 5353/udp          # mDNS/Bonjour
    ufw allow 60000:61000/udp   # mosh default
fi



# ----- CLIENT SETUP -----

if [ "$USER_OPTION" = "client" ]; then
    log "Setting up mosh client..."

    # Download required packages
    apt-get update
    apt-get install -y libnss-mdns mosh
fi
