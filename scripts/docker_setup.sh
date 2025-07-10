#!/usr/bin/env bash
#
# -----------------------------------------------------------------------------
# FILE:         docker_setup.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      07-10-2025
# LAST EDITED:  07-10-2025
# DESCRIPTION:  This script performs all of the necessary commands to build a
#               docker container, compose it with the appropriate compose files,
#               and attach the VSCode window to it.
#
# USAGE:        ./scripts/docker_setup.sh
#
# OPTIONS:
#   -r Rebuild docker images (no cache)
#   -c Recreate containers (--force-recreate)
#   -v Attach VS Code after setup
#   -h Show this help message and exit
#
# DEPENDS:      bash, docker engine, docker engine cli, docker compose, vscode
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

log() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
error() { echo "[ERROR] $*" >&2; exit 1; }

if ! command -v sudo &>/dev/null; then
    error "sudo is required but not installed."
fi

REBUILD_IMAGE=0
RECREATE_CONTAINERs=0
ATTACH_VSCODE=0

usage() {
  cat << EOF
Usage: $(basename "$0") [-r] [-c] [-v]
  -r    Rebuild Docker container (no cache)
  -c    Recreate containers (--force-recreate)
  -v    Attach VS Code after setup
  -h    Show this help message
EOF
  exit 1
}

# Parse options
while getopts ":rcvh" opt; do
  case "$opt" in
    r) REBUILD_IMAGE=1 ;;
    c) RECREATE_CONTAINERs=1 ;;
    v) ATTACH_VSCODE=1 ;;
    h) usage ;;
    \?) echo "Invalid option: -$OPTARG"; usage ;;
  esac
done
shift $((OPTIND - 1))



# ----- DEPENDENCY CHECK ------

# Check that Docker Engine is installed
if command -v docker &>/dev/null; then
    if docker --version &>/dev/null; then
        log "Docker Engine is installed: $(docker --version)"
    else
        warn "Found 'docker' but failed version check."
    fi
else
    error "Docker Engine not found, aborting. Please install manaully via apt repository or run ./scripts/<relevant option>_setup.sh"
fi

# Check that Docker Buildx is installed
if docker buildx version &>/dev/null; then
    log "Docker Buildx plugin is installed: $(docker buildx version)"
else
    error "Docker Buildx not found, aborting. Please install manaully via apt repository or run ./scripts/<relevant option>_setup.sh"
fi

# Check that Docker Compose is installed (v2)
if docker compose version &>/dev/null; then
    log "Docker Compose (v2 plugin) is installed: $(docker compose version)"
else
    error "Docker Compose V2 not found, aborting. Please install manaully via apt repository or run ./scripts/<relevant option>_setup.sh"
fi


# ----- CONFIGURATION CHECK ------

# Buildx must have multiarch enabled
if docker buildx inspect multiarch &>/dev/null; then
    log "Docker Buildx multiarch enabled."
else
    error "Docker does not have multiarch enabled. Aborting."
fi

# Ensure that there is a Docker permission group and that the user is in it
if ! getent group docker >/dev/null; then
    error "There is no docker permission group, aborting. Please create it manually or run ./scripts/<relevant option>_setup.sh"
fi

if ! id -nG "$USER" | grep -qw docker; then
    error "User has not been added to the docker group, aborting. Please join it manually or run ./scripts/<relevant option>_setup.sh"
fi



# ----- CONTAINER CONSTRUCTION -----

COMPOSE_FILES=()

# Pick compose files based on GPU
if lspci | grep -i 'vga' | grep -iq 'nvidia'; then
    log "Nvidia GPU detected, selecting Nvidia compose files"
    COMPOSE_FILES=(-f docker-compose.yaml -f docker-compose.nvidia.yaml)
else
    log "No Nvidia GPU detected, selecting Non-Nvidia compose files"
    COMPOSE_FILES=(-f docker-compose.yaml -f docker-compose.notnvidia.yaml)
fi

# Navigate to the docker/ directory
pushd docker/ >/dev/null || error "Cislune-RE-RASSOR/docker/ directory does not exist. Are you in repository root? Aborting."

if [ "$REBUILD_IMAGE" -eq 1 ]; then
    log "Running compose build with no cache..."
    docker compose "${COMPOSE_FILES[@]}" build --no-cache
else
    log "Running compose build with cache..."
    docker compose "${COMPOSE_FILES[@]}" build
fi

if [ "$RECREATE_CONTAINERs" -eq 1 ]; then
    log "Running compose up with --force-recreate..."
    docker compose "${COMPOSE_FILES[@]}" up --force-recreate --detach
else
    log "Running compose up..."
    docker compose "${COMPOSE_FILES[@]}" up --detach
fi

# Navigate back to repository root
popd >/dev/null



# ----- ATTACH VSCODE -----

if [ "$ATTACH_VSCODE" -eq 1 ]; then
    # Open the folder via VS Code CLI
    CID=$(docker ps -qf name=ros2_dev)
    code --folder-uri="vscode-remote://attached-container+$(printf '%s' "$CID" | xxd -p)/workspaces/Cislune-RE-RASSOR"
fi