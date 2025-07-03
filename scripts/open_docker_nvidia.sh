# Compose and start the AMD container
docker compose -f docker-compose.yaml -f docker-compose.nvidia.yaml up -d

# Give the container access to computer
xhost +local:docker

# Open the folder via VS Code CLI
CID=$(docker ps -qf name=ros2_dev)
code --folder-uri="vscode-remote://attached-container+$(printf '%s' "$CID" | xxd -p)/workspaces/Cislune-RE-RASSOR"
