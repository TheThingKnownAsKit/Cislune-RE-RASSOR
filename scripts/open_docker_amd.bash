# Compose and start the AMD container
docker compose -f docker-compose.yaml -f docker-compose.amd.yaml up -d

# Open the folder via VS Code CLI
CID=$(docker ps -qf name=ros2_dev)
code --folder-uri="vscode-remote://attached-container+$(printf '%s' "$CID" | xxd -p)/workspaces/Cislune-RE-RASSOR"
