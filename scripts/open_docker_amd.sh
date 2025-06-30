# Compose and start the AMD container
cd docker/
docker compose -f docker-compose.yaml -f docker-compose.amd.yaml up -d
cd ..

# Open the folder via VS Code CLI
CID=$(docker ps -qf name=ros2_dev)
code -r --folder-uri="vscode-remote://attached-container+$(printf '%s' "$CID" | xxd -p)/workspaces/Cislune-RE-RASSOR"
