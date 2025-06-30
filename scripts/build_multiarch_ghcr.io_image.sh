#----- THIS FILE ONLY NEEDS TO RUN WHEN UPDATING THE PUBLIC MULTI-ARCH IMAGE IN GHCR.IO -----

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."          # move from project/scripts → project/

# Make sure you set these in a .env file or export them in your shell
# OWNER: your GitHub username or organization
# IMAGE: the name of your image, e.g., "ros2"
if [[ -f .env ]]; then
    set -a               # export every variable we source
    source .env          # load PAT, OWNER, IMAGE, etc.
    set +a
else
    echo "❌ .env not found; aborting"; exit 1
fi
: "${PAT:?PAT missing in .env}"
: "${OWNER:?OWNER missing}"
: "${IMAGE:?IMAGE missing}"
export TAG=ghcr.io/${OWNER}/${IMAGE}

# Build container if it doesn't exist
if ! docker buildx inspect multiarch >/dev/null 2>&1; then
  docker buildx create --name multiarch --driver docker-container --bootstrap --use
else
  docker buildx use multiarch
fi

docker run --privileged --rm tonistiigi/binfmt --install all

# one-time login with a PAT that has write:packages scope
if ! docker --config ~/.docker run --rm ghcr.io/hello-world >/dev/null 2>&1; then
    echo "$PAT" | docker login ghcr.io -u "$OWNER" --password-stdin
fi

# build & push
echo "Building and pushing multi-arch image for $TAG"
docker buildx build \
    --file .devcontainer/Dockerfile \
    --platform linux/amd64,linux/arm64 \
    -t $TAG \
    --push \
    .

echo docker buildx imagetools inspect $TAG   # shows linux/amd64 and linux/arm64