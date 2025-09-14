#!/usr/bin/env bash
set -euo pipefail

# -------- Config (env overridable) --------
PROJECT_NAME="${PROJECT_NAME:-stm32_data_reader}"
BUILD_DIR="${BUILD_DIR:-build}"
PLATFORM="${PLATFORM:-linux/arm64/v8}"
IMAGE_NAME="${IMAGE_NAME:-stm32-dev:arm64}"
CCACHE_VOL="${CCACHE_VOL:-stm32-ccache}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
# Portable CPU count default
if command -v nproc >/dev/null 2>&1; then
  _cpu="$(nproc)"
else
  _cpu="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
fi
NINJA_JOBS="${NINJA_JOBS:-${_cpu}}"
FORCE_RECONFIGURE="${FORCE_RECONFIGURE:-0}"
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"
CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-3G}"

echo "▶ Building $PROJECT_NAME for $PLATFORM into ./$BUILD_DIR using $IMAGE_NAME"
mkdir -p "$BUILD_DIR"

need_builder() {
  if ! docker buildx inspect >/dev/null 2>&1; then
    echo "• No buildx builder found; creating 'cross'..."
    docker buildx create --name cross --use >/dev/null
    docker buildx inspect --bootstrap >/dev/null
  fi
}

ensure_binfmt() {
  # If Docker Desktop provides emulation, skip installing binfmt
  if docker info 2>/dev/null | grep -qi 'docker desktop'; then
    return
  fi
  # Quick probe: can we run uname on target platform?
  if ! docker run --rm --platform="$PLATFORM" alpine:3.20 uname -m >/dev/null 2>&1; then
    echo "• QEMU binfmt not working — installing emulators (requires privileged)..."
    docker run --rm --privileged tonistiigi/binfmt --install all
    docker run --rm --platform="$PLATFORM" alpine:3.20 uname -m >/dev/null
  fi
}

ensure_image() {
  if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1 || [[ "$REBUILD_IMAGE" == "1" ]]; then
    echo "• Building builder image: $IMAGE_NAME (for $PLATFORM)"
    # Important: --platform matches FROM --platform in Dockerfile (use $TARGETPLATFORM inside Dockerfile)
    docker buildx build --platform "$PLATFORM" -t "$IMAGE_NAME" --load .
  fi
}

docker_exec() {
  docker run --rm --platform="$PLATFORM" \
    -v "$PWD":/src \
    -v "$CCACHE_VOL":/ccache \
    -e CCACHE_DIR=/ccache \
    -e CCACHE_MAXSIZE="$CCACHE_MAXSIZE" \
    -e CCACHE_COMPRESS=1 \
    -w /src \
    "$IMAGE_NAME" \
    bash -lc "$*"
}

need_builder
ensure_binfmt
docker volume create "$CCACHE_VOL" >/dev/null
ensure_image

if [[ "$FORCE_RECONFIGURE" == "1" || ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
  echo "• Configuring with CMake ($CMAKE_BUILD_TYPE)"
  docker_exec "cmake -S /src -B /src/$BUILD_DIR -G Ninja \
    -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
else
  echo "• Using existing configuration at $BUILD_DIR/"
fi

echo "• Building with Ninja (-j $NINJA_JOBS)"
docker_exec "cmake --build /src/$BUILD_DIR -j$NINJA_JOBS"

BIN="${BUILD_DIR}/${PROJECT_NAME}"
if [[ ! -f "$BIN" ]]; then
  echo "✖ Build completed but binary not found: $BIN"
  exit 1
fi

chmod +x "$BIN" || true
echo "✓ Built $(basename "$BIN") → $BIN"
file "$BIN" || true
