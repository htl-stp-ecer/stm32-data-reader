#!/usr/bin/env bash
set -euo pipefail

# === Config ===
PLATFORM="${PLATFORM:-linux/arm64/v8}"     # Pi OS 64-bit; for 64-bit use linux/arm64/v8
TARGET_STAGE="${TARGET_STAGE:-artifacts}"
OUT_DIR="${OUT_DIR:-build}"
PROGRESS="${PROGRESS:-plain}"

echo "▶ Building ($PLATFORM) via Docker Buildx → $OUT_DIR/"
mkdir -p "$OUT_DIR"

# Ensure a buildx builder exists (create if missing)
if ! docker buildx inspect >/dev/null 2>&1; then
  echo "• No buildx builder found; creating 'cross'..."
  docker buildx create --name cross --use >/dev/null
  docker buildx inspect --bootstrap >/dev/null
fi

# Ensure binfmt/QEMU is installed and working
if ! docker run --rm --platform=linux/arm/v7 alpine uname -m >/dev/null 2>&1; then
  echo "• QEMU not working — installing binfmt emulators (requires privileged)..."
  docker run --rm --privileged tonistiigi/binfmt --install all
  # verify again
  if ! docker run --rm --platform=linux/arm/v7 alpine uname -m >/dev/null 2>&1; then
    echo "✖ QEMU emulation still not working (check kernel binfmt_misc support)."
    exit 1
  fi
fi

# Build and export the artifact(s)
docker buildx build \
  --platform "$PLATFORM" \
  --target "$TARGET_STAGE" \
  --output "type=local,dest=${OUT_DIR}" \
  --progress="$PROGRESS" \
  .

BIN="${OUT_DIR}/stm32_data_reader"
if [[ ! -f "$BIN" ]]; then
  echo "✖ Build succeeded but ${BIN} not found (check Dockerfile artifacts stage)."
  exit 1
fi

chmod +x "$BIN"
echo "✓ Built $(basename "$BIN") → $BIN"
file "$BIN" || true
