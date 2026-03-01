#!/usr/bin/env bash
# Build and install to a Raspberry Pi.
# This is a thin wrapper: build locally, then delegate to install.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Building..."
bash "${SCRIPT_DIR}/build.sh"

# Copy systemd files next to the binary so install.sh finds them
cp "${SCRIPT_DIR}/systemd/stm32_data_reader.service" "${SCRIPT_DIR}/build/"
cp "${SCRIPT_DIR}/systemd/lcm-loopback-multicast.service" "${SCRIPT_DIR}/build/"
cp "${SCRIPT_DIR}/install.sh" "${SCRIPT_DIR}/build/"

echo "Installing to Pi..."
bash "${SCRIPT_DIR}/build/install.sh"
