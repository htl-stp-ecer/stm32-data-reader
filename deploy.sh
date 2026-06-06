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
cp "${SCRIPT_DIR}/install.py" "${SCRIPT_DIR}/build/"

# iox2_janitor is no longer built — raccoon::Transport switched off
# iceoryx2 to the in-tree raccoon_ring SHM library, so there are no dead
# iceoryx2 nodes to sweep. Leave any stale binary in place so an older
# removed from systemd), but do not error out on the missing file.

# Firmware artifacts are already copied by build.sh (wombat.bin + flash scripts)

echo "Installing to Pi..."
python3 "${SCRIPT_DIR}/build/install.py"
