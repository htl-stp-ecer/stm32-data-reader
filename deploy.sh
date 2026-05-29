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
cp "${SCRIPT_DIR}/systemd/iox2-janitor.service" "${SCRIPT_DIR}/build/"
cp "${SCRIPT_DIR}/install.py" "${SCRIPT_DIR}/build/"

# Pull the iox2 janitor binary out of the raccoon-transport build tree so
# install.py finds it next to itself. raccoon-transport is built as a
# submodule under cmake's iceoryx2_transport/cpp/tools subtree — the binary
# lands wherever CMake's RUNTIME_OUTPUT_DIRECTORY points. We probe a couple
# of likely paths so the build wiring can evolve without breaking the
# installer.
JANITOR_BIN=""
for candidate in \
    "${SCRIPT_DIR}/build/raccoon-transport-build/cpp/tools/iox2_janitor" \
    "${SCRIPT_DIR}/build/cpp/tools/iox2_janitor" \
    "${SCRIPT_DIR}/build/_deps/raccoon_transport-build/cpp/tools/iox2_janitor" \
    "${SCRIPT_DIR}/build/iox2_janitor"; do
  if [ -x "$candidate" ]; then
    JANITOR_BIN="$candidate"
    break
  fi
done
if [ -n "$JANITOR_BIN" ]; then
  echo "Found iox2 janitor at $JANITOR_BIN"
  cp "$JANITOR_BIN" "${SCRIPT_DIR}/build/iox2_janitor"
else
  echo "WARN: iox2_janitor binary not found in build tree — install.py will skip the janitor"
  echo "      (search paths: build/raccoon-transport-build/cpp/tools, build/cpp/tools, ...)"
fi

# Firmware artifacts are already copied by build.sh (wombat.bin + flash scripts)

echo "Installing to Pi..."
python3 "${SCRIPT_DIR}/build/install.py"
