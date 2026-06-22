#!/usr/bin/env bash
# build_deb.sh — assemble an installable .deb from already-built artifacts.
#
# Produces stm32-data-reader_<version>_arm64.deb that, once installed on the
# Pi with `sudo apt install ./<file>.deb` (or `sudo dpkg -i`), drops the
# reader binary, firmware + flash helpers and systemd units in place, flashes
# the STM32 and starts the services — replacing the old tar.gz + install.py
# flow.
#
# Inputs (env overridable):
#   VERSION       package version, no leading 'v'   (default: 1.0.0)
#   READER_BIN    path to the stm32_data_reader binary
#   FIRMWARE_DIR  dir holding wombat.bin + flash_*.sh / *_gpio.sh / reset_*.sh
#   OUTPUT_DIR    where the .deb is written          (default: dist)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

VERSION="${VERSION:-1.0.0}"
READER_BIN="${READER_BIN:-$REPO_ROOT/build/stm32_data_reader}"
FIRMWARE_DIR="${FIRMWARE_DIR:-$REPO_ROOT/firmware/flashFiles}"
OUTPUT_DIR="${OUTPUT_DIR:-$REPO_ROOT/dist}"
PKG=stm32-data-reader
ARCH=arm64

DEBIAN_SRC="$REPO_ROOT/packaging/debian"
SERVICE_SRC="$REPO_ROOT/systemd/stm32_data_reader.service"
LCM_SERVICE_SRC="$REPO_ROOT/systemd/lcm-loopback-multicast.service"

# wombat.bin may sit next to the flash scripts (CI staging) or in firmware/build
WOMBAT_BIN="${WOMBAT_BIN:-}"
if [[ -z "$WOMBAT_BIN" ]]; then
  if [[ -f "$FIRMWARE_DIR/wombat.bin" ]]; then
    WOMBAT_BIN="$FIRMWARE_DIR/wombat.bin"
  elif [[ -f "$REPO_ROOT/firmware/build/Firmware/wombat.bin" ]]; then
    WOMBAT_BIN="$REPO_ROOT/firmware/build/Firmware/wombat.bin"
  fi
fi

echo "▶ Packaging $PKG v$VERSION ($ARCH)"

# --- Preflight ---
missing=0
for f in "$READER_BIN" "$WOMBAT_BIN" \
         "$FIRMWARE_DIR/flash_wombat.sh" "$FIRMWARE_DIR/init_gpio.sh" \
         "$FIRMWARE_DIR/reset_coprocessor.sh" \
         "$SERVICE_SRC" "$LCM_SERVICE_SRC"; do
  if [[ -z "$f" || ! -f "$f" ]]; then
    echo "  ✗ missing: ${f:-<unset>}" >&2
    missing=1
  fi
done
[[ "$missing" -eq 0 ]] || { echo "Aborting: required input files are missing." >&2; exit 1; }

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

# --- Filesystem layout ---
install -d "$STAGE/usr/bin"
install -d "$STAGE/usr/lib/$PKG/firmware"
install -d "$STAGE/lib/systemd/system"
install -d "$STAGE/DEBIAN"

# Reader binary
install -m 0755 "$READER_BIN" "$STAGE/usr/bin/stm32_data_reader"

# Firmware + flash helpers (flash_wombat.sh cd's to its own dir, so keep together)
install -m 0644 "$WOMBAT_BIN"                          "$STAGE/usr/lib/$PKG/firmware/wombat.bin"
install -m 0755 "$FIRMWARE_DIR/flash_wombat.sh"        "$STAGE/usr/lib/$PKG/firmware/flash_wombat.sh"
install -m 0755 "$FIRMWARE_DIR/init_gpio.sh"           "$STAGE/usr/lib/$PKG/firmware/init_gpio.sh"
install -m 0755 "$FIRMWARE_DIR/reset_coprocessor.sh"   "$STAGE/usr/lib/$PKG/firmware/reset_coprocessor.sh"

# systemd units. The reader unit is the single source of truth in systemd/;
# rewrite its home-dir paths to FHS locations for the packaged install:
#   ExecStart binary           -> /usr/bin/stm32_data_reader
#   WorkingDirectory / cmd_trace -> /var/lib/stm32-data-reader
sed -e 's#^ExecStart=/home/pi/stm32_data_reader/stm32_data_reader#ExecStart=/usr/bin/stm32_data_reader#' \
    -e 's#/home/pi/stm32_data_reader#/var/lib/stm32-data-reader#g' \
    "$SERVICE_SRC" > "$STAGE/lib/systemd/system/stm32_data_reader.service"
install -m 0644 "$LCM_SERVICE_SRC" "$STAGE/lib/systemd/system/lcm-loopback-multicast.service"

# --- DEBIAN control + maintainer scripts ---
sed "s/__VERSION__/$VERSION/" "$DEBIAN_SRC/control" > "$STAGE/DEBIAN/control"
for script in postinst prerm postrm; do
  install -m 0755 "$DEBIAN_SRC/$script" "$STAGE/DEBIAN/$script"
done

# --- Build ---
mkdir -p "$OUTPUT_DIR"
DEB_PATH="$OUTPUT_DIR/${PKG}_${VERSION}_${ARCH}.deb"
dpkg-deb --root-owner-group --build "$STAGE" "$DEB_PATH"

echo "✔ Built $DEB_PATH"
dpkg-deb --info "$DEB_PATH" | sed 's/^/    /'
