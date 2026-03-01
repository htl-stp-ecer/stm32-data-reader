#!/usr/bin/env bash
# Installs stm32_data_reader onto a Raspberry Pi over SSH.
# Run this on your laptop after downloading a release.
#
# Usage:
#   ./install.sh                           # uses defaults
#   RPI_HOST=192.168.1.50 ./install.sh     # override Pi address
#
# Expects these files next to this script:
#   stm32_data_reader                      (ARM64 binary)
#   stm32_data_reader.service              (systemd unit)
#   lcm-loopback-multicast.service         (systemd unit)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_NAME="stm32_data_reader"
BINARY="${SCRIPT_DIR}/${PROJECT_NAME}"
SERVICE_FILE="${SCRIPT_DIR}/${PROJECT_NAME}.service"
LCM_SERVICE_FILE="${SCRIPT_DIR}/lcm-loopback-multicast.service"

REMOTE_USER="${RPI_USER:-pi}"
REMOTE_HOST="${RPI_HOST:-10.101.156.14}"
REMOTE_DIR="/home/pi/stm32_data_reader"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

# --- Preflight checks ---
missing=()
[[ -f "$BINARY" ]]           || missing+=("$BINARY")
[[ -f "$SERVICE_FILE" ]]     || missing+=("$SERVICE_FILE")
[[ -f "$LCM_SERVICE_FILE" ]] || missing+=("$LCM_SERVICE_FILE")

if [[ ${#missing[@]} -gt 0 ]]; then
  echo -e "${RED}Missing files:${NC}"
  for f in "${missing[@]}"; do echo "  $f"; done
  echo -e "${YELLOW}Make sure all release files are in the same directory as this script.${NC}"
  exit 1
fi

echo -e "${GREEN}Installing ${PROJECT_NAME} to ${REMOTE_USER}@${REMOTE_HOST}${NC}"

# --- SSH connectivity ---
echo -e "${BLUE}Testing SSH connection...${NC}"
if ! ssh -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "echo 'SSH OK'" >/dev/null 2>&1; then
  echo -e "${RED}Cannot connect to ${REMOTE_USER}@${REMOTE_HOST}${NC}"
  echo -e "${YELLOW}Override with: RPI_HOST=<ip> RPI_USER=<user> $0${NC}"
  exit 1
fi

REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

# --- Stop service ---
echo -e "${BLUE}Stopping ${PROJECT_NAME} service...${NC}"
ssh "$REMOTE" "sudo systemctl stop ${PROJECT_NAME}" 2>/dev/null || true

# --- Upload files ---
echo -e "${BLUE}Uploading files...${NC}"
ssh "$REMOTE" "mkdir -p '${REMOTE_DIR}'"
scp "$BINARY" "$REMOTE:${REMOTE_DIR}/${PROJECT_NAME}"
ssh "$REMOTE" "chmod +x '${REMOTE_DIR}/${PROJECT_NAME}'"

# --- Install systemd units ---
echo -e "${BLUE}Installing systemd services...${NC}"
scp "$LCM_SERVICE_FILE" "$REMOTE:/tmp/lcm-loopback-multicast.service"
scp "$SERVICE_FILE" "$REMOTE:/tmp/${PROJECT_NAME}.service"
ssh "$REMOTE" "sudo mv /tmp/lcm-loopback-multicast.service /etc/systemd/system/ && \
               sudo mv /tmp/${PROJECT_NAME}.service /etc/systemd/system/ && \
               sudo systemctl daemon-reload"

# --- Enable & start ---
echo -e "${BLUE}Enabling and starting services...${NC}"
ssh "$REMOTE" "sudo systemctl enable --now lcm-loopback-multicast.service"
ssh "$REMOTE" "sudo systemctl enable --now ${PROJECT_NAME}.service"

echo -e "${GREEN}Done! ${PROJECT_NAME} is running on ${REMOTE_HOST}.${NC}"
echo -e "${YELLOW}Check status: ssh ${REMOTE} 'sudo systemctl status ${PROJECT_NAME}'${NC}"
