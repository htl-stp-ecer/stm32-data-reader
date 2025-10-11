#!/usr/bin/env bash
set -euo pipefail

PROJECT_NAME="${PROJECT_NAME:-stm32_data_reader}"
BUILD_DIR="${BUILD_DIR:-build}"
BINARY_PATH="${BINARY_PATH:-${BUILD_DIR}/${PROJECT_NAME}}"

REMOTE_USER="${RPI_USER:-pi}"
REMOTE_HOST="${RPI_HOST:-192.168.178.65}"
REMOTE_DIR="${RPI_DIR:-/home/pi/stm32_data_reader}"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

echo -e "${GREEN}Deploying ${PROJECT_NAME} to Raspberry Pi...${NC}"
echo -e "${BLUE}Target: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}${NC}"

echo -e "${YELLOW}Building binary (containerized ARM toolchain)...${NC}"
bash ./build.sh

if [[ ! -f "${BINARY_PATH}" ]]; then
  echo -e "${RED}Error: Binary not found after build at ${BINARY_PATH}${NC}"
  exit 1
fi

echo -e "${GREEN}Testing SSH connection...${NC}"
if ! ssh -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "echo 'SSH OK'"; then
  echo -e "${RED}Error: Cannot connect to ${REMOTE_USER}@${REMOTE_HOST}${NC}"
  echo -e "${YELLOW}Make sure:${NC}"
  echo -e "${YELLOW}  1. SSH is enabled on the Pi${NC}"
  echo -e "${YELLOW}  2. You can reach the Pi at ${REMOTE_HOST}${NC}"
  echo -e "${YELLOW}  3. SSH keys are set up or password auth available${NC}"
  echo -e "${YELLOW}Set environment variables if needed:${NC}"
  echo -e "${YELLOW}  export RPI_USER=your_username${NC}"
  echo -e "${YELLOW}  export RPI_HOST=your_pi_ip_or_hostname${NC}"
  echo -e "${YELLOW}  export RPI_DIR=/path/to/deploy/directory${NC}"
  exit 1
fi

echo -e "${GREEN}Stopping existing service (if any)...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "sudo systemctl stop stm32_data_reader" || true

echo -e "${GREEN}Creating remote directory...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '${REMOTE_DIR}'"

echo -e "${GREEN}Copying binary...${NC}"
scp "${BINARY_PATH}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}/"

if [[ -f "systemd/${PROJECT_NAME}.service" ]]; then
  echo -e "${GREEN}Copying systemd service...${NC}"
  scp "systemd/${PROJECT_NAME}.service" "${REMOTE_USER}@${REMOTE_HOST}:/tmp/"
  ssh "${REMOTE_USER}@${REMOTE_HOST}" "sudo mv /tmp/${PROJECT_NAME}.service /etc/systemd/system/${PROJECT_NAME}.service && sudo systemctl daemon-reload"
fi

echo -e "${GREEN}Making binary executable...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "chmod +x '${REMOTE_DIR}/${PROJECT_NAME}'"

echo -e "${GREEN}Deployment complete!${NC}"
echo -e "${BLUE}Binary deployed to: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}/${PROJECT_NAME}${NC}"
echo -e "${YELLOW}To run: ssh ${REMOTE_USER}@${REMOTE_HOST} '${REMOTE_DIR}/${PROJECT_NAME}'${NC}"
if [[ -f "systemd/${PROJECT_NAME}.service" ]]; then
  echo -e "${YELLOW}To start as service: ssh ${REMOTE_USER}@${REMOTE_HOST} 'sudo systemctl enable --now ${PROJECT_NAME}'${NC}"
fi
