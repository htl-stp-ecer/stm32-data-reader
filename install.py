#!/usr/bin/env python3
"""install.py — Install stm32_data_reader onto a Raspberry Pi over SSH.

Run this on your laptop after downloading a release.

Usage:
    python install.py                           # uses defaults
    RPI_HOST=192.168.1.50 python install.py     # override Pi address

Expects these files next to this script:
    stm32_data_reader                      (ARM64 binary)
    stm32_data_reader.service              (systemd unit)
    lcm-loopback-multicast.service         (systemd unit)
Optional firmware files (if present, firmware will be flashed):
    wombat.bin                             (STM32 firmware binary)
    flash_wombat.sh                        (flash script)
    reset_coprocessor.sh                   (reset script)
    init_gpio.sh                           (GPIO init script)

Env vars:
    RPI_HOST  — Pi IP address (default: 10.101.156.14)
    RPI_USER  — Pi SSH user   (default: pi)
"""

import os
import subprocess
import sys
from pathlib import Path

# ANSI colors
RED = "\033[0;31m"
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
BLUE = "\033[0;34m"
NC = "\033[0m"


def color(text: str, code: str) -> str:
    return f"{code}{text}{NC}"


def ssh(host: str, user: str, command: str, check: bool = True) -> int:
    """Run a command on the Pi via SSH."""
    result = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", f"{user}@{host}", command],
        capture_output=not check,
    )
    if check and result.returncode != 0:
        print(color(f"ERROR: SSH command failed: {command}", RED))
        sys.exit(1)
    return result.returncode


def scp(source: str, dest: str) -> None:
    """Copy a file to the Pi via SCP."""
    result = subprocess.run(["scp", source, dest])
    if result.returncode != 0:
        print(color(f"ERROR: SCP failed: {source} -> {dest}", RED))
        sys.exit(1)


def main() -> None:
    script_dir = Path(__file__).resolve().parent

    project_name = "stm32_data_reader"
    binary = script_dir / project_name
    service_file = script_dir / f"{project_name}.service"
    lcm_service_file = script_dir / "lcm-loopback-multicast.service"

    remote_user = os.environ.get("RPI_USER", "pi")
    remote_host = os.environ.get("RPI_HOST", "10.101.156.14")
    remote_dir = "/home/pi/stm32_data_reader"
    remote_flash_dir = "/home/pi/flashFiles"

    # --- Preflight checks ---
    missing = []
    if not binary.is_file():
        missing.append(str(binary))
    if not service_file.is_file():
        missing.append(str(service_file))
    if not lcm_service_file.is_file():
        missing.append(str(lcm_service_file))

    if missing:
        print(color("Missing files:", RED))
        for f in missing:
            print(f"  {f}")
        print(color("Make sure all release files are in the same directory as this script.", YELLOW))
        sys.exit(1)

    # Check if firmware files are present
    has_firmware = (script_dir / "wombat.bin").is_file() and (script_dir / "flash_wombat.sh").is_file()

    remote = f"{remote_user}@{remote_host}"

    print(color(f"Installing {project_name} to {remote}", GREEN))
    if has_firmware:
        print(color("Firmware files found — will flash STM32", GREEN))

    # --- SSH connectivity ---
    print(color("Testing SSH connection...", BLUE))
    if ssh(remote_host, remote_user, "echo 'SSH OK'", check=False) != 0:
        print(color(f"Cannot connect to {remote}", RED))
        print(color(f"Override with: RPI_HOST=<ip> RPI_USER=<user> python {__file__}", YELLOW))
        sys.exit(1)

    # --- Stop service ---
    print(color(f"Stopping {project_name} service...", BLUE))
    ssh(remote_host, remote_user, f"sudo systemctl stop {project_name}", check=False)

    # --- Flash firmware (if available) ---
    if has_firmware:
        print(color("Uploading firmware files...", BLUE))
        ssh(remote_host, remote_user, f"mkdir -p '{remote_flash_dir}'")
        for fname in ["wombat.bin", "flash_wombat.sh", "reset_coprocessor.sh", "init_gpio.sh"]:
            fpath = script_dir / fname
            if fpath.is_file():
                scp(str(fpath), f"{remote}:{remote_flash_dir}/{fname}")

        print(color("Flashing STM32 firmware...", BLUE))
        rc = ssh(remote_host, remote_user, f"cd '{remote_flash_dir}' && bash ./flash_wombat.sh", check=False)
        if rc != 0:
            print(color("STM32 flash failed!", RED))
            sys.exit(1)

    # --- Upload reader binary ---
    print(color("Uploading reader binary...", BLUE))
    ssh(remote_host, remote_user, f"mkdir -p '{remote_dir}'")
    scp(str(binary), f"{remote}:{remote_dir}/{project_name}")
    ssh(remote_host, remote_user, f"chmod +x '{remote_dir}/{project_name}'")

    # --- Install systemd units ---
    print(color("Installing systemd services...", BLUE))
    scp(str(lcm_service_file), f"{remote}:/tmp/lcm-loopback-multicast.service")
    scp(str(service_file), f"{remote}:/tmp/{project_name}.service")
    ssh(
        remote_host,
        remote_user,
        f"sudo mv /tmp/lcm-loopback-multicast.service /etc/systemd/system/ && "
        f"sudo mv /tmp/{project_name}.service /etc/systemd/system/ && "
        "sudo systemctl daemon-reload",
    )

    # --- Enable & start ---
    print(color("Enabling and starting services...", BLUE))
    ssh(remote_host, remote_user, "sudo systemctl enable --now lcm-loopback-multicast.service")
    ssh(remote_host, remote_user, f"sudo systemctl enable --now {project_name}.service")

    # --- Restart services ---
    print(color("Restarting services...", BLUE))
    ssh(remote_host, remote_user, "sudo systemctl restart lcm-loopback-multicast.service")
    ssh(remote_host, remote_user, f"sudo systemctl restart {project_name}.service")

    print(color(f"Done! {project_name} is running on {remote_host}.", GREEN))
    print(color(f"Check status: ssh {remote} 'sudo systemctl status {project_name}'", YELLOW))


if __name__ == "__main__":
    main()
