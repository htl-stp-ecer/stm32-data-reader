#!/usr/bin/env python3
"""deploy.py — install a local stm32-data-reader .deb on a Pi (for development).

Copies the .deb from this machine to the Pi and installs it there with apt. Releases are
installed by `raccoon update`, not by this script. Standard library only.

Usage:
    python3 deploy.py --host 192.168.4.1                          # the .deb from the last build.py run
    python3 deploy.py --host 192.168.4.1 --artifact my.deb        # a specific .deb

stm32-data-reader is set to "dev" in the Pi's /etc/raccoon-bundle.json: a deployed .deb is a
custom build.

The package flashes the STM32 firmware when it is installed (postinst, if the hardware is there),
so every deploy flashes it again and restarts the reader.
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path

COMPONENT = "stm32-data-reader"
BUNDLE_FILE = "/etc/raccoon-bundle.json"
REMOTE_TMP = "/tmp/stm32-data-reader-install"

# Runs on the Pi as root: set this component's version to "dev" in the bundle file and
# leave everything else in it as it is.
MARK_DEV = r"""
import json, os, tempfile
path = "%(path)s"
try:
    with open(path) as handle:
        document = json.load(handle)
    if not isinstance(document, dict):
        document = {}
except (OSError, ValueError):
    document = {}
document.setdefault("bundle", "dev")
components = document.get("components")
if not isinstance(components, dict):
    components = {}
components["%(component)s"] = "dev"
document["components"] = components
fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".raccoon-bundle.")
with os.fdopen(fd, "w") as handle:
    json.dump(document, handle, indent=2)
    handle.write("\n")
os.chmod(tmp, 0o644)
os.replace(tmp, path)
os.makedirs("/run/raccoon", exist_ok=True)
os.chmod("/run/raccoon", 0o1777)
"""


def fail(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(1)


class Pi:
    def __init__(self, host: str, user: str, ssh_params: str) -> None:
        self.target = f"{user}@{host}"
        self.params = shlex.split(ssh_params)

    def run(self, command: str, *, check: bool = True, stdin: str | None = None) -> int:
        result = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=5", *self.params, self.target, command],
            input=stdin,
            text=True,
        )
        if check and result.returncode != 0:
            fail(f"command failed on the Pi: {command}")
        return result.returncode

    def copy(self, local: Path, remote_dir: str) -> None:
        subprocess.run(["scp", *self.params, str(local), f"{self.target}:{remote_dir}/"], check=True)


def find_deb(artifact: str | None) -> Path:
    if artifact:
        path = Path(artifact)
    else:
        manifest = Path(__file__).resolve().parent / "builds" / "manifest.json"
        if not manifest.is_file():
            fail("no builds/manifest.json. Run build.py first, or pass --artifact <deb>.")
        files = json.loads(manifest.read_text())["files"]
        debs = [f["name"] for f in files if f.get("kind") == "deb"]
        if len(debs) != 1:
            fail(f"expected one .deb in {manifest}, found {len(debs)}")
        path = manifest.parent / debs[0]
    if not path.is_file() or not (path.name.startswith("stm32-data-reader_") and path.suffix == ".deb"):
        fail(f"not a stm32-data-reader .deb: {path}")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="Install a stm32-data-reader .deb on a Pi.")
    parser.add_argument("--host", required=True, help="Pi address")
    parser.add_argument("--user", default="pi", help="SSH user (default: pi)")
    parser.add_argument("--artifact", help=".deb to install (default: the one in builds/manifest.json)")
    parser.add_argument("--ssh-params", default="", help='extra ssh/scp options, e.g. "-o StrictHostKeyChecking=no"')
    args = parser.parse_args()

    deb = find_deb(args.artifact)
    pi = Pi(args.host, args.user, args.ssh_params)
    print(f"Deploying {deb.name} to {pi.target}")

    print("Testing SSH connection...")
    if pi.run("true", check=False) != 0:
        fail(f"cannot connect to {pi.target} (see the ssh message above). "
             f"Host key changed (e.g. after flashing a new image)?  ssh-keygen -R {args.host}")

    print("Uploading .deb...")
    pi.run(f"rm -rf {REMOTE_TMP} && mkdir -p {REMOTE_TMP}")
    pi.copy(deb, REMOTE_TMP)

    # The package restarts the reader when it is installed, and the reader checks itself against
    # the bundle file at start, so the file must be right before apt runs.
    print(f'Marking {COMPONENT} as "dev" in {BUNDLE_FILE}...')
    pi.run("sudo python3 -", stdin=MARK_DEV % {"path": BUNDLE_FILE, "component": COMPONENT})

    # A local build is version 0.0, which Debian sorts below every release: allow the downgrade.
    print("Installing (this flashes the STM32 firmware)...")
    pi.run(
        "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --allow-downgrades "
        f"{shlex.quote(f'{REMOTE_TMP}/{deb.name}')}"
    )

    print()
    pi.run("systemctl is-active stm32_data_reader.service lcm-loopback-multicast.service; "
           "/usr/bin/stm32_data_reader --version", check=False)
    print(f"Deployment to {args.host} completed.")


if __name__ == "__main__":
    main()
