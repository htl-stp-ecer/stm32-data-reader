#!/usr/bin/env python3
"""build.py — build the stm32-data-reader .deb (reader and STM32 firmware) in Docker.

Nothing is installed on the host and nothing is written outside ./builds/. Both parts are
cross compiled on your PC at native speed, nothing is emulated:

  * the reader (C++ for the Pi, aarch64) with the aarch64 cross compiler,
  * the firmware (wombat.bin for the STM32) with arm-none-eabi-gcc.

The .deb contains the reader, the firmware with its flash scripts and the systemd units.
CI runs exactly this script.

Usage:
    python3 build.py                  # local build, version 0.0
    python3 build.py --version 60     # release build (CI)
    python3 build.py --test           # also build the unit tests and run them (see below)
    python3 build.py --skip-firmware  # reuse the firmware of the last build

Result:
    builds/stm32-data-reader_<version>_arm64.deb
    builds/manifest.json

--test builds the tests for the Pi and runs them with qemu-user (`qemu-aarch64`, only the finished
test programs are emulated, nothing is compiled under emulation; no binfmt, no --privileged).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILDS = ROOT / "builds"
READER_IMAGE = "stm32-data-reader-build"
FIRMWARE_IMAGE = "stm32-firmware-build"
VERSION_PATTERN = re.compile(r"^[0-9]+(\.[0-9]+)*$")

PKG = "stm32-data-reader"
SYSROOT = "/usr/aarch64-linux-gnu"
EMULATOR = f"qemu-aarch64;-L;{SYSROOT}"

# Builds the firmware in a copy of firmware/ and shared/ (the firmware includes ../shared).
FIRMWARE_SCRIPT = r"""
set -euo pipefail
rm -rf /tmp/work && mkdir /tmp/work
tar -C /src --exclude=firmware/build --exclude=firmware/build-codex -cf - firmware shared | tar -C /tmp/work -xf -
cd /tmp/work/firmware
mkdir build && cd build
cmake -G "Unix Makefiles" -D CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake ../
cmake --build . -- -j"$(nproc)"
mkdir -p /cache/firmware-out
cp Firmware/wombat.bin /cache/firmware-out/wombat.bin
"""


def fail(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(1)


def run(command: list[str], **kwargs) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, check=True, **kwargs)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ── inside the reader container ──────────────────────────────────────────


def cross_configure(build_dir: Path, version: str, spi_mock: bool) -> None:
    run([
        "cmake", "-S", "/src", "-B", str(build_dir), "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DUSE_SPI_MOCK={'ON' if spi_mock else 'OFF'}",
        f"-DBUILD_NUMBER={version}",
        "-DCMAKE_SYSTEM_NAME=Linux", "-DCMAKE_SYSTEM_PROCESSOR=aarch64",
        "-DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc", "-DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++",
        f"-DCMAKE_FIND_ROOT_PATH={SYSROOT}",
        # CMake runs test programs (gtest discovery, ctest) through this.
        f"-DCMAKE_CROSSCOMPILING_EMULATOR={EMULATOR}",
        "-DCMAKE_C_COMPILER_LAUNCHER=ccache", "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
    ])


def run_tests(cache: Path, version: str) -> None:
    """Build the tests with the mock SPI (as the developers do) and run them with qemu-user."""
    build_dir = cache / "reader-build-test"
    cross_configure(build_dir, version, spi_mock=True)
    run(["cmake", "--build", str(build_dir), "-j", str(os.cpu_count() or 4)])
    # reader_contract_smoke starts the real binary from a shell script, which needs binfmt;
    # the gtest cases cover everything else.
    run(["ctest", "--test-dir", str(build_dir), "--output-on-failure", "-E", "reader_contract_smoke"])


def build_deb(binary: Path, firmware: Path, version: str, output: Path) -> None:
    """Assemble stm32-data-reader_<version>_arm64.deb (what scripts/build_deb.sh did)."""
    stage = Path(tempfile.mkdtemp())
    try:
        def put(source: Path, target: str, mode: int) -> None:
            destination = stage / target
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(source, destination)
            destination.chmod(mode)

        put(binary, "usr/bin/stm32_data_reader", 0o755)
        fw_dir = f"usr/lib/{PKG}/firmware"
        put(firmware, f"{fw_dir}/wombat.bin", 0o644)
        for script in ("flash_wombat.sh", "init_gpio.sh", "reset_coprocessor.sh"):
            put(ROOT_IN_CONTAINER / "firmware/flashFiles" / script, f"{fw_dir}/{script}", 0o755)

        # The reader unit is the single source of truth in systemd/; rewrite its home-dir paths
        # to FHS locations for the packaged install.
        unit = (ROOT_IN_CONTAINER / "systemd/stm32_data_reader.service").read_text()
        unit = re.sub(r"^ExecStart=/home/pi/stm32_data_reader/stm32_data_reader",
                      "ExecStart=/usr/bin/stm32_data_reader", unit, flags=re.MULTILINE)
        unit = unit.replace("/home/pi/stm32_data_reader", "/var/lib/stm32-data-reader")
        service = stage / "lib/systemd/system/stm32_data_reader.service"
        service.parent.mkdir(parents=True, exist_ok=True)
        service.write_text(unit)
        service.chmod(0o644)
        put(ROOT_IN_CONTAINER / "systemd/lcm-loopback-multicast.service",
            "lib/systemd/system/lcm-loopback-multicast.service", 0o644)

        # The reader contains nlohmann/json (MIT) through raccoon-transport: ship its notice.
        doc = f"usr/share/doc/{PKG}"
        put(ROOT_IN_CONTAINER / "THIRD_PARTY_NOTICES.md", f"{doc}/THIRD_PARTY_NOTICES.md", 0o644)
        put(ROOT_IN_CONTAINER / "raccoon-transport/cpp/third_party/nlohmann/LICENSE.MIT",
            f"{doc}/LICENSE.nlohmann-json.MIT", 0o644)

        debian = stage / "DEBIAN"
        debian.mkdir()
        packaging = ROOT_IN_CONTAINER / "packaging/debian"
        (debian / "control").write_text((packaging / "control").read_text().replace("__VERSION__", version))
        for script in ("postinst", "prerm", "postrm"):
            put(packaging / script, f"DEBIAN/{script}", 0o755)

        run(["dpkg-deb", "--root-owner-group", "--build", str(stage), str(output)])
    finally:
        shutil.rmtree(stage, ignore_errors=True)


ROOT_IN_CONTAINER = Path("/src")


def run_in_container(version: str, owner: str, test: bool) -> None:
    """Reader build. /src is the repository (read-only), /cache keeps the build state, /out is ./builds."""
    cache = Path("/cache")
    firmware = cache / "firmware-out" / "wombat.bin"
    if not firmware.is_file():
        fail("no firmware (wombat.bin) found. Run once without --skip-firmware.")

    build_dir = cache / "reader-build"
    cross_configure(build_dir, version, spi_mock=False)
    run(["cmake", "--build", str(build_dir), "--target", "stm32_data_reader", "-j", str(os.cpu_count() or 4)])
    binary = build_dir / "stm32_data_reader"
    run(["file", str(binary)]) if shutil.which("file") else None

    if test:
        run_tests(cache, version)

    deb = Path("/out") / f"{PKG}_{version}_arm64.deb"
    build_deb(binary, firmware, version, deb)
    uid, gid = owner.split(":")
    os.chown(deb, int(uid), int(gid))


# ── on the host ──────────────────────────────────────────────────────────


def build_image(tag: str, dockerfile: str) -> None:
    print(f"▶ Building Docker image {tag}")
    subprocess.run(["docker", "build", "-t", tag, "-f", str(ROOT / dockerfile), str(ROOT)], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the stm32-data-reader .deb in Docker.")
    parser.add_argument("--version", default="0.0", help="version of the package (default: 0.0 = local build)")
    parser.add_argument("--test", action="store_true", help="also build the unit tests and run them with qemu-user")
    parser.add_argument("--skip-firmware", action="store_true", help="reuse the firmware of the last build")
    parser.add_argument("--image", help="use this Docker image for the reader instead of building ./Dockerfile")
    parser.add_argument("--firmware-image", help="use this Docker image for the firmware instead of ./Dockerfile.firmware")
    parser.add_argument(
        "--cache-dir",
        default=str(Path.home() / ".cache" / "raccoon-build" / "stm32-data-reader"),
        help="ccache, build directories and the last firmware, kept between builds (default: %(default)s)",
    )
    parser.add_argument("--in-container", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--owner", default="0:0", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if not VERSION_PATTERN.match(args.version):
        fail(f"--version must be numbers separated by dots (got {args.version!r})")

    if args.in_container:
        run_in_container(args.version, args.owner, args.test)
        return

    if not shutil.which("docker"):
        fail("docker not found. The build runs in containers so the host stays clean.")
    if not (ROOT / "raccoon-transport" / "cpp").is_dir():
        fail("raccoon-transport is empty. Run `git submodule update --init`.")

    cache = Path(args.cache_dir)
    (cache / "ccache").mkdir(parents=True, exist_ok=True)
    (cache / "firmware-out").mkdir(parents=True, exist_ok=True)
    user = f"{os.getuid()}:{os.getgid()}"
    BUILDS.mkdir(exist_ok=True)
    for stale in [*BUILDS.glob(f"{PKG}_*_arm64.deb"), BUILDS / "manifest.json"]:
        stale.unlink(missing_ok=True)

    if not args.skip_firmware:
        firmware_image = args.firmware_image
        if not firmware_image:
            firmware_image = FIRMWARE_IMAGE
            build_image(firmware_image, "Dockerfile.firmware")
        print("▶ Building the firmware")
        (cache / "firmware-out" / "wombat.bin").unlink(missing_ok=True)
        run([
            "docker", "run", "--rm", "--user", user, "-e", "HOME=/tmp",
            "-v", f"{ROOT}:/src:ro", "-v", f"{cache}:/cache",
            firmware_image, "bash", "-c", FIRMWARE_SCRIPT,
        ])

    image = args.image
    if not image:
        image = READER_IMAGE
        build_image(image, "Dockerfile")

    print(f"▶ Building the reader {args.version} (cross compiled for aarch64)")
    command = [
        "docker", "run", "--rm", "--user", user, "-e", "HOME=/tmp",
        "-e", "CCACHE_DIR=/cache/ccache", "-e", "CCACHE_MAXSIZE=3G", "-e", "CCACHE_COMPRESS=1",
        "-v", f"{ROOT}:/src:ro", "-v", f"{BUILDS}:/out", "-v", f"{cache}:/cache",
        image, "python3", "/src/build.py", "--in-container", "--version", args.version, "--owner", user,
    ]
    if args.test:
        command.append("--test")
    run(command)

    debs = sorted(BUILDS.glob(f"{PKG}_*_arm64.deb"))
    if len(debs) != 1:
        fail(f"expected exactly one .deb in {BUILDS}, found {len(debs)}")
    manifest = {
        "name": PKG,
        "version": args.version,
        "files": [{"name": debs[0].name, "kind": "deb", "sha256": sha256(debs[0])}],
    }
    (BUILDS / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"✓ {debs[0].relative_to(ROOT)}")


if __name__ == "__main__":
    main()
