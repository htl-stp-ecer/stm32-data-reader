# Cross compiler for the reader: runs on x86_64 and builds for the Pi (aarch64), at native
# speed. No emulation. build.py builds this image and runs the build inside it.
#
# The reader has no system libraries to link besides glibc and libstdc++: spdlog and googletest
# are built from source by CMake (FetchContent), nlohmann/json is vendored, and raccoon_ring is
# part of raccoon-transport. So the cross toolchain's own sysroot is enough.
#
# qemu-user only runs the finished test programs (qemu-aarch64 -L <sysroot>), it compiles nothing.
FROM debian:13-slim

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates git cmake ninja-build ccache python3 dpkg \
        gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
        qemu-user \
    && rm -rf /var/lib/apt/lists/*
