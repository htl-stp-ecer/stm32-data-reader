# Builder for the reader (Pi, aarch64). Used by build.py, which picks the stage with
# `docker build --target`.
#
#   cross   x86_64 image with the aarch64 cross compiler (a PC). Runs at native speed, no emulation.
#           qemu-user only runs the finished test programs (qemu-aarch64 -L <sysroot>), it compiles nothing.
#   native  image for the machine it is built on (an arm64 machine, e.g. the CI's ARM runner) with the
#           normal compiler. Everything runs natively.
#
# The reader has no system libraries to link besides glibc and libstdc++: spdlog and googletest
# are built from source by CMake (FetchContent), nlohmann/json is vendored, and raccoon_ring is
# part of raccoon-transport. So the compiler's own sysroot is enough.
FROM debian:13-slim AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates git cmake ninja-build ccache python3 dpkg file \
    && rm -rf /var/lib/apt/lists/*


FROM base AS native
RUN apt-get update && apt-get install -y --no-install-recommends gcc g++ \
    && rm -rf /var/lib/apt/lists/*


FROM base AS cross
RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-aarch64-linux-gnu g++-aarch64-linux-gnu qemu-user \
    && rm -rf /var/lib/apt/lists/*
