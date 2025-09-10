# syntax=docker/dockerfile:1.7

############################################
# Build for the TARGET platform (Pi)
############################################
FROM --platform=$TARGETPLATFORM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

# Native ARM deps (note: NO :armhf suffix needed; we are inside an ARM rootfs)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git pkg-config \
      libglib2.0-dev libpcre2-dev zlib1g-dev \
      libfmt-dev libspdlog-dev \
      liblcm-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

# Usual CMake build (no cross toolchain needed)
RUN rm -rf build && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j"$(nproc)"

############################################
# Export artifacts (binary-only) stage
############################################
FROM scratch AS artifacts
COPY --from=build /src/build/stm32_data_reader /out/stm32_data_reader

############################################
# Optional: runnable image for the Pi
############################################
FROM --platform=$TARGETPLATFORM ubuntu:24.04 AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
      libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/build/stm32_data_reader /usr/local/bin/
ENTRYPOINT ["stm32_data_reader"]
