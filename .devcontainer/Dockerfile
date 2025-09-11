FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      pkg-config \
      libglib2.0-dev \
      doxygen \
      graphviz \
    && rm -rf /var/lib/apt/lists/*
