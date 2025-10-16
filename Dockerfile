FROM --platform=linux/arm64/v8 debian:12-slim

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git pkg-config ccache ca-certificates \
    && git config --system http.sslCAinfo /etc/ssl/certs/ca-certificates.crt \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      libglib2.0-dev libpcre2-dev zlib1g-dev \
      libfmt-dev libspdlog-dev \
      liblcm-dev \
    && rm -rf /var/lib/apt/lists/*

ENV CCACHE_DIR=/ccache \
    CCACHE_MAXSIZE=3G \
    CCACHE_COMPRESS=1

WORKDIR /src

CMD ["/bin/bash"]
