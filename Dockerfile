FROM ubuntu

ENV TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && \
    apt-get install -y ssh \
    build-essential \
    gcc \
    g++ \
    git \
    gdb \
    clang \
    cmake \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    wget \
    unzip && \
    apt-get clean

WORKDIR /app

CMD ["bash", "build.sh"]