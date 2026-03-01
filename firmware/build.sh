#!/bin/bash

# Delete the build directory if it exists
if [ -d "build" ]; then
    rm -rf build
fi

mkdir build && cd build || exit

cmake -G "Unix Makefiles" -D "CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake" ../
cmake --build . -- -j "$(nproc)"