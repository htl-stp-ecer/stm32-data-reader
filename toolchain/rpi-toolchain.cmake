# Select target OS/arch early
set(CMAKE_SYSTEM_NAME Linux)
# RPi3B+ = Cortex-A53 (ARMv8). Often runs 32-bit armhf userspace; aarch64 is also common. :contentReference[oaicite:1]{index=1}
set(RPI_ARCH "armhf" CACHE STRING "Target ABI: armhf or aarch64")

# --- Paths you must set (cache allows -D overrides) ---
# Point to your rsynced Pi rootfs (sysroot)
set(RPI_SYSROOT "$ENV{HOME}/rpi-sysroot" CACHE PATH "Raspberry Pi sysroot")

# Cross toolchain prefix directories (Ubuntu packages or your own toolchain)
# For armhf:   sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
# For aarch64: sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
set(RPI_TOOLCHAIN_PATH "/usr/bin" CACHE PATH "Path containing cross compilers")

# --- Compilers & target triple ---
if(RPI_ARCH STREQUAL "aarch64")
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(TRIPLE aarch64-linux-gnu)
    set(CPU_FLAGS "-mcpu=cortex-a53;-mtune=cortex-a53") # 64-bit
else()
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(TRIPLE arm-linux-gnueabihf)
    # 32-bit hard-float NEON on Cortex-A53 (ARMv7 ABI)
    set(CPU_FLAGS "-mfpu=neon-vfpv4;-mfloat-abi=hard;-mcpu=cortex-a53;-mtune=cortex-a53")
endif()

# Tell CMake which compilers to use
set(CMAKE_C_COMPILER   "${RPI_TOOLCHAIN_PATH}/${TRIPLE}-gcc")
set(CMAKE_CXX_COMPILER "${RPI_TOOLCHAIN_PATH}/${TRIPLE}-g++")
set(CMAKE_AR           "${RPI_TOOLCHAIN_PATH}/${TRIPLE}-ar" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB       "${RPI_TOOLCHAIN_PATH}/${TRIPLE}-ranlib" CACHE FILEPATH "" FORCE)
set(CMAKE_STRIP        "${RPI_TOOLCHAIN_PATH}/${TRIPLE}-strip" CACHE FILEPATH "" FORCE)

# Use the sysroot (propagates --sysroot to compiler & affects find_*) :contentReference[oaicite:3]{index=3}
set(CMAKE_SYSROOT "${RPI_SYSROOT}" CACHE PATH "")

# Ensure find_* defaults to the target root
set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)  # keep host tools
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)   # take target libs
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)   # take target headers
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Arch-specific flags
add_compile_options(${CPU_FLAGS})
# Reasonable hardening/size tradeoffs; tweak as needed
add_compile_options(-fPIC)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()

# Correct linker sysroot & LDFLAGS
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-Wl,--as-needed")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-Wl,--as-needed")

# If you keep target libs in nonstandard places inside the sysroot, hint them:
# list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SYSROOT}/usr/lib/${TRIPLE}" "${CMAKE_SYSROOT}/lib/${TRIPLE}")
