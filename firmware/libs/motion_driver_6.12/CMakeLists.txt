cmake_minimum_required(VERSION 3.24)

# Project configuration
project(invensense_motion_driver C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Option to build for specific platform (can be overridden)
option(BUILD_FOR_EMBEDDED "Build for embedded systems with size optimizations" ON)
option(ENABLE_COMPASS "Enable compass/magnetometer support" ON)
option(ENABLE_DMP "Enable Digital Motion Processor features" ON)
option(ENABLE_LOGGING "Enable logging output" OFF)

add_definitions(-DEMPL_TARGET_STM32F4)

# Configure compiler flags based on build type
if (BUILD_FOR_EMBEDDED)
    # Embedded-friendly flags
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -ffunction-sections -fdata-sections")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-unwind-tables")
    # Define single-threaded environment by default for embedded
    add_definitions(-DMOTION_DRIVER_SINGLE_THREADED)
else ()
    # Desktop/test build flags
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g")
endif ()

# Configure logging
if (NOT ENABLE_LOGGING)
    add_definitions(-DMOTION_DRIVER_NO_LOGGING -DMPL_LOG_NDEBUG=1)
endif ()

# Core library compile definitions
add_definitions(
        # Motion Driver specific defines
        -DREMOVE_LOGGING
        -DEMPL
        -DMPU9250
)

# Conditionally enable features
if (ENABLE_DMP)
    add_definitions(-DUSE_DMP)
endif ()

if (ENABLE_COMPASS)
    add_definitions(-DCOMPASS_ENABLED)
endif ()

# Define paths relative to the STM32 project (where the core library lives)
set(MOTION_DRIVER_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Include directories for the core library
include_directories(
        # Motion Driver Core Library headers
        ${MOTION_DRIVER_CORE_DIR}/driver/include
        ${MOTION_DRIVER_CORE_DIR}/driver/eMPL
        ${MOTION_DRIVER_CORE_DIR}/mllite
        ${MOTION_DRIVER_CORE_DIR}/mpl
        ${MOTION_DRIVER_CORE_DIR}/eMPL-hal
        # HAL interface header
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Collect core library source files (hardware-independent)
file(GLOB CORE_DRIVER_SOURCES
        "${MOTION_DRIVER_CORE_DIR}/driver/eMPL/*.c"
)

file(GLOB CORE_MLLITE_SOURCES
        "${MOTION_DRIVER_CORE_DIR}/mllite/*.c"
        "${MOTION_DRIVER_CORE_DIR}/mllite/hal_outputs.c"
)

file(GLOB CORE_EMPL_HAL_SOURCES
        "${MOTION_DRIVER_CORE_DIR}/eMPL-hal/*.c"
        "${MOTION_DRIVER_CORE_DIR}/eMPL-hal/eMPL_outputs.c"
)

# Create the main Motion Driver library
add_library(motion_driver STATIC
        ${CORE_DRIVER_SOURCES}
        ${CORE_MLLITE_SOURCES}
        ${CORE_EMPL_HAL_SOURCES}
        src/driver/include/log.h
)

# Set include directories for the library
target_include_directories(motion_driver PUBLIC
        ${MOTION_DRIVER_CORE_DIR}/driver/include
        ${MOTION_DRIVER_CORE_DIR}/driver/eMPL
        ${MOTION_DRIVER_CORE_DIR}/mllite
        ${MOTION_DRIVER_CORE_DIR}/mpl
        ${MOTION_DRIVER_CORE_DIR}/eMPL-hal
        ${CMAKE_CURRENT_SOURCE_DIR}  # For motion_driver_hal.h
)

# Link the pre-compiled MPL library if it exists
# Note: The libmpllib.a is ARM-compiled, so this only works for ARM targets
if (EXISTS "${MOTION_DRIVER_CORE_DIR}/mpl/libmpllib.a")
    # Check if we're building for ARM
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "[Aa][Rr][Mm]" OR CMAKE_C_COMPILER MATCHES "arm-none-eabi")
        # Create a named imported target for the prebuilt lib
        add_library(mpl_prebuilt STATIC IMPORTED GLOBAL)
        set_target_properties(mpl_prebuilt PROPERTIES
                IMPORTED_LOCATION ${MOTION_DRIVER_CORE_DIR}/mpl/libmpllib.a
        )

        # Link it PRIVATE here (do NOT propagate to the app)
        target_link_libraries(motion_driver PRIVATE mpl_prebuilt)
        message(STATUS "Using pre-compiled ARM MPL library")
    else ()
        message(WARNING "Pre-compiled MPL library is for ARM targets only. You may need to provide x86 version or compile from source.")
        # You would need to either:
        # 1. Get the MPL source code and compile it
        # 2. Use a different MPL library for your target
        # 3. Implement the MPL functions yourself
    endif ()
else ()
    message(WARNING "Pre-compiled MPL library not found. You may need to provide libmpllib for your target platform.")
endif ()

# Link math library (if available)
if (NOT BUILD_FOR_EMBEDDED OR NOT CMAKE_SYSTEM_NAME STREQUAL "Generic")
    target_link_libraries(motion_driver m)
endif ()

# Create a lightweight version without DMP for size-constrained systems
#if(BUILD_FOR_EMBEDDED)
#    add_library(motion_driver_lite STATIC
#            ${CORE_MLLITE_SOURCES}
#            ${CORE_EMPL_HAL_SOURCES}
#            # Only include basic driver, not DMP
#            "${MOTION_DRIVER_CORE_DIR}/driver/eMPL/inv_mpu.c"
#    )
#
#    target_include_directories(motion_driver_lite PUBLIC
#            ${MOTION_DRIVER_CORE_DIR}/driver/include
#            ${MOTION_DRIVER_CORE_DIR}/driver/eMPL
#            ${MOTION_DRIVER_CORE_DIR}/mllite
#            ${MOTION_DRIVER_CORE_DIR}/eMPL-hal
#            ${CMAKE_CURRENT_SOURCE_DIR}
#    )
#
#    target_compile_definitions(motion_driver_lite PRIVATE -UUSE_DMP)
#endif()

# Install targets
install(TARGETS motion_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

if (TARGET motion_driver_lite)
    install(TARGETS motion_driver_lite
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
    )
endif ()

# Install headers
install(FILES motion_driver_hal.h
        DESTINATION include
)

install(DIRECTORY
        ${MOTION_DRIVER_CORE_DIR}/driver/include/
        ${MOTION_DRIVER_CORE_DIR}/driver/eMPL/
        ${MOTION_DRIVER_CORE_DIR}/mllite/
        ${MOTION_DRIVER_CORE_DIR}/mpl/
        ${MOTION_DRIVER_CORE_DIR}/eMPL-hal/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

# Print configuration summary
message(STATUS "Motion Driver Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Target: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  Embedded build: ${BUILD_FOR_EMBEDDED}")
message(STATUS "  Compass support: ${ENABLE_COMPASS}")
message(STATUS "  DMP support: ${ENABLE_DMP}")
message(STATUS "  Logging enabled: ${ENABLE_LOGGING}")
if (TARGET motion_driver_lite)
    message(STATUS "  Lite variant: Available")
endif ()

# Usage instructions
message(STATUS "")
message(STATUS "Usage Instructions:")
message(STATUS "  1. Implement the HAL functions defined in motion_driver_hal.h")
message(STATUS "  2. Link your project with: motion_driver")
message(STATUS "  3. Include: motion_driver_hal.h and inv_mpu.h")
message(STATUS "")
message(STATUS "HAL Functions Required:")
message(STATUS "  - i2c_write() / i2c_read()     : I2C communication")
message(STATUS "  - delay_ms() / get_tick_count(): Timing functions")
message(STATUS "  - inv_malloc() / inv_free()    : Memory management")
message(STATUS "  - _MLPrintLog() (optional)     : Logging function")


if (NOT EXISTS "${MOTION_DRIVER_CORE_DIR}")
    message(FATAL_ERROR "Motion Driver core directory not found at: ${MOTION_DRIVER_CORE_DIR}")
endif ()