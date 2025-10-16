set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

cmake_minimum_required(VERSION 3.16)
project(stm32_data_reader VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_SPI_MOCK "Use mock SPI implementation for testing" OFF)

include_directories(include)
include(FetchContent)

# ---------- spdlog ----------
set(SPDLOG_GIT_TAG v1.15.3)


set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG ${SPDLOG_GIT_TAG}
)
FetchContent_MakeAvailable(spdlog)

# ---------- LCM ----------
set(LCM_ENABLE_JAVA OFF CACHE BOOL "" FORCE)
set(LCM_ENABLE_PYTHON OFF CACHE BOOL "" FORCE)
set(LCM_ENABLE_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        lcm
        GIT_REPOSITORY https://github.com/lcm-proj/lcm.git
        GIT_TAG v1.5.0
)
FetchContent_MakeAvailable(lcm)

add_subdirectory(lcm-messages)

set(CORE_SOURCES
        src/wombat/core/Logger.cpp
        src/wombat/core/Result.cpp
        src/wombat/messaging/LcmBroker.cpp
        src/wombat/services/DeviceController.cpp
        src/wombat/services/DataPublisher.cpp
        src/wombat/services/CommandSubscriber.cpp
        src/wombat/Application.cpp
)

if (USE_SPI_MOCK)
    message(STATUS "Using mock SPI implementation")
    set(SPI_SOURCE src/wombat/hardware/SpiMock.cpp)
else ()
    message(STATUS "Using real SPI implementation")
    set(SPI_SOURCE src/wombat/hardware/Spi.cpp)
endif ()

add_executable(${PROJECT_NAME}
        src/main.cpp
        ${CORE_SOURCES}
        ${SPI_SOURCE}
)


target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog_header_only)

target_link_libraries(${PROJECT_NAME} PRIVATE messages-cpp ${LCM_NAMESPACE}lcm)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif ()

# Precompiled headers tame spdlog/fmt template cost
target_precompile_headers(${PROJECT_NAME} PRIVATE
        <string> <vector> <memory>
        <spdlog/spdlog.h>
        <spdlog/sinks/stdout_color_sinks.h>
)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)