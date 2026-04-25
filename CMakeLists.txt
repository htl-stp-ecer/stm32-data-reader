set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

cmake_minimum_required(VERSION 3.16)
set(BUILD_NUMBER "0" CACHE STRING "CI build number")
project(stm32_data_reader VERSION 1.0.${BUILD_NUMBER} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_SPI_MOCK "Use mock SPI implementation for testing" ON)

include_directories(include)
include_directories(shared)
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

# ---------- Google Test (before LCM to avoid gtest target conflict) ----------
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---------- raccoon-transport (brings LCM + message types) ----------
FetchContent_Declare(
        raccoon_transport
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/raccoon-transport
)
FetchContent_MakeAvailable(raccoon_transport)

set(CORE_SOURCES
        src/wombat/core/Logger.cpp
        src/wombat/core/Result.cpp
        src/wombat/messaging/LcmBroker.cpp
        src/wombat/services/DeviceController.cpp
        src/wombat/services/DataPublisher.cpp
        src/wombat/services/CommandSubscriber.cpp
        src/wombat/services/SystemMonitor.cpp
        src/wombat/services/UartMonitor.cpp
        src/wombat/Application.cpp
)

if (USE_SPI_MOCK)
    message(STATUS "Using mock SPI implementation")
    set(SPI_SOURCE src/wombat/hardware/SpiMock.cpp)
else ()
    message(STATUS "Using real SPI implementation")
    set(SPI_SOURCE src/wombat/hardware/Spi.cpp src/wombat/hardware/SpiReal.cpp)
endif ()

# SpiMock is always needed for tests
set(TEST_SPI_SOURCE src/wombat/hardware/SpiMock.cpp)

# ---------- Static library for tests ----------
add_library(wombat_lib STATIC
        ${CORE_SOURCES}
        ${SPI_SOURCE}
)

target_link_libraries(wombat_lib PUBLIC
        spdlog::spdlog_header_only
        raccoon::transport
)

if (USE_SPI_MOCK)
    target_compile_definitions(wombat_lib PUBLIC USE_SPI_MOCK)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(wombat_lib PRIVATE -Wall -Wextra -Wpedantic)
endif ()

target_precompile_headers(wombat_lib PRIVATE
        <string> <vector> <memory>
        <spdlog/spdlog.h>
        <spdlog/sinks/stdout_color_sinks.h>
)

# ---------- Version header ----------
configure_file(src/version.h.in ${CMAKE_BINARY_DIR}/version.h)

# ---------- Executable ----------
add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE wombat_lib)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif ()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

enable_testing()
add_subdirectory(tests)
