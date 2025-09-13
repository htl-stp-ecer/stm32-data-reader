set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

cmake_minimum_required(VERSION 3.16)
project(stm32_data_reader)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

option(USE_SPI_MOCK "Use mock SPI implementation" OFF)
set(SPI_SOURCE src/spi/Spi.cpp)
if(USE_SPI_MOCK)
    message(STATUS "Using SPI Mock implementation")
    include_directories(${CMAKE_SOURCE_DIR}/include/mock)
    set(SPI_SOURCE src/spi/mock/Spi.cpp)
endif()

include_directories(include)

add_executable(stm32_data_reader
        src/main.cpp
        ${SPI_SOURCE}
        src/lcm/LcmDataWriter.cpp
)


if(USE_SPI_MOCK)
    target_compile_definitions(stm32_data_reader PRIVATE USE_SPI_MOCK)
endif()

FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.15.3
)
FetchContent_MakeAvailable(spdlog)

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

target_link_libraries(stm32_data_reader
        PRIVATE
        spdlog::spdlog
        messages-cpp
        ${LCM_NAMESPACE}lcm
)
