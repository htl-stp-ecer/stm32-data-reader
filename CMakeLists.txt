cmake_minimum_required(VERSION 3.16)
project(stm32_data_reader)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
include_directories(include)

add_executable(stm32_data_reader
        src/main.cpp
        src/Spi.cpp
)

# Add spdlog
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.15.3
)
FetchContent_MakeAvailable(spdlog)

# Add LCM
FetchContent_Declare(
        lcm
        GIT_REPOSITORY https://github.com/lcm-proj/lcm.git
        GIT_TAG v1.5.0
)
FetchContent_MakeAvailable(lcm)

# # Add LCM message types
add_subdirectory(lcm-messages)

target_link_libraries(stm32_data_reader
        PRIVATE
        spdlog::spdlog
        messages-cpp
        ${LCM_NAMESPACE}lcm
        -static
)
