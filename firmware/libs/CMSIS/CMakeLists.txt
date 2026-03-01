
#add_library(cmsis ${sources})
add_library(cmsis INTERFACE)

# add include directories for StdPeriph library and CMSIS
target_include_directories(cmsis INTERFACE Device/ST/STM32F4xx/Include)
target_include_directories(cmsis INTERFACE Include)
#target_include_directories(stm32f4xx PUBLIC Inc/CMSIS)

# set up some defines for the StdPeriph library
target_compile_definitions(cmsis INTERFACE USE_STDPERIPH_DRIVER)
target_compile_definitions(cmsis INTERFACE ${DEVICE_FAMILY})
target_compile_definitions(cmsis INTERFACE HSE_VALUE=${HSE_VALUE})

# additional compiler options: use size-optimized version of library in release build, use -O0 in debug build
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(additional_flags -O0)
else ()
    set(additional_flags -Os)
endif ()

target_compile_options(cmsis INTERFACE ${additional_flags})
#target_link_libraries(stm32f4xx PRIVATE ${additional_flags})
