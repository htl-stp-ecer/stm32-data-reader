file(GLOB_RECURSE sources "${CMAKE_CURRENT_LIST_DIR}/src/*.c")

add_library(stm32f4xx ${sources}
        Inc/stm32f4xx.h
        Inc/stm32f4xx_hal.h)

target_link_libraries(cmsis)
# add include directories for StdPeriph library and CMSIS
target_include_directories(stm32f4xx PUBLIC Inc)
#target_include_directories(stm32f4xx PUBLIC Inc/CMSIS)

# set up some defines for the StdPeriph library
target_compile_definitions(stm32f4xx PUBLIC USE_STDPERIPH_DRIVER)
target_compile_definitions(stm32f4xx PUBLIC ${DEVICE_FAMILY})
target_compile_definitions(stm32f4xx PUBLIC HSE_VALUE=${HSE_VALUE})

# additional compiler options: use size-optimized version of library in release build, use -O0 in debug build
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(additional_flags -O0)
else ()
    set(additional_flags -Os)
endif ()

target_compile_options(stm32f4xx PRIVATE ${additional_flags})
#target_link_libraries(stm32f4xx PRIVATE ${additional_flags})
