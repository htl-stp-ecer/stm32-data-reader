cmake_minimum_required(VERSION 3.24)

# add the sources here
file(GLOB src "src/*.c")
file(GLOB Actors "src/Actors/*.c")
file(GLOB Communication "src/Communication/*.c")
file(GLOB Data_structures "src/Data_structures/*.c")
file(GLOB Hardware "src/Hardware/*.c")
file(GLOB Sensors "src/Sensors/*.c")
file(GLOB IMU "src/Sensors/IMU/*.c")
file(GLOB MotionDriver "src/Sensors/IMU/motion_driver/*.c")
file(GLOB Utility "src/Utility/*.c")

include_directories(${CMAKE_SOURCE_DIR}/../shared)
include_directories(include)
include_directories(include/Actors)
include_directories(include/Communication)
include_directories(include/Sensors)
include_directories(include/Sensors/IMU)
include_directories(src/Sensors/IMU/motion_driver)
include_directories(include/Utility)

set(sources
        #src/main.c
        Startup/startup_stm32f427vitx.s

        ${src}
        ${Actors}
        ${Communication}
        ${Data_structures}
        ${Hardware}
        ${Sensors}
        ${IMU}
        ${MotionDriver}
        ${Utility}
)

set(CMAKE_VERBOSE_MAKEFILE ON)  # or `make VERBOSE=1`

# GCC optimization level: use -O0 in debug build, otherwise -O2
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(opt_level -O0)
else ()
    set(opt_level -O2)
endif ()

# set some project constants
set(elf_file ${APPLICATION_NAME}.elf)
set(bin_file ${APPLICATION_NAME}.bin)
set(hex_file ${APPLICATION_NAME}.hex)
set(map_file ${APPLICATION_NAME}.map)
set(lss_file ${APPLICATION_NAME}.lss)

# add sources to elf file
add_executable(${elf_file} ${sources})

# link StdPeriph and cmsis library to project
# configure_file(${PROJECT_SOURCE_DIR}/Firmware/src/stm32f4xx_hal_conf.h ${PROJECT_BINARY_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf.h)

target_link_libraries(${elf_file} PUBLIC stm32f4xx mpl_prebuilt motion_driver)
#target_link_libraries(${elf_file} PUBLIC MPU9250)

# set additional for compiler and linker: optimization and generate map file
set(additional_compiler_flags
        ${opt_level}
        -Wall
        -gdwarf-2
        -fmessage-length=0
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -Wa,-adhlns="$@.lst"
)
set(additional_linker_flags -Wl,-Map=${map_file},--cref,--no-warn-mismatch)
target_compile_options(${elf_file} PRIVATE ${additional_compiler_flags})
target_link_libraries(${elf_file} PRIVATE ${additional_linker_flags})

# remove unused sections
target_link_libraries(${elf_file} PUBLIC "-g -Wl,--gc-sections -lm")

# link with linker file
target_link_libraries(${elf_file} PUBLIC -T${LINKER_SCRIPT})

# show size of resulting firmware image
add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} -B ${elf_file})

# generate extended listing
add_custom_target(${lss_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJDUMP_EXECUTABLE} -S ${elf_file} > ${lss_file})

# create binary and hex files
add_custom_target(${hex_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Oihex ${elf_file} ${hex_file})
add_custom_target(${bin_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Obinary ${elf_file} ${bin_file})
add_custom_target(${APPLICATION_NAME} ALL DEPENDS ${elf_file}-size ${bin_file} ${hex_file} ${lss_file})
