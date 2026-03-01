cmake_minimum_required(VERSION 3.16)
set(INVN_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(INVN_SOURCES
        ${INVN_ROOT}/driver/eMPL/inv_mpu.c
        ${INVN_ROOT}/driver/eMPL/inv_mpu_dmp_motion_driver.c
        ${INVN_ROOT}/eMPL-hal/eMPL_outputs.c
        ${INVN_ROOT}/mllite/data_builder.c
        ${INVN_ROOT}/mllite/message_layer.c
        ${INVN_ROOT}/mllite/ml_math_func.c
        ${INVN_ROOT}/mllite/mpl.c
        ${INVN_ROOT}/mllite/storage_manager.c
        ${INVN_ROOT}/mllite/results_holder.c
        ${INVN_ROOT}/mllite/start_manager.c
)

add_library(invn_src STATIC ${INVN_SOURCES})
target_include_directories(invn_src PUBLIC
        ${INVN_ROOT}
        ${INVN_ROOT}/driver/include
        ${INVN_ROOT}/driver/eMPL
        ${INVN_ROOT}/eMPL-hal
        ${INVN_ROOT}/mllite
        ${INVN_ROOT}/mpl
)

target_compile_definitions(invn_src PUBLIC MPU9250 REMOVE_LOGGING)
add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

set(MPL_LIB ${INVN_ROOT}/mpl/lib/m4_hardfp/liblibmplmpu.a)
if (NOT EXISTS ${MPL_LIB})
    message(FATAL_ERROR "Missing MPL binary at: ${MPL_LIB}")
endif ()

add_library(mpl STATIC IMPORTED GLOBAL)
set_target_properties(mpl PROPERTIES IMPORTED_LOCATION ${MPL_LIB})