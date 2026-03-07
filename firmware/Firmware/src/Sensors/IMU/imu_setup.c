#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"
#include "Sensors/IMU/mpu9250_config.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpl.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "motion_driver_hal.h"
#include "spi.h"

#define DEFAULT_MPU_HZ   (50)
#define COMPASS_READ_MS  (100)

void setupImu(void)
{
    changeSPIBaudRatePrescaler(&hspi3, SPI_BAUDRATEPRESCALER_64);

    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr, compass_fsr;

    imu_run_self_test();
    mpu_init(&int_param);

    inv_init_mpl();
    inv_enable_quaternion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_in_use_auto_calibration();
    inv_enable_heading_from_gyro();
    inv_enable_eMPL_outputs();
    inv_start_mpl();

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);

    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_compass_fsr(&compass_fsr);

    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
    inv_set_quat_sample_rate(1000000 / gyro_rate);

    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);
    inv_set_compass_orientation_and_scale(
        inv_orientation_matrix_to_scalar(compass_pdata.orientation),
        (long)compass_fsr << 15);

    hal_get_tick_count(&imu_timestamp);

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
        | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
}
