#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "Sensors/IMU/mpu9250_config.h"

struct platform_data_s gyro_pdata = {
    .orientation = IMU_GYRO_ORIENTATION_MATRIX
};
struct platform_data_s compass_pdata = {
    .orientation = IMU_COMPASS_ORIENTATION_MATRIX
};

int imu_run_self_test(void)
{
    long gyro[3], accel[3];
    int result = mpu_run_6500_self_test(gyro, accel, 0);

    if ((result == 0x7) || (result == 0x3))
    {
        for (int i = 0; i < 3; ++i)
        {
            gyro[i] = (long)(gyro[i] * 32.8f);   // to +/-1000 dps
            accel[i] = (long)(accel[i] * 2048.f); // to +/-16 g
            accel[i] >>= 16;
            gyro[i] >>= 16;
        }
        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_6500_reg(accel);
    }
    return result;
}

void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9])
{
    unsigned short gyro_fsr, compass_fsr;
    unsigned char accel_fsr;

    for (int i = 0; i < 9; ++i) {
        gyro_pdata.orientation[i] = gyroOrientation[i];
        compass_pdata.orientation[i] = compassOrientation[i];
    }

    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_compass_fsr(&compass_fsr);

    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);
    inv_set_compass_orientation_and_scale(
        inv_orientation_matrix_to_scalar(compass_pdata.orientation),
        (long)compass_fsr << 15);

    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
}
