#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "communication_with_pi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "motion_driver_hal.h"
#include "spi.h"

#define DEFAULT_MPU_HZ   (50)
#define GYRO_READ_MS     (uint32_t)(1000/DEFAULT_MPU_HZ)
#define COMPASS_READ_MS  (100)
#define TEMP_READ_MS     (500)

ImuData imu = {0};
inv_time_t imu_timestamp;

static unsigned long next_gyro_ms = 0;
static unsigned char new_gyro = 0;
static unsigned long next_compass_ms = 0;
static unsigned char new_compass = 0;
static unsigned long next_temp_ms = 0;
static unsigned char new_temp = 0;

static void poll_fifo(unsigned long* sensor_timestamp)
{
    short gyro[3], accel_short[3], sensors;
    unsigned char more;
    long accel[3], quat[4], temperature;

    dmp_read_fifo(gyro, accel_short, quat, sensor_timestamp, &sensors, &more);
    if (more)
        new_gyro = 1;

    if (sensors & INV_XYZ_GYRO)
    {
        inv_build_gyro(gyro, *sensor_timestamp);
        if (new_temp)
        {
            new_temp = 0;
            mpu_get_temperature(&temperature, sensor_timestamp);
            inv_build_temp(temperature, *sensor_timestamp);
            imu.temperature = inv_q16_to_float(temperature);
        }
    }

    if (sensors & INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, *sensor_timestamp);
    }

    if (sensors & INV_WXYZ_QUAT)
        inv_build_quat(quat, 0, *sensor_timestamp);
}

static void poll_compass(unsigned long* sensor_timestamp)
{
    short compass_short[3];
    long compass[3];

    if (!mpu_get_compass_reg(compass_short, sensor_timestamp))
    {
        compass[0] = (long)compass_short[0];
        compass[1] = (long)compass_short[1];
        compass[2] = (long)compass_short[2];
        imu.compass.data[0] = compass[0];
        imu.compass.data[1] = compass[1];
        imu.compass.data[2] = compass[2];
    }
}

void readImu(void)
{
    unsigned long sensor_timestamp;
    int new_data = 0;

    hal_get_tick_count(&imu_timestamp);
    if (imu_timestamp > next_gyro_ms)
    {
        next_gyro_ms = imu_timestamp + GYRO_READ_MS;
        new_gyro = 1;
    }
    if (imu_timestamp > next_compass_ms)
    {
        next_compass_ms = imu_timestamp + COMPASS_READ_MS;
        new_compass = 1;
    }
    if (imu_timestamp > next_temp_ms)
    {
        next_temp_ms = imu_timestamp + TEMP_READ_MS;
        new_temp = 1;
    }

    if (new_gyro)
    {
        new_gyro = 0;
        poll_fifo(&sensor_timestamp);
        new_data = 1;
    }

    if (new_compass)
    {
        new_compass = 0;
        poll_compass(&sensor_timestamp);
        new_data = 1;
    }

    if (new_data)
    {
        inv_execute_on_data();
        imu_read_from_mpl();

        while (SPI2->SR & SPI_SR_BSY)
            continue;
        txBuffer.imu = imu;
    }
}