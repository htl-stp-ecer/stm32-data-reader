#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "communication_with_pi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "data_builder.h"
#include "motion_driver_hal.h"
#include "spi.h"
#include "Storage/flash_cal.h"

#include <stdio.h>

#define DEFAULT_MPU_HZ   (50)
#define GYRO_READ_MS     (uint32_t)(1000/DEFAULT_MPU_HZ)
#define COMPASS_READ_MS  (100)
#define TEMP_READ_MS     (500)

/* Save shortly after accuracy improves */
#define CAL_SAVE_INTERVAL_MS       30000
/* Re-save periodically to capture refined biases (every 5 min) */
#define CAL_PERIODIC_SAVE_MS      300000

ImuData imu = {0};
inv_time_t imu_timestamp;

static unsigned long next_gyro_ms = 0;
static unsigned char new_gyro = 0;
static unsigned long next_compass_ms = 0;
static unsigned char new_compass = 0;
static unsigned long next_temp_ms = 0;
static unsigned char new_temp = 0;

/* Calibration auto-save state */
static int8_t best_gyro_accuracy = 0;
static int8_t best_accel_accuracy = 0;
static int8_t best_compass_accuracy = 0;
static unsigned long next_cal_save_ms = CAL_SAVE_INTERVAL_MS;
static int cal_needs_save = 0;

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
    {
        inv_build_quat(quat, 0, *sensor_timestamp);

        /* Store DMP 6-axis quaternion (gyro+accel only, no mag) */
        imu.dmpQuat.data[0] = inv_q30_to_float(quat[0]);
        imu.dmpQuat.data[1] = inv_q30_to_float(quat[1]);
        imu.dmpQuat.data[2] = inv_q30_to_float(quat[2]);
        imu.dmpQuat.data[3] = inv_q30_to_float(quat[3]);
        imu.dmpQuat.accuracy = 3; /* DMP quat has no accuracy metric */
    }
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

        /* Feed compass data into MPL for calibration and 9-axis fusion */
        inv_build_compass(compass, INV_NEW_DATA | INV_RAW_DATA | INV_SENSOR_ON, *sensor_timestamp);
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

        /* Track if calibration accuracy improved */
        if (imu.gyro.accuracy > best_gyro_accuracy ||
            imu.accel.accuracy > best_accel_accuracy ||
            imu.compass.accuracy > best_compass_accuracy)
        {
            best_gyro_accuracy = imu.gyro.accuracy;
            best_accel_accuracy = imu.accel.accuracy;
            best_compass_accuracy = imu.compass.accuracy;
            cal_needs_save = 1;
        }

        /* Save after accuracy improves */
        if (cal_needs_save && imu_timestamp > next_cal_save_ms)
        {
            next_cal_save_ms = imu_timestamp + CAL_SAVE_INTERVAL_MS;
            cal_needs_save = 0;
            printf("[IMU] Accuracy improved (gyro=%d accel=%d compass=%d), saving calibration\r\n",
                   best_gyro_accuracy, best_accel_accuracy, best_compass_accuracy);
            cal_save_to_flash();
        }

        /* Periodic re-save to capture refined biases (gyro temp comp, etc.) */
        {
            static unsigned long next_periodic_save = CAL_PERIODIC_SAVE_MS;
            if (imu_timestamp > next_periodic_save)
            {
                next_periodic_save = imu_timestamp + CAL_PERIODIC_SAVE_MS;
                printf("[IMU] Periodic calibration save (gyro=%d accel=%d compass=%d)\r\n",
                       imu.gyro.accuracy, imu.accel.accuracy, imu.compass.accuracy);
                cal_save_to_flash();
            }
        }

        /* Log accuracy periodically (every 10s) */
        {
            static unsigned long next_accuracy_log = 10000;
            if (imu_timestamp > next_accuracy_log)
            {
                next_accuracy_log = imu_timestamp + 10000;
                printf("[IMU] gyro_acc=%d accel_acc=%d compass_acc=%d\r\n",
                       imu.gyro.accuracy, imu.accel.accuracy,
                       imu.compass.accuracy);
            }
        }

        while (SPI2->SR & SPI_SR_BSY)
            continue;
        txBuffer.imu = imu;
    }
}