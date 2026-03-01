#include "Sensors/IMU/imu.h"

#include "communication_with_pi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpl.h"
#include "mltypes.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "motion_driver_hal.h"
#include "mpu9250_config.h"
#include "spi.h"
#include "Utillity/utillity.h"

#define EARTHS_GRAVITY (9.80665f)

// ===== User-facing globals (unchanged) =====
ImuData imu = {0};
inv_time_t timestamp;
unsigned long next_gyro_ms = 0;
unsigned char new_gyro = 0;
unsigned long next_compass_ms = 0;
unsigned char new_compass = 0;
unsigned long next_temp_ms = 0;
unsigned char new_temp = 0;

// ===== Config =====
#define DEFAULT_MPU_HZ   (50)
#define GYRO_READ_MS (uint32_t)(1000/DEFAULT_MPU_HZ)
#define COMPASS_READ_MS  (100)   // 10 Hz magnetometer via timer
#define TEMP_READ_MS    (500)

// ===== Orientation matrices (from your config) =====
struct platform_data_s
{
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = IMU_GYRO_ORIENTATION_MATRIX
};
static struct platform_data_s compass_pdata = {
    .orientation = IMU_COMPASS_ORIENTATION_MATRIX
};

// ===== Internal state =====
static struct
{
    unsigned char sensors;
    unsigned short dmp_features;
} imu_state = {0};

// ==== Internal function declerations ====
int runImuSelfTest(void);

static void rotateBodyToWorld(const long rot_q30[9], const long body[3], long world[3])
{
    for (int i = 0; i < 3; i++)
    {
        world[i] = inv_q30_mult(rot_q30[i * 3 + 0], body[0])
            + inv_q30_mult(rot_q30[i * 3 + 1], body[1])
            + inv_q30_mult(rot_q30[i * 3 + 2], body[2]);
    }
}


/* =========================
 * Init
 * ========================= */

void setupImu(void)
{
    changeSPIBaudRatePrescaler(&hspi3, SPI_BAUDRATEPRESCALER_64);

    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    /* 1) HW init */
    runImuSelfTest(); //calibrate the gyro and accel biases
    mpu_init(&int_param);

    /* 2) MPL init + feature enables */
    inv_init_mpl();
    inv_enable_quaternion();
    //inv_9x_fusion_use_timestamps(1); //don't need to use since alle data is send regularly
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_in_use_auto_calibration();
    inv_enable_heading_from_gyro();
    inv_enable_eMPL_outputs();
    inv_start_mpl();

    /* 3) Turn sensors on + FIFO config */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);

    /* 4) Read back HW config */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Sync rates into MPL */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_quat_sample_rate(1000000 / gyro_rate);

    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);

    hal_get_tick_count(&timestamp);


    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    imu_state.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(imu_state.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);

    // changeSPIBaudRatePrescaler(&hspi3, SPI_BAUDRATEPRESCALER_2);
}

static void read_from_mpl(void)
{
    long data[9];
    long quat[4];
    long rot_mat[9] = {
        ROT_MATRIX_SCALE_LONG, 0, 0,
        0, ROT_MATRIX_SCALE_LONG, 0,
        0, 0, ROT_MATRIX_SCALE_LONG
    };
    unsigned long timestamp;
    long world[3];
    int quat_accuracy = 0;

    inv_get_quaternion_set(quat, &quat_accuracy, (inv_time_t*)&timestamp);
    imu.quat.data[0] = inv_q30_to_float(quat[0]);
    imu.quat.data[1] = inv_q30_to_float(quat[1]);
    imu.quat.data[2] = inv_q30_to_float(quat[2]);
    imu.quat.data[3] = inv_q30_to_float(quat[3]);
    imu.quat.accuracy = (int8_t)quat_accuracy;

    inv_quaternion_to_rotation(quat, rot_mat);

    if (inv_get_sensor_type_gyro(data, &imu.gyro.accuracy,
                                 (inv_time_t*)&timestamp))
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.gyro.data[0] = inv_q16_to_float(world[0]);
        imu.gyro.data[1] = inv_q16_to_float(world[1]);
        imu.gyro.data[2] = inv_q16_to_float(world[2]);
    }

    if (inv_get_sensor_type_accel(data, &imu.accel.accuracy, (inv_time_t*)&timestamp))
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.accel.data[0] = EARTHS_GRAVITY * inv_q16_to_float(world[0]);
        imu.accel.data[1] = EARTHS_GRAVITY * inv_q16_to_float(world[1]);
        imu.accel.data[2] = EARTHS_GRAVITY * inv_q16_to_float(world[2]);
    }

    imu.compass.data[0] = 0.0f;
    imu.compass.data[1] = 0.0f;
    imu.compass.data[2] = 0.0f;
    imu.compass.accuracy = 0;

    {
        long t1, t2, q00, q03, q12, q22;
        float heading_deg;

        q00 = inv_q29_mult(quat[0], quat[0]);
        q03 = inv_q29_mult(quat[0], quat[3]);
        q12 = inv_q29_mult(quat[1], quat[2]);
        q22 = inv_q29_mult(quat[2], quat[2]);
        t1 = q12 - q03;
        t2 = q22 + q00 - (1L << 30);
        heading_deg = atan2f((float)t1, (float)t2) * 180.f / (float)M_PI;
        if (heading_deg < 0.f)
            heading_deg += 360.f;
        imu.heading = heading_deg;
    }

    if (inv_get_linear_accel(data) == INV_SUCCESS)
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.linearAccel.data[0] = inv_q16_to_float(world[0]);
        imu.linearAccel.data[1] = inv_q16_to_float(world[1]);
        imu.linearAccel.data[2] = inv_q16_to_float(world[2]);

        // Integrate linear accel -> velocity (firmware-side)
        static float filtered_accel[3] = {0, 0, 0};
        static uint32_t last_integration_time = 0;

        const float alpha = 0.3f;
        for (int i = 0; i < 3; i++)
        {
            filtered_accel[i] = alpha * imu.linearAccel.data[i] + (1.0f - alpha) * filtered_accel[i];
        }

        inv_time_t now;
        hal_get_tick_count(&now);
        if (last_integration_time != 0)
        {
            float dt = (float)(now - last_integration_time) / 1000.0f;
            if (dt > 0.0f && dt < 0.1f)
            {
                for (int i = 0; i < 2; i++)
                {
                    imu.accelVelocity.data[i] += filtered_accel[i] * dt;
                    imu.accelVelocity.data[i] *= 0.998f;
                }
            }
        }
        last_integration_time = now;
        imu.accelVelocity.accuracy = imu.linearAccel.accuracy;
    }
}


/* =====================================
 * Poll + build -> run MPL -> publish
 * ===================================== */
void readImu(void)
{
    unsigned long sensor_timestamp;
    int new_data = 0;

    hal_get_tick_count(&timestamp);
    if (timestamp > next_gyro_ms)
    {
        next_gyro_ms = timestamp + GYRO_READ_MS;
        new_gyro = 1;
    }

    if (timestamp > next_temp_ms)
    {
        next_temp_ms = timestamp + TEMP_READ_MS;
        new_temp = 1;
    }

    if (new_gyro)
    {
        new_gyro = 0;

        short gyro[3], accel_short[3], sensors;
        unsigned char more;
        long accel[3], quat[4], temperature;
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */
        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
        if (more)
            new_gyro = 1;
        if (sensors & INV_XYZ_GYRO)
        {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            //outpt raw values (only for testing purposes)
            //imu.gyro.data[0] = (float) gyro[0] / 131.0f;
            //imu.gyro.data[1] = (float) gyro[1] / 131.0f;
            //imu.gyro.data[2] = (float) gyro[2] / 131.0f;
            if (new_temp)
            {
                new_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
                imu.temperature = inv_q16_to_float(temperature);
            }
        }
        if (sensors & INV_XYZ_ACCEL)
        {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            //outpt raw values (only for testing purposes)
            //imu.accel.data[0] = (float) accel_short[0] / 16384 * 9.81;
            //imu.accel.data[1] = (float) accel_short[1] / 16384 * 9.81;
            //imu.accel.data[2] = (float) accel_short[2] / 16384 * 9.81;
        }
        if (sensors & INV_WXYZ_QUAT)
        {
            inv_build_quat(quat, 0, sensor_timestamp);
            new_data = 1;
            //outpt raw values (only for testing purposes)
            // imu.quat.data[0] = inv_q30_to_float(quat[0]);
            // imu.quat.data[1] = inv_q30_to_float(quat[1]);
            // imu.quat.data[2] = inv_q30_to_float(quat[2]);
            // imu.quat.data[3] = inv_q30_to_float(quat[3]);
        }
    }

    if (new_data)
    {
        inv_execute_on_data();
        /* This function reads bias-compensated sensor data and sensor
         * fusion outputs from the MPL. The outputs are formatted as seen
         * in eMPL_outputs.c. This function only needs to be called at the
         * rate requested by the host.
         */
        read_from_mpl();
    }

    //try updating the txBuffer
    if (new_data)
    {
        while (SPI2->SR & SPI_SR_BSY) //check if the spi buffer is in use
            continue;

        txBuffer.imu = imu;
    }
}

/* =====================================
 * Runtime orientation update
 * ===================================== */
void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9])
{
    unsigned short gyro_fsr, compass_fsr;
    unsigned char accel_fsr;

    for (int i = 0; i < 9; ++i)
    {
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

/* ================
 * Self-test passthrough
 * ================ */
int runImuSelfTest(void)
{
    long gyro[3], accel[3];
    int result = mpu_run_6500_self_test(gyro, accel, 0);

    if ((result == 0x7) || (result == 0x3)) // it is enouth if gyro and accel works
    //(compass not needed for setting bais; compass is fucked since we use SPI and I am to lacy to fix invesence shit)
    {
        // Convert and push to HW offset regs
        for (int i = 0; i < 3; ++i)
        {
            gyro[i] = (long)(gyro[i] * 32.8f); // to ±1000 dps
            accel[i] = (long)(accel[i] * 2048.f); // to ±16 g
            accel[i] >>= 16;
            gyro[i] >>= 16;
        }
        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_6500_reg(accel);
    }
    return result;
}