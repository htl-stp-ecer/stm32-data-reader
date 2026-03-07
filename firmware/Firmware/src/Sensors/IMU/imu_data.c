#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "mpl.h"
#include "mltypes.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "eMPL_outputs.h"
#include "motion_driver_hal.h"

#include <math.h>

static void rotateBodyToWorld(const long rot_q30[9], const long body[3], long world[3])
{
    for (int i = 0; i < 3; i++)
    {
        world[i] = inv_q30_mult(rot_q30[i * 3 + 0], body[0])
            + inv_q30_mult(rot_q30[i * 3 + 1], body[1])
            + inv_q30_mult(rot_q30[i * 3 + 2], body[2]);
    }
}

static void read_heading(const long quat[4])
{
    long q00, q03, q12, q22;
    long t1, t2;
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

static void integrate_linear_accel(void)
{
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

void imu_read_from_mpl(void)
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

    if (inv_get_sensor_type_gyro(data, &imu.gyro.accuracy, (inv_time_t*)&timestamp))
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

    read_heading(quat);

    if (inv_get_linear_accel(data) == INV_SUCCESS)
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.linearAccel.data[0] = inv_q16_to_float(world[0]);
        imu.linearAccel.data[1] = inv_q16_to_float(world[1]);
        imu.linearAccel.data[2] = inv_q16_to_float(world[2]);

        integrate_linear_accel();
    }
}