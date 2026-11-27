#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "mpl.h"
#include "mltypes.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "eMPL_outputs.h"
#include "motion_driver_hal.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile float imuFusedHeading = 0.0f;   // deg, imu.heading/CW frame, continuous

static void rotateBodyToWorld(const long rot_q30[9], const long body[3], long world[3])
{
    for (int i = 0; i < 3; i++)
    {
        world[i] = inv_q30_mult(rot_q30[i * 3 + 0], body[0])
            + inv_q30_mult(rot_q30[i * 3 + 1], body[1])
            + inv_q30_mult(rot_q30[i * 3 + 2], body[2]);
    }
}

// Heading from the DMP 6-axis quaternion: sum of the rotation about the world
// vertical (gravity) between consecutive DMP samples. Independent of the angle
// the IMU is mounted at.
void imu_update_orientation(const long dmp_quat[4])
{
    static float prev_w, prev_x, prev_y, prev_z;
    static uint8_t has_prev = 0;

    const float w = inv_q30_to_float(dmp_quat[0]);
    const float x = inv_q30_to_float(dmp_quat[1]);
    const float y = inv_q30_to_float(dmp_quat[2]);
    const float z = inv_q30_to_float(dmp_quat[3]);

    if (has_prev)
    {
        // W and Z of q_new * conjugate(q_previous), the world-frame rotation
        // between the two samples; they give its twist about world vertical.
        float delta_w = w * prev_w + x * prev_x + y * prev_y + z * prev_z;
        float delta_z = z * prev_w - w * prev_z - x * prev_y + y * prev_x;
        if (delta_w < 0.0f)            // q and -q are the same rotation
        {
            delta_w = -delta_w;
            delta_z = -delta_z;
        }
        // Heading is clockwise positive, the quaternion counter-clockwise.
        imuFusedHeading -= atan2f(delta_z, delta_w) * (360.0f / (float)M_PI);
    }
    prev_w = w;
    prev_x = x;
    prev_y = y;
    prev_z = z;
    has_prev = 1;

    float wrapped_heading = fmodf(imuFusedHeading, 360.0f);
    if (wrapped_heading < 0.0f)
        wrapped_heading += 360.0f;
    imu.heading = wrapped_heading < 360.0f ? wrapped_heading : 0.0f;

    imu.dmpQuat.data[0] = w;
    imu.dmpQuat.data[1] = x;
    imu.dmpQuat.data[2] = y;
    imu.dmpQuat.data[3] = z;
    imu.dmpQuat.accuracy = 3; // DMP provides no accuracy metric.
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
    long rot_mat[9] = {
        ROT_MATRIX_SCALE_LONG, 0, 0,
        0, ROT_MATRIX_SCALE_LONG, 0,
        0, 0, ROT_MATRIX_SCALE_LONG
    };
    unsigned long timestamp;
    long world[3];

    // DMP 6-axis quaternion for body-to-world rotation
    long dmp_quat[4];
    inv_get_6axis_quaternion(dmp_quat);

    // Use DMP 6-axis quat for body-to-world rotation (stable, no mag influence)
    inv_quaternion_to_rotation(dmp_quat, rot_mat);

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

    if (inv_get_sensor_type_compass(data, &imu.compass.accuracy, (inv_time_t*)&timestamp))
    {
        imu.compass.data[0] = inv_q16_to_float(data[0]);
        imu.compass.data[1] = inv_q16_to_float(data[1]);
        imu.compass.data[2] = inv_q16_to_float(data[2]);
    }

    if (inv_get_linear_accel(data) == INV_SUCCESS)
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.linearAccel.data[0] = inv_q16_to_float(world[0]);
        imu.linearAccel.data[1] = inv_q16_to_float(world[1]);
        imu.linearAccel.data[2] = inv_q16_to_float(world[2]);

        integrate_linear_accel();
    }
}
