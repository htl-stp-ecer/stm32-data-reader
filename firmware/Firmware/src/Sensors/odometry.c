//
// STM32-side dead reckoning odometry using BEMF + IMU heading.
// Includes per-motor velocity tracking (~200Hz), midpoint integration,
// rotational slip detection (IMU vs kinematics wz),
// linear slip detection (per-wheel residuals + accel sanity check).
// IMU heading/wz is always authoritative — wheels provide vx/vy only.
//

#include "Sensors/odometry.h"
#include "Actors/motor.h"
#include "Sensors/IMU/imu.h"
#include "Sensors/bemf.h"
#include "Hardware/timer.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Tuning constants ---
// Max angular velocity disagreement before flagging rotational slip (rad/s)
#define WZ_SLIP_THRESHOLD 0.5f
// Max wheel residual before flagging that wheel as slipping (rad/s)
#define WHEEL_RESIDUAL_THRESHOLD 1.5f
// Max velocity-change vs accel disagreement (m/s^2) before flagging accel slip
#define ACCEL_SLIP_THRESHOLD 1.0f
// Dampen factor for linear velocity when rotational slip detected (0=freeze, 1=full trust)
#define ROTATION_SLIP_VEL_DAMPEN 0.5f
// Max staleness before zeroing a motor's velocity (microseconds) — 5ms
#define MAX_STALE_US 5000

// Kinematics config (copied from RxBuffer when flag is set)
static KinematicsConfig kin = {0};
static uint8_t configured = 0;

// Odometry state
static float pos_x = 0.0f;
static float pos_y = 0.0f;
static float heading_baseline = 0.0f; // IMU heading at last reset (degrees, CW+)

// Per-motor velocity tracking
static float w[4] = {0}; // current wheel angular velocity (rad/s)
static int32_t prev_position[4] = {0}; // previous position for delta
static uint32_t last_conv[4] = {0}; // per-motor conversion count at last update
static uint32_t last_motor_update_us[4] = {0}; // timestamp of last velocity update per motor
static uint8_t prev_position_init = 0;

// Last computed body-frame velocities (for SPI output)
static float last_vx = 0.0f;
static float last_vy = 0.0f;
static float last_wz = 0.0f;

// Previous heading for midpoint integration
static float prev_heading_rad = 0.0f;

// Previous body-frame velocity for accel sanity check
static float prev_body_vx = 0.0f;
static float prev_body_vy = 0.0f;

// Global odometry tick tracking
static uint32_t last_bemf_conv = 0;
static uint32_t last_update_us = 0;

void odometry_configure(const volatile KinematicsConfig* cfg)
{
    memcpy((void*)&kin, (const void*)cfg, sizeof(KinematicsConfig));
    configured = 1;

    uint32_t now = microSeconds;
    for (int i = 0; i < 4; i++)
    {
        prev_position[i] = motor_data.position[i];
        last_conv[i] = bemfConvCountPerMotor[i];
        last_motor_update_us[i] = now;
        w[i] = 0.0f;
    }
    prev_position_init = 1;
    last_update_us = now;
    last_bemf_conv = bemfConvCount;

    float heading_deg = -(imu.heading - heading_baseline);
    prev_heading_rad = heading_deg * (M_PI / 180.0f);
    prev_body_vx = 0.0f;
    prev_body_vy = 0.0f;
}

void odometry_reset(void)
{
    pos_x = 0.0f;
    pos_y = 0.0f;
    heading_baseline = imu.heading;

    last_vx = 0.0f;
    last_vy = 0.0f;
    last_wz = 0.0f;

    uint32_t now = microSeconds;
    for (int i = 0; i < 4; i++)
    {
        prev_position[i] = motor_data.position[i];
        last_conv[i] = bemfConvCountPerMotor[i];
        last_motor_update_us[i] = now;
        w[i] = 0.0f;
    }
    prev_position_init = 1;
    last_update_us = now;
    last_bemf_conv = bemfConvCount;

    prev_heading_rad = 0.0f;
    prev_body_vx = 0.0f;
    prev_body_vy = 0.0f;
}

void odometry_update(void)
{
    if (!configured)
        return;

    // Run whenever any motor gets a new BEMF reading (~200Hz)
    uint32_t current_conv = bemfConvCount;
    if (current_conv == last_bemf_conv)
        return;
    last_bemf_conv = current_conv;

    uint32_t now = microSeconds;

    // Compute global dt for position integration
    uint32_t elapsed = now - last_update_us;
    if (elapsed == 0)
        return;
    float dt = (float)elapsed * 1e-6f;
    if (dt > 0.1f)
        dt = 0.1f;
    last_update_us = now;

    // Initialize position tracking on first call
    if (!prev_position_init)
    {
        for (int i = 0; i < 4; i++)
        {
            prev_position[i] = motor_data.position[i];
            last_conv[i] = bemfConvCountPerMotor[i];
            last_motor_update_us[i] = now;
        }
        prev_position_init = 1;
        return;
    }

    // ========================================================================
    // Step 1: Update per-motor velocities (only for motors with new data)
    // ========================================================================
    for (int i = 0; i < 4; i++)
    {
        uint32_t conv_i = bemfConvCountPerMotor[i];
        if (conv_i != last_conv[i])
        {
            // This motor has a fresh BEMF reading
            uint32_t motor_elapsed = now - last_motor_update_us[i];
            if (motor_elapsed > 0)
            {
                float motor_dt = (float)motor_elapsed * 1e-6f;
                if (motor_dt > 0.1f)
                    motor_dt = 0.1f;

                int32_t delta = motor_data.position[i] - prev_position[i];
                prev_position[i] = motor_data.position[i];
                w[i] = (float)delta * kin.ticks_to_rad[i] / motor_dt;
            }
            last_conv[i] = conv_i;
            last_motor_update_us[i] = now;
        }
        else
        {
            // Stale — if too old, zero the velocity (motor probably stopped)
            uint32_t stale_us = now - last_motor_update_us[i];
            if (stale_us > MAX_STALE_US)
                w[i] = 0.0f;
        }
    }

    // ========================================================================
    // Step 2: IMU heading — always the authority for heading and wz
    // ========================================================================
    float heading_deg = -(imu.heading - heading_baseline);
    float heading_rad = heading_deg * (M_PI / 180.0f);
    float imu_wz = (heading_rad - prev_heading_rad) / dt;

    // ========================================================================
    // Step 3: Compute body velocity from wheels (vx/vy — wz from IMU)
    // ========================================================================
    float vx = 0.0f, vy = 0.0f, wz_wheels = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        vx += kin.inv_matrix[0][i] * w[i];
        vy += kin.inv_matrix[1][i] * w[i];
        wz_wheels += kin.inv_matrix[2][i] * w[i];
    }

    // ========================================================================
    // Step 4: Rotational slip detection — if wheels disagree with IMU wz,
    //         the wheels are slipping, so also dampen trust in vx/vy
    // ========================================================================
    float wz_error = fabsf(wz_wheels - imu_wz);
    uint8_t rotation_slip = (wz_error > WZ_SLIP_THRESHOLD);

    if (rotation_slip)
    {
        vx *= ROTATION_SLIP_VEL_DAMPEN;
        vy *= ROTATION_SLIP_VEL_DAMPEN;
    }

    // ========================================================================
    // Step 5: Linear slip detection — per-wheel residuals via forward matrix
    // ========================================================================
    uint8_t wheel_slipping[4] = {0};
    int slip_count = 0;
    for (int i = 0; i < 4; i++)
    {
        float w_expected = kin.fwd_matrix[i][0] * vx
            + kin.fwd_matrix[i][1] * vy
            + kin.fwd_matrix[i][2] * imu_wz;
        float residual = fabsf(w[i] - w_expected);
        if (residual > WHEEL_RESIDUAL_THRESHOLD)
        {
            wheel_slipping[i] = 1;
            slip_count++;
        }
    }

    // If 1-2 wheels are slipping, clamp them to predicted and recompute vx/vy
    if (slip_count > 0 && slip_count <= 2)
    {
        for (int i = 0; i < 4; i++)
        {
            if (wheel_slipping[i])
            {
                w[i] = kin.fwd_matrix[i][0] * vx
                    + kin.fwd_matrix[i][1] * vy
                    + kin.fwd_matrix[i][2] * imu_wz;
            }
        }
        vx = 0.0f;
        vy = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            vx += kin.inv_matrix[0][i] * w[i];
            vy += kin.inv_matrix[1][i] * w[i];
        }
    }

    // ========================================================================
    // Step 6: Accel sanity check — flag if velocity change disagrees with IMU
    // ========================================================================
    float imu_ax = imu.linearAccel.data[0] * 9.80665f;
    float imu_ay = imu.linearAccel.data[1] * 9.80665f;

    // Rotate IMU world-frame accel to body frame for comparison
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);
    float imu_ax_body = cos_h * imu_ax + sin_h * imu_ay;
    float imu_ay_body = -sin_h * imu_ax + cos_h * imu_ay;

    float dvx = (vx - prev_body_vx) / dt;
    float dvy = (vy - prev_body_vy) / dt;

    float accel_error = sqrtf((dvx - imu_ax_body) * (dvx - imu_ax_body)
        + (dvy - imu_ay_body) * (dvy - imu_ay_body));

    if (accel_error > ACCEL_SLIP_THRESHOLD)
    {
        vx = 0.5f * vx + 0.5f * prev_body_vx;
        vy = 0.5f * vy + 0.5f * prev_body_vy;
    }

    prev_body_vx = vx;
    prev_body_vy = vy;

    // ========================================================================
    // Step 7: Midpoint integration — use average heading for world-frame rotation
    // ========================================================================
    float mid_heading = 0.5f * (prev_heading_rad + heading_rad);
    float cos_mid = cosf(mid_heading);
    float sin_mid = sinf(mid_heading);

    float vx_world = cos_mid * vx - sin_mid * vy;
    float vy_world = sin_mid * vx + cos_mid * vy;

    // Store body-frame velocities for SPI output — wz always from IMU
    last_vx = vx;
    last_vy = vy;
    last_wz = imu_wz;

    // Integrate position using midpoint heading
    pos_x += vx_world * dt;
    pos_y += vy_world * dt;

    prev_heading_rad = heading_rad;
}

void odometry_write_to_spi_buffer(volatile OdometryData* out)
{
    out->pos_x = pos_x;
    out->pos_y = pos_y;
    out->heading = -(imu.heading - heading_baseline) * (M_PI / 180.0f);
    out->vx = last_vx;
    out->vy = last_vy;
    out->wz = last_wz;
}
