//
// STM32-side dead reckoning odometry using BEMF + IMU heading.
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

// Kinematics config (copied from RxBuffer when flag is set)
static KinematicsConfig kin = {0};
static uint8_t configured = 0;

// Odometry state
static float pos_x = 0.0f;
static float pos_y = 0.0f;
static float heading_baseline = 0.0f; // IMU heading at last reset (degrees, CW+)

// Previous motor positions for delta computation
static int32_t prev_position[4] = {0};
static uint8_t prev_position_init = 0;

// Last computed body-frame velocities (for SPI output)
static float last_vx = 0.0f;
static float last_vy = 0.0f;
static float last_wz = 0.0f;

// Track BEMF full cycles (all 4 motors measured = every 4th conversion)
static uint32_t last_bemf_cycle = 0;

// Timing
static uint32_t last_update_us = 0;

void odometry_configure(const volatile KinematicsConfig* cfg)
{
    memcpy((void*)&kin, (const void*)cfg, sizeof(KinematicsConfig));
    configured = 1;

    // Re-initialize position tracking
    for (int i = 0; i < 4; i++)
        prev_position[i] = motor_data.position[i];
    prev_position_init = 1;
    last_update_us = microSeconds;
    last_bemf_cycle = bemfConvCount / MOTOR_COUNT;
}

void odometry_reset(void)
{
    pos_x = 0.0f;
    pos_y = 0.0f;
    heading_baseline = imu.heading; // snapshot current IMU heading as zero

    last_vx = 0.0f;
    last_vy = 0.0f;
    last_wz = 0.0f;

    // Reset position deltas
    for (int i = 0; i < 4; i++)
        prev_position[i] = motor_data.position[i];
    prev_position_init = 1;
    last_update_us = microSeconds;
    last_bemf_cycle = bemfConvCount / MOTOR_COUNT;
}

void odometry_update(void)
{
    if (!configured)
        return;

    // Only update when a new BEMF conversion has completed.
    // This gives stable velocity at the BEMF sample rate (~200Hz per motor)
    // instead of spiky zero/peak alternation from the fast main loop.
    uint32_t current_cycle = bemfConvCount / MOTOR_COUNT;
    if (current_cycle == last_bemf_cycle)
        return;
    last_bemf_cycle = current_cycle;

    // Compute dt from microsecond timer
    uint32_t now = microSeconds;
    uint32_t elapsed = now - last_update_us;
    if (elapsed == 0)
        return;
    float dt = (float)elapsed * 1e-6f;

    // Clamp dt to avoid huge jumps on first update or timer wrap
    if (dt > 0.1f)
        dt = 0.1f;

    last_update_us = now;

    // Initialize position tracking on first call
    if (!prev_position_init)
    {
        for (int i = 0; i < 4; i++)
            prev_position[i] = motor_data.position[i];
        prev_position_init = 1;
        return;
    }

    // Compute wheel angular velocities from position deltas
    float w[4];
    for (int i = 0; i < 4; i++)
    {
        int32_t delta = motor_data.position[i] - prev_position[i];
        prev_position[i] = motor_data.position[i];
        // ticks_to_rad converts BEMF ticks to radians
        w[i] = (float)delta * kin.ticks_to_rad[i] / dt;
    }

    // Apply pre-baked inverse kinematics matrix: wheel speeds -> body velocity
    // Matrix already includes wheel radius and geometry constants
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        vx += kin.inv_matrix[0][i] * w[i];
        vy += kin.inv_matrix[1][i] * w[i];
        wz += kin.inv_matrix[2][i] * w[i];
    }

    // Get heading from IMU (degrees, CW-positive firmware convention)
    // Negate to CCW-positive (library/ENU convention) for correct world-frame rotation
    float heading_deg = -(imu.heading - heading_baseline);
    float heading_rad = heading_deg * (M_PI / 180.0f);

    // Rotate body velocity to world frame
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);
    float vx_world = cos_h * vx - sin_h * vy;
    float vy_world = sin_h * vx + cos_h * vy;

    // Store body-frame velocities for SPI output
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;

    // Integrate position
    pos_x += vx_world * dt;
    pos_y += vy_world * dt;
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