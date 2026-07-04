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
#include <math.h>

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
        // NOTE: sourcing the body gyro from mpu_get_gyro_reg() (raw registers)
        // gives a clean 100Hz gyro at rest but corrupts it during rotation
        // (axis/sign vs the DMP frame, or SPI contention with the FIFO read) —
        // fused heading blows up while turning. Reverted to the DMP FIFO
        // cal-gyro at 50Hz, which is correct at both rest and motion. Raising the
        // rate cleanly is deferred (see task: raw-gyro axis/SPI fix).
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

// ── DMP/FIFO-free high-rate heading path ────────────────────────────────────
// Reads the raw gyro+accel registers directly (~200Hz — the useful ceiling for
// the 98Hz gyro DLPF; faster only adds SPI load, not information) and computes
// the yaw rate about the WORLD VERTICAL as gyro . gravity_unit. The dot product
// is frame-independent, so no orientation matrix or DMP quaternion is needed and
// a fixed mounting tilt cancels out. Gravity_unit is a slow LPF of the accel
// direction (stable during on-table spins; linear accel is smoothed out). Feeds
// heading_fusion_update() with the true measured dt.
#define FASTGYRO_GATE_MS   5u       // ~200Hz
#define HF_FAST_SIGN       (-1.0f)  // robot MPU is z-down (gravity ~-Z); this makes
                                    // omega_vert match the imu.heading/CW sign convention
// Gyro scale trim: the raw MPU9250 gyro over-reads yaw by ~2.25% vs the calib
// ICM-42688-P ground truth (measured over ~3.5 turns each way, spread <0.2%,
// consistent both directions -> a stable sensor-scale difference, unaffected by
// the ICM/MPU mounting offset since angular velocity is body-wide). 1/1.0225.
#define HF_GYRO_SCALE      (0.9780f)
static inv_time_t fast_last_ms = 0;
static float g_gyro_dps_per_lsb = 0.0f;              // set lazily from gyro FSR
static float grav_x = 0.0f, grav_y = 0.0f, grav_z = 1.0f;  // LPF gravity unit (body frame)
static uint8_t grav_seeded = 0;

void readGyroFast(void)
{
    inv_time_t now_ms;
    hal_get_tick_count(&now_ms);

    if (g_gyro_dps_per_lsb == 0.0f)
    {
        unsigned short fsr = 2000;
        mpu_get_gyro_fsr(&fsr);
        g_gyro_dps_per_lsb = (fsr > 0) ? ((float)fsr / 32768.0f) : (2000.0f / 32768.0f);
    }

    if (fast_last_ms != 0 && (inv_time_t)(now_ms - fast_last_ms) < FASTGYRO_GATE_MS)
        return;
    float dt = (fast_last_ms == 0) ? 0.005f : (float)(now_ms - fast_last_ms) * 1e-3f;
    fast_last_ms = now_ms;

    short g[3], a[3];
    unsigned long ts;
    if (mpu_get_gyro_reg(g, &ts) != 0) return;
    if (mpu_get_accel_reg(a, &ts) != 0) return;

    // gravity direction (unit) via slow LPF of the accel vector
    float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 1.0f)
    {
        if (!grav_seeded)
        {
            // Seed gravity from the first accel read (no LPF ramp-up transient —
            // the robot MPU is z-down, so the default (0,0,1) would take ~1s to
            // flip to (0,0,-1), adding a boot-time heading error).
            grav_x = ax / an; grav_y = ay / an; grav_z = az / an;
            grav_seeded = 1;
        }
        else
        {
            const float alpha = 0.02f;  // tau ~0.25s @200Hz — gravity moves slowly
            grav_x += alpha * (ax / an - grav_x);
            grav_y += alpha * (ay / an - grav_y);
            grav_z += alpha * (az / an - grav_z);
        }
    }
    float gn = sqrtf(grav_x * grav_x + grav_y * grav_y + grav_z * grav_z);
    if (gn < 0.1f) return;

    // yaw rate about world vertical = gyro . gravity_unit  (deg/s)
    float gx = (float)g[0] * g_gyro_dps_per_lsb;
    float gy = (float)g[1] * g_gyro_dps_per_lsb;
    float gz = (float)g[2] * g_gyro_dps_per_lsb;
    float omega_vert = HF_FAST_SIGN * HF_GYRO_SCALE * (gx * grav_x + gy * grav_y + gz * grav_z) / gn;

    heading_fusion_update(omega_vert, dt);
}

void readImu(void)
{
    unsigned long sensor_timestamp;
    int new_data = 0;

    readGyroFast();   // DMP/FIFO-free heading path (self-gated to ~200Hz)

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

        /* Trigger save when any sensor accuracy improves */
        if (imu.gyro.accuracy > best_gyro_accuracy ||
            imu.accel.accuracy > best_accel_accuracy ||
            imu.compass.accuracy > best_compass_accuracy)
        {
            best_gyro_accuracy = imu.gyro.accuracy > best_gyro_accuracy
                                     ? imu.gyro.accuracy
                                     : best_gyro_accuracy;
            best_accel_accuracy = imu.accel.accuracy > best_accel_accuracy
                                      ? imu.accel.accuracy
                                      : best_accel_accuracy;
            best_compass_accuracy = imu.compass.accuracy > best_compass_accuracy
                                        ? imu.compass.accuracy
                                        : best_compass_accuracy;
            cal_needs_save = 1;
        }

        if (cal_needs_save && imu_timestamp >= next_cal_save_ms)
        {
            cal_needs_save = 0;
            next_cal_save_ms = imu_timestamp + CAL_PERIODIC_SAVE_MS;
            cal_save_to_flash();
        }

        if (!spi2_wait_idle())
            return; // SPI2 stuck — skip this IMU buffer update
        txBuffer.imu = imu;
    }
}