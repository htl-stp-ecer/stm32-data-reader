#ifndef IMU_INTERNAL_H
#define IMU_INTERNAL_H

#include <stdint.h>
#include "mltypes.h"

#define EARTHS_GRAVITY (9.80665f)

struct platform_data_s
{
    signed char orientation[9];
};

extern struct platform_data_s gyro_pdata;
extern struct platform_data_s compass_pdata;
extern inv_time_t imu_timestamp;

void imu_read_from_mpl(void);
int imu_run_self_test(void);

// High-rate DMP/FIFO-free heading path (imu.c) + its fusion step (imu_data.c).
// readGyroFast() reads the raw gyro+accel registers at ~200Hz, computes the yaw
// rate about the world-vertical as gyro . gravity_unit (tilt-agnostic, no DMP
// quaternion needed), and feeds heading_fusion_update(rate_dps, dt).
void readGyroFast(void);
void heading_fusion_update(float rate_dps, float dt);
// Load the reboot-persisted gyro bias-vs-temperature model. Call once at setup,
// before motion (may erase-compact its flash log). See gyro_tc_persist.h.
void heading_fusion_load_persisted(void);

#endif // IMU_INTERNAL_H