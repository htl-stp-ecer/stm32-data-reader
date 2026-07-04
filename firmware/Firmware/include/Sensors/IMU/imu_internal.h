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

#endif // IMU_INTERNAL_H