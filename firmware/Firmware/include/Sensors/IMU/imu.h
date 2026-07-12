#ifndef IMU
#define IMU
#include "spi/pi_buffer.h"

// Single source of truth for the gyro/DMP output rate. Must be a clean divisor
// of 200 (200,100,50,40,...). The heading fusion derives its fixed sample dt
// from this (imu_data.c: HF_SAMPLE_DT = 1/DEFAULT_MPU_HZ), so they can't drift.
#define DEFAULT_MPU_HZ   (50)

extern ImuData imu;

// Drift-corrected fused heading (degrees, same frame/sign as imu.heading, but
// continuous — does not wrap at 360). Produced entirely in imu_data.c at the
// gyro-FIFO cadence: integrate the world-frame yaw rate (imu.gyro.z) and remove
// its slow bias via ZUPT (zero-rate update while the yaw rate is small). The
// odometry heading consumes this instead of the raw DMP imu.heading.
extern volatile float imuFusedHeading;

void setupImu();
void readImu();
void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9]);

#endif //IMU