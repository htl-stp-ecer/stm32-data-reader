#ifndef IMU
#define IMU
#include "spi/pi_buffer.h"

extern ImuData imu;

void setupImu();
void readImu();
void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9]);

#endif //IMU