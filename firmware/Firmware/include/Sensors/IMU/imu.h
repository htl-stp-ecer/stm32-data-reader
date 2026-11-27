#ifndef IMU
#define IMU
#include "spi/pi_buffer.h"

// DMP FIFO output rate.
#define DEFAULT_MPU_HZ   (50)

extern ImuData imu;

// Continuous clockwise heading in degrees relative to startup: the rotation of the
// DMP quaternion about world vertical (gravity), summed over all DMP samples.
extern volatile float imuFusedHeading;

void setupImu();
void readImu();
void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9]);

#endif //IMU
