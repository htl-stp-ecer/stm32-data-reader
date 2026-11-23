#ifndef IMU
#define IMU
#include "spi/pi_buffer.h"

// DMP FIFO output rate. Heading fusion samples the raw sensors separately.
#define DEFAULT_MPU_HZ   (50)

extern ImuData imu;

// Continuous clockwise heading in degrees relative to startup. The raw gyro
// rate around gravity is integrated after correcting for the estimated bias.
extern volatile float imuFusedHeading;

void setupImu();
void readImu();
void updateImuOrientation(const int8_t gyroOrientation[9], const int8_t compassOrientation[9]);

#endif //IMU
