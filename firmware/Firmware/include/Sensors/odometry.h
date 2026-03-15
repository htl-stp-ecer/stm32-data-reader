//
// STM32-side dead reckoning odometry using BEMF + IMU heading.
// Kinematics matrix is sent once from the Pi at startup.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "spi/pi_buffer.h"

// Call once when PI_BUFFER_UPDATE_KINEMATICS flag is set.
// Copies the kinematics config from the RxBuffer.
void odometry_configure(const volatile KinematicsConfig* cfg);

// Call when PI_BUFFER_UPDATE_ODOM_RESET flag is set.
// Zeros the integrated position and resets the heading baseline.
void odometry_reset(void);

// Call from main loop after processBEMF() and readImu().
// Integrates wheel velocities into world-frame position using current heading.
void odometry_update(void);

// Copy current odometry state into the TxBuffer field.
void odometry_write_to_spi_buffer(volatile OdometryData* out);

#endif // ODOMETRY_H