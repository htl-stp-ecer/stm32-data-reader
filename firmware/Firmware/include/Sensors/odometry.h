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

// Map a body-frame velocity command [vx (m/s), vy (m/s), wz (rad/s)] to the
// per-wheel velocity-PID setpoint (BEMF velocity units) for MOT_MODE_CHASSIS,
// using the configured forward kinematics matrix + per-motor ticks_to_rad.
// Returns 0 for an out-of-range wheel or uncalibrated motor.
int32_t odometry_chassis_wheel_target(uint8_t wheel, float vx, float vy, float wz);

#endif // ODOMETRY_H