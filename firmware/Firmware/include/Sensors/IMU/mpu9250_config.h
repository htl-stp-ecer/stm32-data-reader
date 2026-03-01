#ifndef _MPU9250_CONFIG_H_
#define _MPU9250_CONFIG_H_

// Platform-specific configurations
#define MPU9250_SPI			hspi3
#define	MPU9250_CS_GPIO     IMU_SPI_CS0_GPIO_Port
#define	MPU9250_CS_PIN      IMU_SPI_CS0_Pin

// DMP and sensor configuration
#define MPU9250_ENABLED
#define AK8963_SECONDARY
#define COMPASS_ENABLED

// Target platform definition for InvenSense drivers
#define EMPL_TARGET_STM32F4

// IMU sensor ranges and settings
#define IMU_DEFAULT_GYRO_FSR        2000    // ±2000 dps
#define IMU_DEFAULT_ACCEL_FSR       2       // ±2g
#define IMU_DEFAULT_LPF             42      // 42 Hz low-pass filter
#define IMU_DEFAULT_SAMPLE_RATE     100     // 100 Hz sample rate
#define IMU_DEFAULT_COMPASS_RATE    100     // 100 Hz compass rate

// DMP configuration
#define IMU_DMP_FEATURES    (DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL)
#define IMU_DMP_FIFO_RATE   100     // 100 Hz DMP FIFO rate

// Default orientation matrix (adapt as needed for your board)
// This maps chip axes to board axes
// Current setting: chip X = +E, chip Y = -N, chip Z = +D
#define IMU_GYRO_ORIENTATION_MATRIX  { \
    0, 1, 0, \
    -1, 0, 0, \
    0, 0, -1 \
}

#define IMU_COMPASS_ORIENTATION_MATRIX  { \
    1, 0, 0, \
    0, -1, 0, \
    0, 0, 1 \
}


#endif // _MPU9250_CONFIG_H_