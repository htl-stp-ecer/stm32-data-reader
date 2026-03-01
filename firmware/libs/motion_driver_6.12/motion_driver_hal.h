/**
 * @file motion_driver_hal.h
 * @brief Hardware Abstraction Layer interface for Motion Driver 6.12
 * 
 * This file defines the hardware abstraction layer that you must implement
 * to use the Motion Driver library with your own hardware platform.
 * 
 * The Motion Driver library requires the following HAL functions:
 * - I2C communication (read/write)
 * - Timing functions (delays and timestamps)
 * - Memory management (malloc/free)
 * - Logging functions (optional)
 */

#ifndef MOTION_DRIVER_HAL_H
#define MOTION_DRIVER_HAL_H

#include "mltypes.h"

/**
 * @brief Write data to an I2C device register
 * @param slave_addr    7-bit I2C slave address
 * @param reg_addr      Register address to write to
 * @param length        Number of bytes to write
 * @param data          Pointer to data to write
 * @return 0 on success, non-zero on error
 * 
 * @note This function must perform a complete I2C write transaction
 */
int hal_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                  unsigned short length, const unsigned char* data);

/**
 * @brief Read data from an I2C device register
 * @param slave_addr    7-bit I2C slave address  
 * @param reg_addr      Register address to read from
 * @param length        Number of bytes to read
 * @param data          Pointer to buffer for read data
 * @return 0 on success, non-zero on error
 * 
 * @note This function must perform a complete I2C read transaction
 */
int hal_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                 unsigned short length, unsigned char* data);

/**
 * @brief Delay execution for specified milliseconds
 * @param num_ms Number of milliseconds to delay
 * 
 * @note This function should block for the specified time
 */
void hal_delay_ms(unsigned long num_ms);

/**
 * @brief Get current timestamp in milliseconds
 * @param timestamp Pointer to store current timestamp
 * @return INV_SUCCESS on success
 * 
 * @note The timestamp should be monotonic and ideally millisecond resolution.
 *       It's used for sensor data timestamping and timing calculations.
 */
inv_error_t hal_get_tick_count(inv_time_t * timestamp);

/* Alternative timestamp function signature (used by some library variants) */
inv_time_t hal_inv_get_tick_count(void);

#endif /* MOTION_DRIVER_HAL_H */