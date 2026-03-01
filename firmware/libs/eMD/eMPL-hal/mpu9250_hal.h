#ifndef _MPU9250_HAL_H_
#define _MPU9250_HAL_H_

#include <stdint.h>

/**
 * @brief Hardware abstraction layer for MPU9250 InvenSense motion drivers
 * 
 * This module provides the platform-specific interface required by the
 * original InvenSense motion drivers. It adapts the driver's I2C-style
 * interface to work with the STM32 SPI implementation.
 */

/**
 * @brief Hardware abstraction layer interface structure
 * 
 * Contains function pointers for all hardware-specific operations
 * required by the InvenSense motion drivers.
 */
typedef struct
{
    int (*read)(unsigned char addr, unsigned char reg, unsigned char len, unsigned char* data);
    int (*write)(unsigned char addr, unsigned char reg, unsigned char len, unsigned char* data);
    int (*get_ms)(unsigned long* count);
    int (*delay_ms)(unsigned long ms);
} mpu9250_hal_t;

/**
 * @brief Initialize the hardware abstraction layer
 * 
 * Sets up the HAL to use the existing SPI implementation from MPU9250.c
 * 
 * @return 0 on success, non-zero on failure
 */
int mpu9250_hal_init(void);

/**
 * @brief Get the hardware abstraction layer interface
 * 
 * @return Pointer to the HAL interface structure
 */
mpu9250_hal_t* mpu9250_get_hal(void);

/**
 * @brief Platform-specific I2C write function (adapted for SPI)
 * 
 * This function implements the interface expected by InvenSense drivers,
 * but internally uses SPI communication through the existing MPU_SPI_Write function.
 * 
 * @param slave_addr I2C slave address (ignored for SPI)
 * @param reg_addr Register address to write to
 * @param length Number of bytes to write
 * @param data Pointer to data buffer
 * @return 0 on success, non-zero on failure
 */
int mpu9250_hal_write(unsigned char slave_addr, unsigned char reg_addr,
                      unsigned char length, unsigned char* data);

/**
 * @brief Platform-specific I2C read function (adapted for SPI)
 * 
 * This function implements the interface expected by InvenSense drivers,
 * but internally uses SPI communication through the existing MPU_SPI_Read function.
 * 
 * @param slave_addr I2C slave address (ignored for SPI)
 * @param reg_addr Register address to read from
 * @param length Number of bytes to read
 * @param data Pointer to data buffer
 * @return 0 on success, non-zero on failure
 */
int mpu9250_hal_read(unsigned char slave_addr, unsigned char reg_addr,
                     unsigned char length, unsigned char* data);

/**
 * @brief Get current time in milliseconds
 * 
 * @param count Pointer to store current time in milliseconds
 * @return 0 on success, non-zero on failure
 */
int mpu9250_hal_get_clock_ms(unsigned long* count);

/**
 * @brief Delay for specified number of milliseconds
 * 
 * @param num_ms Number of milliseconds to delay
 * @return 0 on success, non-zero on failure
 */
int mpu9250_hal_delay_ms(unsigned long num_ms);

#endif // _MPU9250_HAL_H_