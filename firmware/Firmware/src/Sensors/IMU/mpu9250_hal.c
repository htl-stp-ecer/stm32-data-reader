#include <stdlib.h>

#include "motion_driver_hal.h"
#include "MPU9250.h"
#include "stm32f4xx_hal.h"

unsigned char* mpl_key = (unsigned char*)"eMPL 5.1";

int hal_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                  unsigned short length, const unsigned char* data)
{
    (void)slave_addr; // SPI doesn't use slave address

    // SPI write: MSB must be 0 for write operation, burst allowed
    uint8_t reg = (uint8_t)(reg_addr & 0x7F);

    // Use existing SPI write function from MPU9250.c
    MPU_SPI_Write((uint8_t*)data, reg, length);

    return 0; // InvenSense motion driver uses 0 == success
}

int hal_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                 unsigned short length, unsigned char* data)
{
    (void)slave_addr; // SPI doesn't use slave address

    // SPI read: use register address directly
    uint8_t reg = reg_addr;

    // Use existing SPI read function from MPU9250.c
    MPU_SPI_Read(data, reg, length);

    return 0; // InvenSense motion driver uses 0 == success
}

inv_error_t hal_get_tick_count(inv_time_t* timestamp)
{
    // Use HAL to get current tick count in milliseconds
    *timestamp = HAL_GetTick();
    return INV_SUCCESS; // InvenSense motion driver uses 0 == success
}

inv_time_t hal_inv_get_tick_count(void)
{
    inv_time_t timestamp;
    hal_get_tick_count(&timestamp);
    return timestamp; // Return the current tick count
}

void hal_delay_ms(unsigned long num_ms)
{
    HAL_Delay(num_ms);
}

int _MLPrintLog(int priority, const char* tag, const char* fmt, ...)
{
    (void)priority;
    (void)tag;
    (void)fmt;
    // Do nothing - logging disabled
}