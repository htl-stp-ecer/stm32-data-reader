#ifndef FLASH_CAL_H
#define FLASH_CAL_H

#include "mltypes.h"

/**
 * @brief Calibration flash storage for MPL state persistence.
 *
 * Uses sector 12 (first 16KB sector of Bank 2) at 0x08100000.
 * Bank 2 can be erased while the CPU executes code from Bank 1 (dual-bank RWW).
 * Requires DB1M option byte = 1, which is the factory default on 2MB STM32F427VI parts.
 *
 * Data layout in flash:
 *   [4 bytes magic] [4 bytes version] [4 bytes data_len] [data_len bytes MPL state]
 *
 * Bump CAL_VERSION whenever the MPL feature set or IMU configuration changes
 * (e.g. switching between 6-axis/9-axis fusion, changing DMP features).
 * Old cal data with a mismatched version is silently rejected on load.
 */

#define CAL_FLASH_SECTOR       12
#define CAL_FLASH_ADDR         0x08100000U
#define CAL_FLASH_MAGIC        0xCA1BDA7AU  /* "CALBDATA" */
#define CAL_VERSION            2            /* v2: 6-axis DMP quat + MPL compass */
#define CAL_MAX_SIZE           4096         /* Max bytes for MPL state */

/**
 * @brief Save current MPL calibration state to flash.
 * @return INV_SUCCESS on success, error code otherwise.
 */
inv_error_t cal_save_to_flash(void);

/**
 * @brief Load MPL calibration state from flash.
 * Must be called after inv_init_mpl() and feature enables,
 * but before inv_start_mpl().
 * @return INV_SUCCESS on success, error code otherwise.
 */
inv_error_t cal_load_from_flash(void);

/**
 * @brief Check if valid calibration data exists in flash.
 * @return 1 if valid data found, 0 otherwise.
 */
int cal_has_saved_data(void);

#endif /* FLASH_CAL_H */