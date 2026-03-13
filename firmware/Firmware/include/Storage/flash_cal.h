#ifndef FLASH_CAL_H
#define FLASH_CAL_H

#include "mltypes.h"

/**
 * @brief Calibration flash storage for MPL state persistence.
 *
 * Uses the last 128KB sector of flash bank 1 (sector 11) at 0x080E0000.
 * Firmware is ~101KB and fits in sectors 0-4, so sector 11 is safe.
 *
 * Data layout in flash:
 *   [4 bytes magic] [4 bytes data_len] [data_len bytes MPL state]
 */

#define CAL_FLASH_SECTOR       11
#define CAL_FLASH_ADDR         0x080E0000U
#define CAL_FLASH_MAGIC        0xCA1BDA7AU  /* "CALBDATA" */
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