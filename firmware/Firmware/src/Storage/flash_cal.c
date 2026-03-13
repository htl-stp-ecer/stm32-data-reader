#include "Storage/flash_cal.h"

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "invensense.h"
#include "storage_manager.h"

/* Flash data header stored at CAL_FLASH_ADDR */
typedef struct __attribute__ ((packed))
{
    uint32_t magic;
    uint32_t version;
    uint32_t data_len;
}

cal_flash_header_t;

static uint8_t cal_buffer[CAL_MAX_SIZE];

static HAL_StatusTypeDef flash_erase_cal_sector(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = CAL_FLASH_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7-3.6V */

    return HAL_FLASHEx_Erase(&erase, &sector_error);
}

static HAL_StatusTypeDef flash_write_bytes(uint32_t addr, const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        HAL_StatusTypeDef status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_BYTE, addr + i, data[i]);
        if (status != HAL_OK)
            return status;
    }
    return HAL_OK;
}

int cal_has_saved_data(void)
{
    const cal_flash_header_t* hd = (const cal_flash_header_t*)CAL_FLASH_ADDR;
    return (hd->magic == CAL_FLASH_MAGIC &&
        hd->version == CAL_VERSION &&
        hd->data_len > 0 &&
        hd->data_len <= CAL_MAX_SIZE);
}

inv_error_t cal_save_to_flash(void)
{
    size_t mpl_size = 0;
    inv_error_t result;

    result = inv_get_mpl_state_size(&mpl_size);
    if (result != INV_SUCCESS)
    {
        printf("[CAL] Failed to get MPL state size: %d\r\n", (int)result);
        return result;
    }

    if (mpl_size == 0 || mpl_size > CAL_MAX_SIZE)
    {
        printf("[CAL] MPL state size %u exceeds buffer (%u)\r\n",
               (unsigned)mpl_size, (unsigned)CAL_MAX_SIZE);
        return INV_ERROR;
    }

    result = inv_save_mpl_states(cal_buffer, mpl_size);
    if (result != INV_SUCCESS)
    {
        printf("[CAL] Failed to serialize MPL state: %d\r\n", (int)result);
        return result;
    }

    printf("[CAL] Saving %u bytes to flash sector %d\r\n",
           (unsigned)mpl_size, CAL_FLASH_SECTOR);

    HAL_StatusTypeDef hal_status;

    hal_status = HAL_FLASH_Unlock();
    if (hal_status != HAL_OK)
    {
        printf("[CAL] Flash unlock failed: %d\r\n", (int)hal_status);
        return INV_ERROR;
    }

    hal_status = flash_erase_cal_sector();
    if (hal_status != HAL_OK)
    {
        printf("[CAL] Flash erase failed: %d\r\n", (int)hal_status);
        HAL_FLASH_Lock();
        return INV_ERROR;
    }

    /* Write header */
    cal_flash_header_t hd;
    hd.magic = CAL_FLASH_MAGIC;
    hd.version = CAL_VERSION;
    hd.data_len = (uint32_t)mpl_size;

    hal_status = flash_write_bytes(CAL_FLASH_ADDR, (const uint8_t*)&hd, sizeof(hd));
    if (hal_status != HAL_OK)
    {
        printf("[CAL] Flash write header failed: %d\r\n", (int)hal_status);
        HAL_FLASH_Lock();
        return INV_ERROR;
    }

    /* Write MPL state data */
    hal_status = flash_write_bytes(
        CAL_FLASH_ADDR + sizeof(cal_flash_header_t), cal_buffer, mpl_size);
    if (hal_status != HAL_OK)
    {
        printf("[CAL] Flash write data failed: %d\r\n", (int)hal_status);
        HAL_FLASH_Lock();
        return INV_ERROR;
    }

    HAL_FLASH_Lock();
    printf("[CAL] Calibration saved successfully\r\n");
    return INV_SUCCESS;
}

inv_error_t cal_load_from_flash(void)
{
    const cal_flash_header_t* hd = (const cal_flash_header_t*)CAL_FLASH_ADDR;

    if (hd->magic != CAL_FLASH_MAGIC)
    {
        printf("[CAL] No calibration data in flash (no magic)\r\n");
        return INV_ERROR_CALIBRATION_LOAD;
    }

    if (hd->version != CAL_VERSION)
    {
        printf("[CAL] Stale calibration (version %u, need %u) — erasing\r\n",
               (unsigned)hd->version, (unsigned)CAL_VERSION);
        /* Erase so we don't log this every boot */
        HAL_FLASH_Unlock();
        flash_erase_cal_sector();
        HAL_FLASH_Lock();
        return INV_ERROR_CALIBRATION_LOAD;
    }

    if (hd->data_len == 0 || hd->data_len > CAL_MAX_SIZE)
    {
        printf("[CAL] Invalid calibration data length: %u\r\n",
               (unsigned)hd->data_len);
        return INV_ERROR_CALIBRATION_LOAD;
    }

    const uint8_t* flash_data = (const uint8_t*)(CAL_FLASH_ADDR + sizeof(cal_flash_header_t));

    printf("[CAL] Loading %u bytes from flash\r\n", (unsigned)hd->data_len);

    inv_error_t result = inv_load_mpl_states(flash_data, hd->data_len);
    if (result != INV_SUCCESS)
    {
        printf("[CAL] Failed to load MPL states: %d\r\n", (int)result);
        return result;
    }

    printf("[CAL] Calibration loaded successfully\r\n");
    return INV_SUCCESS;
}