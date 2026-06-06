// ============================================================================
// MPL calibration flash persistence — DISABLED.
//
// The previous implementation erased + reprogrammed sector 12 (first 16 KiB of
// Bank 2) every time the in-use auto-calibration produced a "better" accuracy
// reading. On our STM32F427VI parts this blocked the firmware main loop for
// multiple minutes per save in practice: hb counters froze for ~9 min wallclock
// while the sensor read paths (SPI ISR / DMA) stayed alive. During that window
// every Pi-issued servo position_cmd reached the STM32 over SPI but the PWM
// registers never updated — symptoms users saw as "servos no longer move".
//
// We don't actually need persisted calibration: the DMP runs `inv_init_mpl` +
// `inv_enable_in_use_auto_calibration` + `inv_enable_fast_nomot` on every boot
// and converges to a usable bias within 2-3 minutes of normal motion. Saving
// that across boots only mattered if we wanted a cold-start IMU to be accurate
// in the first few seconds — which we don't, the robot always re-calibrates
// during M000SetupMission anyway.
//
// All three public functions are now no-ops:
//   - cal_save_to_flash() returns INV_SUCCESS so the auto-save site in imu.c
//     and the Pi-triggered save in main.c silently skip without log noise.
//   - cal_load_from_flash() returns INV_ERROR_CALIBRATION_LOAD so imu_setup
//     takes the existing "starting fresh" branch.
//   - cal_has_saved_data() reports 0.
//
// If we ever bring this back: do it via the IT (interrupt) variants
// (HAL_FLASH_Program_IT / HAL_FLASHEx_Erase_IT), drive it from a low-priority
// background task that yields between sectors, and make absolutely sure the
// SysTick + PWM-update timers stay in their NVIC priority group above the
// flash IRQ. Don't restore the polling variants.
// ============================================================================

#include "Storage/flash_cal.h"

#include "invensense.h"

inv_error_t cal_save_to_flash(void)
{
    return INV_SUCCESS;
}

inv_error_t cal_load_from_flash(void)
{
    return INV_ERROR_CALIBRATION_LOAD;
}

int cal_has_saved_data(void)
{
    return 0;
}
