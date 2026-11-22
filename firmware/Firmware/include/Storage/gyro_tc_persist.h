#ifndef GYRO_TC_PERSIST_H
#define GYRO_TC_PERSIST_H

#include <stdint.h>

/**
 * @brief Reboot-persistent storage for the gyro bias-vs-temperature model.
 *
 * The heading fusion (imu_data.c) learns a linear model of the gyro yaw-rate
 * bias against chip temperature. That model only becomes useful once it has seen
 * a wide enough temperature spread — which mostly happens during the cold-boot
 * warmup ramp. If the model started empty every boot, it would never be trained
 * in time to compensate the very warmup it exists for. So we persist it.
 *
 * IMPORTANT — no flash erase during operation. The old MPL-cal persistence was
 * disabled because per-save sector erases blocked the main loop (froze PWM). We
 * avoid that: the model is stored in an APPEND LOG (sector 13, Bank 2). Runtime
 * saves only PROGRAM one small record into a free slot (0xFF -> data, sub-ms,
 * RWW-safe since code runs from Bank 1). The sector is only ERASED at boot, and
 * only when the log is full or corrupt — i.e. before any motion. See the .c.
 *
 * The instantaneous bias is NOT persisted (ZUPT re-learns it every boot in
 * seconds); only the slow-to-learn temperature SLOPE and its fit accumulators.
 */

#define GTC_FLASH_SECTOR   13            /* Bank 2, 16 KiB, RWW-safe */
#define GTC_FLASH_ADDR     0x08104000U
#define GTC_FLASH_SIZE     0x4000U       /* 16 KiB */
#define GTC_SLOT_SIZE      64U           /* one padded record per slot */
#define GTC_SLOT_COUNT     (GTC_FLASH_SIZE / GTC_SLOT_SIZE)  /* 256 */
#define GTC_MAGIC          0x47544331U   /* "GTC1" */
#define GTC_VERSION        2U            /* v2: added sBB for the R^2 gate */

/* Persisted state: the fit accumulators + derived slope + validity. */
typedef struct
{
    float slope;   /* learned bias-vs-temp slope (dps/degC) */
    float n;       /* forgetting-weighted fit-point count */
    float sT;      /* sum w*T    */
    float sB;      /* sum w*bias */
    float sTT;     /* sum w*T^2  */
    float sTB;     /* sum w*T*bias */
    float sBB;     /* sum w*bias^2 (for the R^2 significance gate) */
    uint8_t valid; /* slope trustworthy */
} gyro_tc_state_t;

/**
 * @brief Load the newest valid model record from flash into @p st.
 * Scans the append log for the last uncorrupted record. If the log is full or
 * corrupt it erases+compacts the sector ONCE (safe at boot, pre-motion). Sets up
 * the internal write cursor for subsequent appends. Call once at setup.
 * @return 1 if a valid model was loaded (st populated), 0 otherwise (st unchanged).
 */
int gyro_tc_persist_load(gyro_tc_state_t* st);

/**
 * @brief Append @p st as a new record into the next free log slot (program only,
 * no erase). Silently skips if the log is full — never erases during runtime.
 * Call rate-limited (e.g. at most once per minute, only on a material change).
 */
void gyro_tc_persist_save(const gyro_tc_state_t* st);

#endif /* GYRO_TC_PERSIST_H */
