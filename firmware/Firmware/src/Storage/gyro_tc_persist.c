// ============================================================================
// Reboot-persistent gyro bias-vs-temperature model — append-log flash storage.
//
// Layout: sector 13 (Bank 2, 16 KiB) as a log of 256 fixed 64-byte slots. Each
// save PROGRAMS one record into the next erased slot (0xFF -> data); we never
// erase during runtime, so the main loop never stalls the way the old MPL-cal
// persistence did. The sector is erased only at boot, and only when the log is
// full or corrupt (before any motion). On boot we scan for the newest valid
// record and load it; the write cursor points at the first free slot after it.
//
// Record (44 of the 64 bytes used, word-aligned), as 11 little-endian u32 words:
//   [0] magic  [1] version  [2] slope  [3] n  [4] sT  [5] sB  [6] sTT  [7] sTB
//   [8] sBB  [9] valid  [10] checksum(words 0..9)
// The checksum rejects partial writes (e.g. power loss mid-program).
// ============================================================================

#include "Storage/gyro_tc_persist.h"

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define GTC_WORDS      11U            /* u32 words actually used per record */
#define GTC_ERASED32   0xFFFFFFFFU

static uint32_t gtc_write_idx = 0;    /* next free slot index */
static uint8_t  gtc_full = 0;         /* log full — stop appending this session */
static uint8_t  gtc_inited = 0;

static inline volatile uint32_t* slot_ptr(uint32_t idx)
{
    return (volatile uint32_t*)(GTC_FLASH_ADDR + idx * GTC_SLOT_SIZE);
}

static uint32_t gtc_checksum(const uint32_t* w)
{
    // Simple non-zero mixing sum over words 0..8. Avoids 0/0xFFFFFFFF so an
    // all-erased or all-zero slot never passes as a valid record.
    uint32_t c = 0x9E3779B9U;
    for (uint32_t i = 0; i < GTC_WORDS - 1U; i++)
        c = (c * 33U) ^ w[i];
    return c;
}

// Pack a state struct into the 10-word record image.
static void pack(const gyro_tc_state_t* st, uint32_t* w)
{
    w[0] = GTC_MAGIC;
    w[1] = GTC_VERSION;
    memcpy(&w[2], &st->slope, 4);
    memcpy(&w[3], &st->n,     4);
    memcpy(&w[4], &st->sT,    4);
    memcpy(&w[5], &st->sB,    4);
    memcpy(&w[6], &st->sTT,   4);
    memcpy(&w[7], &st->sTB,   4);
    memcpy(&w[8], &st->sBB,   4);
    w[9]  = st->valid ? 1U : 0U;
    w[10] = gtc_checksum(w);
}

// Validate a slot in flash and, if good, unpack it into @p st.
static int read_slot(uint32_t idx, gyro_tc_state_t* st)
{
    volatile uint32_t* p = slot_ptr(idx);
    uint32_t w[GTC_WORDS];
    for (uint32_t i = 0; i < GTC_WORDS; i++)
        w[i] = p[i];

    if (w[0] != GTC_MAGIC || w[1] != GTC_VERSION)
        return 0;
    if (w[10] != gtc_checksum(w))
        return 0;

    if (st)
    {
        memcpy(&st->slope, &w[2], 4);
        memcpy(&st->n,     &w[3], 4);
        memcpy(&st->sT,    &w[4], 4);
        memcpy(&st->sB,    &w[5], 4);
        memcpy(&st->sTT,   &w[6], 4);
        memcpy(&st->sTB,   &w[7], 4);
        memcpy(&st->sBB,   &w[8], 4);
        st->valid = (uint8_t)w[9];
    }
    return 1;
}

static int slot_is_erased(uint32_t idx)
{
    volatile uint32_t* p = slot_ptr(idx);
    for (uint32_t i = 0; i < GTC_WORDS; i++)
        if (p[i] != GTC_ERASED32)
            return 0;
    return 1;
}

static HAL_StatusTypeDef gtc_erase_sector(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0;
    erase.TypeErase   = FLASH_TYPEERASE_SECTORS;
    erase.Banks       = FLASH_BANK_2;
    erase.Sector      = GTC_FLASH_SECTOR;
    erase.NbSectors   = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7-3.6V: word programming */
    return HAL_FLASHEx_Erase(&erase, &sector_error);
}

// Program a record image into slot @p idx (assumes the slot is erased).
static HAL_StatusTypeDef gtc_program_slot(uint32_t idx, const uint32_t* w)
{
    uint32_t addr = GTC_FLASH_ADDR + idx * GTC_SLOT_SIZE;
    for (uint32_t i = 0; i < GTC_WORDS; i++)
    {
        HAL_StatusTypeDef s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                                addr + i * 4U, w[i]);
        if (s != HAL_OK)
            return s;
    }
    return HAL_OK;
}

int gyro_tc_persist_load(gyro_tc_state_t* st)
{
    gtc_inited = 1;
    gtc_full = 0;

    // Find the newest valid record: each valid slot overwrites @p st, so after
    // the scan st holds the last one. Its index is the append frontier.
    int have = 0;
    uint32_t last_valid = 0;
    for (uint32_t i = 0; i < GTC_SLOT_COUNT; i++)
    {
        if (read_slot(i, st))
        {
            have = 1;
            last_valid = i;
        }
    }

    gyro_tc_state_t loaded;
    int loaded_ok = 0;
    if (have)
    {
        loaded = *st;                 // st currently holds the last valid record
        loaded_ok = 1;
        gtc_write_idx = last_valid + 1;
    }
    else
    {
        gtc_write_idx = 0;
    }

    // Can we append cleanly? The target slot must exist and be erased.
    int need_compact = 0;
    if (gtc_write_idx >= GTC_SLOT_COUNT)
        need_compact = 1;             // log full
    else if (!slot_is_erased(gtc_write_idx))
        need_compact = 1;             // stale/corrupt frontier

    if (need_compact)
    {
        // Safe here: setup runs before any motion. Erase once, then re-seed with
        // the loaded state (if any) so the model survives the compaction.
        printf("[gtc] log needs compaction — erasing sector %u\r\n",
               (unsigned)GTC_FLASH_SECTOR);
        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            gtc_full = 1;             // give up persistence, keep running
            return loaded_ok;
        }
        HAL_StatusTypeDef es = gtc_erase_sector();
        if (es != HAL_OK)
        {
            HAL_FLASH_Lock();
            gtc_full = 1;
            printf("[gtc] erase failed (%d) — persistence disabled\r\n", (int)es);
            return loaded_ok;
        }
        gtc_write_idx = 0;
        if (loaded_ok)
        {
            uint32_t w[GTC_WORDS];
            pack(&loaded, w);
            if (gtc_program_slot(0, w) == HAL_OK)
                gtc_write_idx = 1;
        }
        HAL_FLASH_Lock();
    }

    if (loaded_ok)
        printf("[gtc] loaded model: slope=%ld mdps/degC valid=%u (slot %lu)\r\n",
               (long)(loaded.slope * 1000.0f), (unsigned)loaded.valid,
               (unsigned long)last_valid);
    else
        printf("[gtc] no saved model — learning fresh\r\n");

    return loaded_ok;
}

void gyro_tc_persist_save(const gyro_tc_state_t* st)
{
    if (!gtc_inited || gtc_full)
        return;
    if (gtc_write_idx >= GTC_SLOT_COUNT)
    {
        gtc_full = 1;                 // no runtime erase — wait for next boot
        return;
    }

    uint32_t w[GTC_WORDS];
    pack(st, w);

    if (HAL_FLASH_Unlock() != HAL_OK)
        return;
    HAL_StatusTypeDef s = gtc_program_slot(gtc_write_idx, w);
    HAL_FLASH_Lock();

    if (s == HAL_OK)
        gtc_write_idx++;
    else
        gtc_full = 1;                 // stop trying this session
}
