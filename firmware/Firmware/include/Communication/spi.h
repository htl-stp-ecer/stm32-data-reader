/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "../main.h"

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/**
 * @brief Wait for SPI2 BSY flag to clear, with a 100 µs timeout.
 * @return true if SPI2 is idle, false if timed out (SPI2 stuck).
 *
 * Uses the DWT cycle counter (CYCCNT) which runs independently of
 * interrupts, so this is safe to call from any context including ISRs.
 * At 180 MHz, 100 µs = 18 000 cycles.
 */
#define SPI2_WAIT_TIMEOUT_CYCLES 18000u // 100 µs at 180 MHz

static inline bool spi2_wait_idle(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    while (SPI2->SR & SPI_SR_BSY)
    {
        if (DWT->CYCCNT >= SPI2_WAIT_TIMEOUT_CYCLES)
            return false;
    }
    return true;
}

void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

void changeSPIBaudRatePrescaler(SPI_HandleTypeDef* spiX, const uint32_t baudRatePresacler);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */