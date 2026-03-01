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
#include "../main.h"

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;


void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

void changeSPIBaudRatePrescaler(SPI_HandleTypeDef* spiX, const uint32_t baudRatePresacler);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */