/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adcInit.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../main.h"

extern ADC_HandleTypeDef hadc1; //analog sensor ADCs
extern ADC_HandleTypeDef hadc2; //motor BEMF ADCs

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */