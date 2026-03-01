/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "Hardware/gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, IMU_SPI_CS0_Pin | IMU_SPI_CS1_Pin | MOT1_D0_Pin | SERVO_6V0_ENABLE_Pin,
                      //6V for Servo should be disabled by default
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, USER_LED_Pin,
                      GPIO_PIN_SET); //LED logic reversed - turned of as default

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, MOT1_D1_Pin | MOT2_D0_Pin | MOT2_D1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, MOT3_D0_Pin | MOT3_D1_Pin | MOT0_D0_Pin | MOT0_D1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : IMU_SPI_CS0_Pin IMU_SPI_CS1_Pin USER_LED_Pin SERVO_6V0_ENABLE_Pin
                           MOT1_D0_Pin */
    GPIO_InitStruct.Pin = IMU_SPI_CS0_Pin | IMU_SPI_CS1_Pin | USER_LED_Pin | SERVO_6V0_ENABLE_Pin
        | MOT1_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    /*Configure GPIO pins : MOT1_D1_Pin MOT2_D0_Pin MOT2_D1_Pin */
    GPIO_InitStruct.Pin = MOT1_D1_Pin | MOT2_D0_Pin | MOT2_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    /*Configure GPIO pins : MOT3_D0_Pin MOT3_D1_Pin MOT0_D0_Pin MOT0_D1_Pin */
    GPIO_InitStruct.Pin = MOT3_D0_Pin | MOT3_D1_Pin | MOT0_D0_Pin | MOT0_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* --- Digital Inputs --- */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = DIGITAL_INPUT_PULL_STATE;

    /*Configure GPIO pins : DIN0_Pin DIN1_Pin DIN2_Pin DIN3_Pin */
    GPIO_InitStruct.Pin = DIN0_Pin | DIN1_Pin | DIN2_Pin | DIN3_Pin;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : Button_Pin DIN5_Pin DIN4_Pin */
    GPIO_InitStruct.Pin = Button_Pin | DIN5_Pin | DIN4_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : DIN6_Pin */
    GPIO_InitStruct.Pin = DIN6_Pin;
    HAL_GPIO_Init(DIN6_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DIN9_Pin DIN7_Pin DIN8_Pin */
    GPIO_InitStruct.Pin = DIN7_Pin | DIN8_Pin | DIN9_Pin;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}