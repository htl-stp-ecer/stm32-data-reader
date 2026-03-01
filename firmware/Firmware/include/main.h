/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
/*----------<user LED>----------*/
#define USER_LED_Pin GPIO_PIN_9
#define USER_LED_GPIO_Port GPIOE

/*----------<SPI>----------*/
//SPI for IMU
#define IMU_SPI_CS0_Pin GPIO_PIN_2
#define IMU_SPI_CS0_GPIO_Port GPIOE
#define IMU_SPI_CS1_Pin GPIO_PIN_3
#define IMU_SPI_CS1_GPIO_Port GPIOE
#define IMU_SPI_SCK_Pin GPIO_PIN_10
#define IMU_SPI_SCK_GPIO_Port GPIOC
#define IMU_SPI_MISO_Pin GPIO_PIN_11
#define IMU_SPI_MISO_GPIO_Port GPIOC
#define IMU_SPI_MOSI_Pin GPIO_PIN_12
#define IMU_SPI_MOSI_GPIO_Port GPIOC

//SPI communication with the Raspberry Pi
#define PI_SPI_CS_Pin GPIO_PIN_12
#define PI_SPI_CS_GPIO_Port GPIOB
#define PI_SPI_SCK_Pin GPIO_PIN_13
#define PI_SPI_SCK_GPIO_Port GPIOB
#define PI_SPI_MISO_Pin GPIO_PIN_14
#define PI_SPI_MISO_GPIO_Port GPIOB
#define PI_SPI_MOSI_Pin GPIO_PIN_15
#define PI_SPI_MOSI_GPIO_Port GPIOB

/*----------<UART>----------*/
//UART communication with the Raspberry Pi
#define PI_UART_TX_Pin GPIO_PIN_10
#define PI_UART_TX_GPIO_Port GPIOB

#define PI_UART_RX_Pin GPIO_PIN_11
#define PI_UART_RX_GPIO_Port GPIOB

/*----------<Servo>----------*/
//6V0 Enable (Servo Voltage Source)
#define SERVO_6V0_ENABLE_Pin GPIO_PIN_10
#define SERVO_6V0_ENABLE_GPIO_Port GPIOE

//servo PWM pins
#define S0_PWM_Pin GPIO_PIN_8
#define S0_PWM_GPIO_Port GPIOC

#define S1_PWM_Pin GPIO_PIN_7
#define S1_PWM_GPIO_Port GPIOC

#define S2_PWM_Pin GPIO_PIN_6
#define S2_PWM_GPIO_Port GPIOE

#define S3_PWM_Pin GPIO_PIN_5
#define S3_PWM_GPIO_Port GPIOE

/*----------<motor>----------*/
//motor PWM pins
#define MOT0_PWM_Pin GPIO_PIN_8
#define MOT0_PWM_GPIO_Port GPIOA

#define MOT1_PWM_Pin GPIO_PIN_9
#define MOT1_PWM_GPIO_Port GPIOA

#define MOT2_PWM_Pin GPIO_PIN_10
#define MOT2_PWM_GPIO_Port GPIOA

#define MOT3_PWM_Pin GPIO_PIN_6
#define MOT3_PWM_GPIO_Port GPIOC

//motor direction pins
//motor 0 direction pins
#define MOT0_D0_Pin GPIO_PIN_1
#define MOT0_D0_GPIO_Port GPIOD
#define MOT0_D1_Pin GPIO_PIN_7
#define MOT0_D1_GPIO_Port GPIOD

//motor 1 direction pins
#define MOT1_D1_Pin GPIO_PIN_13
#define MOT1_D1_GPIO_Port GPIOC
#define MOT1_D0_Pin GPIO_PIN_15
#define MOT1_D0_GPIO_Port GPIOE

//motor 2 direction pins
#define MOT2_D0_Pin GPIO_PIN_14
#define MOT2_D0_GPIO_Port GPIOC
#define MOT2_D1_Pin GPIO_PIN_15
#define MOT2_D1_GPIO_Port GPIOC

//motor 3 direction pins
#define MOT3_D0_Pin GPIO_PIN_10
#define MOT3_D0_GPIO_Port GPIOD
#define MOT3_D1_Pin GPIO_PIN_11
#define MOT3_D1_GPIO_Port GPIOD


/*----------<digital inputs>----------*/
//build in button
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOB

//digital Ports
#define DIN0_Pin GPIO_PIN_12
#define DIN0_GPIO_Port GPIOD

#define DIN1_Pin GPIO_PIN_13
#define DIN1_GPIO_Port GPIOD

#define DIN2_Pin GPIO_PIN_14
#define DIN2_GPIO_Port GPIOD

#define DIN3_Pin GPIO_PIN_15
#define DIN3_GPIO_Port GPIOD

#define DIN4_Pin GPIO_PIN_9
#define DIN4_GPIO_Port GPIOB

#define DIN5_Pin GPIO_PIN_8
#define DIN5_GPIO_Port GPIOB

#define DIN6_Pin GPIO_PIN_9
#define DIN6_GPIO_Port GPIOC

#define DIN7_Pin GPIO_PIN_0
#define DIN7_GPIO_Port GPIOE

#define DIN8_Pin GPIO_PIN_1
#define DIN8_GPIO_Port GPIOE

#define DIN9_Pin GPIO_PIN_4
#define DIN9_GPIO_Port GPIOE

/*----------<analog inputs>----------*/
//battery voltage
#define VCC_BATT_SENSOR_Pin GPIO_PIN_0
#define VCC_BATT_SENSOR_GPIO_Port GPIOC

//analog ports
#define AIN0_Pin GPIO_PIN_1
#define AIN0_GPIO_Port GPIOB

#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOC

#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOC

#define AIN3_Pin GPIO_PIN_3
#define AIN3_GPIO_Port GPIOC

#define AIN4_Pin GPIO_PIN_4
#define AIN4_GPIO_Port GPIOC

#define AIN5_Pin GPIO_PIN_5
#define AIN5_GPIO_Port GPIOC

//motor BEMF pin
//motor 0 BEMF pins
#define MOT0_BEMFH_Pin GPIO_PIN_0
#define MOT0_BEMFH_GPIO_Port GPIOA
#define MOT0_BEMFL_Pin GPIO_PIN_1
#define MOT0_BEMFL_GPIO_Port GPIOA

//motor 1 BEMF pins
#define MOT1_BEMFH_Pin GPIO_PIN_2
#define MOT1_BEMFH_GPIO_Port GPIOA
#define MOT1_BEMFL_Pin GPIO_PIN_3
#define MOT1_BEMFL_GPIO_Port GPIOA

//motor 2 BEMF pins
#define MOT2_BEMFH_Pin GPIO_PIN_4
#define MOT2_BEMFH_GPIO_Port GPIOA
#define MOT2_BEMFL_Pin GPIO_PIN_5
#define MOT2_BEMFL_GPIO_Port GPIOA

//motor 3 BEMF pins
#define MOT3_BEMFH_Pin GPIO_PIN_6
#define MOT3_BEMFH_GPIO_Port GPIOA
#define MOT3_BEMFL_Pin GPIO_PIN_7
#define MOT3_BEMFL_GPIO_Port GPIOA


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */