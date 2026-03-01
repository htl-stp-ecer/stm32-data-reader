/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "main.h"

#include "adcPorts-batteryVoltage.h"
#include "bemf.h"
#include "MPU9250.h"
#include "Communication/communication_with_pi.h"

#include "Actors/servo.h"
#include "Actors/motor.h"
#include "Hardware/timer.h"

#include "Sensors/adcInit.h"
#include "Hardware/dma.h"
#include "Communication/spi.h"
#include "Hardware/timerInit.h"
#include "Hardware/gpio.h"
#include "Sensors/IMU/imu.h"
#include "Utillity/utillity.h"

#define SERVO_UPDATE_INTERVAL 100 //ms (only update the servos with 10Hz to avoid servo jitter)

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();

    //initialising systems
    systemTimerStart();
    //starting bemf meashurment cycle - startet with starting the systemTimer -> is called periodicly
    //starting analog port meashurment cycle - startet with starting the systemTimer -> is called periodicly

    initPiCommunication();
    initMotors();
    setupImu();

    uint32_t last_update = HAL_GetTick();

    /* Infinite loop */
    while (1)
    {
        const uint32_t current_time = HAL_GetTick();
        if (current_time - last_update >= SERVO_UPDATE_INTERVAL)
        {
            update_servo_cmd();
            last_update = current_time;
        }

        if (updateFlags & PI_BUFFER_UPDATE_MOTOR_PID_SPEED)
        {
            updateFlags &= ~PI_BUFFER_UPDATE_MOTOR_PID_SPEED;
            update_motor_pidSettings();
        }

        if (updateFlags & PI_BUFFER_UPDATE_MOTOR_PID_POS)
        {
            updateFlags &= ~PI_BUFFER_UPDATE_MOTOR_PID_POS;
            update_motor_pidSettings();
        }

        if (updateFlags & PI_BUFFER_UPDATE_IMU_ORIENTATION)
        {
            updateFlags &= ~PI_BUFFER_UPDATE_IMU_ORIENTATION;
            updateImuOrientation(
                (const int8_t*)rxBuffer.imuGyroOrientation,
                (const int8_t*)rxBuffer.imuCompassOrientation);
        }

        readImu();

        //update spi buffer if possible
        updatingAnalogValuesInSpiBuffer();
        updatingMotorsInSpiBuffer();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
        HAL_Delay(500);
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */