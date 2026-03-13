//
// Created by matthias on 5/4/25.
//
#include "main.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/bemf.h"
#include "Hardware/timer.h"
#include "communication_with_pi.h"
#include "Hardware/timerInit.h"

volatile uint32_t microSeconds = 0;


void systemTimerStart(void)
{
    //starts the timer 6 = systemtimer
    microSeconds = 0;
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    //timer for time measurement - is called every
    if (htim->Instance == TIM6)
    {
        microSeconds++;

        if (!(rxBuffer.systemShutdown & SHUTDOWN_MOTOR))
        {
            bemf_watchdog_check(microSeconds);

            static uint32_t bemfLastStart = 0;
            doEveryXuSeconds(stop_motors_for_bemf_conv(), BEMF_SAMPLING_INTERVAL, bemfLastStart);

            if (bemfState == WAITING_TO_START)
            {
                doAfterXuSeconds(startBemfReading(), BEMF_CONVERSION_START_DELAY_TIME, bemfLastStart);
            }
        }

        static uint32_t analogOutputLastStart = 0;
        doEveryXuSeconds(updatingAnalogValuesInSpiBuffer(), ANALOG_OUTPUT_INTERVAL, analogOutputLastStart);
    }
}

HAL_TIM_ActiveChannel convertChanalToActiveChanal(uint16_t chanal)
{
    switch (chanal)
    {
    case TIM_CHANNEL_1:
        return HAL_TIM_ACTIVE_CHANNEL_1;
    case TIM_CHANNEL_2:
        return HAL_TIM_ACTIVE_CHANNEL_2;
    case TIM_CHANNEL_3:
        return HAL_TIM_ACTIVE_CHANNEL_3;
    case TIM_CHANNEL_4:
        return HAL_TIM_ACTIVE_CHANNEL_4;
    default:
        return HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
}