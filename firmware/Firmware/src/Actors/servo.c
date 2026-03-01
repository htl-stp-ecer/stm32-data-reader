//
// Created by matthias on 4/28/25.
//

#include "../../include/Actors/servo.h"

#include "main.h"
#include "Utillity/utillity.h"
#include "Communication/communication_with_pi.h"
#include  "Hardware/timerInit.h"

const volatile Servo servos[NUM_SERVOS] =
{
    {
        //PWM_Pin =
        S0_PWM_Pin,
        //PWM_GPIO_PORT =
        S0_PWM_GPIO_Port,

        //timer =
        &htim3,
        //chanal =
        TIM_CHANNEL_3
    },
    {
        //PWM_Pin =
        S1_PWM_Pin,
        //PWM_GPIO_PORT =
        S1_PWM_GPIO_Port,

        //timer =
        &htim3,
        //chanal =
        TIM_CHANNEL_2
    },
    {
        //PWM_Pin =
        S2_PWM_Pin,
        //PWM_GPIO_PORT =
        S2_PWM_GPIO_Port,

        //timer =
        &htim9,
        //chanal =
        TIM_CHANNEL_2
    },
    {
        //PWM_Pin =
        S3_PWM_Pin,
        //PWM_GPIO_PORT =
        S3_PWM_GPIO_Port,

        //timer =
        &htim9,
        //chanal =
        TIM_CHANNEL_1
    }
};


void servo_enable(const int servoPort)
{
    //if (HAL_TIM_Base_GetState(servos[servoPort].timer)) //it should be checked if TIMER CHannal is acitve
    HAL_TIM_PWM_Start(servos[servoPort].timer, servos[servoPort].chanal);
}

void servo_disable(const int servoPort)
{
    //HAL_TIM_PWM_Stop(servos[servoPort].timer, servos[servoPort].chanal);
}

void servo_fullyEnable()
{
    if (HAL_GPIO_ReadPin(SERVO_6V0_ENABLE_GPIO_Port, SERVO_6V0_ENABLE_Pin) == GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(SERVO_6V0_ENABLE_GPIO_Port, SERVO_6V0_ENABLE_Pin, GPIO_PIN_SET);
        delayus(10); //short delay to give the 6V supply time to start (may not be needed)
        for (int i = 0; i < NUM_SERVOS; i++)
            servo_enable(i);
    }
}

void servo_fullyDisable()
{
    for (int i = 0; i < NUM_SERVOS; i++)
        servo_disable(i);

    delayus(10); //short delay to give the 6V supply time to start (may not be needed)
    HAL_GPIO_WritePin(SERVO_6V0_ENABLE_GPIO_Port, SERVO_6V0_ENABLE_Pin, GPIO_PIN_RESET);
}

void servo_setPositon(const int servoPort, const uint16_t position)
{
    __HAL_TIM_SET_COMPARE(servos[servoPort].timer, servos[servoPort].chanal, (uint32_t)position);
}

void update_servo_cmd()
{
    static uint8_t lastServoModes = 0;

    if (rxBuffer.systemShutdown & SHUTDOWN_SERVO) //disables the servos if servos are shutdown
    {
        servo_fullyDisable();
        return;
    }
    else
        servo_fullyEnable();

    //if Servos are fully disabled
    if (rxBuffer.servoMode == ALL_SERVOS_FULLY_DISABLED)
    {
        servo_fullyDisable();
        lastServoModes = rxBuffer.servoMode;
        return;
    }

    if (lastServoModes == ALL_SERVOS_FULLY_DISABLED && rxBuffer.servoMode != ALL_SERVOS_FULLY_DISABLED)
        servo_fullyEnable();

    for (int servoPort = 0; servoPort < NUM_SERVOS; servoPort++)
    {
        const uint8_t servoMode = (rxBuffer.servoMode >> (2 * servoPort)) & 0x03;

        switch (servoMode)
        {
        case SERVO_FULLY_DISABLED:
        case SERVO_DISABLED:
            servo_disable(servoPort);
            break;

        case SERVO_ENABLED:
            static uint16_t lastServoCmd[NUM_SERVOS] = {0};
            const uint16_t servoPos = rxBuffer.servoPos[servoPort];
            const uint8_t lastMode = (lastServoModes >> (2 * servoPort)) & 0x03;

            if (lastServoCmd[servoPort] != servoPos)
                servo_setPositon(servoPort, servoPos);

            if (lastMode != servoMode)
                servo_enable(servoPort);

            lastServoCmd[servoPort] = servoPos;

            break;

        default:
            break;
        }
    }
    lastServoModes = rxBuffer.servoMode;
}