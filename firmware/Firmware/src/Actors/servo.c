//
// Created by matthias on 4/28/25.
//

#include "../../include/Actors/servo.h"

#include <stdio.h>
#include <stdint.h>
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

typedef struct
{
    uint8_t mode;
    uint16_t position;
    uint8_t pwmEnabled;
} ServoPortRuntimeState;

typedef struct
{
    uint8_t railEnabled;
    uint8_t shutdownActive;
    ServoPortRuntimeState port[NUM_SERVOS];
} ServoRuntimeState;

static ServoRuntimeState servoState = {0};

static uint8_t servo_valid_port(const int servoPort)
{
    return servoPort >= 0 && servoPort < NUM_SERVOS;
}

static void servo_set_rail(const uint8_t enabled)
{
    if (servoState.railEnabled == enabled)
        return;

    HAL_GPIO_WritePin(SERVO_6V0_ENABLE_GPIO_Port,
                      SERVO_6V0_ENABLE_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    servoState.railEnabled = enabled;
    delayus(10);
    printf("[servo] rail %s\r\n", enabled ? "on" : "off");
}

static void servo_set_compare(const int servoPort, const uint16_t position)
{
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(servos[servoPort].timer);
    const uint32_t compare = position > period ? period : position;
    if (compare != position)
        printf("[servo] port %d compare clamped %u->%lu\r\n", servoPort, position, compare);

    __HAL_TIM_SET_COMPARE(servos[servoPort].timer, servos[servoPort].chanal, compare);
    servoState.port[servoPort].position = position;
}

static void servo_set_pwm(const int servoPort, const uint8_t enabled)
{
    if (!servo_valid_port(servoPort) || servoState.port[servoPort].pwmEnabled == enabled)
        return;

    if (enabled)
        HAL_TIM_PWM_Start(servos[servoPort].timer, servos[servoPort].chanal);
    else
        HAL_TIM_PWM_Stop(servos[servoPort].timer, servos[servoPort].chanal);

    servoState.port[servoPort].pwmEnabled = enabled;
    printf("[servo] port %d pwm %s\r\n", servoPort, enabled ? "start" : "stop");
}

static void servo_apply_port(const int servoPort, const uint8_t mode, const uint16_t position)
{
    ServoPortRuntimeState* state = &servoState.port[servoPort];

    if (state->mode != mode)
        printf("[servo] port %d mode %u->%u\r\n", servoPort, state->mode, mode);

    switch (mode)
    {
    case SERVO_ENABLED:
        if (state->position != position)
        {
            printf("[servo] port %d pos %u->%u\r\n", servoPort, state->position, position);
            servo_set_compare(servoPort, position);
        }
        servo_set_pwm(servoPort, 1);
        break;

    case SERVO_DISABLED:
    case SERVO_FULLY_DISABLED:
    default:
        servo_set_pwm(servoPort, 0);
        break;
    }

    state->mode = mode;
}

void servo_enable(const int servoPort)
{
    servo_set_pwm(servoPort, 1);
}

void servo_disable(const int servoPort)
{
    servo_set_pwm(servoPort, 0);
}

void servo_fullyEnable()
{
    servo_set_rail(1);
    for (int i = 0; i < NUM_SERVOS; i++)
        servo_enable(i);
}

void servo_fullyDisable()
{
    for (int i = 0; i < NUM_SERVOS; i++)
        servo_disable(i);
    servo_set_rail(0);
}

void servo_setPositon(const int servoPort, const uint16_t position)
{
    if (!servo_valid_port(servoPort))
        return;

    servo_set_compare(servoPort, position);
}

void update_servo_cmd()
{
    if (rxBuffer.systemShutdown & SHUTDOWN_SERVO) //disables the servos if servos are shutdown
    {
        if (!servoState.shutdownActive)
            printf("[servo] shutdown active, disabling all PWM channels\r\n");
        servo_fullyDisable();
        for (int servoPort = 0; servoPort < NUM_SERVOS; servoPort++)
            servoState.port[servoPort].mode = SERVO_FULLY_DISABLED;
        servoState.shutdownActive = 1;
        return;
    }

    if (servoState.shutdownActive)
    {
        printf("[servo] shutdown cleared, PWM channels will re-enable from modes=0x%02X\r\n", rxBuffer.servoMode);
        servoState.shutdownActive = 0;
    }

    uint8_t anyEnabled = 0;
    for (int servoPort = 0; servoPort < NUM_SERVOS; servoPort++)
    {
        const uint8_t mode = (rxBuffer.servoMode >> (2 * servoPort)) & 0x03;
        if (mode == SERVO_ENABLED)
            anyEnabled = 1;
    }

    servo_set_rail(anyEnabled);

    for (int servoPort = 0; servoPort < NUM_SERVOS; servoPort++)
    {
        const uint8_t mode = (rxBuffer.servoMode >> (2 * servoPort)) & 0x03;
        servo_apply_port(servoPort, mode, rxBuffer.servoPos[servoPort]);
    }
}
