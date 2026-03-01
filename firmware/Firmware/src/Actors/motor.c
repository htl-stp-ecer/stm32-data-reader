//
// Created by matthias on 4/24/25.
//
#include "Actors/motor.h"

#include "main.h"
#include "Communication/communication_with_pi.h"
#include "Sensors/bemf.h"
#include "Hardware/timer.h"
#include "Hardware/timerInit.h"
#include "Actors/pid.h"

#include <stdlib.h>

#define MTP_DONE_THRESHOLD 50  // position error deadband for "done" in BEMF units

const volatile Motor motors[MOTOR_COUNT] = {
    //motor 0 and 1 as well as 2 and 3 are switched
    //to get right sequenc of motor ports
    //Motor 1
    {
        //PWM_Pin =
        MOT1_PWM_Pin,
        //PWM_GPIO PORT =
        MOT1_PWM_GPIO_Port,

        //timer =
        &htim1,
        //chanal =
        TIM_CHANNEL_2,

        //D0_Pin=
        MOT1_D0_Pin,
        //D0_GPIO_Port=
        MOT1_D0_GPIO_Port,
        //D1_Pin=
        MOT1_D1_Pin,
        //D1_GPIO_Port=
        MOT1_D1_GPIO_Port

    },
    //Motor 0
    {
        //PWM_Pin =
        MOT0_PWM_Pin,
        //PWM_GPIO PORT =
        MOT0_PWM_GPIO_Port,

        //timer =
        &htim1,
        //chanal =
        TIM_CHANNEL_1,

        //D0_Pin=
        MOT0_D0_Pin,
        //D0_GPIO_Port=
        MOT0_D0_GPIO_Port,
        //D1_Pin=
        MOT0_D1_Pin,
        //D1_GPIO_Port=
        MOT0_D1_GPIO_Port
    },
    //Motor 3
    {
        //PWM_Pin =
        MOT3_PWM_Pin,
        //PWM_GPIO PORT =
        MOT3_PWM_GPIO_Port,

        //timer =
        &htim8,
        //chanal =
        TIM_CHANNEL_1,

        //D0_Pin=
        MOT3_D0_Pin,
        //D0_GPIO_Port=
        MOT3_D0_GPIO_Port,
        //D1_Pin=
        MOT3_D1_Pin,
        //D1_GPIO_Port=
        MOT3_D1_GPIO_Port
    },
    //Motor 2
    {
        //PWM_Pin =
        MOT2_PWM_Pin,
        //PWM_GPIO PORT =
        MOT2_PWM_GPIO_Port,

        //timer =
        &htim1,
        //chanal =
        TIM_CHANNEL_3,

        //D0_Pin=
        MOT2_D0_Pin,
        //D0_GPIO_Port=
        MOT2_D0_GPIO_Port,
        //D1_Pin=
        MOT2_D1_Pin,
        //D1_GPIO_Port=
        MOT2_D1_GPIO_Port
    }
};

static PidController pidControllers[MOTOR_COUNT]; // velocity (inner) loop
static PidController posPidControllers[MOTOR_COUNT]; // position (outer) loop
volatile MotorData motor_data = {0};

void initMotors()
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        pid_init(&pidControllers[i]);
        pid_init_position(&posPidControllers[i]);
    }
}

static void applyMotorOutput(uint8_t ch, int32_t motor_cmd)
{
    if (motor_cmd >= 0)
    {
        motor_setDirection(ch, CW);
        motor_setDutycycle(ch, (uint32_t)motor_cmd);
    }
    else
    {
        motor_setDirection(ch, CCW);
        motor_setDutycycle(ch, (uint32_t)(-motor_cmd));
    }
}

void motor_setDutycycle(int portNumber, uint32_t dutycycle)
{
    if (dutycycle > MOTOR_MAX_DUTYCYCLE)
        dutycycle = MOTOR_MAX_DUTYCYCLE;

    //if PWM Chanal of timer is inactive -> activate timer chanal
    if (!(HAL_TIM_GetActiveChannel(motors[portNumber].timer)
        & convertChanalToActiveChanal(motors[portNumber].chanal)))
        HAL_TIM_PWM_Start(motors[portNumber].timer, motors[portNumber].chanal);

    //change the duty cycle of the chanal
    __HAL_TIM_SET_COMPARE(motors[portNumber].timer, motors[portNumber].chanal, dutycycle);
}

void motor_setDirection(int portNumber, enum MOTOR_DIRECTION_CTL direction)
{
    switch (direction)
    {
    case OFF:
        HAL_GPIO_WritePin(motors[portNumber].D0_GPIO_Port, motors[portNumber].D0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[portNumber].D1_GPIO_Port, motors[portNumber].D1_Pin, GPIO_PIN_RESET);
        break;

    case CCW:
        HAL_GPIO_WritePin(motors[portNumber].D0_GPIO_Port, motors[portNumber].D0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motors[portNumber].D1_GPIO_Port, motors[portNumber].D1_Pin, GPIO_PIN_RESET);
        break;

    case CW:
        HAL_GPIO_WritePin(motors[portNumber].D0_GPIO_Port, motors[portNumber].D0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[portNumber].D1_GPIO_Port, motors[portNumber].D1_Pin, GPIO_PIN_SET);
        break;

    case SHORT_BREAK:
        HAL_GPIO_WritePin(motors[portNumber].D0_GPIO_Port, motors[portNumber].D0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motors[portNumber].D1_GPIO_Port, motors[portNumber].D1_Pin, GPIO_PIN_SET);
        break;

    default:
        break;
    }
}

void motor_stop(int portNumber)
{
    motor_setDirection(portNumber, SHORT_BREAK);
}

void motors_forceOff(void)
{
    for (int ch = 0; ch < MOTOR_COUNT; ch++)
    {
        motor_setDirection(ch, OFF);
        motor_setDutycycle(ch, 0);
    }
}


void update_motor_pidSettings()
{
    for (int port = 0; port < MOTOR_COUNT; port++)
    {
        pidControllers[port].kP = rxBuffer.motorPidSettings.pids[port].Kp;
        pidControllers[port].kI = rxBuffer.motorPidSettings.pids[port].Ki;
        pidControllers[port].kD = rxBuffer.motorPidSettings.pids[port].Kd;
    }
    // Apply global clamp settings if provided (non-zero), otherwise keep defaults
    if (rxBuffer.motorPidSettings.limMaxInt > 0.0f)
    {
        for (int port = 0; port < MOTOR_COUNT; port++)
            pidControllers[port].iMax = rxBuffer.motorPidSettings.limMaxInt;
    }
    if (rxBuffer.motorPidSettings.limMax > 0.0f)
    {
        for (int port = 0; port < MOTOR_COUNT; port++)
            pidControllers[port].outMax = rxBuffer.motorPidSettings.limMax;
    }
}


void update_motor_posPidSettings()
{
    for (int port = 0; port < MOTOR_COUNT; port++)
    {
        posPidControllers[port].kP = rxBuffer.motorPidSettings.pids[port].Kp;
        posPidControllers[port].kI = rxBuffer.motorPidSettings.pids[port].Ki;
        posPidControllers[port].kD = rxBuffer.motorPidSettings.pids[port].Kd;
    }
    // Apply global clamp settings if provided (non-zero), otherwise keep defaults
    if (rxBuffer.motorPidSettings.limMaxInt > 0.0f)
    {
        for (int port = 0; port < MOTOR_COUNT; port++)
            posPidControllers[port].iMax = rxBuffer.motorPidSettings.limMaxInt;
    }
    if (rxBuffer.motorPidSettings.limMax > 0.0f)
    {
        for (int port = 0; port < MOTOR_COUNT; port++)
            posPidControllers[port].outMax = rxBuffer.motorPidSettings.limMax;
    }
}

void update_motor(const uint8_t channel, const int16_t bemf_filtered)
{
    // NEVER EXECUTE THIS FUNTION IN MAIN LOOP!!!!!!!
    if (channel >= MOTOR_COUNT)
        return;

    if ((rxBuffer.systemShutdown & SHUTDOWN_MOTOR)) //disable the motor if motors are shutdowned
    {
        motor_setDirection(channel, OFF);
        motor_setDutycycle(channel, 0);
        return;
    }

    const uint8_t ctlMode = (rxBuffer.motorControlMode >> (3 * channel)) & 0x07;
    const int32_t target = rxBuffer.motorTarget[channel];
    const int32_t goalPos = rxBuffer.motorGoalPosition[channel];
    // Track previous control mode per motor to detect mode changes
    static uint8_t prevControlMode[MOTOR_COUNT] = {OFF};

    // Detect mode change and reset PID + MRP state
    if (ctlMode != prevControlMode[channel])
    {
        pid_reset(&pidControllers[channel]);
        pid_reset(&posPidControllers[channel]);
        prevControlMode[channel] = ctlMode;
        // Clear done flag on mode change
        motor_data.done &= ~(1u << channel);
    }

    switch (ctlMode)
    {
    case MOT_MODE_OFF:
        {
            motor_setDirection(channel, OFF);
            motor_setDutycycle(channel, 0);
        }
        break;
    case MOT_MODE_PASSIV_BRAKE:
        {
            motor_setDirection(channel, SHORT_BREAK);
            motor_setDutycycle(channel, 0);
        }
        break;

    case MOT_MODE_PWM:
        applyMotorOutput(channel, target);
        break;

    case MOT_MODE_MAV:
        {
            // Velocity PID: goal = target velocity, current = BEMF reading
            // BEMF sign is inverted w.r.t. motor direction, so negate measurement
            int32_t pidOut = pid_update(&pidControllers[channel], target, bemf_filtered);
            applyMotorOutput(channel, pidOut);
            break;
        }

    case MOT_MODE_MTP:
        {
            // Cascaded PID: position loop → velocity target → velocity loop → PWM
            int32_t currentPos = motor_data.position[channel];
            int32_t posError = goalPos - currentPos;
            int32_t absError = posError < 0 ? -posError : posError;

            int32_t velTarget = pid_update(&posPidControllers[channel], goalPos, currentPos);

            int32_t speedLimit = rxBuffer.motorTarget[channel];
            if (speedLimit > 0)
            {
                if (velTarget > speedLimit) velTarget = speedLimit;
                else if (velTarget < -speedLimit) velTarget = -speedLimit;
            }

            int32_t pidOut = pid_update(&pidControllers[channel], velTarget, bemf_filtered);
            applyMotorOutput(channel, pidOut);


            if (absError <= MTP_DONE_THRESHOLD)
            {
                motor_data.done |= (1u << channel);
            }
            else
            {
                motor_data.done &= ~(1u << channel);
            }
            break;
        }

    default:
        break;
    }
}