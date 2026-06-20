//
// Created by matthias on 4/24/25.
//
#include "Actors/motor.h"

#include "main.h"
#include "Communication/communication_with_pi.h"
#include "Sensors/bemf.h"
#include "Sensors/odometry.h"
#include "Hardware/timer.h"
#include "Hardware/timerInit.h"
#include "Actors/pid.h"

#include <stdio.h>
#include <stdlib.h>

#define MTP_DONE_THRESHOLD 40   // position error deadband for "done" in BEMF units
#define MTP_MIN_VEL 15          // minimum crawl velocity to overcome static friction
#define MTP_ACCEL_PER_TICK 150  // profile velocity acceleration rate (faster ramp-up)
#define MTP_DECEL_FACTOR 10     // decel for sqrt curve: v = sqrt(2*factor*dist)

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
static PidController posPidControllers[MOTOR_COUNT]; // position (outer) loop — unused for MTP, kept for API
static int32_t profileVel[MOTOR_COUNT]; // trapezoidal profile current velocity
static int32_t prevGoalPos[MOTOR_COUNT]; // track goal changes for sticky done reset
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

// ---------------------------------------------------------------------------
// Motor control state machine
//
// update_motor() is invoked once per motor per BEMF cycle. It is structured as
// a per-channel state machine whose state is the SPI-commanded control mode
// (MOT_MODE_*). The function body is a fixed preamble followed by a dispatch to
// a per-mode handler:
//
//   1. shutdown guard      — SHUTDOWN_MOTOR forces motor off and returns early
//   2. read inputs         — ctlMode / target / goalPos from the Rx buffer
//   3. per-channel dt       — real elapsed time since this motor's last update
//   4. transition handler   — on a mode change, reset both PID loops, the
//                            profile velocity, the done flag, and log it
//   5. BEMF-disable guard   — MAV/CHASSIS need BEMF; hold off if disabled
//   6. dispatch             — call the handler for the current mode
//
// Each handler is a small static function that performs exactly the work the
// original monolithic switch did for that mode; behavior is unchanged.
// ---------------------------------------------------------------------------

// OFF: coast — both direction pins low, duty zero.
static void motor_mode_off(const uint8_t ch)
{
    motor_setDirection(ch, OFF);
    motor_setDutycycle(ch, 0);
}

// PASSIV_BRAKE: passive (short) brake — both direction pins high, duty zero.
static void motor_mode_brake(const uint8_t ch)
{
    motor_setDirection(ch, SHORT_BREAK);
    motor_setDutycycle(ch, 0);
}

// PWM: open-loop — drive the commanded duty/direction directly.
static void motor_mode_pwm(const uint8_t ch, const int32_t target)
{
    applyMotorOutput(ch, target);
}

// MAV (Move At Velocity): velocity PID closed on the BEMF reading.
//   goal = target velocity, current = BEMF reading.
//   BEMF sign is inverted w.r.t. motor direction, so negate measurement.
static void motor_mode_mav(const uint8_t ch, const int32_t target,
                           const int16_t bemf_filtered, const float pidDt)
{
    int32_t pidOut = pid_update(&pidControllers[ch], target, bemf_filtered, pidDt);
    applyMotorOutput(ch, pidOut);
}

// CHASSIS: full chassis velocity loop on-MCU: derive this wheel's velocity
// setpoint from the body-frame command via forward kinematics, then run the
// same per-motor MAV PID. Closing the chassis loop here (next to BEMF + IMU)
// keeps it deterministic — no SPI round-trip in-loop.
//
// wz is the gyro-corrected setpoint from the chassis yaw-rate controller
// (odometry_chassis_corrected_wz(), updated once per odometry cycle), so the
// rotation axis is regulated on the IMU rather than fed forward open-loop.
// vx/vy remain feedforward.
static void motor_mode_chassis(const uint8_t ch, const int16_t bemf_filtered,
                               const float pidDt)
{
    const int32_t chassisTarget = odometry_chassis_wheel_target(
        ch,
        rxBuffer.chassisVelocity[0],
        rxBuffer.chassisVelocity[1],
        odometry_chassis_corrected_wz());
    int32_t pidOut = pid_update(&pidControllers[ch], chassisTarget, bemf_filtered, pidDt);
    applyMotorOutput(ch, pidOut);
}

// MTP (Move To Position): sqrt deceleration profile → velocity PID → PWM.
// Generates a velocity setpoint from the remaining position error
// (v = sqrt(2 * decel_factor * distance)), rate-limits acceleration, allows
// instant deceleration, and latches a sticky "done" + active brake once within
// MTP_DONE_THRESHOLD of the goal.
static void motor_mode_mtp(const uint8_t ch, const int32_t goalPos,
                           const int16_t bemf_filtered, const float pidDt)
{
    // Clear done if goal position changed (new MTP command)
    if (goalPos != prevGoalPos[ch])
    {
        prevGoalPos[ch] = goalPos;
        motor_data.done &= ~(1u << ch);
        profileVel[ch] = 0;
        pid_reset(&pidControllers[ch]);
    }

    // Sticky done: once reached, stay done until goal changes or mode changes
    if (motor_data.done & (1u << ch))
    {
        motor_setDirection(ch, SHORT_BREAK);
        motor_setDutycycle(ch, 0);
        return;
    }

    // Sqrt decel curve → velocity target → velocity PID → PWM
    int32_t currentPos = motor_data.position[ch];
    int32_t posError = goalPos - currentPos;
    int32_t absError = posError < 0 ? -posError : posError;
    int32_t speedLimit = rxBuffer.motorTarget[ch];
    if (speedLimit <= 0) speedLimit = 300;

    if (absError <= MTP_DONE_THRESHOLD)
    {
        // Done — active brake, set sticky flag
        profileVel[ch] = 0;
        motor_data.done |= (1u << ch);
        motor_setDirection(ch, SHORT_BREAK);
        motor_setDutycycle(ch, 0);
        return;
    }
    int32_t dir = (posError > 0) ? 1 : -1;

    // Deceleration curve: v = sqrt(2 * decel_factor * distance)
    // Conservative decel_factor means early deceleration start
    uint32_t val = 2u * MTP_DECEL_FACTOR * (uint32_t)absError;
    uint32_t vDecel = 0;
    if (val > 0)
    {
        uint32_t x = val;
        uint32_t r = 0;
        for (int b = 15; b >= 0; b--)
        {
            uint32_t test = r | (1u << b);
            if (test * test <= x)
                r = test;
        }
        vDecel = r;
    }

    // Apply minimum velocity (overcome friction)
    if (vDecel < MTP_MIN_VEL) vDecel = MTP_MIN_VEL;

    // Apply speed limit
    int32_t desiredVel = (int32_t)vDecel;
    if (desiredVel > speedLimit) desiredVel = speedLimit;
    desiredVel *= dir;

    // Rate-limit acceleration only; allow instant deceleration
    int32_t absDesired = desiredVel < 0 ? -desiredVel : desiredVel;
    int32_t absCurrent = profileVel[ch] < 0 ? -profileVel[ch] : profileVel[ch];
    if (absDesired >= absCurrent)
    {
        // Accelerating — rate limit
        int32_t velDiff = desiredVel - profileVel[ch];
        if (velDiff > MTP_ACCEL_PER_TICK) velDiff = MTP_ACCEL_PER_TICK;
        else if (velDiff < -MTP_ACCEL_PER_TICK) velDiff = -MTP_ACCEL_PER_TICK;
        profileVel[ch] += velDiff;
    }
    else
    {
        // Decelerating — follow curve directly and reset PID integral
        // to prevent windup from acceleration phase fighting the brake
        profileVel[ch] = desiredVel;
        pidControllers[ch].iErr = 0.0f;
    }

    int32_t pidOut = pid_update(&pidControllers[ch], profileVel[ch], bemf_filtered, pidDt);
    applyMotorOutput(ch, pidOut);
}

// On a control-mode transition, reset all per-channel control state so the new
// mode starts clean: both PID loops, the trapezoidal profile velocity, and the
// done flag. Logs the transition for debugging.
static void motor_on_mode_change(const uint8_t ch, const uint8_t prevMode,
                                 const uint8_t newMode)
{
    printf("[stp] mot%d mode %u->%u pos=%ld\r\n",
           ch, (unsigned)prevMode, (unsigned)newMode,
           (long)motor_data.position[ch]);
    pid_reset(&pidControllers[ch]);
    pid_reset(&posPidControllers[ch]);
    profileVel[ch] = 0;
    // Clear done flag on mode change
    motor_data.done &= ~(1u << ch);
}

void update_motor(const uint8_t channel, const int16_t bemf_filtered)
{
    // NEVER EXECUTE THIS FUNTION IN MAIN LOOP!!!!!!!
    if (channel >= MOTOR_COUNT)
        return;

    // --- shutdown guard: motors off + return -------------------------------
    if ((rxBuffer.systemShutdown & SHUTDOWN_MOTOR)) //disable the motor if motors are shutdowned
    {
        motor_setDirection(channel, OFF);
        motor_setDutycycle(channel, 0);
        return;
    }

    // --- read inputs -------------------------------------------------------
    const uint8_t ctlMode = (rxBuffer.motorControlMode >> (3 * channel)) & 0x07;
    const int32_t target = rxBuffer.motorTarget[channel];
    const int32_t goalPos = rxBuffer.motorGoalPosition[channel];
    // Track previous control mode per motor to detect mode changes
    static uint8_t prevControlMode[MOTOR_COUNT] = {OFF};

    // --- per-channel real dt -----------------------------------------------
    // Real elapsed time since this motor's previous PID update, for the
    // dt-explicit velocity/position PID. update_motor() runs once per motor per
    // BEMF cycle, so this captures the actual (jittery) control period rather
    // than assuming a fixed rate. Unsigned subtraction wraps correctly.
    static uint32_t lastPidUs[MOTOR_COUNT] = {0};
    const uint32_t nowUs = microSeconds;
    const float pidDt = (float)(nowUs - lastPidUs[channel]) * 1e-6f;
    lastPidUs[channel] = nowUs;

    // --- transition handler: reset state on mode change --------------------
    if (ctlMode != prevControlMode[channel])
    {
        motor_on_mode_change(channel, prevControlMode[channel], ctlMode);
        prevControlMode[channel] = ctlMode;
    }

    // --- BEMF-disable guard ------------------------------------------------
    if ((ctlMode == MOT_MODE_MAV || ctlMode == MOT_MODE_CHASSIS)
        && (rxBuffer.featureFlags & FEATURE_BEMF_DISABLE))
    {
        // MAV/CHASSIS require BEMF feedback — ignore command, hold motor off.
        // The Pi-side reader is the actual guard; this is defense in depth.
        motor_setDirection(channel, OFF);
        motor_setDutycycle(channel, 0);
        return;
    }

    // --- dispatch to the current mode's handler ----------------------------
    switch (ctlMode)
    {
    case MOT_MODE_OFF:
        motor_mode_off(channel);
        break;
    case MOT_MODE_PASSIV_BRAKE:
        motor_mode_brake(channel);
        break;
    case MOT_MODE_PWM:
        motor_mode_pwm(channel, target);
        break;
    case MOT_MODE_MAV:
        motor_mode_mav(channel, target, bemf_filtered, pidDt);
        break;
    case MOT_MODE_CHASSIS:
        motor_mode_chassis(channel, bemf_filtered, pidDt);
        break;
    case MOT_MODE_MTP:
        motor_mode_mtp(channel, goalPos, bemf_filtered, pidDt);
        break;
    default:
        break;
    }
}