//
// Created by matthias on 4/24/25.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

#define MOTOR_COUNT 4

#define MOTOR_MAX_DUTYCYCLE (400-1)

#include "spi/pi_buffer.h"

enum MOTOR_DIRECTION_CTL
{
    OFF = 0b00,
    CCW = 0b01,
    CW = 0b10,
    SHORT_BREAK = 0b11,
};

typedef struct
{
    //PWM pin
    uint16_t PWM_Pin;

    GPIO_TypeDef* PWM_GPIO_Port;

    //PWM Chanal and Timer
    TIM_HandleTypeDef* timer;
    uint32_t chanal;

    //direction pins
    uint16_t D0_Pin;
    GPIO_TypeDef* D0_GPIO_Port;
    uint16_t D1_Pin;
    GPIO_TypeDef* D1_GPIO_Port;
} Motor;

extern volatile uint8_t updateMotorPidsFlag;
extern volatile MotorData motor_data;

void initMotors();
void motor_setDutycycle(int portNumber, uint32_t dutycycle);
void motor_setDirection(int portNumber, enum MOTOR_DIRECTION_CTL direction);
void motor_stop(int protNumber);
void motors_forceOff(void);

//updates PID settings from Rx buffer
void update_motor_pidSettings();
void update_motor_posPidSettings();

//updates motor cmd from Rx buffer
void update_motor(uint8_t channel, int16_t bemf_filtered);
#endif //MOTOR_H