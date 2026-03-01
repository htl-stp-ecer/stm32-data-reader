//
// Created by matthias on 4/28/25.
//

#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

#define NUM_SERVOS 4

#define ALL_SERVOS_FULLY_DISABLED 0

typedef struct
{
    //PWM pin
    uint16_t PWM_Pin;
    GPIO_TypeDef* PWM_GPIO_Port;

    //PWM Chanal and Timer
    TIM_HandleTypeDef* timer;
    uint32_t chanal;
} Servo;


typedef enum
{
    SERVO_FULLY_DISABLED = 0,
    SERVO_DISABLED,
    SERVO_ENABLED
} ServoMode;

extern const volatile Servo servos[NUM_SERVOS];

//enable / disable pwm prots of servo
void servo_enable(const int servoPort);
void servo_disable(const int servoPort);

//enable / disable 6v power supply for servos
void servo_fullyEnable(); //turn on the 6V supplay for the servos
void servo_fullyDisable(); //turn off the 6V supplay for the servos
void servo_setPositon(const int servoPort, uint16_t position);

void update_servo_cmd();
#endif //SERVO_H