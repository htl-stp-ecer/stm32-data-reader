#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kP;
    float kI;
    float kD;
    float iMax;
    float outMax;
    float prevErr;
    float iErr;
} PidController;

void pid_init(PidController* pid);
void pid_init_position(PidController* pid);
void pid_reset(PidController* pid);
int32_t pid_update(PidController* pid, int32_t goal, int32_t current);

#endif //PID_H