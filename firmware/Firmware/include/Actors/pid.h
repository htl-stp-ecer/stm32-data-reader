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
// dt is the elapsed time (seconds) since this controller's previous update.
// The loop is dt-EXPLICIT: the integral accumulates err*dt and the derivative
// divides by dt, so kI/kD are in physical (per-second) units and stay correct
// under the variable BEMF cadence / watchdog jitter the firmware experiences.
int32_t pid_update(PidController* pid, int32_t goal, int32_t current, float dt);

#endif //PID_H