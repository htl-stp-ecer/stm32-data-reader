#include "Actors/pid.h"
#include "Actors/motor.h"

// dt-EXPLICIT gains (per-second units). kI was rescaled from the old
// dt-implicit default 0.045 by the nominal MAV rate (~200 Hz/motor):
// kI_explicit = kI_implicit * rate = 0.045 * 200 = 9.0, so boot behaviour
// matches the previous loop until the autotuner pushes optimized gains.
#define PID_DEFAULT_P  1.22f
#define PID_DEFAULT_I  9.0f
#define PID_DEFAULT_D  0.000f

// Fallback dt (s) when no valid timestamp delta is available (first call after
// a mode change or a long BEMF gap). Matches the nominal per-motor MAV rate.
#define PID_NOMINAL_DT (1.0f / 200.0f)

// Position loop outputs a velocity target (BEMF ticks), not PWM directly.
// Pure proportional: the inner velocity PID already provides damping and
// steady-state correction, so I and D on the outer loop are unnecessary.
// P=0.15: at 1000-tick error → ~150 velocity target; speedLimit clamps large errors.
#define PID_POS_DEFAULT_P  1.0f
#define PID_POS_DEFAULT_I  0.0f
#define PID_POS_DEFAULT_D  0.0f
// Max velocity the position loop can request (in BEMF ticks)

void pid_init(PidController* pid)
{
    pid->kP = PID_DEFAULT_P;
    pid->kI = PID_DEFAULT_I;
    pid->kD = PID_DEFAULT_D;
    pid->iMax = (float)MOTOR_MAX_DUTYCYCLE;
    pid->outMax = (float)MOTOR_MAX_DUTYCYCLE;
    pid->prevErr = 0.0f;
    pid->iErr = 0.0f;
}

void pid_init_position(PidController* pid)
{
    pid->kP = PID_POS_DEFAULT_P;
    pid->kI = PID_POS_DEFAULT_I;
    pid->kD = PID_POS_DEFAULT_D;
    pid->iMax = MOTOR_MAX_DUTYCYCLE;
    pid->outMax = MOTOR_MAX_DUTYCYCLE;
    pid->prevErr = 0.0f;
    pid->iErr = 0.0f;
}

void pid_reset(PidController* pid)
{
    pid->prevErr = 0.0f;
    pid->iErr = 0.0f;
}

int32_t pid_update(PidController* pid, int32_t goal, int32_t current, float dt)
{
    // Guard against a missing/garbage timestamp delta so the loop never blows
    // up (divide-by-zero on dErr, or a huge integral jump after a long gap).
    if (dt <= 0.0f || dt > 0.1f) dt = PID_NOMINAL_DT;

    float pErr = (float)(goal - current);

    pid->iErr += pErr * dt;

    float dErr = (pErr - pid->prevErr) / dt;
    pid->prevErr = pErr;

    // Clamp integral contribution (not raw accumulator) to prevent windup
    float iTerm = pid->kI * pid->iErr;
    if (pid->kI > 0.0f)
    {
        if (iTerm > pid->iMax)
        {
            iTerm = pid->iMax;
            pid->iErr = pid->iMax / pid->kI;
        }
        else if (iTerm < -pid->iMax)
        {
            iTerm = -pid->iMax;
            pid->iErr = -pid->iMax / pid->kI;
        }
    }

    float cmd = pid->kP * pErr + iTerm + pid->kD * dErr;

    // Clamp output to valid PWM range
    if (cmd > pid->outMax) cmd = pid->outMax;
    else if (cmd < -pid->outMax) cmd = -pid->outMax;

    return (int32_t)cmd;
}