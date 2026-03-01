#include "Data_structures/PID.h"
#include "stdint.h"

void PIDController_Init(volatile PIDController* pid)
{
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PIDController* pid, const int32_t setpoint, const float measurement)
{
    /*
	* Error signal
	*/
    const float error = (float)setpoint - measurement;


    /*
	* Proportional
	*/
    const float proportional = pid->Kp * (float)error;


    /*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * ((float)error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt)
    {
        pid->integrator = pid->limMaxInt;
    }
    else if (pid->integrator < pid->limMinInt)
    {
        pid->integrator = pid->limMinInt;
    }


    /*
	* Derivative (band-limited differentiator)
	*/

    pid->differentiator = -(2.0f * pid->Kd * ((float)measurement - pid->prevMeasurement)
            /* Note: derivative on measurement, therefore minus sign in front of equation! */
            + (2.0f * pid->tau - pid->T) * pid->differentiator)
        / (2.0f * pid->tau + pid->T);


    /*
	* Compute output and apply limits
	*/
    pid->out = proportional; //+ pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {
        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}