//
// Created by matthias on 5/4/25.
//

#ifndef TIMER_H
#define TIMER_H

#include "../main.h"
extern volatile uint32_t microSeconds;

void delayus(uint32_t delay);

void systemTimerStart(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);

HAL_TIM_ActiveChannel convertChanalToActiveChanal(uint16_t chanal);

#define doEveryXuSeconds(func, interval, lastExecutionTime)\
{ \
        if((microSeconds) - (interval) >= (lastExecutionTime)){ \
            lastExecutionTime = microSeconds; \
            func; \
        } \
}

#define doAfterXuSeconds(func, interval, startTime) \
{ \
    if((microSeconds) - (startTime) >=  (interval)){ \
        func; \
    } \
}


#endif //TIMER_H