#include "stm32f4xx_hal.h"

#include "Utillity/utillity.h"
#include "Hardware/timer.h"

void delayus(uint32_t delay)
{
    uint32_t startTime = microSeconds;

    while (startTime + delay > microSeconds);
}