//
// Created by matthias on 7/15/25.
//

#ifndef UTILLITY_H
#define UTILLITY_H

#include "main.h"

#define TOGGLE_USERLED do{ HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin); }while(0);

void delayus(uint32_t delay);


#endif //UTILLITY_H