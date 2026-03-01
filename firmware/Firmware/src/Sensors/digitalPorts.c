//
// Created by matthias on 5/1/25.
//
#include "main.h"

uint16_t readDigitalInputs(void)
{
    uint16_t DIN_Pins[10] = {
        DIN0_Pin, DIN1_Pin, DIN2_Pin, DIN3_Pin, DIN4_Pin, DIN5_Pin, DIN6_Pin, DIN7_Pin, DIN8_Pin, DIN9_Pin
    };
    GPIO_TypeDef* DIN_Ports[10] = {
        DIN0_GPIO_Port, DIN1_GPIO_Port, DIN2_GPIO_Port, DIN3_GPIO_Port, DIN4_GPIO_Port, DIN5_GPIO_Port, DIN6_GPIO_Port,
        DIN7_GPIO_Port, DIN8_GPIO_Port, DIN9_GPIO_Port
    };
    uint16_t result = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        if (HAL_GPIO_ReadPin(DIN_Ports[i], DIN_Pins[i]) == GPIO_PIN_RESET)
        {
            result |= (1 << i);
        }
    }

    if (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_SET)
    {
        //button has reversed logic in coperison to digital ports
        result |= (1 << 10);
    }

    return result;
}