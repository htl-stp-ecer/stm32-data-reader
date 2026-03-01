#include <string.h>

#include "Communication/communication_with_pi.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/adcInit.h"
#include "Sensors/bemf.h"
#include "Actors/motor.h"

#define ADC_IN_USE 0x01
#define NEW_DATA 0x02

typedef volatile struct
{
    uint8_t state;

    int16_t adcRaw[ANALOG_SENSOR_COUNT];
} AnalogReader;

volatile AnalogReader analogReader = {0};


/* @brief:
 * start the sampling of the adc's; conversion complete callback will be called when the conversion is done
 */
void sampleAnalogPorts()
{
    analogReader.state |= NEW_DATA;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)analogReader.adcRaw, ANALOG_SENSOR_COUNT);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* --- analog Ports + battery voltage --- */
    if (hadc->Instance == ADC1)
    {
        analogReader.state &= ~ADC_IN_USE;
        analogReader.state |= NEW_DATA;
    }

    /* --- bemf --- */
    else if (hadc->Instance == ADC2)
    {
        bemfState = CONVERSION_DONE;
        processBEMF();
        for (int ch = 0; ch < MOTOR_COUNT; ch++)
            update_motor(ch, bemfLastReadings[ch]);
    }
}

int updatingAnalogValuesInSpiBuffer()
{
    if ((analogReader.state & NEW_DATA) == 0)
        return 2;

    if ((analogReader.state & ADC_IN_USE) == ADC_IN_USE)
        return 3;

    while (SPI2->SR & SPI_SR_BSY) //check if the spi buffer is in use
        continue;

    //coppys analog port readings and battery voltage
    memcpy(&txBuffer.analogSensor, &analogReader.adcRaw,
           sizeof(txBuffer.analogSensor) + sizeof(txBuffer.batteryVoltage));
    return 0;
}