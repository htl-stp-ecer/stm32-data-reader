#include "stm32f4xx_hal.h"
#include "Actors/motor.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/bemf.h"

#include <stdbool.h>
#include <string.h>


#include "adcInit.h"
#include "communication_with_pi.h"
#include "Data_structures/filter.h"

#define BEMF_FILTER_ALPHA 0.2f
#define MAX_BEMF_READING 1700

volatile uint16_t adc_dma_bemf_buffer[BEMF_ADC_CHANNELS] = {0};
volatile float bemfLastReadings[MOTOR_COUNT] = {0};
volatile float bemfRawReadings[MOTOR_COUNT] = {0};
volatile enum BemfState bemfState = STOPPED;


void stop_motors_for_bemf_conv()
{
    if (bemfState == STOPPED)
    {
        //turn all motors off
        for (int i = 0; i < 4; i++)
        {
            motor_setDirection(i, OFF);
        }
        bemfState = WAITING_TO_START;
        //waits now until the delay of BEMF_CONVERSION_START_DELAY_TIME is done then start BEMFadcConversion is called
    }
}

void startBemfReading()
{
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_dma_bemf_buffer, BEMF_ADC_CHANNELS);
    bemfState = CONVERSION_ONGOING;
    //call back interupt will be called when ADC conversion is done
}

void processBEMF()
{
    if (bemfState == CONVERSION_DONE)
    {
        bemfRawReadings[0] = (float)adc_dma_bemf_buffer[3] - (float)adc_dma_bemf_buffer[2];
        bemfRawReadings[1] = (float)adc_dma_bemf_buffer[1] - (float)adc_dma_bemf_buffer[0];
        bemfRawReadings[2] = (float)adc_dma_bemf_buffer[7] - (float)adc_dma_bemf_buffer[6];
        bemfRawReadings[3] = (float)adc_dma_bemf_buffer[5] - (float)adc_dma_bemf_buffer[4];

        for (int ch = 0; ch < MOTOR_COUNT; ch++)
        {
            bemfLastReadings[ch] = lowPassFilter(bemfRawReadings[ch], bemfLastReadings[ch], BEMF_FILTER_ALPHA);
            if (bemfLastReadings[ch] > MAX_BEMF_READING) break; // if bemf reading is above max then dump the reading

            // Store raw BEMF for telemetry
            motor_data.bemf[ch] = (int32_t)bemfLastReadings[ch];

            // Accumulate position using filtered values to reduce drift
            if (bemfLastReadings[ch] > 8 || bemfLastReadings[ch] < -8)
            {
                motor_data.position[ch] += (int32_t)bemfLastReadings[ch];
            }
        }
        bemfState = STOPPED;
    }
}

void updatingMotorsInSpiBuffer()
{
    //check the bemf-State
    while (SPI2->SR & SPI_SR_BSY) //check if the spi buffer is in use
        continue;

    txBuffer.motor = motor_data;
}