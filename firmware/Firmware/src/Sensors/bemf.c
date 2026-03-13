#include "stm32f4xx_hal.h"
#include "Actors/motor.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/bemf.h"

#include <stdbool.h>
#include <string.h>


#include "adcInit.h"
#include "communication_with_pi.h"
#include "Hardware/timer.h"
#include "Data_structures/filter.h"

#define BEMF_FILTER_ALPHA 0.2f
#define MAX_BEMF_READING 2000
#define MEDIAN_WINDOW 3

volatile uint16_t adc_dma_bemf_buffer[BEMF_ADC_CHANNELS] = {0};
volatile float bemfLastReadings[MOTOR_COUNT] = {0};
volatile float bemfRawReadings[MOTOR_COUNT] = {0};
volatile enum BemfState bemfState = STOPPED;
volatile uint32_t bemfConvCount = 0; // how many times processBEMF actually ran
static volatile uint32_t bemfCycleStartTime = 0;

// Circular buffer for median-of-3 pre-filter
static float medianBuf[MOTOR_COUNT][MEDIAN_WINDOW] = {{0}};
static uint8_t medianIdx = 0;

static float median3(float a, float b, float c)
{
    if (a > b)
    {
        float t = a;
        a = b;
        b = t;
    }
    if (b > c)
    {
        float t = b;
        b = c;
        c = t;
    }
    if (a > b) { b = a; }
    return b;
}


void stop_motors_for_bemf_conv()
{
    if (bemfState == STOPPED)
    {
        //turn all motors off
        for (int i = 0; i < 4; i++)
        {
            motor_setDirection(i, OFF);
        }
        bemfCycleStartTime = microSeconds;
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
        bemfConvCount++;
        // Compute differential BEMF and normalize for VDDA drift
        float scale = vddaScale;
        bemfRawReadings[0] = ((float)adc_dma_bemf_buffer[3] - (float)adc_dma_bemf_buffer[2]) * scale;
        bemfRawReadings[1] = ((float)adc_dma_bemf_buffer[1] - (float)adc_dma_bemf_buffer[0]) * scale;
        bemfRawReadings[2] = ((float)adc_dma_bemf_buffer[7] - (float)adc_dma_bemf_buffer[6]) * scale;
        bemfRawReadings[3] = ((float)adc_dma_bemf_buffer[5] - (float)adc_dma_bemf_buffer[4]) * scale;

        // Store raw values into median ring buffer
        for (int ch = 0; ch < MOTOR_COUNT; ch++)
            medianBuf[ch][medianIdx] = bemfRawReadings[ch];
        medianIdx = (medianIdx + 1) % MEDIAN_WINDOW;

        for (int ch = 0; ch < MOTOR_COUNT; ch++)
        {
            float filtered = median3(medianBuf[ch][0], medianBuf[ch][1], medianBuf[ch][2]);
            bemfLastReadings[ch] = lowPassFilter(filtered, bemfLastReadings[ch], BEMF_FILTER_ALPHA);

            if (bemfLastReadings[ch] > MAX_BEMF_READING || bemfLastReadings[ch] < -MAX_BEMF_READING)
                continue;

            motor_data.bemf[ch] = (int32_t)bemfLastReadings[ch];

            if (bemfLastReadings[ch] > 8 || bemfLastReadings[ch] < -8)
            {
                motor_data.position[ch] += (int32_t)bemfLastReadings[ch];
            }
        }
        bemfState = STOPPED;
    }
}

void bemf_watchdog_check(uint32_t now)
{
    if (bemfState == STOPPED)
        return;

    if (now - bemfCycleStartTime < BEMF_WATCHDOG_TIMEOUT)
        return;

    // BEMF cycle is stuck — abort ADC/DMA and discard any partial results
    HAL_ADC_Stop_DMA(&hadc2);

    // Wipe the DMA buffer so stale data can't leak through
    memset((void*)adc_dma_bemf_buffer, 0, sizeof(adc_dma_bemf_buffer));

    bemfState = STOPPED;
}

void updatingMotorsInSpiBuffer()
{
    //check the bemf-State
    while (SPI2->SR & SPI_SR_BSY) //check if the spi buffer is in use
        continue;

    txBuffer.motor = motor_data;
}