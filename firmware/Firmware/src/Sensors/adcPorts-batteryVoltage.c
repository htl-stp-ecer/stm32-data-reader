#include <string.h>

#include "Communication/communication_with_pi.h"
#include "Communication/spi.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/adcInit.h"
#include "Sensors/bemf.h"
#include "Actors/motor.h"

// Factory VREFINT calibration value (12-bit ADC reading at VDDA=3.3V, 30°C)
#define VREFINT_CAL (*(volatile uint16_t*)0x1FFF7A2AU)

// DMA target buffer — continuously overwritten by circular DMA
static volatile uint16_t adcDmaBuffer[ANALOG_SENSOR_COUNT];

// Oversampling accumulators
static volatile uint32_t adcAccum[ANALOG_SENSOR_COUNT];
static volatile uint32_t adcSampleCount;

// VDDA scale: multiply raw ADC counts by this to normalize to 3.3V-equivalent
volatile float vddaScale = 1.0f;

void startContinuousAnalogSampling(void)
{
    adcSampleCount = 0;
    memset((void*)adcAccum, 0, sizeof(adcAccum));
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDmaBuffer, ANALOG_SENSOR_COUNT);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2)
    {
        // DMA/ADC error during BEMF conversion — abort and let watchdog recover
        HAL_ADC_Stop_DMA(hadc);
        bemfState = STOPPED;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* --- analog Ports + battery voltage (continuous oversampling) --- */
    if (hadc->Instance == ADC1)
    {
        for (int i = 0; i < ANALOG_SENSOR_COUNT; i++)
            adcAccum[i] += adcDmaBuffer[i];
        adcSampleCount++;
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

int updatingAnalogValuesInSpiBuffer(void)
{
    if (adcSampleCount == 0)
        return 2;

    if (!spi2_wait_idle())
        return 1; // SPI2 stuck — skip rather than blocking ISR

    // Atomic snapshot and reset — prevent DMA callback from firing mid-swap
    __disable_irq();
    uint32_t count = adcSampleCount;
    uint32_t localAccum[ANALOG_SENSOR_COUNT];
    memcpy(localAccum, (void*)adcAccum, sizeof(localAccum));
    memset((void*)adcAccum, 0, sizeof(adcAccum));
    adcSampleCount = 0;
    __enable_irq();

    // Update VDDA scale from averaged VREFINT reading (index 7)
    uint32_t vrefintAvg = localAccum[7] / count;
    if (vrefintAvg > 0)
        vddaScale = (float)VREFINT_CAL / (float)vrefintAvg;

    // Compute averages, apply VDDA compensation, and write to SPI buffer
    float scale = vddaScale;
    for (int i = 0; i < 6; i++)
        txBuffer.analogSensor[i] = (int16_t)((float)(localAccum[i] / count) * scale);
    txBuffer.batteryVoltage = (int16_t)((float)(localAccum[6] / count) * scale);

    return 0;
}