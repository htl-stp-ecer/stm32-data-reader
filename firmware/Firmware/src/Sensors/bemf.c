#include "stm32f4xx_hal.h"
#include "Actors/motor.h"
#include "Sensors/adcPorts-batteryVoltage.h"
#include "Sensors/bemf.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>


#include "adcInit.h"
#include "communication_with_pi.h"
#include "Communication/spi.h"
#include "Hardware/timer.h"
#include "Data_structures/filter.h"

#define BEMF_FILTER_ALPHA 0.2f
#define MAX_BEMF_READING 2000
#define MEDIAN_WINDOW 3

// --- BEMF zero-offset correction ---
// The differential BEMF reading is not proportional to wheel speed through the
// origin: extrapolating the (linear) BEMF-vs-speed relation to zero speed gives
// a non-zero per-motor offset (~20-40 counts; an ADC/amplifier + coast-measure
// settling artifact). Integrating that offset every cycle (no dt, no dead-zone)
// adds phantom ticks that grow with time -> odometry over-reports, worst at low
// speed / during acceleration. The Pi calibrates the offset per motor against
// the external calibration board (auto_tune_bemf_velocity: B = -intercept/slope)
// and sends it in KinematicsConfig.bemf_offset; we subtract it before integrating
// so the tick integral stays proportional to wheel angle (speed-independent).
// The dead-zone must exceed (driving_offset - standstill_offset) so the corrected
// reading is zeroed when the wheel is actually stopped.
#define BEMF_DEADZONE 25.0f  // counts; |corrected| below this => 0

volatile uint16_t adc_dma_bemf_buffer[BEMF_CHANNELS_PER_MOTOR] = {0};
volatile float bemfLastReadings[MOTOR_COUNT] = {0};
volatile float bemfRawReadings[MOTOR_COUNT] = {0};
volatile enum BemfState bemfState = STOPPED;
volatile uint32_t bemfConvCount = 0; // how many times processBEMF actually ran
volatile uint32_t bemfConvCountPerMotor[MOTOR_COUNT] = {0}; // per-motor conversion count
volatile uint8_t bemfCurrentMotor = 0;
static volatile uint32_t bemfCycleStartTime = 0;

// Circular buffer for median-of-3 pre-filter (per motor)
static float medianBuf[MOTOR_COUNT][MEDIAN_WINDOW] = {{0}};
static uint8_t medianIdx[MOTOR_COUNT] = {0};

// Float accumulator per motor — keeps fractional ticks between updates
static float positionAccum[MOTOR_COUNT] = {0};

// Per-motor timestamp (microSeconds) of the last BEMF sample, for dt-aware
// integration. 0 = no previous sample yet. Integrating bemf*dt instead of a
// fixed += bemf makes the position a true ∫ω dt, immune to the round-robin
// cadence, loop jitter and watchdog skips that vary the real sample period.
static uint32_t bemf_last_us[MOTOR_COUNT] = {0};

// Per-motor BEMF zero-offset (ADC counts), supplied by the Pi via
// KinematicsConfig.bemf_offset (see bemf_set_offset). 0 until configured =>
// no correction (original behaviour).
static float bemf_offset_cfg[MOTOR_COUNT] = {0};

void bemf_set_offset(const volatile float off[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        bemf_offset_cfg[i] = off[i];
}

// ADC channel pairs per software motor: {low, high}
static const uint32_t bemfAdcChannels[MOTOR_COUNT][2] = {
    {ADC_CHANNEL_2, ADC_CHANNEL_3}, // Motor 0
    {ADC_CHANNEL_0, ADC_CHANNEL_1}, // Motor 1
    {ADC_CHANNEL_6, ADC_CHANNEL_7}, // Motor 2
    {ADC_CHANNEL_4, ADC_CHANNEL_5}, // Motor 3
};

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

// Reconfigure ADC2 to scan only the 2 channels for the given motor
static void configureBemfAdc(uint8_t motor)
{
    hadc2.Init.NbrOfConversion = 2;
    HAL_ADC_Init(&hadc2);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    sConfig.Channel = bemfAdcChannels[motor][0]; // low
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    sConfig.Channel = bemfAdcChannels[motor][1]; // high
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}


void stop_motors_for_bemf_conv()
{
    if (bemfState == STOPPED)
    {
        // Only stop the motor we're about to measure
        motor_setDirection(bemfCurrentMotor, OFF);
        bemfCycleStartTime = microSeconds;
        bemfState = WAITING_TO_START;
    }
}

void startBemfReading()
{
    configureBemfAdc(bemfCurrentMotor);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_dma_bemf_buffer, BEMF_CHANNELS_PER_MOTOR);
    bemfState = CONVERSION_ONGOING;
    //call back interupt will be called when ADC conversion is done
}

void processBEMF()
{
    if (rxBuffer.featureFlags & FEATURE_BEMF_DISABLE)
    {
        bemfState = STOPPED;
        return;
    }
    if (bemfState == CONVERSION_DONE)
    {
        bemfConvCount++;
        uint8_t ch = bemfCurrentMotor;
        bemfConvCountPerMotor[ch]++;

        // Compute differential BEMF and normalize for VDDA drift
        // buf[0] = low channel, buf[1] = high channel
        float scale = vddaScale;
        bemfRawReadings[ch] = ((float)adc_dma_bemf_buffer[1] - (float)adc_dma_bemf_buffer[0]) * scale;

        // Store into per-motor median ring buffer
        medianBuf[ch][medianIdx[ch]] = bemfRawReadings[ch];
        medianIdx[ch] = (medianIdx[ch] + 1) % MEDIAN_WINDOW;

        float filtered = median3(medianBuf[ch][0], medianBuf[ch][1], medianBuf[ch][2]);
        bemfLastReadings[ch] = lowPassFilter(filtered, bemfLastReadings[ch], BEMF_FILTER_ALPHA);

        if (bemfLastReadings[ch] <= MAX_BEMF_READING && bemfLastReadings[ch] >= -MAX_BEMF_READING)
        {
            // Subtract the per-motor RAW BEMF offset (signed; the Pi sends it in
            // raw-reading space) + dead-zone. raw = ±k·ω + offset, so raw - offset
            // = ±k·ω → the position integral stays proportional to wheel angle for
            // both directions (the old "no dead zone" behaviour drifted at
            // standstill and over-counted at low speed). At rest raw≈offset →
            // corrected≈0 → zeroed by the dead-zone.
            float corrected = bemfLastReadings[ch] - bemf_offset_cfg[ch];
            if (corrected < BEMF_DEADZONE && corrected > -BEMF_DEADZONE)
                corrected = 0.0f;

            motor_data.bemf[ch] = (int32_t)corrected;

            // dt-aware integration: position += bemf * dt (seconds since this
            // motor was last sampled). Unsigned subtraction handles the
            // microSeconds wrap. The first sample only seeds the timestamp.
            // NOTE: this rescales `position` units (per second), so ticks_to_rad
            // must be re-tuned after flashing.
            const uint32_t now_us = microSeconds;
            if (bemf_last_us[ch] != 0)
            {
                const float dt_s = (float)(now_us - bemf_last_us[ch]) * 1e-6f;
                positionAccum[ch] += corrected * dt_s;
            }
            bemf_last_us[ch] = now_us;

            // Transfer whole ticks to integer position, keep remainder
            int32_t whole = (int32_t)positionAccum[ch];
            if (whole != 0)
            {
                motor_data.position[ch] += whole;
                positionAccum[ch] -= (float)whole;
            }
        }

        // Advance to next motor for the next cycle
        bemfCurrentMotor = (bemfCurrentMotor + 1) % MOTOR_COUNT;
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

    // Skip this motor, move on
    bemfCurrentMotor = (bemfCurrentMotor + 1) % MOTOR_COUNT;
    bemfState = STOPPED;
}

void updatingMotorsInSpiBuffer()
{
    if (!spi2_wait_idle())
        return; // SPI2 stuck — skip this update rather than hang

    if (rxBuffer.featureFlags & FEATURE_BEMF_DISABLE)
    {
        // BEMF measurements disabled — surface zeros instead of stale values.
        // motor_data.position[] keeps its last frozen value as a stable anchor.
        for (int i = 0; i < MOTOR_COUNT; i++) motor_data.bemf[i] = 0;
    }
    txBuffer.motor = motor_data;
}
