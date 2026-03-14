//
// Created by matthias on 7/16/25.
//

#ifndef BEMF_H
#define BEMF_H

#define BEMF_CHANNELS_PER_MOTOR 2 // high + low ADC channels per motor

#define BEMF_SAMPLING_INTERVAL (uint32_t) 1250 //us - interval between individual motor measurements (each motor measured every 4 * 1250 = 5000us)

#define BEMF_CONVERSION_START_DELAY_TIME (uint32_t) 500 //us - time to delay the start of the bemf adc conversion - waiting until the bemf signal is stable again


#define BEMF_WATCHDOG_TIMEOUT (uint32_t) (BEMF_SAMPLING_INTERVAL * 2) //us - force-recover if cycle stuck longer than this


enum BemfState
{
    STOPPED = 0, //motors are running, bemf meashurment is finished
    WAITING_TO_START, //waits x us until bemf signal is stabel
    CONVERSION_ONGOING, //adc conversion is running
    CONVERSION_DONE //adc conversion finished waiting for processing
};

extern volatile uint16_t adc_dma_bemf_buffer[BEMF_CHANNELS_PER_MOTOR];
extern volatile float bemfLastReadings[];
extern volatile float bemfRawReadings[];
extern volatile enum BemfState bemfState;
extern volatile uint32_t bemfConvCount;
extern volatile uint8_t bemfCurrentMotor;

void stop_motors_for_bemf_conv();
void startBemfReading();
void processBEMF();
void bemf_watchdog_check(uint32_t now);
void updatingMotorsInSpiBuffer();

#endif //BEMF_H