//
// Created by matthias on 5/2/25.
//

#ifndef ADC_VALUES_H
#define ADC_VALUES_H

/*ANALOG PORTS + BATTERY VOLTAGE + VREFINT*/
#define ANALOG_SENSOR_COUNT 8 //6 analog ports + Vbat + VREFINT

#define ANALOG_OUTPUT_INTERVAL 4000 // 250 Hz output rate (oversample between updates)

// VDDA compensation: scale factor = VREFIN_CAL / measured_vrefint
// When VDDA == 3.3V (factory cal), this is 1.0. Multiply raw ADC counts by this
// to normalize readings as if VDDA were always 3.3V.
extern volatile float vddaScale;

void startContinuousAnalogSampling(void);
int updatingAnalogValuesInSpiBuffer(void);

#endif //ADC_VALUES_H