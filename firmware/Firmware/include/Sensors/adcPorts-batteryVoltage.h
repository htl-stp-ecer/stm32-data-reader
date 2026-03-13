//
// Created by matthias on 5/2/25.
//

#ifndef ADC_VALUES_H
#define ADC_VALUES_H

/*ANALOG PORTS + BATTERY VOLTAGE*/
#define ANALOG_SENSOR_COUNT 7 //6 analog ports + Vbat measurment

#define ANALOG_OUTPUT_INTERVAL 4000 // 250 Hz output rate (oversample between updates)

void startContinuousAnalogSampling(void);
int updatingAnalogValuesInSpiBuffer(void);

#endif //ADC_VALUES_H