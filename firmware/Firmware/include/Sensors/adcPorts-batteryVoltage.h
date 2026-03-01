//
// Created by matthias on 5/2/25.
//

#ifndef ADC_VALUES_H
#define ADC_VALUES_H

/*ANALOG PORTS + BATTERY VOLTAGE*/
#define ANALOG_SENSOR_COUNT 7 //6 analog ports + Vbat measurment

#define ANALOG_SENSOR_SAMPLING_INTERVAL 1000

void sampleAnalogPorts();
int updatingAnalogValuesInSpiBuffer();

#endif //ADC_VALUES_H