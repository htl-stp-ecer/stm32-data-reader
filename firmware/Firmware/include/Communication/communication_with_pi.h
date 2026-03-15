//
// Created by matthias on 5/1/25.
//
#ifndef COMMUNICATION_WITH_PI_H
#define COMMUNICATION_WITH_PI_H

#include <stdint.h>
#include "spi/pi_buffer.h"

extern volatile RxBuffer rxBuffer;
extern volatile TxBuffer txBuffer;

/**
 * @brief flags that command a subsystem to update
 * @details
 * Each bit is related to one specific system.
 * If set it will update the system.
 * Flags are accepted when non-zero and cleared after processing.
 *
 * (should be used for systems, that don't have/should to update constantly)
 */
extern volatile uint8_t updateFlags;

void initPiCommunication();

#endif //COMMUNICATION_WITH_PI_H