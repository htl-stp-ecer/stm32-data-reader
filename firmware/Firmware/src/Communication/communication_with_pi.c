//
// Created by matthias on 5/1/25.
//
#include "string.h"

#include "main.h"
#include "Communication/spi.h"
#include "Communication/communication_with_pi.h"


volatile RxBuffer rxBuffer = {0};
volatile TxBuffer txBuffer = {
    .transferVersion = TRANSFER_VERSION,
};

//when set to true the STM will update the corresponding actuators from the rxBuffer data
volatile uint8_t updateServoFlag = 0;
volatile uint8_t updateFlags = 0;

void initPiCommunication()
{
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
        continue;
    HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)&txBuffer, (uint8_t*)&rxBuffer, BUFFER_LENGTH_DUPLEX_COMMUNICATION);
}