//
// Created by matthias on 6/3/25.
//

#ifndef INTERUPT_PRIORYTY_H
#define INTERUPT_PRIORYTY_H

/*--- ADC ---*/
//ADC 1
#define IRQ_ADC1_PRIORITY 2
#define IRQ_ADC1_SUB_PRIORITY 4

//ADC 2
#define IRQ_ADC2_PRIORITY 2
#define IRQ_ADC2_SUB_PRIORITY 3

/*--- DMA ---*/
// SPI3 Rx
#define IRQ_DMA1_Stream0_PRIORITY 1
#define IRQ_DMA1_Stream0_SUB_PRIORITY 1

// SPI2 Rx
#define IRQ_DMA1_Stream3_PRIORITY 0
#define IRQ_DMA1_Stream3_SUB_PRIORITY 1

// SPI2 Tx
#define IRQ_DMA1_Stream4_PRIORITY 0
#define IRQ_DMA1_Stream4_SUB_PRIORITY 2

// SPI3 Tx
#define IRQ_DMA1_Stream5_PRIORITY 1
#define IRQ_DMA1_Stream5_SUB_PRIORITY 2

//ADC 1
#define IRQ_DMA2_Stream0_PRIORITY 2
#define IRQ_DMA2_Stream0_SUB_PRIORITY 1

//ADC 2
#define IRQ_DMA2_Stream2_PRIORITY 2
#define IRQ_DMA2_Stream2_SUB_PRIORITY 0

/*-- SPI ---*/
//SPI 2
#define IRQ_SPI2_PRIORITY 0
#define IRQ_SPI2_SUB_PRIORITY 0

//SPI 3
#define IRQ_SPI3_PRIORITY 1
#define IRQ_SPI3_SUB_PRIORITY 0

/*--- Timer ---*/
//Timer 6
#define IRQ_TIM6_PRIORITY 3
#define IRQ_TIM6_SUB_PRIORITY 0

/*--- UART ---*/
#define IRQ_UART3_PRIORITY 4
#define IRQ_UART3_SUB_PRIORITY 0


#endif //INTERUPT_PRIORYTY_H