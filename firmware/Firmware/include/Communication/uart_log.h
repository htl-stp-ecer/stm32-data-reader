#ifndef UART_LOG_H
#define UART_LOG_H

#include <stddef.h>

/*
 * Non-blocking log output to the Pi over USART3.
 *
 * printf and vprintf are replaced (uart_log.c): each call formats into a stack
 * buffer and copies the whole line into a ring buffer in one go, so lines from
 * the main loop and from interrupts never get mixed. The USART3 TXE interrupt
 * (lowest priority) then sends the buffer out byte by byte.
 *
 * USART3 TX can't use DMA: its only two DMA streams (DMA1 Stream3/Stream4) are
 * used by SPI2, the Pi link.
 *
 * - Safe to call from interrupts, but don't use %f there: newlib-nano's float
 *   formatting is not reentrant.
 * - Never blocks: when the buffer is full the whole message is dropped, and
 *   "[log] dropped N msgs" is sent once there is space again.
 */

void uart_log_write(const char* data, size_t len);

/* Call from USART3_IRQHandler. */
void uart_log_irq_handler(void);

/* Blocking drain of the buffer, for fatal error paths (works with IRQs off). */
void uart_log_flush(void);

#endif // UART_LOG_H
