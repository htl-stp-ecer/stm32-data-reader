#include "Communication/uart_log.h"
#include "Communication/usart.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define LOG_BUF_SIZE 4096u // must be a power of two
#define LOG_BUF_MASK (LOG_BUF_SIZE - 1u)
#define LOG_LINE_MAX 256   // longest single printf (the heartbeat is ~200)

static char buf[LOG_BUF_SIZE];
static volatile uint32_t head;    // free-running, advanced by producers
static volatile uint32_t tail;    // free-running, advanced by the TX interrupt
static volatile uint32_t dropped; // messages dropped since the last report

// Copies the whole message into the ring buffer, or nothing if it doesn't fit.
static bool push(const char* data, const size_t len)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool fits = len <= LOG_BUF_SIZE - (head - tail);
    if (fits)
    {
        const uint32_t start = head & LOG_BUF_MASK;
        const size_t first = (len < LOG_BUF_SIZE - start) ? len : LOG_BUF_SIZE - start;
        memcpy(&buf[start], data, first);
        memcpy(buf, data + first, len - first);
        head += len;
        __HAL_UART_ENABLE_IT(&uart_pi, UART_IT_TXE);
    }
    __set_PRIMASK(primask);
    return fits;
}

static void report_drops(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t n = dropped;
    dropped = 0;
    __set_PRIMASK(primask);

    char msg[40];
    const int len = snprintf(msg, sizeof msg, "[log] dropped %lu msgs\r\n", (unsigned long)n);
    if (!push(msg, (size_t)len))
    {
        dropped += n; // still no space, report later
    }
}

void uart_log_write(const char* data, const size_t len)
{
    if (dropped != 0)
    {
        report_drops();
    }
    if (!push(data, len))
    {
        dropped++;
    }
}

void uart_log_irq_handler(void)
{
    if (!__HAL_UART_GET_FLAG(&uart_pi, UART_FLAG_TXE))
    {
        return;
    }
    // IRQs off so a producer can't push between the empty check and disabling TXEIE.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tail != head)
    {
        uart_pi.Instance->DR = (uint8_t)buf[tail & LOG_BUF_MASK];
        tail++;
    }
    else
    {
        __HAL_UART_DISABLE_IT(&uart_pi, UART_IT_TXE);
    }
    __set_PRIMASK(primask);
}

void uart_log_flush(void)
{
    if (uart_pi.gState == HAL_UART_STATE_RESET)
    {
        return; // USART3 not initialised yet, polling TXE would hang
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_UART_DISABLE_IT(&uart_pi, UART_IT_TXE);
    while (tail != head)
    {
        while (!__HAL_UART_GET_FLAG(&uart_pi, UART_FLAG_TXE))
        {
        }
        uart_pi.Instance->DR = (uint8_t)buf[tail & LOG_BUF_MASK];
        tail++;
    }
    while (!__HAL_UART_GET_FLAG(&uart_pi, UART_FLAG_TC))
    {
    }
    __set_PRIMASK(primask);
}

/* Replace newlib's printf/vprintf, which go through the (not interrupt-safe)
 * stdout FILE. The firmware is built with -fno-builtin-printf so GCC doesn't
 * turn printf calls into puts/putchar. */
int vprintf(const char* fmt, va_list args)
{
    char line[LOG_LINE_MAX];
    int len = vsnprintf(line, sizeof line, fmt, args);
    if (len < 0)
    {
        return len;
    }
    if ((size_t)len >= sizeof line)
    {
        // Truncated: keep the line ending so the Pi still splits lines correctly.
        len = sizeof line - 1;
        line[len - 2] = '\r';
        line[len - 1] = '\n';
    }
    uart_log_write(line, (size_t)len);
    return len;
}

int printf(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    const int len = vprintf(fmt, args);
    va_end(args);
    return len;
}

/* Everything else that still writes to stdout (puts, fwrite, libraries) ends up here. */
int _write(int file, char* ptr, int len)
{
    (void)file;
    uart_log_write(ptr, (size_t)len);
    return len;
}
