/* spi.c – Pure-C port of the original C++ Spi wrapper
 * ---------------------------------------------------
 * Maintainer: Tobias <tobias.madlberger@gmail.com>
 * Last update: 2025-07-06
 */

#include "wombat/hardware/Spi.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <fcntl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "spi/pi_buffer.h"

#ifndef SPI_DEVICE
#define SPI_DEVICE "/dev/spidev0.0"
#endif

#define SPI_MINIMUM_UPDATE_DELAY_MS 20u
#define SERVO_MINIMUM_DUTYCYCLE 300u

typedef struct
{
    int fd;
    struct spi_ioc_transfer tr;
    uint8_t tx[BUFFER_LENGTH_DUPELX_COMMUNICATION];
    uint8_t rx[BUFFER_LENGTH_DUPELX_COMMUNICATION];
    uint32_t speed_hz;
    uint64_t last_call_ms;
} SpiCtx;

static SpiCtx ctx = {
    .fd = -1,
    .speed_hz = 20000000,
    .last_call_ms = 0,
};

static bool allowUpdates = false;

static inline uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static void reset_stm(void)
{
    int rc = system("bash ~/flashFiles/wallaby_reset_coproc");
    if (rc != 0)
    {
        fprintf(stderr, "[spi] STM reset script failed (%d).  Power-cycle the Wombat.\n", rc);
        exit(EXIT_FAILURE);
    }
    usleep(1 * 1000 * 1000);
}

static bool spi_reopen(void)
{
    if (ctx.fd >= 0)
    {
        close(ctx.fd);
        ctx.fd = -1;
    }

    ctx.fd = open(SPI_DEVICE, O_RDWR | O_CLOEXEC);
    if (ctx.fd < 0)
    {
        perror("[spi] open");
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(ctx.fd, SPI_IOC_WR_MODE, &mode) || ioctl(ctx.fd, SPI_IOC_WR_BITS_PER_WORD, &bits) ||
        ioctl(ctx.fd, SPI_IOC_WR_MAX_SPEED_HZ, &ctx.speed_hz))
    {
        perror("[spi] ioctl");
        close(ctx.fd);
        ctx.fd = -1;
        return false;
    }

    ctx.tx[TX_TRANSFER_VERSION] = TRANSFER_VERSION;
    memset(&ctx.tr, 0, sizeof(ctx.tr));
    ctx.tr.tx_buf = (unsigned long)ctx.tx;
    ctx.tr.rx_buf = (unsigned long)ctx.rx;
    ctx.tr.len = BUFFER_LENGTH_DUPELX_COMMUNICATION;
    ctx.tr.speed_hz = ctx.speed_hz;
    ctx.tr.bits_per_word = bits;

    return true;
}

static bool spi_do_transfer(void)
{
    if (ioctl(ctx.fd, SPI_IOC_MESSAGE(1), &ctx.tr) < 0)
        return false;
    return ctx.rx[RX_TRANSFER_VERSION] == TRANSFER_VERSION;
}

bool spi_init(uint32_t speed_hz)
{
    ctx.speed_hz = speed_hz;
    ctx.last_call_ms = now_ms() - SPI_MINIMUM_UPDATE_DELAY_MS;
    return spi_reopen();
}

void set_spi_mode(bool mode)
{
    allowUpdates = mode;
}

bool spi_update(void)
{
    if (!allowUpdates)
    {
        fprintf(stderr, "[spi] SPI updates are disabled.\n");
        return true;
    }
    if (ctx.fd < 0)
        return false;

    uint64_t now = now_ms();
    if (now - ctx.last_call_ms < SPI_MINIMUM_UPDATE_DELAY_MS)
        return ctx.rx[RX_TRANSFER_VERSION] == TRANSFER_VERSION;

    ctx.last_call_ms = now;

    const int max_tries = 3;
    for (int t = 0; t < max_tries; ++t)
    {
        if (spi_do_transfer())
            return true;

        fprintf(stderr, "[spi] transfer-version mismatch; Received: %d, Expected: %d (attempt %d/%d).\n",
                ctx.rx[RX_TRANSFER_VERSION],
                TRANSFER_VERSION,
                t + 1, max_tries);

        if (t == max_tries - 1)
        {
            fprintf(stderr, "[spi] fatal – unable to recover SPI link.\n");
            exit(EXIT_FAILURE);
        }

        if (t > 0)
        {
            fprintf(stderr, "[spi] resetting STM …\n");
            reset_stm();
        }

        if (!spi_reopen())
        {
            fprintf(stderr, "[spi] failed to reopen spidev.\n");
            exit(EXIT_FAILURE);
        }
    }
    return false;
}

bool spi_force_update(void)
{
    usleep(SPI_MINIMUM_UPDATE_DELAY_MS * 1000);
    return spi_update();
}

void spi_close(void)
{
    if (ctx.fd >= 0)
    {
        close(ctx.fd);
        ctx.fd = -1;
    }
}

static inline uint8_t* spi_tx(void)
{
    return ctx.tx;
}

static inline const uint8_t* spi_rx(void)
{
    return ctx.rx;
}

void set_shutdown_flag(uint8_t bit, bool value)
{
    uint8_t* tx = spi_tx();
    if (value)
        tx[TX_SYSTEM_SHUTDOWN] |= (1u << bit);
    else
        tx[TX_SYSTEM_SHUTDOWN] &= ~(1u << bit);

    if (!spi_update())
        exit(EXIT_FAILURE);
}

void set_motor(uint8_t port, MotorDir dir, uint32_t value)
{
    if (port > 3)
        return;
    uint8_t* tx = spi_tx();
    tx[TX_MOT_MODE] = (tx[TX_MOT_MODE] & ~(0b11u << (port * 2))) | ((uint8_t)dir << (port * 2));
    memcpy(&tx[TX_SPEED_POS_MOT_0 + port * 4], &value, sizeof value);
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_servo_mode(uint8_t port, ServoMode mode)
{
    if (port > 3)
        return;
    uint8_t* tx = spi_tx();
    uint8_t bitPos = port * 2;
    tx[TX_SERVO_MODE] &= ~(0b11u << bitPos);
    tx[TX_SERVO_MODE] |= (((uint8_t)mode & 0b11u) << bitPos);
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

uint16_t get_servo_pos(uint8_t port)
{
    if (port > 3)
        return 0;
    uint16_t pos;
    memcpy(&pos, &spi_tx()[TX_POS_SERVO_0 + port * 2], sizeof pos);
    double degrees = ((double)pos - 1500.0) / 10.0;
    double dval = (degrees + 90.0) * 2047.0 / 180.0;
    if (dval < 0.0)
        dval = 0.0;
    if (dval > 2047.)
        dval = 2047.01;
    return (uint16_t)(dval + 0.5);
}

void set_servo_pos(uint8_t port, uint16_t raw)
{
    if (port > 3)
        return;
    unsigned short val = 1500 + (unsigned short)round(1800.0 * ((double)raw / 2047.0)) - 900;
    memcpy(&spi_tx()[TX_POS_SERVO_0 + port * 2], &val, sizeof val);
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

uint32_t last_update_us(void)
{
    uint32_t v;
    memcpy(&v, &spi_rx()[RX_UPDATE_TIME], sizeof v);
    return v;
}

#define SENSOR_TRIPLE(name, base)                   \
    float name##X(void) {                           \
        if (!spi_update())                          \
            exit(EXIT_FAILURE);                     \
        float f;                                    \
        memcpy(&f, &spi_rx()[base + 0], sizeof(f)); \
        return f;                                   \
    }                                               \
    float name##Y(void) {                           \
        if (!spi_update())                          \
            exit(EXIT_FAILURE);                     \
        float f;                                    \
        memcpy(&f, &spi_rx()[base + 4], sizeof(f)); \
        return f;                                   \
    }                                               \
    float name##Z(void) {                           \
        if (!spi_update())                          \
            exit(EXIT_FAILURE);                     \
        float f;                                    \
        memcpy(&f, &spi_rx()[base + 8], sizeof(f)); \
        return f;                                   \
    }

SENSOR_TRIPLE(gyro, RX_GYRO_X)
SENSOR_TRIPLE(accel, RX_ACCEL_X)
SENSOR_TRIPLE(mag, RX_MAG_X)
#undef SENSOR_TRIPLE

float imuTemperature(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    float f;
    memcpy(&f, &spi_rx()[RX_IMU_TEMPERATUR], sizeof(f));
    return f;
}

int32_t bemf(uint8_t mot)
{
    if (mot > 3)
        return 0;
    if (!spi_update())
        exit(EXIT_FAILURE);
    int32_t v;
    memcpy(&v, &spi_rx()[RX_BEMF_READING_MOT0 + mot * 4], sizeof v);
    return v / 250;
}

uint16_t analog_in(uint8_t idx)
{
    if (idx > 5)
        return 0;
    if (!spi_update())
        exit(EXIT_FAILURE);
    uint16_t v;
    memcpy(&v, &spi_rx()[RX_ANALOG_SENSOR_0 + idx * 2], sizeof v);
    return v;
}

uint16_t digital_raw(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    uint16_t v;
    memcpy(&v, &spi_rx()[RX_DIGITAL_VALUES], sizeof v);
    return v;
}

bool digital(uint8_t bit)
{
    return (bit < 16) && (digital_raw() & (1u << bit));
}

float battery_voltage(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    uint16_t v;
    memcpy(&v, &spi_rx()[RX_BATTERY_VOLTAGE], sizeof v);

    const float stmVoltage = 3.3f; // Voltage of the STM32 ADC reference
    const float voltageDeviderFactor = 11.0f; // Voltage divider factor (110k:10k resistors)
    const float adcResulution = 4096.0f; // 12-bit ADC resolution

    const float voltage = (float)v * stmVoltage * voltageDeviderFactor / adcResulution;

    static float filtered_voltage = 0.0f;
    float alpha = 0.1f; // Smoothing factor
    if (filtered_voltage == 0.0f)
    {
        filtered_voltage = voltage;
    }
    else
    {
        filtered_voltage = filtered_voltage * (1 - alpha) + voltage * alpha;
    }

    return filtered_voltage;
}
