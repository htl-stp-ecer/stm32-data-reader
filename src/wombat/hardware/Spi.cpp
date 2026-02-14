/* spi.c – Pure-C port of the original C++ Spi wrapper
 * ---------------------------------------------------
 * Maintainer: Tobias <tobias.madlberger@gmail.com>
 * Last update: 2026-01-03
 * Updated to use struct-based protocol format
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

#include "spdlog/spdlog.h"
#include "spi/pi_buffer.h"

#ifndef SPI_DEVICE
#define SPI_DEVICE "/dev/spidev0.0"
#endif

#define SERVO_MINIMUM_DUTYCYCLE 300u

typedef struct
{
    int fd;
    struct spi_ioc_transfer tr;
    RxBuffer tx; // What we send to STM32 (commands)
    TxBuffer rx; // What we receive from STM32 (sensor data)
    uint32_t speed_hz;
} SpiCtx;

static SpiCtx ctx = {
    .fd = -1,
    .speed_hz = 20000000,
};

static bool allowUpdates = false;

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

    ctx.tx.transferVersion = TRANSFER_VERSION;
    memset(&ctx.tr, 0, sizeof(ctx.tr));
    ctx.tr.tx_buf = (unsigned long)&ctx.tx;
    ctx.tr.rx_buf = (unsigned long)&ctx.rx;
    ctx.tr.len = BUFFER_LENGTH_DUPLEX_COMMUNICATION;
    ctx.tr.speed_hz = ctx.speed_hz;
    ctx.tr.bits_per_word = bits;

    return true;
}

static bool spi_do_transfer(void)
{
    const uint32_t pending_updates = ctx.tx.updates;
    struct timespec ts_before, ts_after;
    if (pending_updates & PI_BUFFER_UPDATE_MOTOR_CMD)
        clock_gettime(CLOCK_MONOTONIC, &ts_before);

    if (ioctl(ctx.fd, SPI_IOC_MESSAGE(1), &ctx.tr) < 0)
        return false;

    if (pending_updates & PI_BUFFER_UPDATE_MOTOR_CMD)
    {
        clock_gettime(CLOCK_MONOTONIC, &ts_after);
        const long elapsed_us = (ts_after.tv_sec - ts_before.tv_sec) * 1000000L
            + (ts_after.tv_nsec - ts_before.tv_nsec) / 1000L;
        const uint32_t stm_time = ctx.rx.updateTime;
        SPDLOG_INFO("[TIMING] spi_ioctl motor_cmd elapsed_us={} stm32_update_us={}",
                    elapsed_us, stm_time);
    }

    // Clear update flags after transmission so subsequent sensor-only
    // transfers don't re-trigger actuator updates on the STM32
    ctx.tx.updates = 0;
    return ctx.rx.transferVersion == TRANSFER_VERSION;
}

const TxBuffer* get_rx_buffer(void)
{
    return &ctx.rx;
}

bool spi_init(uint32_t speed_hz)
{
    ctx.speed_hz = speed_hz;
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

    const int max_tries = 3;
    for (int t = 0; t < max_tries; ++t)
    {
        if (spi_do_transfer())
            return true;

        fprintf(stderr, "[spi] transfer-version mismatch; Received: %d, Expected: %d (attempt %d/%d).\n",
                ctx.rx.transferVersion,
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

void set_shutdown_flag(uint8_t bit, bool value)
{
    uint8_t prev = ctx.tx.systemShutdown;
    if (value)
        ctx.tx.systemShutdown |= (1u << bit);
    else
        ctx.tx.systemShutdown &= ~(1u << bit);

    if (ctx.tx.systemShutdown == prev)
        return;

    // Shutdown affects both servo and motor processing on the firmware side
    ctx.tx.updates |= PI_BUFFER_UPDATE_SERVO_CMD | PI_BUFFER_UPDATE_MOTOR_CMD;
    if (!spi_update())
        exit(EXIT_FAILURE);
}

static void set_motor_control_mode(uint8_t port, MotorControlMode mode)
{
    uint8_t newCtlMode = (ctx.tx.motorControlMode & ~(0b11u << (port * 2))) | ((uint8_t)mode << (port * 2));
    ctx.tx.motorControlMode = newCtlMode;
}

void set_motor(uint8_t port, MotorDir dir, uint32_t value)
{
    if (port > 3)
        return;
    uint8_t newDir = (ctx.tx.motorDirection & ~(0b11u << (port * 2))) | ((uint8_t)dir << (port * 2));
    set_motor_control_mode(port, MOTOR_CTL_PWM);
    ctx.tx.motorDirection = newDir;
    ctx.tx.motorTarget[port] = (int32_t)value;
    ctx.tx.updates |= PI_BUFFER_UPDATE_MOTOR_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_motor_velocity(uint8_t port, int32_t velocity)
{
    if (port > 3)
        return;
    set_motor_control_mode(port, MOTOR_CTL_MAV);
    ctx.tx.motorTarget[port] = velocity;
    ctx.tx.updates |= PI_BUFFER_UPDATE_MOTOR_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_motor_position(uint8_t port, int32_t velocity, int32_t goal_position)
{
    if (port > 3)
        return;
    set_motor_control_mode(port, MOTOR_CTL_MTP);
    ctx.tx.motorTarget[port] = velocity;
    ctx.tx.motorGoalPosition[port] = goal_position;
    ctx.tx.updates |= PI_BUFFER_UPDATE_MOTOR_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_motor_relative(uint8_t port, int32_t velocity, int32_t delta_position)
{
    if (port > 3)
        return;
    set_motor_control_mode(port, MOTOR_CTL_MRP);
    ctx.tx.motorTarget[port] = velocity;
    ctx.tx.motorGoalPosition[port] = delta_position;
    ctx.tx.updates |= PI_BUFFER_UPDATE_MOTOR_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_servo_mode(uint8_t port, ServoMode mode)
{
    if (port > 3)
        return;
    uint8_t bitPos = port * 2;
    ctx.tx.servoMode = (ctx.tx.servoMode & ~(0b11u << bitPos)) | (((uint8_t)mode & 0b11u) << bitPos);
    ctx.tx.updates |= PI_BUFFER_UPDATE_SERVO_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

uint16_t get_servo_pos(uint8_t port)
{
    if (port > 3)
        return 0;
    uint16_t pos = ctx.tx.servoPos[port];
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
    ctx.tx.servoPos[port] = val;
    ctx.tx.updates |= PI_BUFFER_UPDATE_SERVO_CMD;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

uint32_t last_update_us(void)
{
    return ctx.rx.updateTime;
}

// Gyroscope data access
float gyroX(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.gyro.data[0];
}

float gyroY(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.gyro.data[1];
}

float gyroZ(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.gyro.data[2];
}

// Accelerometer data access
float accelX(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.accel.data[0];
}

float accelY(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.accel.data[1];
}

float accelZ(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.accel.data[2];
}

// Magnetometer data access
float magX(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.compass.data[0];
}

float magY(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.compass.data[1];
}

float magZ(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.compass.data[2];
}

// Quaternion data access (note: struct order is w, x, y, z)
float quatX(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.quat.data[1];
}

float quatY(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.quat.data[2];
}

float quatZ(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.quat.data[3];
}

float quatW(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.quat.data[0];
}

// Linear acceleration data access (gravity removed)
float linearAccelX(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.linearAccel.data[0];
}

float linearAccelY(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.linearAccel.data[1];
}

float linearAccelZ(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.linearAccel.data[2];
}

int8_t linear_accel_accuracy(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.linearAccel.accuracy;
}

float imuTemperature(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.temperature;
}

int32_t bemf(uint8_t mot)
{
    if (mot > 3)
        return 0;
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.motorBemf[mot];
}

int32_t get_motor_position(uint8_t port)
{
    if (port > 3)
        return 0;
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.motorPosition[port];
}

uint8_t get_motor_done(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.motorDone;
}

uint16_t analog_in(uint8_t idx)
{
    if (idx > 5)
        return 0;
    if (!spi_update())
        exit(EXIT_FAILURE);
    // analogSensor is now int16_t, cast to uint16_t for compatibility
    return (uint16_t)ctx.rx.analogSensor[idx];
}

uint16_t digital_raw(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.digitalSensors;
}

bool digital(uint8_t bit)
{
    return (bit < 16) && (digital_raw() & (1u << bit));
}

float battery_voltage(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    // batteryVoltage is now int16_t
    int16_t v = ctx.rx.batteryVoltage;

    const float stmVoltage = 3.3f; // Voltage of the STM32 ADC reference
    const float voltageDeviderFactor = 11.0f; // Voltage divider factor (110k:10k resistors)
    const float adcResulution = 4096.0f; // 12-bit ADC resolution

    const float voltage = (float)v * stmVoltage * voltageDeviderFactor / adcResulution;

    static float filtered_voltage = 0.0f;
    float alpha = 0.0001f; // Smoothing factor
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

int8_t gyro_accuracy(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.gyro.accuracy;
}

int8_t accel_accuracy(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.accel.accuracy;
}

int8_t compass_accuracy(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.compass.accuracy;
}

int8_t quaternion_accuracy(void)
{
    if (!spi_update())
        exit(EXIT_FAILURE);
    return ctx.rx.imu.quat.accuracy;
}

void set_motor_pid(uint8_t port, float kp, float ki, float kd)
{
    if (port > 3)
        return;
    ctx.tx.motorPidSettings.pids[port].Kp = kp;
    ctx.tx.motorPidSettings.pids[port].Ki = ki;
    ctx.tx.motorPidSettings.pids[port].Kd = kd;
    ctx.tx.updates |= PI_BUFFER_UPDATE_MOTOR_PID;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_bemf_scale(uint8_t port, float scale)
{
    if (port > 3)
        return;
    ctx.tx.bemfScale[port] = scale;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_bemf_offset(uint8_t port, float offset)
{
    if (port > 3)
        return;
    ctx.tx.bemfOffset[port] = offset;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}

void set_bemf_nominal_voltage(int16_t adc_value)
{
    ctx.tx.nominalVoltageAdc = adc_value;
    if (!spi_force_update())
        exit(EXIT_FAILURE);
}