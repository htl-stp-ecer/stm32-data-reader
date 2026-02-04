/* spi.h – Public API for Raspberry‑Pi ⇆ STM32 SPI link
* -----------------------------------------------------
 * Generated 2025‑07‑06 (split from the original monolithic spi.c).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------- */
/*  lifecycle                                           */
/* ---------------------------------------------------- */

bool spi_init(uint32_t speed_hz);
bool spi_update(void);
void set_spi_mode(bool);
bool spi_force_update(void);
void spi_close(void);

/* ---------------------------------------------------- */
/*  typed helpers                                       */
/* ---------------------------------------------------- */

typedef enum
{
    MOTOR_DIR_OFF = 0,
    MOTOR_DIR_CCW = 1,
    MOTOR_DIR_CW = 2,
    MOTOR_DIR_BRAKE = 3  /* Active braking - short-circuits motor windings */
} MotorDir;

typedef enum
{
    SERVO_MODE_FULLY_DISABLED = 0,
    SERVO_MODE_DISABLED = 1,
    SERVO_MODE_ENABLED = 2
} ServoMode;

/* Shutdown flags used with set_shutdown_flag() */
enum
{
    SHUTDOWN_SERVO_FLAG = 0,
    SHUTDOWN_MOTOR_FLAG = 1,
};

/* ---------------- setters (TX) ---------------------- */
void set_shutdown_flag(uint8_t bit, bool value);
void set_motor(uint8_t port, MotorDir dir, uint32_t value);
void set_servo_mode(uint8_t port, ServoMode mode);
void set_servo_pos(uint8_t port, uint16_t raw /* 0‑2047 */);

/* BEMF calibration setters */
void set_bemf_scale(uint8_t port, float scale);
void set_bemf_offset(uint8_t port, float offset);
void set_bemf_nominal_voltage(int16_t adc_value);

/* ---------------- getters (RX) ---------------------- */
uint16_t get_servo_pos(uint8_t port);
uint32_t last_update_us(void);

float gyroX(void);
float gyroY(void);
float gyroZ(void);

float accelX(void);
float accelY(void);
float accelZ(void);

float magX(void);
float magY(void);
float magZ(void);

float quatX(void);
float quatY(void);
float quatZ(void);
float quatW(void);

float imuTemperature(void);

int32_t bemf(uint8_t mot);
uint16_t analog_in(uint8_t idx);
uint16_t digital_raw(void);
bool digital(uint8_t bit);

float battery_voltage(void);

int8_t gyro_accuracy(void);
int8_t accel_accuracy(void);
int8_t compass_accuracy(void);
int8_t quaternion_accuracy(void);

#ifdef __cplusplus
} /* extern "C" */
#endif
