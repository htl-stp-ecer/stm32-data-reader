/* spi.h – Public API for Raspberry-Pi <-> STM32 SPI link
* -----------------------------------------------------
 * Generated 2025-07-06 (split from the original monolithic spi.c).
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
void set_motor_off(uint8_t port);
void set_motor_brake(uint8_t port);
void set_motor_pwm(uint8_t port, int32_t duty); /* duty: -400..400, sign = direction */
void set_motor_velocity(uint8_t port, int32_t velocity);
void set_motor_position(uint8_t port, int32_t velocity, int32_t goal_position);
void set_servo_mode(uint8_t port, ServoMode mode);
void set_servo_pos(uint8_t port, uint16_t microseconds);

/* Motor PID settings */
void set_motor_pid(uint8_t port, float kp, float ki, float kd);

/* Odometry: send kinematics config to STM32 */
void set_kinematics_config(const float inv_matrix[3][4], const float ticks_to_rad[4]);

/* Odometry: request STM32 to reset its integrated pose */
void reset_stm32_odometry(void);

/* ---------------- bulk read (RX) -------------------- */
/* Returns pointer to the internal RX buffer (last received data).
 * Does NOT trigger an SPI transfer – call spi_update() first. */
const struct TxBuffer_tag* get_rx_buffer(void);

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

float linearAccelX(void);
float linearAccelY(void);
float linearAccelZ(void);
int8_t linear_accel_accuracy(void);

float imuTemperature(void);

int32_t bemf(uint8_t mot);
int32_t get_motor_position(uint8_t port);
uint8_t get_motor_done(void);
uint16_t analog_in(uint8_t idx);
uint16_t digital_raw(void);
bool digital(uint8_t bit);

float battery_voltage(void);

/* Odometry data (computed on STM32) */
float odom_pos_x(void);
float odom_pos_y(void);
float odom_heading(void);
float odom_vx(void);
float odom_vy(void);
float odom_wz(void);

int8_t gyro_accuracy(void);
int8_t accel_accuracy(void);
int8_t compass_accuracy(void);
int8_t quaternion_accuracy(void);

#ifdef __cplusplus
} /* extern "C" */
#endif