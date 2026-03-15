//
// Created by matthias on 5/23/25.
// Updated to struct-based protocol format
// Shared protocol header — single source of truth for STM32 <-> Pi SPI protocol
//

#ifndef PI_BUFFER_INV_H
#define PI_BUFFER_INV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRANSFER_VERSION 15

#define PI_BUFFER_UPDATE_MOTOR_PID_SPEED 0x01
#define PI_BUFFER_UPDATE_MOTOR_PID_POS   0x02
#define PI_BUFFER_UPDATE_IMU_ORIENTATION 0x04
#define PI_BUFFER_UPDATE_SAVE_IMU_CAL    0x08
#define PI_BUFFER_UPDATE_KINEMATICS      0x10
#define PI_BUFFER_UPDATE_ODOM_RESET      0x20

#define SHUTDOWN_SERVO 0x01
#define SHUTDOWN_MOTOR 0x02

#define MOTOR_CONTR_MOD_LENGTH 3 // Bits per motor in motorControlMode

enum MOTOR_CMD_MODE
{
    MOT_MODE_OFF = 0b000,
    MOT_MODE_PASSIV_BRAKE = 0b001,
    MOT_MODE_PWM = 0b010,
    MOT_MODE_MAV = 0b011, // Move At Velocity - PID velocity control
    MOT_MODE_MTP = 0b100, // Move To Position - PID position control (absolute)
};

typedef struct __attribute__ ((packed))
{
    /* --- { x, y, z } --- */
    float data[3];
    /* Accuracy of the measurement from 0 (least accurate) to 3 (most accurate). */
    int8_t accuracy;
}

SensorData;

typedef struct __attribute__ ((packed))
{
    /* --- { w, x, y, z } --- */
    float data[4];
    /* Accuracy of the measurement from 0 (least accurate) to 3 (most accurate). */
    int8_t accuracy;
}

QuaternionData;

typedef struct __attribute__ ((packed))
{
    SensorData gyro;
    SensorData accel;
    SensorData compass;
    SensorData linearAccel;
    SensorData accelVelocity;
    QuaternionData dmpQuat; /* DMP 6-axis quaternion (gyro+accel only) */
    float heading; /* MPL heading (mag-corrected when calibrated) */
    float temperature;
}

ImuData;

typedef struct __attribute__ ((packed))
{
    int32_t bemf[4]; // Instantaneous filtered BEMF reading per motor
    int32_t position[4]; // Motor position (accumulated BEMF ticks)
    uint8_t done; // Bit N set when motor N reached position goal
}

MotorData;

typedef struct __attribute__ ((packed))
{
    float pos_x; // meters, world frame
    float pos_y; // meters, world frame
    float heading; // radians, CCW-positive (library/ENU convention)
    float vx; // m/s, body frame
    float vy; // m/s, body frame
    float wz; // rad/s, body frame
}

OdometryData;

typedef struct __attribute__ ((packed))
{
    /* Inverse kinematics matrix: wheel speeds (rad/s) -> [vx, vy, wz]
     * Pre-baked with wheel radius and geometry constants.
     * Row 0: vx coefficients for [fl, fr, bl, br]
     * Row 1: vy coefficients
     * Row 2: wz coefficients */
    float inv_matrix[3][4];

    /* Per-motor encoder calibration: radians per BEMF tick */
    float ticks_to_rad[4];
}

KinematicsConfig;

typedef struct __attribute__ ((packed)) TxBuffer_tag
{
    /* --- Version of SPI buffer --- */
    uint8_t transferVersion;
    /* -- last transfer --- */
    uint32_t updateTime;

    /* --- MOTOR DATA --- */
    MotorData motor;

    /* --- SENSOR PORTS / BATTERY VOLTAGE READING --- */
    int16_t analogSensor[6];
    int16_t batteryVoltage;
    uint16_t digitalSensors;
    //bits from 0 to 9 represent the state of a digital Port; bit 10 represents the state of the built in button

    /* --- IMU DATA --- */
    ImuData imu;

    /* --- ODOMETRY (computed on STM32 from BEMF + IMU) --- */
    OdometryData odometry;
}

TxBuffer;

typedef struct __attribute__ ((packed))
{
    float Kp;
    float Ki;
    float Kd;
}

BasicPidSettings;

typedef struct __attribute__ ((packed))
{
    //global PID settings
    float limMin;
    float limMax;
    float limMinInt;
    float limMaxInt;
    float tau;

    //motor specific PID settings
    BasicPidSettings pids[4];
}

MotorPidSettings;

typedef struct __attribute__ ((packed))
{
    /* --- Version of SPI buffer --- */
    uint8_t transferVersion;

    /* --- UPDATE FLAGS / which data of the buffer is new and should be processed --- */
    uint32_t updates;

    /* --- SHUTDOWN CMD / every bit represents a system that can be turned off --- */
    uint8_t systemShutdown;

    /* --- MOTOR CONTROL MODE (3 bits per motor) --- */
    uint16_t motorControlMode; // OFF/PASSIV_BRAKE/PWM/MAV/MTP

    /* --- MOTOR TARGET --- */
    int32_t motorTarget[4]; // PWM: duty (0-400), MAV: velocity goal, MTP: speed limit

    /* --- MOTOR GOAL POSITION (for MTP mode) --- */
    int32_t motorGoalPosition[4]; // target position in accumulated BEMF ticks

    /* --- SERVO MODE --- */
    uint8_t servoMode;
    uint16_t servoPos[4];

    /* --- MOTOR PID SETTINGS (configurable from Pi) --- */
    MotorPidSettings motorPidSettings;

    /* --- IMU ORIENTATION MATRICES (configurable from Pi) --- */
    // Row-major 3x3 signed char matrices, each element is -1, 0, or 1
    int8_t imuGyroOrientation[9]; // Gyro/accel chip-to-board mapping
    int8_t imuCompassOrientation[9]; // Compass chip-to-board mapping

    /* --- KINEMATICS CONFIG (sent once at startup from Pi) --- */
    KinematicsConfig kinematics;
}

RxBuffer;

#define BUFFER_LENGTH_DUPLEX_COMMUNICATION ((sizeof(TxBuffer) < sizeof(RxBuffer)) ? sizeof(RxBuffer) : sizeof(TxBuffer))

#ifdef __cplusplus
}
#endif

#endif //PI_BUFFER_INV_H