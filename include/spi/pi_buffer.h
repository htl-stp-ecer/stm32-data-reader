//
// Created by matthias on 5/23/25.
// Updated to struct-based protocol format
//

#ifndef PI_BUFFER_INV_H
#define PI_BUFFER_INV_H

#include <stdint.h>

#define TRANSFER_VERSION 5

#define PI_BUFFER_UPDATE_SERVO_CMD 0x01
#define PI_BUFFER_UPDATE_MOTOR_CMD 0x02
#define PI_BUFFER_UPDATE_MOTOR_PID 0x04

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
    QuaternionData quat;
    float temperature;
}

ImuData;

typedef struct __attribute__ ((packed)) TxBuffer_tag
{
    /* --- Version of SPI buffer --- */
    uint8_t transferVersion;
    /* -- last transfer --- */
    uint32_t updateTime;

    /* --- Instantaneous filtered BEMF reading per motor (not accumulated) --- */
    int32_t motorBemf[4];

    /* --- MOTOR POSITION (accumulated BEMF ticks) --- */
    int32_t motorPosition[4];

    /* --- MOTOR DONE FLAGS (bit N set when motor N reached position goal) --- */
    uint8_t motorDone;

    /* --- SENSOR PORTS / BATTERY VOLTAGE READING --- */
    int16_t analogSensor[6];
    int16_t batteryVoltage;
    uint16_t digitalSensors;
    //bits from 0 to 9 represent the state of a digital Port; bit 10 represents the state of the built in button

    /* --- IMU DATA --- */
    ImuData imu;
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

    /* --- MOTOR DIRECTION --- */
    uint8_t motorDirection; // 2 bits per motor: OFF/CCW/CW/BRAKE

    /* --- MOTOR CONTROL MODE --- */
    uint8_t motorControlMode; // 2 bits per motor: PWM/MAV/MTP/MRP

    /* --- MOTOR TARGET --- */
    int32_t motorTarget[4]; // PWM: duty (0-400), MAV: velocity goal, MTP/MRP: velocity setpoint

    /* --- MOTOR GOAL POSITION (for MTP/MRP modes) --- */
    int32_t motorGoalPosition[4]; // target position in accumulated BEMF ticks

    /* --- SERVO MODE --- */
    uint8_t servoMode;
    uint16_t servoPos[4];

    /* --- BEMF CALIBRATION (per motor) --- */
    // Formula: calibrated = (raw * batteryScale) * bemfScale + bemfOffset
    // Set bemfScale=1.0 and bemfOffset=0.0 for uncalibrated (default)
    float bemfScale[4]; // Multiplier per motor (default: 1.0)
    float bemfOffset[4]; // Offset per motor (default: 0.0)
    int16_t nominalVoltageAdc; // Nominal battery voltage in ADC counts (default: 3000)

    /* --- MOTOR PID SETTINGS (configurable from Pi) --- */
    MotorPidSettings motorPidSettings;
}

RxBuffer;

#define BUFFER_LENGTH_DUPLEX_COMMUNICATION ((sizeof(TxBuffer) < sizeof(RxBuffer)) ? sizeof(RxBuffer) : sizeof(TxBuffer))

#endif //PI_BUFFER_INV_H