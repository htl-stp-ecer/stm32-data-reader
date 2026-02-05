//
// Created by matthias on 5/23/25.
// Updated to struct-based protocol format
//

#ifndef PI_BUFFER_INV_H
#define PI_BUFFER_INV_H

#include <stdint.h>

#define TRANSFER_VERSION 3

#define PI_BUFFER_UPDATE_SERVO_CMD 0x01
#define PI_BUFFER_UPDATE_MOTOR_CMD 0x02

typedef struct __attribute__((packed))
{
    /* --- { x, y, z } --- */
    float data[3];
    /* Accuracy of the measurement from 0 (least accurate) to 3 (most accurate). */
    int8_t accuracy;
} SensorData;

typedef struct __attribute__((packed))
{
    /* --- { w, x, y, z } --- */
    float data[4];
    /* Accuracy of the measurement from 0 (least accurate) to 3 (most accurate). */
    int8_t accuracy;
} QuaternionData;

typedef struct __attribute__((packed))
{
    SensorData gyro;
    SensorData accel;
    SensorData compass;
    SensorData linearAccel;
    QuaternionData quat;
    float temperature;
} ImuData;

typedef struct __attribute__((packed))
{
    /* --- Version of SPI buffer --- */
    uint8_t transferVersion;
    /* -- last transfer --- */
    uint32_t updateTime;

    /* --- BEMF reading of Motors --- */
    int32_t motorBemfSum[4];

    /* --- SENSOR PORTS / BATTERY VOLTAGE READING --- */
    int16_t analogSensor[6];
    int16_t batteryVoltage;
    uint16_t digitalSensors;
    //bits from 0 to 9 represent the state of a digital Port; bit 10 represents the state of the built in button

    /* --- IMU DATA --- */
    ImuData imu;
} TxBuffer;


typedef struct __attribute__((packed))
{
    /* --- Version of SPI buffer --- */
    uint8_t transferVersion;

    /* --- UPDATE FLAGS / which data of the buffer is new and should be processed --- */
    uint32_t updates;

    /* --- SHUTDOWN CMD / every bit represents a system that can be turned off --- */
    uint8_t systemShutdown;

    /* --- MOTOR MODE --- */
    uint8_t motorMode;
    //represents either speed or position
    uint32_t motorSpeedPos[4];

    /* --- SERVO MODE --- */
    uint8_t servoMode;
    uint16_t servoPos[4];

    /* --- BEMF CALIBRATION (per motor) --- */
    // Formula: calibrated = (raw * batteryScale) * bemfScale + bemfOffset
    // Set bemfScale=1.0 and bemfOffset=0.0 for uncalibrated (default)
    float bemfScale[4];      // Multiplier per motor (default: 1.0)
    float bemfOffset[4];     // Offset per motor (default: 0.0)
    int16_t nominalVoltageAdc; // Nominal battery voltage in ADC counts (default: 3000)
} RxBuffer;

#define BUFFER_LENGTH_DUPLEX_COMMUNICATION ((sizeof(TxBuffer) < sizeof(RxBuffer)) ? sizeof(RxBuffer) : sizeof(TxBuffer))

#endif //PI_BUFFER_INV_H
