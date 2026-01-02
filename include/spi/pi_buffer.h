//
// Created by matthias on 5/23/25.
//

#ifndef PI_BUFFER_INV_H
#define PI_BUFFER_INV_H

#define TRANSFER_VERSION 2

#define RX_BUFFER_LENGTH 97 //bytes
#define TX_BUFFER_LENGTH 32 //bytes

#define BUFFER_LENGTH_DUPELX_COMMUNICATION ((TX_BUFFER_LENGTH < RX_BUFFER_LENGTH) ? RX_BUFFER_LENGTH : TX_BUFFER_LENGTH)

/* <--- RX ---> */
// The version describes what kind of communication is used (uint8)
#define RX_TRANSFER_VERSION        0

// time when the buffer was last updated (uint32_t)
#define RX_UPDATE_TIME                 1

// BEMF readings (4 x uint32_t)
#define RX_BEMF_READING_MOT0       5
#define RX_BEMF_READING_MOT1       9
#define RX_BEMF_READING_MOT2       13
#define RX_BEMF_READING_MOT3       17

// Analog Values (6 x uint16)
#define RX_ANALOG_SENSOR_0         21
#define RX_ANALOG_SENSOR_1         23
#define RX_ANALOG_SENSOR_2         25
#define RX_ANALOG_SENSOR_3         27
#define RX_ANALOG_SENSOR_4         29
#define RX_ANALOG_SENSOR_5         31

// Battery Voltage (1 x uint16)
#define RX_BATTERY_VOLTAGE         33

// Digital Values (1 x uint16)
#define RX_DIGITAL_VALUES          35

// IMU Measurements (12 x float)
#define RX_GYRO_X                37
#define RX_GYRO_Y                41
#define RX_GYRO_Z                45

#define RX_ACCEL_X               49
#define RX_ACCEL_Y               53
#define RX_ACCEL_Z               57

#define RX_MAG_X                 61
#define RX_MAG_Y                 65
#define RX_MAG_Z                 69

#define RX_QUATERNION_X         73
#define RX_QUATERNION_Y         77
#define RX_QUATERNION_Z         81
#define RX_QUATERNION_W         85

// Temperature (1 x float)
#define RX_IMU_TEMPERATUR              89

//Accuracy of MPL data (4x int8_t)
#define RX_GYRO_ACCURACY         93
#define RX_ACCEL_ACCURACY        94
#define RX_COMPASS_ACCURACY      95
#define RX_QUATERNION_ACCURACY   96

/*----------------------------------------------------------------------------------------------------*/

/* <--- TX ---> */
// The Version describes what kind of communication is used (uint8)
#define TX_TRANSFER_VERSION        0

// Update flags (uint32_t)
#define TX_UPDATES                 1

// Shutdown command (bool = 1 byte)
#define TX_SYSTEM_SHUTDOWN         5

// Motor Mode (uint8)
#define TX_MOT_MODE                6

// Motor Speeds/Positions (4 x uint32_t)
#define TX_SPEED_POS_MOT_0         7
#define TX_SPEED_POS_MOT_1         11
#define TX_SPEED_POS_MOT_2         15
#define TX_SPEED_POS_MOT_3         19

// Servo Mode (uint8)
#define TX_SERVO_MODE              23

// Servo Positions (4 x uint16)
#define TX_POS_SERVO_0             24
#define TX_POS_SERVO_1             26
#define TX_POS_SERVO_2             28
#define TX_POS_SERVO_3             30

/*----------------------------------------------------------------------------------------------------*/

#endif //PI_BUFFER_INV_H
