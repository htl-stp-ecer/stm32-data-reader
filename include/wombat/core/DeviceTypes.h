//
// Created by tobias on 9/14/25.
// Split from Types.h during architecture restructuring.
//
#pragma once

#include <cstdint>
#include <array>
#include <cmath>

namespace wombat
{
    using PortId = uint8_t;
    using AnalogValue = uint16_t;
    using MotorSpeed = uint32_t;
    using ServoPosition = uint16_t;
    using Timestamp = uint32_t;
    using DigitalValue = uint16_t;

    constexpr PortId MAX_MOTOR_PORTS = 4;
    constexpr PortId MAX_SERVO_PORTS = 4;
    constexpr PortId MAX_ANALOG_PORTS = 6;
    constexpr PortId MAX_DIGITAL_BITS = 16;

    enum class MotorDirection : uint8_t
    {
        Off = 0b00,
        CounterClockwise = 0b01,
        Clockwise = 0b10,
        Brake = 0b11 // Active braking - short-circuits motor windings for fast stopping
    };

    enum class MotorControlMode : uint8_t
    {
        Pwm = 0, // Direct PWM, no PID
        MoveAtVelocity = 1, // PID velocity control
        MoveToPosition = 2, // PID position control (absolute)
        MoveRelativePosition = 3 // PID position control (relative, converted to absolute on STM32)
    };

    enum class ServoMode : uint8_t
    {
        FullyDisabled = 0,
        Disabled = 1,
        Enabled = 2
    };

    struct Vector3f
    {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};

        bool operator==(const Vector3f& other) const noexcept
        {
            constexpr float epsilon = 1e-6f;
            return std::abs(x - other.x) < epsilon &&
                std::abs(y - other.y) < epsilon &&
                std::abs(z - other.z) < epsilon;
        }
    };

    struct Quaternionf
    {
        float w{1.0f};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};

        bool operator==(const Quaternionf& other) const noexcept
        {
            constexpr float epsilon = 1e-6f;
            return std::abs(w - other.w) < epsilon &&
                std::abs(x - other.x) < epsilon &&
                std::abs(y - other.y) < epsilon &&
                std::abs(z - other.z) < epsilon;
        }
    };

    struct ImuAccuracy
    {
        int8_t gyro{0};
        int8_t accelerometer{0};
        int8_t linearAcceleration{0};
        int8_t compass{0};
        int8_t quaternion{0};

        bool operator==(const ImuAccuracy& other) const noexcept
        {
            return gyro == other.gyro &&
                accelerometer == other.accelerometer &&
                linearAcceleration == other.linearAcceleration &&
                compass == other.compass &&
                quaternion == other.quaternion;
        }
    };

    struct MotorState
    {
        MotorDirection direction{MotorDirection::Off};
        MotorControlMode controlMode{MotorControlMode::Pwm};
        MotorSpeed speed{0};
        mutable int32_t backEmf{0};
        int32_t position{0};
        bool done{false};
    };

    struct ServoState
    {
        ServoMode mode{ServoMode::Disabled};
        ServoPosition position{0};
    };

    struct SensorData
    {
        Vector3f gyro{};
        Vector3f accelerometer{};
        Vector3f magnetometer{};
        Vector3f linearAcceleration{};
        Vector3f accelVelocity{};
        Quaternionf orientation{};
        ImuAccuracy accuracy{};
        float temperature{0.0f};
        float batteryVoltage{0.0f};
        std::array<AnalogValue, MAX_ANALOG_PORTS> analogValues{};
        DigitalValue digitalBits{0};
        Timestamp lastUpdate{0};
    };
}