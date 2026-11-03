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
    using ServoPosition = float; // Degrees
    using Timestamp = uint32_t;
    using DigitalValue = uint16_t;

    constexpr PortId MAX_MOTOR_PORTS = 4;
    constexpr PortId MAX_SERVO_PORTS = 4;
    constexpr PortId MAX_ANALOG_PORTS = 6;
    constexpr PortId MAX_DIGITAL_BITS = 10;

    enum class MotorControlMode : uint8_t
    {
        Off = 0, // Motor off, coasting
        PassiveBrake = 1, // Active braking - short-circuits motor windings
        Pwm = 2, // Direct PWM, no PID
        MoveAtVelocity = 3, // PID velocity control
        MoveToPosition = 4, // PID position control (absolute)
        Chassis = 5, // Chassis velocity: per-wheel target = fwd_matrix * chassisVelocity (set on all 4 motors)
    };

    enum class ServoMode : uint8_t
    {
        FullyDisabled = 0,
        Disabled = 1,
        Enabled = 2, // PWM servo
        Bus = 3 // Feetech STS serial bus servo (e.g. STS3215)
    };

    // Bus servo geometry: one revolution is 4096 steps; position_cmd degrees
    // 0..360 map onto the servo's single-turn range 0..4095.
    constexpr float BUS_SERVO_STEPS_PER_DEGREE = 4096.0f / 360.0f;

    enum class BusServoMode : uint8_t
    {
        Relaxed = 0, // torque off, position still reported
        Position = 1, // go to goalPosition at speed/acc, then hold
        Wheel = 2 // continuous rotation at signed speed
    };

    struct BusServoCommand
    {
        BusServoMode mode{BusServoMode::Relaxed};
        uint16_t goalPosition{0}; // steps, 0..4095
        int16_t speed{0}; // steps/s; Position: max speed (0 = servo maximum), Wheel: signed velocity
        uint8_t acc{0}; // x100 steps/s^2 (0 = servo default)
    };

    struct BusServoFeedback
    {
        bool connected{false};
        bool moving{false};
        bool done{false}; // command commandSeq finished
        bool stalled{false}; // ... but stopped short of the goal
        bool torque{false};
        int32_t position{0}; // steps, multi-turn
        uint16_t rawPosition{0}; // steps, 0..4095
        int16_t speed{0}; // steps/s
        int16_t load{0}; // 0.1 % of max torque
        float voltage{0.0f}; // V
        uint8_t temperature{0}; // deg C
        uint8_t status{0}; // servo error bits
        uint8_t commandSeq{0};
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
        MotorControlMode controlMode{MotorControlMode::Off};
        int32_t target{0}; // PWM: signed duty (-400..400), MAV: velocity, MTP: speed limit
        int32_t goalPosition{0}; // Target position for MTP mode (BEMF ticks)
        mutable int32_t backEmf{0};
        int32_t position{0};
        bool done{false};

        bool hasSameCommand(const MotorState& other) const noexcept
        {
            return controlMode == other.controlMode &&
                target == other.target &&
                goalPosition == other.goalPosition;
        }
    };

    struct ServoState
    {
        ServoMode mode{ServoMode::Disabled};
        ServoPosition position{0};
    };

    struct OdometryState
    {
        float pos_x{0.0f}; // meters, world frame
        float pos_y{0.0f}; // meters, world frame
        float heading{0.0f}; // radians (firmware CW-positive convention)
        float vx{0.0f}; // m/s, body frame
        float vy{0.0f}; // m/s, body frame
        float wz{0.0f}; // rad/s, body frame
    };

    struct SensorData
    {
        Vector3f gyro{};
        Vector3f accelerometer{};
        Vector3f magnetometer{};
        Vector3f linearAcceleration{};
        Vector3f accelVelocity{};
        Quaternionf dmpOrientation{};
        float heading{0.0f};
        float temperature{0.0f};
        ImuAccuracy accuracy{};
        float batteryVoltage{0.0f};
        std::array<AnalogValue, MAX_ANALOG_PORTS> analogValues{};
        DigitalValue digitalBits{0};
        Timestamp lastUpdate{0};
        OdometryState odometry{};
        std::array<BusServoFeedback, MAX_SERVO_PORTS> busServos{};
    };
}
