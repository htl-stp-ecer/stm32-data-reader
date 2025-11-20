//
// Created by tobias on 9/14/25.
//
#pragma once

#include <cstdint>
#include <array>
#include <chrono>
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
        ServoLike = 0b11
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

    struct MotorState
    {
        MotorDirection direction{MotorDirection::Off};
        MotorSpeed speed{0};
        mutable int32_t backEmf{0};
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
        Quaternionf orientation{};
        float temperature{0.0f};
        float batteryVoltage{0.0f};
        std::array<AnalogValue, MAX_ANALOG_PORTS> analogValues{};
        DigitalValue digitalBits{0};
        Timestamp lastUpdate{0};
    };

    struct Configuration
    {
        struct Spi
        {
            std::string devicePath{"/dev/spidev0.0"};
            uint32_t speedHz{20'000'000};
            uint8_t mode{0};
            uint8_t bitsPerWord{8};
            std::chrono::milliseconds minimumUpdateDelay{20};
            uint8_t maxRetryAttempts{3};
            uint8_t protocolVersion{1};
        } spi;

        struct Logging
        {
            enum class Level { Debug, Info, Warn, Error };

            Level logLevel{Level::Info};
            std::string pattern{"[%H:%M:%S.%e] [%l] %v"};
        } logging;

        std::chrono::milliseconds mainLoopDelay{1};
    };

    namespace Channels
    {
        constexpr auto GYRO = "libstp/gyro/value";
        constexpr auto ACCELEROMETER = "libstp/accel/value";
        constexpr auto MAGNETOMETER = "libstp/mag/value";
        constexpr auto ORIENTATION = "libstp/imu/quaternion";
        constexpr auto TEMPERATURE = "libstp/imu/temp/value";  // IMU temperature from sensor
        constexpr auto BATTERY_VOLTAGE = "libstp/battery/voltage";
        constexpr auto CPU_TEMPERATURE = "libstp/cpu/temp/value";  // Raspberry Pi CPU temperature

        inline std::string motorPower(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/value";
        }

        inline std::string servoMode(const PortId port)
        {
            return "libstp/servo/" + std::to_string(port) + "/mode";
        }

        inline std::string servoPosition(const PortId port)
        {
            return "libstp/servo/" + std::to_string(port) + "/position";
        }

        inline std::string backEmf(const PortId port)
        {
            return "libstp/bemf/" + std::to_string(port) + "/value";
        }

        inline std::string bemfResetCommand(const PortId port)
        {
            return "libstp/bemf/" + std::to_string(port) + "/reset_cmd";
        }

        inline std::string analog(const PortId port)
        {
            return "libstp/analog/" + std::to_string(port) + "/value";
        }

        inline std::string digital(const PortId bit)
        {
            return "libstp/digital/" + std::to_string(bit) + "/value";
        }

        inline std::string motorPowerCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/power_cmd";
        }

        inline std::string servoPositionCommand(const PortId port)
        {
            return "libstp/servo/" + std::to_string(port) + "/position_cmd";
        }

        constexpr auto DATA_DUMP_REQUEST = "libstp/system/dump_request";
        constexpr auto ERROR_MESSAGES = "libstp/errors";
    }
}
