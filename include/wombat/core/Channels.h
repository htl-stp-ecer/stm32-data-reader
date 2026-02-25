//
// Created by tobias on 9/14/25.
// Split from Types.h during architecture restructuring.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include <string>

namespace wombat
{
    namespace Channels
    {
        constexpr auto GYRO = "libstp/gyro/value";
        constexpr auto ACCELEROMETER = "libstp/accel/value";
        constexpr auto LINEAR_ACCELERATION = "libstp/linear_accel/value";
        constexpr auto ACCEL_VELOCITY = "libstp/accel_velocity/value";
        constexpr auto MAGNETOMETER = "libstp/mag/value";
        constexpr auto ORIENTATION = "libstp/imu/quaternion";
        constexpr auto TEMPERATURE = "libstp/imu/temp/value"; // IMU temperature from sensor
        constexpr auto BATTERY_VOLTAGE = "libstp/battery/voltage";
        constexpr auto GYRO_ACCURACY = "libstp/gyro/accuracy";
        constexpr auto ACCEL_ACCURACY = "libstp/accel/accuracy";
        constexpr auto COMPASS_ACCURACY = "libstp/mag/accuracy";
        constexpr auto QUATERNION_ACCURACY = "libstp/imu/quaternion_accuracy";
        constexpr auto CPU_TEMPERATURE = "libstp/cpu/temp/value"; // Raspberry Pi CPU temperature

        // IMU orientation matrix commands
        constexpr auto IMU_GYRO_ORIENTATION_CMD = "libstp/imu/gyro_orientation_cmd";
        constexpr auto IMU_COMPASS_ORIENTATION_CMD = "libstp/imu/compass_orientation_cmd";

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

        inline std::string bemfScaleCommand(const PortId port)
        {
            return "libstp/bemf/" + std::to_string(port) + "/scale_cmd";
        }

        inline std::string bemfOffsetCommand(const PortId port)
        {
            return "libstp/bemf/" + std::to_string(port) + "/offset_cmd";
        }

        constexpr auto BEMF_NOMINAL_VOLTAGE_CMD = "libstp/bemf/nominal_voltage_cmd";

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

        inline std::string motorStopCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/stop_cmd";
        }

        inline std::string motorVelocityCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/velocity_cmd";
        }

        inline std::string motorPositionCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/position_cmd";
        }

        inline std::string motorPidCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/pid_cmd";
        }

        inline std::string motorPositionResetCommand(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/position_reset_cmd";
        }

        inline std::string motorPower(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/power";
        }

        inline std::string motorPosition(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/position";
        }

        inline std::string motorDone(const PortId port)
        {
            return "libstp/motor/" + std::to_string(port) + "/done";
        }

        inline std::string servoPositionCommand(const PortId port)
        {
            return "libstp/servo/" + std::to_string(port) + "/position_cmd";
        }

        constexpr auto DATA_DUMP_REQUEST = "libstp/system/dump_request";
        constexpr auto ERROR_MESSAGES = "libstp/errors";

        // STM32 shutdown flag command (disables motors and servos at firmware level)
        constexpr auto SHUTDOWN_CMD = "libstp/system/shutdown_cmd";

        // STM32 shutdown flag status (published when shutdown state changes)
        // Value is a bitmask: bit 0 = servo shutdown, bit 1 = motor shutdown
        constexpr auto SHUTDOWN_STATUS = "libstp/system/shutdown_status";
    }
}