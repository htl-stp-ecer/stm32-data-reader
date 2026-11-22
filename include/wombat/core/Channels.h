#pragma once

// The channels the reader uses, named after what they do here. Every name comes from the
// generated channel table of raccoon-transport (raccoon/ChannelTable.h); this file only
// gives the daemon a short spelling and std::string values for its maps.

#include <raccoon/ChannelTable.h>

#include <string>

namespace wombat::Channels
{
    namespace table = raccoon::channels;

    // ── Sensor data ──────────────────────────────────────────────────────────
    inline const std::string GYRO{table::hardware::imu::state::GYRO};
    inline const std::string ACCELEROMETER{table::hardware::imu::state::ACCEL};
    inline const std::string MAGNETOMETER{table::hardware::imu::state::MAG};
    inline const std::string LINEAR_ACCELERATION{table::hardware::imu::state::LINEAR_ACCEL};
    inline const std::string ACCEL_VELOCITY{table::hardware::imu::state::ACCEL_VELOCITY};
    inline const std::string DMP_ORIENTATION{table::hardware::imu::state::QUATERNION};
    inline const std::string HEADING{table::hardware::imu::state::HEADING};
    inline const std::string TEMPERATURE{table::hardware::imu::state::TEMPERATURE};
    inline const std::string GYRO_ACCURACY{table::hardware::imu::state::GYRO_ACCURACY};
    inline const std::string ACCEL_ACCURACY{table::hardware::imu::state::ACCEL_ACCURACY};
    inline const std::string COMPASS_ACCURACY{table::hardware::imu::state::MAG_ACCURACY};
    inline const std::string QUATERNION_ACCURACY{table::hardware::imu::state::QUATERNION_ACCURACY};
    inline const std::string BATTERY_VOLTAGE{table::hardware::battery::state::VOLTAGE};
    inline const std::string CPU_TEMPERATURE{table::system::state::CPU_TEMPERATURE};

    inline std::string analog(int port) { return std::string{table::hardware::analog::state::value(port)}; }
    inline std::string digital(int port) { return std::string{table::hardware::digital::state::value(port)}; }

    // ── Motors ───────────────────────────────────────────────────────────────
    inline std::string motorPower(int port) { return std::string{table::hardware::motor::state::power(port)}; }
    inline std::string motorPosition(int port) { return std::string{table::hardware::motor::state::position(port)}; }
    inline std::string backEmf(int port) { return std::string{table::hardware::motor::state::bemf(port)}; }

    inline std::string motorPowerCommand(int port) { return std::string{table::hardware::motor::cmd::power(port)}; }
    inline std::string motorVelocityCommand(int port) { return std::string{table::hardware::motor::cmd::velocity(port)}; }
    inline std::string motorModeCommand(int port) { return std::string{table::hardware::motor::cmd::mode(port)}; }
    inline std::string motorStopCommand(int port) { return std::string{table::hardware::motor::cmd::stop(port)}; }
    inline std::string motorPositionCommand(int port) { return std::string{table::hardware::motor::cmd::position(port)}; }
    inline std::string motorPidCommand(int port) { return std::string{table::hardware::motor::cmd::pid(port)}; }
    inline std::string motorPositionResetCommand(int port)
    {
        return std::string{table::hardware::motor::cmd::reset_position(port)};
    }

    // ── Servos ───────────────────────────────────────────────────────────────
    inline std::string servoMode(int port) { return std::string{table::hardware::servo::state::mode(port)}; }
    inline std::string servoPosition(int port) { return std::string{table::hardware::servo::state::position(port)}; }
    inline std::string servoFeedback(int port) { return std::string{table::hardware::servo::state::feedback(port)}; }
    inline std::string servoHealth(int port) { return std::string{table::hardware::servo::state::health(port)}; }
    inline std::string servoDone(int port) { return std::string{table::hardware::servo::state::done(port)}; }

    inline std::string servoPositionCommand(int port) { return std::string{table::hardware::servo::cmd::position(port)}; }
    inline std::string servoModeCommand(int port) { return std::string{table::hardware::servo::cmd::mode(port)}; }
    inline std::string servoSmoothPositionCommand(int port) { return std::string{table::hardware::servo::cmd::smooth(port)}; }
    inline std::string servoVelocityCommand(int port) { return std::string{table::hardware::servo::cmd::velocity(port)}; }

    // ── Chassis, odometry, system ────────────────────────────────────────────
    inline const std::string CHASSIS_VELOCITY_CMD{table::hardware::chassis::cmd::VELOCITY};
    inline const std::string CHASSIS_STOP_CMD{table::hardware::chassis::cmd::STOP};
    inline const std::string KINEMATICS_CONFIG_CMD{table::hardware::kinematics::cmd::CONFIG};
    inline const std::string ODOM_RESET_CMD{table::hardware::odometry::cmd::RESET};
    inline const std::string ODOM_POS_X{table::hardware::odometry::state::POS_X};
    inline const std::string ODOM_POS_Y{table::hardware::odometry::state::POS_Y};
    inline const std::string ODOM_HEADING{table::hardware::odometry::state::HEADING};
    inline const std::string ODOM_VX{table::hardware::odometry::state::VX};
    inline const std::string ODOM_VY{table::hardware::odometry::state::VY};
    inline const std::string ODOM_WZ{table::hardware::odometry::state::WZ};

    // The latch of the motors (see ShutdownFlags.h): engaged by a command, reported as flags.
    inline const std::string SHUTDOWN_CMD{table::hardware::latch::cmd::ENGAGED};
    inline const std::string SHUTDOWN_STATUS{table::hardware::latch::state::FLAGS};
    inline const std::string HEARTBEAT_CMD{table::hardware::watchdog::cmd::HEARTBEAT};
    inline const std::string BEMF_ENABLED_CMD{table::hardware::feature::cmd::BEMF_ACTIVE};
    inline const std::string BEMF_ENABLED{table::hardware::feature::state::BEMF_ACTIVE};

    inline const std::string ERROR_MESSAGES{table::messages::ERROR};
    inline const std::string WARN_MESSAGES{table::messages::WARN};
}
