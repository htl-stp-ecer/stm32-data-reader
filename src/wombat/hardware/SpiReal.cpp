#include "wombat/hardware/SpiReal.h"
#include <algorithm>
#include <cmath>
#include <string>

#include "spdlog/spdlog.h"

// C API
extern "C" {
#include "wombat/hardware/Spi.h"
#include "spi/pi_buffer.h"
}

namespace wombat
{
    SpiReal::SpiReal(const Configuration::Spi& cfg, std::shared_ptr<Logger> logger)
        : cfg_(cfg), logger_(std::move(logger))
    {
    }

    Result<void> SpiReal::initialize()
    {
        if (!spi_init(cfg_.speedHz))
        {
            return Result<void>::failure("spi_init() failed");
        }
        set_spi_mode(true);
        return Result<void>::success();
    }

    Result<void> SpiReal::shutdown()
    {
        set_spi_mode(false);
        spi_close();
        return Result<void>::success();
    }

    Result<void> SpiReal::forceUpdate()
    {
        if (!spi_force_update()) return Result<void>::failure("spi_force_update() failed");
        return Result<void>::success();
    }

    Result<SensorData> SpiReal::readSensorData()
    {
        // Single SPI transfer — read all sensor data at once
        if (!spi_update()) return Result<SensorData>::failure("spi_update() failed");

        const auto* rx = get_rx_buffer();
        SensorData d{};

        d.gyro.x = rx->imu.gyro.data[0];
        d.gyro.y = rx->imu.gyro.data[1];
        d.gyro.z = rx->imu.gyro.data[2];

        d.accelerometer.x = rx->imu.accel.data[0];
        d.accelerometer.y = rx->imu.accel.data[1];
        d.accelerometer.z = rx->imu.accel.data[2];

        d.magnetometer.x = rx->imu.compass.data[0];
        d.magnetometer.y = rx->imu.compass.data[1];
        d.magnetometer.z = rx->imu.compass.data[2];

        d.linearAcceleration.x = rx->imu.linearAccel.data[0];
        d.linearAcceleration.y = rx->imu.linearAccel.data[1];
        d.linearAcceleration.z = rx->imu.linearAccel.data[2];

        d.accelVelocity.x = rx->imu.accelVelocity.data[0];
        d.accelVelocity.y = rx->imu.accelVelocity.data[1];
        d.accelVelocity.z = rx->imu.accelVelocity.data[2];

        d.dmpOrientation.w = rx->imu.dmpQuat.data[0];
        d.dmpOrientation.x = rx->imu.dmpQuat.data[1];
        d.dmpOrientation.y = rx->imu.dmpQuat.data[2];
        d.dmpOrientation.z = rx->imu.dmpQuat.data[3];

        d.heading = rx->imu.heading;
        d.temperature = rx->imu.temperature;

        d.accuracy.gyro = rx->imu.gyro.accuracy;
        d.accuracy.accelerometer = rx->imu.accel.accuracy;
        d.accuracy.linearAcceleration = rx->imu.linearAccel.accuracy;
        d.accuracy.compass = rx->imu.compass.accuracy;
        d.accuracy.quaternion = rx->imu.dmpQuat.accuracy;

        // Debug: log IMU values periodically
        static uint32_t imuLogCounter = 0;
        if (++imuLogCounter % 500 == 0)
        {
            logger_->info("SPI rx gyro=[" + std::to_string(d.gyro.x) + ","
                + std::to_string(d.gyro.y) + "," + std::to_string(d.gyro.z)
                + "] accel=[" + std::to_string(d.accelerometer.x) + ","
                + std::to_string(d.accelerometer.y) + "," + std::to_string(d.accelerometer.z)
                + "] quat=[" + std::to_string(d.dmpOrientation.w) + ","
                + std::to_string(d.dmpOrientation.x) + ","
                + std::to_string(d.dmpOrientation.y) + ","
                + std::to_string(d.dmpOrientation.z)
                + "] heading=" + std::to_string(d.heading));
        }

        // Battery voltage filtering
        const float stmVoltage = 3.3f;
        const float voltageDividerFactor = 11.0f;
        const float adcResolution = 4096.0f;
        const float rawVoltage = static_cast<float>(rx->batteryVoltage) * stmVoltage * voltageDividerFactor /
            adcResolution;
        constexpr float alpha = 0.05f;
        if (filteredBatteryVoltage_ == 0.0f)
            filteredBatteryVoltage_ = rawVoltage;
        else
            filteredBatteryVoltage_ = filteredBatteryVoltage_ * (1.0f - alpha) + rawVoltage * alpha;
        d.batteryVoltage = filteredBatteryVoltage_;

        for (uint8_t i = 0; i < 6; ++i)
            d.analogValues[i] = static_cast<uint16_t>(rx->analogSensor[i]);

        d.digitalBits = rx->digitalSensors;
        d.lastUpdate = rx->updateTime;

        const uint8_t doneFlags = rx->motor.done;
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            motors_[i].backEmf = rx->motor.bemf[i];
            motors_[i].position = rx->motor.position[i];
            motors_[i].done = (doneFlags & (1u << i)) != 0;
        }

        // Debug: log raw BEMF values from SPI buffer
        static uint32_t bemfLogCounter = 0;
        if (++bemfLogCounter % 200 == 0) // ~every 200 SPI reads
        {
            logger_->info("SPI rx bemf=["
                + std::to_string(rx->motor.bemf[0]) + ","
                + std::to_string(rx->motor.bemf[1]) + ","
                + std::to_string(rx->motor.bemf[2]) + ","
                + std::to_string(rx->motor.bemf[3]) + "] pos=["
                + std::to_string(rx->motor.position[0]) + ","
                + std::to_string(rx->motor.position[1]) + ","
                + std::to_string(rx->motor.position[2]) + ","
                + std::to_string(rx->motor.position[3]) + "]");
        }

        // Odometry (computed on STM32)
        d.odometry.pos_x = rx->odometry.pos_x;
        d.odometry.pos_y = rx->odometry.pos_y;
        d.odometry.heading = rx->odometry.heading;
        d.odometry.vx = rx->odometry.vx;
        d.odometry.vy = rx->odometry.vy;
        d.odometry.wz = rx->odometry.wz;

        return Result<SensorData>::success(d);
    }

    Result<void> SpiReal::setMotorOff(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_off(port);
        motors_[port].controlMode = MotorControlMode::Off;
        motors_[port].target = 0;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorBrake(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_brake(port);
        motors_[port].controlMode = MotorControlMode::PassiveBrake;
        motors_[port].target = 0;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorPwm(PortId port, int32_t duty)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        SPDLOG_TRACE("SPI setMotorPwm port={} duty={}", static_cast<int>(port), duty);
        set_motor_pwm(port, duty);
        motors_[port].controlMode = MotorControlMode::Pwm;
        motors_[port].target = duty;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorVelocity(PortId port, int32_t velocity)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_velocity(port, velocity);
        motors_[port].controlMode = MotorControlMode::MoveAtVelocity;
        motors_[port].target = velocity;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_position(port, velocity, goalPosition);
        motors_[port].controlMode = MotorControlMode::MoveToPosition;
        motors_[port].target = velocity;
        motors_[port].goalPosition = goalPosition;
        return Result<void>::success();
    }

    Result<int32_t> SpiReal::getMotorPosition(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<int32_t>::failure("motor port out of range");
        return Result<int32_t>::success(get_motor_position(port));
    }

    Result<uint8_t> SpiReal::getMotorDone()
    {
        return Result<uint8_t>::success(get_motor_done());
    }

    Result<MotorState> SpiReal::getMotorState(PortId port) const
    {
        if (port >= MAX_MOTOR_PORTS) return Result<MotorState>::failure("motor port out of range");
        return Result<MotorState>::success(motors_[port]);
    }

    Result<void> SpiReal::setServoState(PortId port, const ServoState& st)
    {
        if (port >= MAX_SERVO_PORTS) return Result<void>::failure("servo port out of range");
        ::ServoMode mode = SERVO_MODE_DISABLED;
        switch (st.mode)
        {
        case ServoMode::Disabled: mode = SERVO_MODE_DISABLED;
            break;
        case ServoMode::Enabled: mode = SERVO_MODE_ENABLED;
            break;
        case ServoMode::FullyDisabled: mode = SERVO_MODE_FULLY_DISABLED;
            break;
        }
        set_servo_mode(port, mode);

        // Convert degrees to microseconds for the PWM timer (10µs per degree, centered at 1500µs)
        const auto microseconds = static_cast<uint16_t>(std::round(600.0f + st.position * 10.0f));
        set_servo_pos(port, microseconds);

        servos_[port] = st;
        return Result<void>::success();
    }

    Result<ServoState> SpiReal::getServoState(PortId port) const
    {
        if (port >= MAX_SERVO_PORTS) return Result<ServoState>::failure("servo port out of range");
        return Result<ServoState>::success(servos_[port]);
    }

    Result<void> SpiReal::resetMotorPosition(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        reset_motor_position_on_stm32(port);
        motors_[port].position = 0;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorPid(PortId port, float kp, float ki, float kd)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_pid(port, kp, ki, kd);
        return Result<void>::success();
    }

    Result<void> SpiReal::setShutdown(bool enabled)
    {
        set_shutdown_flag(SHUTDOWN_MOTOR_FLAG, enabled);
        set_shutdown_flag(SHUTDOWN_SERVO_FLAG, enabled);
        if (logger_) logger_->info("SPI: Shutdown " + std::string(enabled ? "enabled" : "disabled"));
        return Result<void>::success();
    }

    Result<void> SpiReal::sendKinematicsConfig(const float inv_matrix[3][4], const float ticks_to_rad[4])
    {
        set_kinematics_config(inv_matrix, ticks_to_rad);
        if (logger_) logger_->info("SPI: Kinematics config sent to STM32");
        return Result<void>::success();
    }

    Result<void> SpiReal::resetOdometry()
    {
        reset_stm32_odometry();
        if (logger_) logger_->info("SPI: STM32 odometry reset requested");
        return Result<void>::success();
    }
}