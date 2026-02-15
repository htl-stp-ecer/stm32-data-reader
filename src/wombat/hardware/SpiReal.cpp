#include "wombat/hardware/SpiReal.h"
#include <algorithm>
#include <string>

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

        d.orientation.w = rx->imu.quat.data[0];
        d.orientation.x = rx->imu.quat.data[1];
        d.orientation.y = rx->imu.quat.data[2];
        d.orientation.z = rx->imu.quat.data[3];

        d.accuracy.gyro = rx->imu.gyro.accuracy;
        d.accuracy.accelerometer = rx->imu.accel.accuracy;
        d.accuracy.linearAcceleration = rx->imu.linearAccel.accuracy;
        d.accuracy.compass = rx->imu.compass.accuracy;
        d.accuracy.quaternion = rx->imu.quat.accuracy;
        d.temperature = rx->imu.temperature;

        // Battery voltage filtering
        const float stmVoltage = 3.3f;
        const float voltageDividerFactor = 11.0f;
        const float adcResolution = 4096.0f;
        const float rawVoltage = static_cast<float>(rx->batteryVoltage) * stmVoltage * voltageDividerFactor /
            adcResolution;
        constexpr float alpha = 0.0001f;
        if (filteredBatteryVoltage_ == 0.0f)
            filteredBatteryVoltage_ = rawVoltage;
        else
            filteredBatteryVoltage_ = filteredBatteryVoltage_ * (1.0f - alpha) + rawVoltage * alpha;
        d.batteryVoltage = filteredBatteryVoltage_;

        for (uint8_t i = 0; i < 6; ++i)
            d.analogValues[i] = static_cast<uint16_t>(rx->analogSensor[i]);

        d.digitalBits = rx->digitalSensors;
        d.lastUpdate = rx->updateTime;

        const uint8_t doneFlags = rx->motorDone;
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            motors_[i].backEmf = rx->motorBemf[i] - bemfOffsets_[i];
            motors_[i].position = rx->motorPosition[i];
            motors_[i].done = (doneFlags & (1u << i)) != 0;
        }
        return Result<SensorData>::success(d);
    }

    Result<void> SpiReal::setMotorState(PortId port, const MotorState& st)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        MotorDir dir = MOTOR_DIR_OFF;
        switch (st.direction)
        {
        case MotorDirection::Off: dir = MOTOR_DIR_OFF;
            break;
        case MotorDirection::CounterClockwise: dir = MOTOR_DIR_CCW;
            break;
        case MotorDirection::Clockwise: dir = MOTOR_DIR_CW;
            break;
        case MotorDirection::Brake: dir = MOTOR_DIR_BRAKE;
            break;
        }

        const double percentAbs = std::min(st.speed / 100.0, 1.0);

        uint32_t duty = 0;
        if (percentAbs > 0.01f)
        {
            duty = static_cast<uint32_t>(percentAbs * 400.0f);
        }

        SPDLOG_TRACE(
            "SPI setMotorState port={} dir={} duty={}",
            static_cast<int>(port),
            static_cast<int>(dir),
            duty);
        set_motor(port, dir, duty);
        st.backEmf = motors_[port].backEmf;
        motors_[port] = st;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorVelocity(PortId port, int32_t velocity)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_velocity(port, velocity);
        motors_[port].controlMode = MotorControlMode::MoveAtVelocity;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_position(port, velocity, goalPosition);
        motors_[port].controlMode = MotorControlMode::MoveToPosition;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorRelative(PortId port, int32_t velocity, int32_t deltaPosition)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_relative(port, velocity, deltaPosition);
        motors_[port].controlMode = MotorControlMode::MoveRelativePosition;
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
        set_servo_pos(port, st.position);
        servos_[port] = st;
        return Result<void>::success();
    }

    Result<ServoState> SpiReal::getServoState(PortId port) const
    {
        if (port >= MAX_SERVO_PORTS) return Result<ServoState>::failure("servo port out of range");
        return Result<ServoState>::success(servos_[port]);
    }

    Result<void> SpiReal::resetBemfSum(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        const auto rawValue = motors_[port].backEmf + bemfOffsets_[port];
        bemfOffsets_[port] = rawValue;
        motors_[port].backEmf = 0;
        return Result<void>::success();
    }

    Result<void> SpiReal::setMotorPid(PortId port, float kp, float ki, float kd)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_motor_pid(port, kp, ki, kd);
        return Result<void>::success();
    }

    Result<void> SpiReal::setBemfScale(PortId port, float scale)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_bemf_scale(port, scale);
        return Result<void>::success();
    }

    Result<void> SpiReal::setBemfOffset(PortId port, float offset)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        set_bemf_offset(port, offset);
        return Result<void>::success();
    }

    Result<void> SpiReal::setBemfNominalVoltage(int16_t adcValue)
    {
        set_bemf_nominal_voltage(adcValue);
        return Result<void>::success();
    }

    Result<void> SpiReal::setShutdown(bool enabled)
    {
        set_shutdown_flag(SHUTDOWN_MOTOR_FLAG, enabled);
        set_shutdown_flag(SHUTDOWN_SERVO_FLAG, enabled);
        if (logger_) logger_->info("SPI: Shutdown " + std::string(enabled ? "enabled" : "disabled"));
        return Result<void>::success();
    }
}