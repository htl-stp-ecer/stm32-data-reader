#include "wombat/hardware/SpiMock.h"
#include <cmath>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace wombat
{
    SpiMock::SpiMock(const Configuration::Spi& cfg, std::shared_ptr<Logger> logger)
        : cfg_(cfg), logger_(std::move(logger))
    {
    }

    Result<void> SpiMock::initialize()
    {
        lastUpdateTime_ = std::chrono::steady_clock::now();
        elapsedSecs_ = 0.0;
        initialized_ = true;
        if (logger_) logger_->info("SPI Mock initialized");
        return Result<void>::success();
    }

    Result<void> SpiMock::shutdown()
    {
        initialized_ = false;
        if (logger_) logger_->info("SPI Mock shut down");
        return Result<void>::success();
    }

    Result<void> SpiMock::forceUpdate()
    {
        // Just trigger a sensor read cycle in mock mode
        return Result<void>::success();
    }

    Result<SensorData> SpiMock::readSensorData()
    {
        if (!initialized_) return Result<SensorData>::failure("SPI Mock not initialized");

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
        if (dt < 0) dt = 0;
        if (dt > 0.2) dt = 0.2; // clamp if app stalls
        elapsedSecs_ += dt;
        lastUpdateTime_ = now;

        const float w = 2.0f * static_cast<float>(M_PI) * 0.2f; // 0.2 Hz
        const float t = static_cast<float>(elapsedSecs_);

        SensorData d{};

        // Gyro
        d.gyro.x = 5.0f * sinf(w * t);
        d.gyro.y = 3.0f * cosf(w * t * 0.7f);
        d.gyro.z = 2.0f * sinf(w * t * 1.3f);

        // Accelerometer
        d.accelerometer.x = 0.02f * sinf(w * t * 0.5f);
        d.accelerometer.y = 0.02f * cosf(w * t * 0.5f);
        d.accelerometer.z = 1.00f + 0.01f * sinf(w * t);

        // Magnetometer
        d.magnetometer.x = 0.30f + 0.05f * sinf(w * t * 0.3f);
        d.magnetometer.y = 0.10f + 0.04f * cosf(w * t * 0.4f);
        d.magnetometer.z = 0.50f + 0.03f * sinf(w * t * 0.5f);

        // Linear acceleration (gravity removed)
        d.linearAcceleration.x = d.accelerometer.x;
        d.linearAcceleration.y = d.accelerometer.y;
        d.linearAcceleration.z = d.accelerometer.z - 1.0f;

        // Accel velocity (mock: small sinusoidal)
        d.accelVelocity.x = 0.01f * sinf(w * t * 0.3f);
        d.accelVelocity.y = 0.01f * cosf(w * t * 0.3f);
        d.accelVelocity.z = 0.0f;

        // DMP orientation: smoothly varying quaternion from small Euler rotations
        const float roll = 0.12f * sinf(w * t * 0.8f);
        const float pitch = 0.10f * cosf(w * t * 0.6f);
        const float yaw = 0.18f * sinf(w * t * 0.4f);

        const float cr = cosf(roll * 0.5f);
        const float sr = sinf(roll * 0.5f);
        const float cp = cosf(pitch * 0.5f);
        const float sp = sinf(pitch * 0.5f);
        const float cy = cosf(yaw * 0.5f);
        const float sy = sinf(yaw * 0.5f);

        d.dmpOrientation.w = cr * cp * cy + sr * sp * sy;
        d.dmpOrientation.x = sr * cp * cy - cr * sp * sy;
        d.dmpOrientation.y = cr * sp * cy + sr * cp * sy;
        d.dmpOrientation.z = cr * cp * sy - sr * sp * cy;

        d.heading = 180.0f + 90.0f * sinf(w * t * 0.2f);

        // IMU accuracy (mock: calibrated)
        d.accuracy.gyro = 3;
        d.accuracy.accelerometer = 3;
        d.accuracy.linearAcceleration = 3;
        d.accuracy.compass = 3;
        d.accuracy.quaternion = 3;

        // Temperature
        d.temperature = 28.0f + 0.3f * sinf(w * t * 0.2f);

        // Battery: slow drift + ripple
        static float batteryV = 12.3f;
        batteryV -= static_cast<float>(dt * 0.0005);
        if (batteryV < 10.5f) batteryV = 12.3f;
        d.batteryVoltage = batteryV + 0.02f * sinf(2.0f * static_cast<float>(M_PI) * 2.0f * t);

        // Analog: deterministic pattern per index + wobble
        for (uint8_t i = 0; i < MAX_ANALOG_PORTS; ++i)
        {
            float base = 800.0f + 100.0f * static_cast<float>(i);
            float wob = 50.0f * sinf(w * t * (0.6f + 0.1f * static_cast<float>(i)));
            int val = static_cast<int>(base + wob);
            if (val < 0) val = 0;
            if (val > 4095) val = 4095;
            d.analogValues[i] = static_cast<uint16_t>(val);
        }

        // Digital: rotating single '1' bit
        static uint16_t digitalBits = 0x0001;
        digitalBits = static_cast<uint16_t>((digitalBits << 1) | (digitalBits >> 15));
        d.digitalBits = digitalBits;

        d.lastUpdate = static_cast<uint32_t>(dt * 1e6);

        // Update motor BEMF and position from mock state
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            int32_t rawBemf = 0;
            if (motors_[i].controlMode == MotorControlMode::Pwm ||
                motors_[i].controlMode == MotorControlMode::MoveAtVelocity ||
                motors_[i].controlMode == MotorControlMode::MoveToPosition)
            {
                // Back EMF: toy model proportional to target magnitude
                rawBemf = motors_[i].target / 4;
            }
            motors_[i].backEmf = rawBemf;

            // Position: simple mock based on target
            motors_[i].position = motors_[i].target;

            // All motors report done in mock mode
            motors_[i].done = true;
        }

        return Result<SensorData>::success(d);
    }

    Result<void> SpiMock::setMotorOff(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].controlMode = MotorControlMode::Off;
        motors_[port].target = 0;
        return Result<void>::success();
    }

    Result<void> SpiMock::setMotorBrake(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].controlMode = MotorControlMode::PassiveBrake;
        motors_[port].target = 0;
        return Result<void>::success();
    }

    Result<void> SpiMock::setMotorPwm(PortId port, int32_t duty)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].controlMode = MotorControlMode::Pwm;
        motors_[port].target = duty;
        return Result<void>::success();
    }

    Result<void> SpiMock::setMotorVelocity(PortId port, int32_t velocity)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].controlMode = MotorControlMode::MoveAtVelocity;
        motors_[port].target = velocity;
        return Result<void>::success();
    }

    Result<void> SpiMock::setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].controlMode = MotorControlMode::MoveToPosition;
        motors_[port].target = velocity;
        motors_[port].goalPosition = goalPosition;
        return Result<void>::success();
    }

    Result<int32_t> SpiMock::getMotorPosition(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<int32_t>::failure("motor port out of range");
        return Result<int32_t>::success(motors_[port].position);
    }

    Result<uint8_t> SpiMock::getMotorDone()
    {
        return Result<uint8_t>::success(0x0F); // All motors done
    }

    Result<MotorState> SpiMock::getMotorState(PortId port) const
    {
        if (port >= MAX_MOTOR_PORTS) return Result<MotorState>::failure("motor port out of range");
        return Result<MotorState>::success(motors_[port]);
    }

    Result<void> SpiMock::setServoState(PortId port, const ServoState& st)
    {
        if (port >= MAX_SERVO_PORTS) return Result<void>::failure("servo port out of range");
        servos_[port] = st;
        return Result<void>::success();
    }

    Result<ServoState> SpiMock::getServoState(PortId port) const
    {
        if (port >= MAX_SERVO_PORTS) return Result<ServoState>::failure("servo port out of range");
        return Result<ServoState>::success(servos_[port]);
    }

    Result<void> SpiMock::resetMotorPosition(PortId port)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        motors_[port].position = 0;
        return Result<void>::success();
    }

    Result<void> SpiMock::setMotorPid(PortId port, float /*kp*/, float /*ki*/, float /*kd*/)
    {
        if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
        // Mock: just store, no actual PID control
        return Result<void>::success();
    }

    Result<void> SpiMock::setShutdown(bool enabled)
    {
        if (logger_) logger_->info("SPI Mock: Shutdown " + std::string(enabled ? "enabled" : "disabled"));
        return Result<void>::success();
    }

    Result<void> SpiMock::sendKinematicsConfig(const float /*inv_matrix*/[3][4], const float /*ticks_to_rad*/[4],
                                               const float /*fwd_matrix*/[4][3])
    {
        if (logger_) logger_->info("SPI Mock: Kinematics config sent (no-op)");
        return Result<void>::success();
    }

    Result<void> SpiMock::resetOdometry()
    {
        if (logger_) logger_->info("SPI Mock: Odometry reset (no-op)");
        return Result<void>::success();
    }
}