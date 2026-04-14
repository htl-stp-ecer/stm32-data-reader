#include "wombat/services/DataPublisher.h"
#include "wombat/messaging/LcmConversions.h"
#include <string>
#include <cmath>

namespace wombat
{
    DataPublisher::DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)}, logger_{std::move(logger)}
    {
    }

    Result<void> DataPublisher::publishSensorData(const SensorData& data)
    {
        auto gyroResult = broker_->publish(Channels::GYRO, toLcm(data.gyro));
        if (gyroResult.isFailure())
        {
            logger_->warn("Failed to publish gyro data: " + gyroResult.error());
        }

        auto accelResult = broker_->publish(Channels::ACCELEROMETER, toLcm(data.accelerometer));
        if (accelResult.isFailure())
        {
            logger_->warn("Failed to publish accelerometer data: " + accelResult.error());
        }

        auto magResult = broker_->publish(Channels::MAGNETOMETER, toLcm(data.magnetometer));
        if (magResult.isFailure())
        {
            logger_->warn("Failed to publish magnetometer data: " + magResult.error());
        }

        auto linAccelResult = broker_->publish(Channels::LINEAR_ACCELERATION, toLcm(data.linearAcceleration));
        if (linAccelResult.isFailure())
        {
            logger_->warn("Failed to publish linear acceleration data: " + linAccelResult.error());
        }

        auto accelVelResult = broker_->publish(Channels::ACCEL_VELOCITY, toLcm(data.accelVelocity));
        if (accelVelResult.isFailure())
        {
            logger_->warn("Failed to publish accel velocity data: " + accelVelResult.error());
        }

        auto dmpOrientResult = broker_->publish(Channels::DMP_ORIENTATION, toLcm(data.dmpOrientation));
        if (dmpOrientResult.isFailure())
        {
            logger_->warn("Failed to publish DMP orientation: " + dmpOrientResult.error());
        }

        auto headingResult = broker_->publishRetained(Channels::HEADING, toLcmScalarF(data.heading));
        if (headingResult.isFailure())
        {
            logger_->warn("Failed to publish heading data: " + headingResult.error());
        }

        // Publish IMU accuracy (throttled)
        publishAccuracy(data.accuracy);

        auto tempResult = broker_->publish(Channels::TEMPERATURE, toLcmScalarF(data.temperature));
        if (tempResult.isFailure())
        {
            logger_->warn("Failed to publish temperature data: " + tempResult.error());
        }

        {
            const float rounded = std::round(data.batteryVoltage * 1000.0f) / 1000.0f;
            auto batteryResult = broker_->publishRetained(Channels::BATTERY_VOLTAGE, toLcmScalarF(rounded));
            if (batteryResult.isFailure())
            {
                logger_->warn("Failed to publish battery voltage data: " + batteryResult.error());
            }
        }

        auto analogResult = publishAnalogValues(data.analogValues);
        if (analogResult.isFailure())
        {
            logger_->warn("Failed to publish analog values: " + analogResult.error());
        }

        auto digitalResult = publishDigitalBits(data.digitalBits);
        if (digitalResult.isFailure())
        {
            logger_->warn("Failed to publish digital bits: " + digitalResult.error());
        }

        // Odometry (computed on STM32) — verbose, keep at debug
        logger_->debug("Odometry: pos_x=" + std::to_string(data.odometry.pos_x) +
            " pos_y=" + std::to_string(data.odometry.pos_y) +
            " heading=" + std::to_string(data.odometry.heading) +
            " vx=" + std::to_string(data.odometry.vx) +
            " vy=" + std::to_string(data.odometry.vy) +
            " wz=" + std::to_string(data.odometry.wz));
        broker_->publishRetained(Channels::ODOM_POS_X, toLcmScalarF(data.odometry.pos_x));
        broker_->publishRetained(Channels::ODOM_POS_Y, toLcmScalarF(data.odometry.pos_y));
        broker_->publishRetained(Channels::ODOM_HEADING, toLcmScalarF(data.odometry.heading));
        broker_->publishRetained(Channels::ODOM_VX, toLcmScalarF(data.odometry.vx));
        broker_->publishRetained(Channels::ODOM_VY, toLcmScalarF(data.odometry.vy));
        broker_->publishRetained(Channels::ODOM_WZ, toLcmScalarF(data.odometry.wz));

        if (logger_) logger_->debug("Sensor data publish completed");

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorState(PortId port, const MotorState& state)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto powerResult = broker_->publishRetained(
            Channels::motorPower(port),
            toLcmScalarI32(state.target)
        );
        if (powerResult.isFailure())
        {
            logger_->warn("Failed to publish motor power: " + powerResult.error());
        }

        auto bemfResult = broker_->publishRetained(
            Channels::backEmf(port),
            toLcmScalarI32(state.backEmf)
        );
        if (bemfResult.isFailure())
        {
            logger_->warn("Failed to publish back EMF: " + bemfResult.error());
        }

        auto posResult = broker_->publishRetained(
            Channels::motorPosition(port),
            toLcmScalarI32(state.position)
        );
        if (posResult.isFailure())
        {
            logger_->warn("Failed to publish motor position: " + posResult.error());
        }

        auto doneResult = broker_->publishRetained(
            Channels::motorDone(port),
            toLcmScalarI32(state.done ? 1 : 0)
        );
        if (doneResult.isFailure())
        {
            logger_->warn("Failed to publish motor done: " + doneResult.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorPosition(PortId port, int32_t position)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto result = broker_->publish(
            Channels::motorPosition(port),
            toLcmScalarI32(position)
        );
        if (result.isFailure())
        {
            logger_->warn("Failed to publish motor position: " + result.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorDone(PortId port, bool done)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto result = broker_->publish(
            Channels::motorDone(port),
            toLcmScalarI32(done ? 1 : 0)
        );
        if (result.isFailure())
        {
            logger_->warn("Failed to publish motor done: " + result.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishServoState(PortId port, const ServoState& state)
    {
        if (port >= MAX_SERVO_PORTS)
        {
            return Result<void>::failure("Invalid servo port: " + std::to_string(port));
        }

        auto modeResult = broker_->publishRetained(
            Channels::servoMode(port),
            toLcmScalarI8(static_cast<uint8_t>(state.mode))
        );
        if (modeResult.isFailure())
        {
            logger_->warn("Failed to publish servo mode: " + modeResult.error());
        }

        auto positionResult = broker_->publishRetained(
            Channels::servoPosition(port),
            toLcmScalarF(state.position)
        );
        if (positionResult.isFailure())
        {
            logger_->warn("Failed to publish servo position: " + positionResult.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values)
    {
        for (size_t i = 0; i < values.size(); ++i)
        {
            auto result = broker_->publish(
                Channels::analog(static_cast<PortId>(i)),
                toLcmScalarI32(static_cast<int32_t>(values[i]))
            );
            if (result.isFailure())
            {
                logger_->warn("Failed to publish analog value " + std::to_string(i) + ": " + result.error());
            }
        }
        return Result<void>::success();
    }

    Result<void> DataPublisher::publishDigitalBits(DigitalValue digitalBits)
    {
        for (PortId bit = 0; bit < 11; ++bit)
        {
            const int32_t value = (digitalBits >> bit) & 1u;
            auto result = broker_->publish(
                Channels::digital(bit),
                toLcmScalarI32(value)
            );
            if (result.isFailure())
            {
                logger_->warn("Failed to publish digital bit " + std::to_string(bit) + ": " + result.error());
            }
        }
        return Result<void>::success();
    }

    Result<void> DataPublisher::publishAccuracy(const ImuAccuracy& accuracy)
    {
        const bool isFirstTime = !lastAccuracy_.has_value();
        const bool hasChanged = lastAccuracy_.has_value() && !(accuracy == lastAccuracy_.value());

        if (isFirstTime)
        {
            logger_->info("IMU accuracy (initial): gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", lin_accel=" + std::to_string(accuracy.linearAcceleration) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }
        else if (hasChanged)
        {
            logger_->info("IMU accuracy changed: gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", lin_accel=" + std::to_string(accuracy.linearAcceleration) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }

        if (isFirstTime || hasChanged)
        {
            broker_->publish(Channels::GYRO_ACCURACY, toLcmScalarI8(accuracy.gyro));
            broker_->publish(Channels::ACCEL_ACCURACY, toLcmScalarI8(accuracy.accelerometer));
            broker_->publish(Channels::COMPASS_ACCURACY, toLcmScalarI8(accuracy.compass));
            broker_->publish(Channels::QUATERNION_ACCURACY, toLcmScalarI8(accuracy.quaternion));
        }

        lastAccuracy_ = accuracy;

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishShutdownStatus(uint8_t shutdownFlags)
    {
        auto message = toLcmScalarI32(static_cast<int32_t>(shutdownFlags));

        auto result = broker_->publishRetained(Channels::SHUTDOWN_STATUS, message);
        if (result.isFailure())
        {
            return Result<void>::failure("Failed to publish shutdown status: " + result.error());
        }

        logger_->info("Published shutdown status: " + std::to_string(shutdownFlags) +
            " (servo=" + (shutdownFlags & 0x01 ? "on" : "off") +
            ", motor=" + (shutdownFlags & 0x02 ? "on" : "off") + ")");

        return Result<void>::success();
    }
} // namespace wombat