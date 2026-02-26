#include "wombat/services/DataPublisher.h"
#include "wombat/messaging/LcmConversions.h"
#include <string>
#include <chrono>

namespace wombat
{
    DataPublisher::DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)}, logger_{std::move(logger)}
    {
    }

    void DataPublisher::setAxisRemap(const int8_t matrix[9])
    {
        remap_.setFromMatrix(matrix);
        logger_->info("Axis remap set: identity=" + std::string(remap_.identity ? "true" : "false"));
    }

    Result<void> DataPublisher::publishSensorData(const SensorData& data)
    {
        // Apply body-to-world axis remap to all IMU data before publishing
        const auto gyro = remap_.remapVector(data.gyro);
        const auto accel = remap_.remapVector(data.accelerometer);
        const auto mag = remap_.remapVector(data.magnetometer);
        const auto linAccel = remap_.remapVector(data.linearAcceleration);
        const auto accelVel = remap_.remapVector(data.accelVelocity);
        const auto orientation = remap_.remapQuaternion(data.orientation);

        auto gyroResult = broker_->publish(Channels::GYRO, toLcm(gyro));
        if (gyroResult.isFailure())
        {
            logger_->warn("Failed to publish gyro data: " + gyroResult.error());
        }

        auto accelResult = broker_->publish(Channels::ACCELEROMETER, toLcm(accel));
        if (accelResult.isFailure())
        {
            logger_->warn("Failed to publish accelerometer data: " + accelResult.error());
        }

        auto magResult = broker_->publish(Channels::MAGNETOMETER, toLcm(mag));
        if (magResult.isFailure())
        {
            logger_->warn("Failed to publish magnetometer data: " + magResult.error());
        }

        auto linAccelResult = broker_->publish(Channels::LINEAR_ACCELERATION, toLcm(linAccel));
        if (linAccelResult.isFailure())
        {
            logger_->warn("Failed to publish linear acceleration data: " + linAccelResult.error());
        }

        auto accelVelResult = broker_->publish(Channels::ACCEL_VELOCITY, toLcm(accelVel));
        if (accelVelResult.isFailure())
        {
            logger_->warn("Failed to publish accel velocity data: " + accelVelResult.error());
        }

        auto orientationResult = broker_->publish(Channels::ORIENTATION, toLcm(orientation));
        if (orientationResult.isFailure())
        {
            logger_->warn("Failed to publish orientation data: " + orientationResult.error());
        }

        // Publish IMU accuracy (throttled)
        publishAccuracy(data.accuracy);

        auto tempResult = broker_->publish(Channels::TEMPERATURE, toLcmScalarF(data.temperature));
        if (tempResult.isFailure())
        {
            logger_->warn("Failed to publish temperature data: " + tempResult.error());
        }

        auto batteryResult = broker_->publish(Channels::BATTERY_VOLTAGE, toLcmScalarF(data.batteryVoltage));
        if (batteryResult.isFailure())
        {
            logger_->warn("Failed to publish battery voltage data: " + batteryResult.error());
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

        if (logger_) logger_->debug("Sensor data publish completed");

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorState(PortId port, const MotorState& state)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto powerResult = broker_->publish(
            Channels::motorPower(port),
            toLcmScalarI32(state.target)
        );
        if (powerResult.isFailure())
        {
            logger_->warn("Failed to publish motor power: " + powerResult.error());
        }

        auto bemfResult = broker_->publish(
            Channels::backEmf(port),
            toLcmScalarI32(state.backEmf)
        );
        if (bemfResult.isFailure())
        {
            logger_->warn("Failed to publish back EMF: " + bemfResult.error());
        }

        auto posResult = broker_->publish(
            Channels::motorPosition(port),
            toLcmScalarI32(state.position)
        );
        if (posResult.isFailure())
        {
            logger_->warn("Failed to publish motor position: " + posResult.error());
        }

        auto doneResult = broker_->publish(
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

        auto modeResult = broker_->publish(
            Channels::servoMode(port),
            toLcmScalarI8(static_cast<uint8_t>(state.mode))
        );
        if (modeResult.isFailure())
        {
            logger_->warn("Failed to publish servo mode: " + modeResult.error());
        }

        auto positionResult = broker_->publish(
            Channels::servoPosition(port),
            toLcmScalarI32(static_cast<int32_t>(state.position))
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
        const auto now = std::chrono::steady_clock::now();
        const bool isFirstTime = !lastAccuracy_.has_value();
        const bool hasChanged = lastAccuracy_.has_value() && !(accuracy == lastAccuracy_.value());
        const bool intervalElapsed = (now - lastAccuracyPublishTime_) >= accuracyPublishInterval_;

        if (!isFirstTime && !hasChanged && !intervalElapsed)
        {
            return Result<void>::success();
        }

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
        else if (intervalElapsed)
        {
            logger_->info("IMU accuracy (periodic): gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", lin_accel=" + std::to_string(accuracy.linearAcceleration) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }

        broker_->publishForce(Channels::GYRO_ACCURACY, toLcmScalarI8(accuracy.gyro));
        broker_->publishForce(Channels::ACCEL_ACCURACY, toLcmScalarI8(accuracy.accelerometer));
        broker_->publishForce(Channels::COMPASS_ACCURACY, toLcmScalarI8(accuracy.compass));
        broker_->publishForce(Channels::QUATERNION_ACCURACY, toLcmScalarI8(accuracy.quaternion));

        lastAccuracy_ = accuracy;
        lastAccuracyPublishTime_ = now;

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishShutdownStatus(uint8_t shutdownFlags)
    {
        auto message = toLcmScalarI32(static_cast<int32_t>(shutdownFlags));

        auto result = broker_->publishForce(Channels::SHUTDOWN_STATUS, message);
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