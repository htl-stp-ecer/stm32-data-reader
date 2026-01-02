#include "wombat/services/DataPublisher.h"
#include <fstream>
#include <string>

namespace wombat
{
    DataPublisher::DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)}, logger_{std::move(logger)}
    {
    }

    Result<void> DataPublisher::publishSensorData(const SensorData& data)
    {
        // Publish IMU data
        auto gyroResult = broker_->publishVector3f(Channels::GYRO, convertVector3f(data.gyro));
        if (gyroResult.isFailure())
        {
            logger_->warn("Failed to publish gyro data: " + gyroResult.error());
        }

        auto accelResult = broker_->publishVector3f(Channels::ACCELEROMETER, convertVector3f(data.accelerometer));
        if (accelResult.isFailure())
        {
            logger_->warn("Failed to publish accelerometer data: " + accelResult.error());
        }

        auto magResult = broker_->publishVector3f(Channels::MAGNETOMETER, convertVector3f(data.magnetometer));
        if (magResult.isFailure())
        {
            logger_->warn("Failed to publish magnetometer data: " + magResult.error());
        }

        auto orientationResult = broker_->publishQuaternion(Channels::ORIENTATION, convertQuaternion(data.orientation));
        if (orientationResult.isFailure())
        {
            logger_->warn("Failed to publish orientation data: " + orientationResult.error());
        }

        // Publish IMU accuracy (throttled)
        publishAccuracy(data.accuracy);

        // Publish temperature
        auto tempResult = broker_->publishScalarF(Channels::TEMPERATURE, convertScalarF(data.temperature));
        if (tempResult.isFailure())
        {
            logger_->warn("Failed to publish temperature data: " + tempResult.error());
        }

        // Publish battery voltage
        auto batteryResult = broker_->publishScalarF(Channels::BATTERY_VOLTAGE, convertScalarF(data.batteryVoltage));
        if (batteryResult.isFailure())
        {
            logger_->warn("Failed to publish battery voltage data: " + batteryResult.error());
        }


        // Publish analog and digital values
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

        // Publish motor value (speed)
        auto valueResult = broker_->publishScalarI32(
            Channels::motorPower(port),
            convertScalarI32(
                static_cast<int32_t>(state.speed * (state.direction == MotorDirection::CounterClockwise ? -1 : 1)))
        );
        if (valueResult.isFailure())
        {
            logger_->warn("Failed to publish motor value: " + valueResult.error());
        }

        // Publish back EMF
        auto bemfResult = broker_->publishScalarI32(
            Channels::backEmf(port),
            convertScalarI32(state.backEmf)
        );
        if (bemfResult.isFailure())
        {
            logger_->warn("Failed to publish back EMF: " + bemfResult.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishServoState(PortId port, const ServoState& state)
    {
        if (port >= MAX_SERVO_PORTS)
        {
            return Result<void>::failure("Invalid servo port: " + std::to_string(port));
        }

        // Publish servo mode
        auto modeResult = broker_->publishScalarI8(
            Channels::servoMode(port),
            convertScalarI8(static_cast<uint8_t>(state.mode))
        );
        if (modeResult.isFailure())
        {
            logger_->warn("Failed to publish servo mode: " + modeResult.error());
        }

        // Publish servo position
        auto positionResult = broker_->publishScalarI32(
            Channels::servoPosition(port),
            convertScalarI32(static_cast<int32_t>(state.position))
        );
        if (positionResult.isFailure())
        {
            logger_->warn("Failed to publish servo position: " + positionResult.error());
        }

        return Result<void>::success();
    }

    exlcm::vector3f_t DataPublisher::convertVector3f(const Vector3f& vector) const
    {
        exlcm::vector3f_t message{};
        message.x = vector.x;
        message.y = vector.y;
        message.z = vector.z;
        return message;
    }

    exlcm::quaternion_t DataPublisher::convertQuaternion(const Quaternionf& quaternion) const
    {
        exlcm::quaternion_t message{};
        message.w = quaternion.w;
        message.x = quaternion.x;
        message.y = quaternion.y;
        message.z = quaternion.z;
        return message;
    }

    exlcm::scalar_f_t DataPublisher::convertScalarF(float value) const
    {
        exlcm::scalar_f_t message{};
        message.value = value;
        return message;
    }

    exlcm::scalar_i32_t DataPublisher::convertScalarI32(int32_t value) const
    {
        exlcm::scalar_i32_t message{};
        message.value = value;
        return message;
    }

    exlcm::scalar_i8_t DataPublisher::convertScalarI8(uint8_t value) const
    {
        exlcm::scalar_i8_t message{};
        message.dir = value;
        return message;
    }

    Result<void> DataPublisher::publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values)
    {
        for (size_t i = 0; i < values.size(); ++i)
        {
            auto result = broker_->publishScalarI32(
                Channels::analog(static_cast<PortId>(i)),
                convertScalarI32(static_cast<int32_t>(values[i]))
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
            auto result = broker_->publishScalarI32(
                Channels::digital(bit),
                convertScalarI32(value)
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

        // Publish on first call, on change, or every 15 seconds
        if (!isFirstTime && !hasChanged && !intervalElapsed)
        {
            return Result<void>::success();
        }

        // Log on first time, change, or periodic
        if (isFirstTime)
        {
            logger_->info("IMU accuracy (initial): gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }
        else if (hasChanged)
        {
            logger_->info("IMU accuracy changed: gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }
        else if (intervalElapsed)
        {
            logger_->info("IMU accuracy (periodic): gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }

        // Publish all accuracy values (force to bypass change detection)
        broker_->publishScalarI8Force(Channels::GYRO_ACCURACY, convertScalarI8(accuracy.gyro));
        broker_->publishScalarI8Force(Channels::ACCEL_ACCURACY, convertScalarI8(accuracy.accelerometer));
        broker_->publishScalarI8Force(Channels::COMPASS_ACCURACY, convertScalarI8(accuracy.compass));
        broker_->publishScalarI8Force(Channels::QUATERNION_ACCURACY, convertScalarI8(accuracy.quaternion));

        lastAccuracy_ = accuracy;
        lastAccuracyPublishTime_ = now;

        return Result<void>::success();
    }

    Result<void> DataPublisher::updateCpuTemperature(std::chrono::milliseconds publishInterval)
    {
        // Check if enough time has elapsed since last publish
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCpuTempPublishTime_);

        if (elapsed < publishInterval)
        {
            // Not time to publish yet
            return Result<void>::success();
        }

        // Read CPU temperature
        auto tempResult = readCpuTemperature();
        if (tempResult.isFailure())
        {
            logger_->warn("Failed to read CPU temperature: " + tempResult.error());
            return Result<void>::failure("Failed to read CPU temperature: " + tempResult.error());
        }

        float temperature = tempResult.value();

        // Publish the temperature
        auto publishResult = publishCpuTemperature(temperature);
        if (publishResult.isFailure())
        {
            logger_->warn("Failed to publish CPU temperature: " + publishResult.error());
            return publishResult;
        }

        // Update last publish time and value
        lastCpuTempPublishTime_ = now;
        lastPublishedCpuTemperature_ = temperature;

        return Result<void>::success();
    }

    Result<float> DataPublisher::readCpuTemperature()
    {
        // Read from Linux thermal zone (temperature in millidegrees Celsius)
        const std::string thermalPath = "/sys/class/thermal/thermal_zone0/temp";

        std::ifstream thermalFile(thermalPath);
        if (!thermalFile.is_open())
        {
            return Result<float>::failure("Unable to open thermal sensor file: " + thermalPath);
        }

        std::string tempStr;
        std::getline(thermalFile, tempStr);
        thermalFile.close();

        if (tempStr.empty())
        {
            return Result<float>::failure("Empty temperature reading from thermal sensor");
        }

        try
        {
            // Convert from millidegrees to degrees Celsius
            int milliTemp = std::stoi(tempStr);
            float temperature = static_cast<float>(milliTemp) / 1000.0f;

            // Sanity check: CPU temperature should be between -40°C and 125°C
            if (temperature < -40.0f || temperature > 125.0f)
            {
                return Result<float>::failure("CPU temperature out of valid range: " + std::to_string(temperature));
            }

            return Result<float>::success(temperature);
        }
        catch (const std::exception& e)
        {
            return Result<float>::failure("Failed to parse temperature value: " + std::string(e.what()));
        }
    }

    Result<void> DataPublisher::publishCpuTemperature(float temperature)
    {
        // Convert to LCM message format
        auto message = convertScalarF(temperature);

        // Publish on the CPU temperature channel
        auto result = broker_->publishScalarF(Channels::CPU_TEMPERATURE, message);
        if (result.isFailure())
        {
            return Result<void>::failure("Failed to publish CPU temperature: " + result.error());
        }

        return Result<void>::success();
    }

} // namespace wombat
