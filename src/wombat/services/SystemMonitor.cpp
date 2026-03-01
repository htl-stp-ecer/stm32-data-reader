#include "wombat/services/SystemMonitor.h"
#include <raccoon/Channels.h>
#include "wombat/messaging/LcmConversions.h"
#include <fstream>
#include <string>

namespace wombat
{
    namespace Channels = raccoon::Channels;

    SystemMonitor::SystemMonitor(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)}, logger_{std::move(logger)}
    {
    }

    Result<void> SystemMonitor::updateCpuTemperature(std::chrono::milliseconds publishInterval)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCpuTempPublishTime_);

        if (elapsed < publishInterval)
        {
            return Result<void>::success();
        }

        auto tempResult = readCpuTemperature();
        if (tempResult.isFailure())
        {
            logger_->warn("Failed to read CPU temperature: " + tempResult.error());
            return Result<void>::failure("Failed to read CPU temperature: " + tempResult.error());
        }

        float temperature = tempResult.value();

        auto publishResult = publishCpuTemperature(temperature);
        if (publishResult.isFailure())
        {
            logger_->warn("Failed to publish CPU temperature: " + publishResult.error());
            return publishResult;
        }

        lastCpuTempPublishTime_ = now;
        lastPublishedCpuTemperature_ = temperature;

        return Result<void>::success();
    }

    Result<float> SystemMonitor::readCpuTemperature()
    {
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
            int milliTemp = std::stoi(tempStr);
            float temperature = static_cast<float>(milliTemp) / 1000.0f;

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

    Result<void> SystemMonitor::publishCpuTemperature(float temperature)
    {
        auto message = toLcmScalarF(temperature);

        auto result = broker_->publish(Channels::CPU_TEMPERATURE, message);
        if (result.isFailure())
        {
            return Result<void>::failure("Failed to publish CPU temperature: " + result.error());
        }

        return Result<void>::success();
    }
}