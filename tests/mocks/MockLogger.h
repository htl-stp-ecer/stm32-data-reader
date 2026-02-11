#pragma once

#include "wombat/core/Logger.h"
#include <vector>
#include <string>
#include <mutex>

namespace wombat::test
{
    class MockLogger : public Logger
    {
    public:
        void info(const std::string& message) override
        {
            std::lock_guard lock(mutex_);
            infoMessages.push_back(message);
        }

        void warn(const std::string& message) override
        {
            std::lock_guard lock(mutex_);
            warnMessages.push_back(message);
        }

        void error(const std::string& message) override
        {
            std::lock_guard lock(mutex_);
            errorMessages.push_back(message);
        }

        void debug(const std::string& message) override
        {
            std::lock_guard lock(mutex_);
            debugMessages.push_back(message);
        }

        void clear()
        {
            std::lock_guard lock(mutex_);
            infoMessages.clear();
            warnMessages.clear();
            errorMessages.clear();
            debugMessages.clear();
        }

        std::vector<std::string> infoMessages;
        std::vector<std::string> warnMessages;
        std::vector<std::string> errorMessages;
        std::vector<std::string> debugMessages;

    private:
        std::mutex mutex_;
    };
}