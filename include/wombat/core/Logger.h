//
// Created by tobias on 9/14/25.
//
#pragma once

#include <string>
#include <memory>
#include "Types.h"

namespace wombat
{
    class LcmBroker;

    class Logger
    {
    public:
        virtual ~Logger() = default;
        virtual void info(const std::string& message) = 0;
        virtual void warn(const std::string& message) = 0;
        virtual void error(const std::string& message) = 0;
        virtual void debug(const std::string& message) = 0;

        static std::unique_ptr<Logger> create(const Configuration::Logging& config);
        static std::unique_ptr<Logger> create(const Configuration::Logging& config, std::shared_ptr<LcmBroker> lcmBroker);

        virtual void setLcmBroker(std::shared_ptr<LcmBroker> /*lcmBroker*/) {};
    };
}
