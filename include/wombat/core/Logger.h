//
// Created by tobias on 9/14/25.
//
#pragma once

#include <string>
#include <memory>
#include "wombat/core/Configuration.h"

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

        // A message for the user (a dialog in the UI): a short title and the text below it. The journal
        // gets one line, "title: content". The overloads above show the text under the title "STM32 reader".
        virtual void warn(const std::string& title, const std::string& content)
        {
            warn(title + ": " + content);
        }

        virtual void error(const std::string& title, const std::string& content)
        {
            error(title + ": " + content);
        }

        static std::unique_ptr<Logger> create(const Configuration::Logging& config);
        static std::unique_ptr<Logger> create(const Configuration::Logging& config,
                                              std::shared_ptr<LcmBroker> lcmBroker);

        virtual void setLcmBroker(std::shared_ptr<LcmBroker> /*lcmBroker*/)
        {
        };
    };
}