//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmConcepts.h"
#include "exlcm/vector3f_t.hpp"
#include "exlcm/quaternion_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/scalar_i8_t.hpp"
#include "exlcm/string_t.hpp"
#include <memory>
#include <functional>

namespace wombat
{
    class LcmBroker
    {
    public:
        explicit LcmBroker(std::shared_ptr<Logger> logger);
        ~LcmBroker();

        Result<void> initialize();
        Result<void> shutdown();
        Result<void> processMessages();
        [[nodiscard]] bool isHealthy() const;

        template <LcmMessage T>
        Result<void> publish(const std::string& channel, const T& message);

        template <LcmMessage T>
        Result<void> publishForce(const std::string& channel, const T& message);

        template <LcmMessage T>
        Result<void> subscribe(const std::string& channel, std::function<void(const T &)> handler);

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };
}