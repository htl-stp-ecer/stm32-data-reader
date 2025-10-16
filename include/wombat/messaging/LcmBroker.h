//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "exlcm/vector3f_t.hpp"
#include "exlcm/quaternion_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/scalar_i8_t.hpp"
#include "exlcm/string_t.hpp"
#include <memory>
#include <functional>

namespace wombat {

class LcmBroker {
public:
    explicit LcmBroker(std::shared_ptr<Logger> logger);
    ~LcmBroker();

    Result<void> initialize();
    Result<void> shutdown();
    Result<void> processMessages();
    [[nodiscard]] bool isHealthy() const;

    // Typed publishers with change detection
    Result<void> publishVector3f(const std::string& channel, const exlcm::vector3f_t& message);
    Result<void> publishQuaternion(const std::string& channel, const exlcm::quaternion_t& message);
    Result<void> publishScalarF(const std::string& channel, const exlcm::scalar_f_t& message);
    Result<void> publishScalarI32(const std::string& channel, const exlcm::scalar_i32_t& message);
    Result<void> publishScalarI8(const std::string& channel, const exlcm::scalar_i8_t& message);
    Result<void> publishString(const std::string& channel, const exlcm::string_t& message);

    // Typed subscribers
    Result<void> subscribeVector3f(const std::string& channel,
                                  std::function<void(const exlcm::vector3f_t&)> handler);
    Result<void> subscribeQuaternion(const std::string& channel,
                                    std::function<void(const exlcm::quaternion_t&)> handler);
    Result<void> subscribeScalarI32(const std::string& channel,
                                   std::function<void(const exlcm::scalar_i32_t&)> handler);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}
