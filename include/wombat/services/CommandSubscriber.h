//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/services/DeviceController.h"
#include <memory>

namespace wombat {

class CommandSubscriber {
public:
    CommandSubscriber(std::shared_ptr<LcmBroker> broker,
                     std::shared_ptr<DeviceController> deviceController,
                     std::shared_ptr<Logger> logger);

    Result<void> initialize();
    Result<void> shutdown();

private:
    std::shared_ptr<LcmBroker> broker_;
    std::shared_ptr<DeviceController> deviceController_;
    std::shared_ptr<Logger> logger_;

    void onMotorPowerCommand(const exlcm::scalar_i32_t& command);
    void onServoPositionCommand(const exlcm::scalar_i32_t& command);

    bool isInitialized_{false};
};

}