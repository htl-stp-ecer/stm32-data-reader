//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include <raccoon/Channels.h>
#include <raccoon/Concepts.h>
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include <raccoon/vector3f_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/orientation_matrix_t.hpp>
#include "wombat/services/DeviceController.h"
#include "wombat/services/DataPublisher.h"
#include <memory>
#include <unordered_map>
#include <string>
#include <cstdint>

namespace wombat
{
    namespace Channels = raccoon::Channels;
    using raccoon::LcmMessage;

    class CommandSubscriber
    {
    public:
        CommandSubscriber(std::shared_ptr<LcmBroker> broker,
                          std::shared_ptr<DeviceController> deviceController,
                          std::shared_ptr<DataPublisher> dataPublisher,
                          std::shared_ptr<Logger> logger);

        Result<void> initialize();
        Result<void> shutdown();

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<DeviceController> deviceController_;
        std::shared_ptr<DataPublisher> dataPublisher_;
        std::shared_ptr<Logger> logger_;

        void onMotorPowerCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onMotorModeCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onMotorStopCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onMotorVelocityCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onMotorPositionCommand(PortId port, const raccoon::vector3f_t& command);
        void onServoPositionCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onServoModeCommand(PortId port, const raccoon::scalar_i8_t& command);
        void onMotorPositionResetCommand(PortId port, const raccoon::scalar_i32_t& command);
        void onMotorPidCommand(PortId port, const raccoon::vector3f_t& command);
        void onShutdownCommand(const raccoon::scalar_i32_t& command);

        bool isTimestampNewer(const std::string& channel, int64_t timestamp);

        template <LcmMessage MsgT>
        Result<void> subscribeForPorts(
            PortId maxPorts,
            std::function<std::string(PortId)> channelFn,
            std::function<void(PortId, const MsgT &)> handler,
            const std::string& description,
            const raccoon::SubscribeOptions& options = {});

        bool isInitialized_{false};
        std::unordered_map<std::string, int64_t> latestTimestamps_;
    };
}