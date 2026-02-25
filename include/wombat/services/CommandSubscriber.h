//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include "wombat/core/Channels.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/messaging/LcmConcepts.h"
#include "exlcm/orientation_matrix_t.hpp"
#include "wombat/services/DeviceController.h"
#include "wombat/services/DataPublisher.h"
#include <memory>
#include <unordered_map>
#include <string>
#include <cstdint>

namespace wombat
{
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

        void onMotorPowerCommand(PortId port, const exlcm::scalar_i32_t& command);
        void onMotorStopCommand(PortId port, const exlcm::scalar_i32_t& command);
        void onMotorVelocityCommand(PortId port, const exlcm::scalar_i32_t& command);
        void onMotorPositionCommand(PortId port, const exlcm::vector3f_t& command);
        void onServoPositionCommand(PortId port, const exlcm::scalar_i32_t& command);
        void onServoModeCommand(PortId port, const exlcm::scalar_i8_t& command);
        void onDataDumpRequest(const exlcm::scalar_i32_t& command) const;
        void onMotorPositionResetCommand(PortId port, const exlcm::scalar_i32_t& command);
        void onBemfScaleCommand(PortId port, const exlcm::scalar_f_t& command);
        void onBemfOffsetCommand(PortId port, const exlcm::scalar_f_t& command);
        void onBemfNominalVoltageCommand(const exlcm::scalar_i32_t& command);
        void onMotorPidCommand(PortId port, const exlcm::vector3f_t& command);
        void onShutdownCommand(const exlcm::scalar_i32_t& command);
        void onImuGyroOrientationCommand(const exlcm::orientation_matrix_t& command);
        void onImuCompassOrientationCommand(const exlcm::orientation_matrix_t& command);

        bool isTimestampNewer(const std::string& channel, int64_t timestamp);

        template <LcmMessage MsgT>
        Result<void> subscribeForPorts(
            PortId maxPorts,
            std::function<std::string(PortId)> channelFn,
            std::function<void(PortId, const MsgT &)> handler,
            const std::string& description);

        bool isInitialized_{false};
        std::unordered_map<std::string, int64_t> latestTimestamps_;
    };
}