#include "wombat/Application.h"

#ifdef USE_SPI_MOCK
#include "wombat/hardware/SpiMock.h"
#else
#include "wombat/hardware/SpiReal.h"
extern "C" {
#include "wombat/hardware/Spi.h"
#include "spi/pi_buffer.h"
}
#endif

#include "version.h"

#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <stdexcept>

namespace wombat
{
    // Global pointer for signal handling
    static Application* g_application = nullptr;

    void signalHandler(int signal)
    {
        if (g_application && (signal == SIGINT || signal == SIGTERM))
        {
            g_application->requestShutdown();
        }
    }

    Application::Application(const Configuration& config)
        : config_{config}
    {
        g_application = this;
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
    }

    Application::~Application()
    {
        shutdown();
        g_application = nullptr;
    }

    Result<void> Application::initialize()
    {
        if (isInitialized_)
        {
            return Result<void>::success();
        }

        auto servicesResult = createServices();
        if (servicesResult.isFailure())
        {
            return servicesResult;
        }

        auto initResult = initializeServices();
        if (initResult.isFailure())
        {
            return initResult;
        }

        isInitialized_ = true;
        logger_->info("Application initialized successfully");
        return Result<void>::success();
    }

    Result<void> Application::run()
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Application not initialized");
        }

        logger_->info("Starting Wombat-Pi LCM interface...");

        while (!shouldShutdown_.load())
        {
            auto result = processMainLoop();
            if (result.isFailure())
            {
                logger_->error("Main loop error: " + result.error());
                fatalShutdown_ = true;
                shouldShutdown_ = true;
            }

            std::this_thread::sleep_for(config_.mainLoopDelay);
        }

        logger_->info("Application main loop finished");
        if (fatalShutdown_)
        {
            return Result<void>::failure("Fatal STM32 health failure");
        }
        return Result<void>::success();
    }

    Result<void> Application::shutdown()
    {
        if (!isInitialized_)
        {
            return Result<void>::success();
        }

        logger_->info("Shutting down application...");
        shouldShutdown_ = true;

        auto result = shutdownServices();
        if (result.isFailure())
        {
            logger_->error("Failed to shutdown services cleanly: " + result.error());
        }

        isInitialized_ = false;
        logger_->info("Application shut down complete");
        return Result<void>::success();
    }

    void Application::requestShutdown()
    {
        shouldShutdown_ = true;
        if (logger_)
        {
            logger_->info("Shutdown requested");
        }
    }

    Result<void> Application::createServices()
    {
        // Create logger first
        logger_ = Logger::create(config_.logging);

        // Create message broker
        messageBroker_ = std::make_shared<LcmBroker>(logger_);

        // Connect logger to LCM broker for error message publishing
        logger_->setLcmBroker(messageBroker_);

        // Create SPI hardware
#ifdef USE_SPI_MOCK
        auto spi = std::make_unique<SpiMock>(config_.spi, logger_);
#else
        auto spi = std::make_unique<SpiReal>(config_.spi, logger_);
#endif
        deviceController_ = std::make_shared<DeviceController>(std::move(spi), logger_);
        dataPublisher_ = std::make_shared<DataPublisher>(messageBroker_, logger_);
        systemMonitor_ = std::make_unique<SystemMonitor>(messageBroker_, logger_);
        commandSubscriber_ = std::make_shared<CommandSubscriber>(messageBroker_, deviceController_, dataPublisher_,
                                                                 logger_, motorWatchdog_);

        if (config_.uart.enabled)
        {
            uartMonitor_ = std::make_unique<UartMonitor>(logger_, config_.uart);
        }

        logger_->debug("All services created successfully");
        return Result<void>::success();
    }

    Result<void> Application::initializeServices()
    {
        // 1. Message broker
        auto messageBrokerResult = messageBroker_->initialize();
        if (messageBrokerResult.isFailure())
        {
            return Result<void>::failure("Failed to initialize message broker: " + messageBrokerResult.error());
        }

        // 2. UART monitor — must be up before reset so boot output is captured
        if (uartMonitor_)
        {
            auto uartResult = uartMonitor_->initialize();
            if (uartResult.isFailure())
            {
                logger_->warn("Failed to initialize UART monitor: " + uartResult.error());
            }
        }

#ifndef USE_SPI_MOCK
        // 3. Reset STM32 and capture boot output
        logger_->info("Resetting STM32 coprocessor...");
        spi_reset_stm32(); // calls reset script + 1 s sleep

        if (uartMonitor_)
        {
            logger_->info("Waiting for STM32 boot output...");
            uartMonitor_->drainFor(std::chrono::milliseconds(2000));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
#endif

        // 4. Initialize device controller (opens SPI fd)
        auto deviceControllerResult = deviceController_->initialize();
        if (deviceControllerResult.isFailure())
        {
            return Result<void>::failure("Failed to initialize device controller: " + deviceControllerResult.error());
        }

#ifndef USE_SPI_MOCK
        // 5. Probe the protocol version actually running on the STM32
        {
            const uint8_t stm32Version = spi_probe_version();
            const bool versionMatch = (stm32Version == TRANSFER_VERSION);

            logger_->info("=== Startup version report ===");
            logger_->info("  Reader version      : " + std::string(STMREADER_VERSION));
            logger_->info("  SPI protocol expected: " + std::to_string(TRANSFER_VERSION));
            logger_->info("  SPI protocol from STM32: " + std::to_string(stm32Version));
            if (versionMatch)
            {
                logger_->info("  Version check       : OK — no reflash needed");
            }
            else
            {
                logger_->warn("  Version check       : MISMATCH — firmware reflash will be triggered on first update");
            }
            logger_->info("==============================");
        }
#else
        logger_->info("Reader version: " + std::string(STMREADER_VERSION) + " (SPI mock — no STM32 version check)");
#endif

        // 6. Initialize command subscriber
        auto commandSubscriberResult = commandSubscriber_->initialize();
        if (commandSubscriberResult.isFailure())
        {
            logger_->warn("Failed to initialize command subscriber: " + commandSubscriberResult.error());
        }

        // 7. Apply startup-only opt-in feature flags.
        // disableBemfOnStartup is the "speed mode" toggle — push it to the STM32 on the
        // very first SPI transfer so the firmware skips BEMF measurement from the start.
        if (config_.disableBemfOnStartup)
        {
            logger_->warn("Startup config: disableBemfOnStartup=true — entering speed mode "
                "(no BEMF, no encoder ticks, MAV commands will be rejected).");
            auto bemfResult = deviceController_->setBemfEnabled(false);
            if (bemfResult.isFailure())
            {
                logger_->error("Failed to apply startup BEMF disable: " + bemfResult.error());
            }
            else if (dataPublisher_)
            {
                dataPublisher_->publishBemfEnabled(false);
            }
        }

        logger_->debug("All services initialized successfully");
        return Result<void>::success();
    }

    Result<void> Application::shutdownServices()
    {
        // Shutdown in reverse order

        if (uartMonitor_)
        {
            auto result = uartMonitor_->shutdown();
            if (result.isFailure())
            {
                logger_->warn("Failed to shutdown UART monitor: " + result.error());
            }
        }

        if (commandSubscriber_)
        {
            auto result = commandSubscriber_->shutdown();
            if (result.isFailure())
            {
                logger_->warn("Failed to shutdown command subscriber: " + result.error());
            }
        }

        if (deviceController_)
        {
            auto result = deviceController_->shutdown();
            if (result.isFailure())
            {
                logger_->warn("Failed to shutdown device controller: " + result.error());
            }
        }

        if (messageBroker_)
        {
            auto result = messageBroker_->shutdown();
            if (result.isFailure())
            {
                logger_->warn("Failed to shutdown message broker: " + result.error());
            }
        }

        logger_->debug("Services shut down successfully");
        return Result<void>::success();
    }

    Result<void> Application::processMainLoop()
    {
        const auto loopNow = std::chrono::steady_clock::now();

        // Process incoming messages. SPI guards (e.g. BEMF-disable MAV reject)
        // throw std::runtime_error from inside command handlers — catch here so
        // the loop survives, log, and force the affected motor(s) to OFF.
        if (messageBroker_)
        {
            try
            {
                auto messageResult = messageBroker_->processMessages();
                if (messageResult.isFailure())
                {
                    logger_->warn("Failed to process messages: " + messageResult.error());
                }
            }
            catch (const std::runtime_error& ex)
            {
                logger_->error(std::string("SPI guard rejected command: ") + ex.what());
                // Recovery: force all motors to OFF so the rejected command can't
                // get re-driven on the next transfer. Caller (raccoon-lib) gets the
                // motor/done channel back at zero and can re-issue a valid command.
                for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
                {
                    auto offResult = deviceController_->setMotorOff(port);
                    if (offResult.isFailure())
                    {
                        logger_->warn("Recovery: failed to OFF motor " + std::to_string(port) +
                            ": " + offResult.error());
                    }
                }
            }
        }

        // Update motor watchdog (fires hardware shutdown if heartbeat is missing)
        motorWatchdog_.update(*deviceController_, *dataPublisher_);

        // Update device controller
        auto deviceResult = deviceController_->processUpdate();
        if (deviceResult.isFailure())
        {
            logger_->error("Device controller update failed: " + deviceResult.error());
            return deviceResult;
        }

        // Update CPU temperature (publishes periodically - every 1 second)
        if (systemMonitor_)
        {
            auto cpuTempResult = systemMonitor_->updateCpuTemperature(std::chrono::milliseconds(1000));
            if (cpuTempResult.isFailure())
            {
                logger_->debug("CPU temperature update failed: " + cpuTempResult.error());
            }
        }

        // Read STM32 UART debug output
        if (uartMonitor_)
        {
            uartMonitor_->noteLoopTime(loopNow);
            auto uartResult = uartMonitor_->processUpdate();
            if (uartResult.isFailure())
            {
                logger_->debug("UART monitor update failed: " + uartResult.error());
            }
        }

        // STM32 health check: updateTime must change within 10 seconds
        checkStm32Health();

        auto heartbeatResult = checkStm32Heartbeat(loopNow);
        if (heartbeatResult.isFailure())
        {
            return heartbeatResult;
        }

        // Publish current data
        auto publishResult = publishCurrentData();
        if (publishResult.isFailure())
        {
            logger_->warn("Failed to publish data: " + publishResult.error());
        }

        return Result<void>::success();
    }

    void Application::checkStm32Health()
    {
        auto sensorDataResult = deviceController_->getCurrentSensorData();
        if (sensorDataResult.isFailure())
            return;

        const Timestamp ts = sensorDataResult.value().lastUpdate;
        const auto now = std::chrono::steady_clock::now();

        if (ts != lastStm32Timestamp_)
        {
            lastStm32Timestamp_ = ts;
            lastStm32Activity_ = now;
            stm32HealthArmed_ = true;
            return;
        }

        if (!stm32HealthArmed_)
        {
            lastStm32Activity_ = now;
            return;
        }

        constexpr auto kTimeout = std::chrono::seconds(10);
        if (now - lastStm32Activity_ > kTimeout)
        {
            logger_->error("STM32 health check failed: updateTime has not changed for >10s — shutting down");
            fatalShutdown_ = true;
            shouldShutdown_ = true;
        }
    }

    Result<void> Application::checkStm32Heartbeat(const std::chrono::steady_clock::time_point now)
    {
        // UART heartbeat is a diagnostic signal, NOT a liveness signal.
        // The STM32 firmware writes calibration to flash periodically, which
        // disables UART TX interrupts for >12s while SPI/DMA keeps running
        // (verified 2026-06-02: heartbeats stop after `[CAL] Saving 124 bytes
        // to flash sector 12`). Killing the reader here causes a probe-fail
        // cascade in the Pi-side robot program even though sensors are live.
        //
        // checkStm32Health() above checks SPI updateTime — that is the
        // authoritative liveness signal. Here we only emit rate-limited
        // warnings and never terminate the service.
        if (!uartMonitor_)
        {
            return Result<void>::success();
        }

        if (!uartMonitor_->heartbeatEverSeen())
        {
            return Result<void>::success();
        }

        constexpr auto kHeartbeatWarnTimeout = std::chrono::seconds(12);
        constexpr auto kWarnInterval = std::chrono::seconds(30);
        const auto lastHeartbeat = uartMonitor_->lastHeartbeatTime();
        if (!lastHeartbeat.has_value())
        {
            return Result<void>::success();
        }

        const bool stale = (now - *lastHeartbeat) > kHeartbeatWarnTimeout;

        if (stale)
        {
            if (!uartHeartbeatDegraded_
                || lastUartHeartbeatWarn_.time_since_epoch().count() == 0
                || (now - lastUartHeartbeatWarn_) >= kWarnInterval)
            {
                const auto silentFor = std::chrono::duration_cast<std::chrono::seconds>(now - *lastHeartbeat);
                logger_->warn(
                    "STM32 UART heartbeat silent for " + std::to_string(silentFor.count())
                    + "s (SPI still live; likely flash-write stall — not fatal)");
                lastUartHeartbeatWarn_ = now;
            }
            uartHeartbeatDegraded_ = true;
        }
        else if (uartHeartbeatDegraded_)
        {
            logger_->info("STM32 UART heartbeat recovered");
            uartHeartbeatDegraded_ = false;
        }

        return Result<void>::success();
    }

    Result<void> Application::publishCurrentData()
    {
        // Get current sensor data
        auto sensorDataResult = deviceController_->getCurrentSensorData();
        if (sensorDataResult.isFailure())
        {
            return Result<void>::failure("Failed to get sensor data: " + sensorDataResult.error());
        }

        const auto& sensorData = sensorDataResult.value();

        // Only publish if timestamp has changed
        if (sensorData.lastUpdate == lastPublishedTimestamp_)
        {
            return Result<void>::success();
        }

        lastPublishedTimestamp_ = sensorData.lastUpdate;

        // Publish sensor data
        auto sensorResult = dataPublisher_->publishSensorData(sensorData);
        if (sensorResult.isFailure())
        {
            logger_->warn("Failed to publish sensor data: " + sensorResult.error());
        }

        // Publish motor states
        for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
        {
            auto motorStateResult = deviceController_->getMotorState(port);
            if (motorStateResult.isSuccess())
            {
                const auto& ms = motorStateResult.value();

                auto publishResult = dataPublisher_->publishMotorState(port, ms);
                if (publishResult.isFailure())
                {
                    logger_->warn(
                        "Failed to publish motor " + std::to_string(port) + " state: " + publishResult.error());
                }
            }
        }

        // Publish servo states
        for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
        {
            auto servoStateResult = deviceController_->getServoState(port);
            if (servoStateResult.isSuccess())
            {
                auto publishResult = dataPublisher_->publishServoState(port, servoStateResult.value());
                if (publishResult.isFailure())
                {
                    logger_->warn(
                        "Failed to publish servo " + std::to_string(port) + " state: " + publishResult.error());
                }
            }
        }

        return Result<void>::success();
    }
} // namespace wombat
