#include "wombat/Application.h"
#include <thread>
#include <csignal>

namespace wombat {

// Global pointer for signal handling
static Application* g_application = nullptr;

void signalHandler(int signal) {
    if (g_application && (signal == SIGINT || signal == SIGTERM)) {
        g_application->requestShutdown();
    }
}

Application::Application(const Configuration& config)
    : config_{config} {
    g_application = this;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
}

Application::~Application() {
    shutdown();
    g_application = nullptr;
}

Result<void> Application::initialize() {
    if (isInitialized_) {
        return Result<void>::success();
    }

    auto servicesResult = createServices();
    if (servicesResult.isFailure()) {
        return servicesResult;
    }

    auto initResult = initializeServices();
    if (initResult.isFailure()) {
        return initResult;
    }

    isInitialized_ = true;
    logger_->info("Application initialized successfully");
    return Result<void>::success();
}

Result<void> Application::run() {
    if (!isInitialized_) {
        return Result<void>::failure("Application not initialized");
    }

    logger_->info("Starting Wombat-Pi LCM interface...");

    while (!shouldShutdown_.load()) {
        auto result = processMainLoop();
        if (result.isFailure()) {
            logger_->error("Main loop error: " + result.error());
        }

        std::this_thread::sleep_for(config_.mainLoopDelay);
    }

    logger_->info("Application main loop finished");
    return Result<void>::success();
}

Result<void> Application::shutdown() {
    if (!isInitialized_) {
        return Result<void>::success();
    }

    logger_->info("Shutting down application...");
    shouldShutdown_ = true;

    auto result = shutdownServices();
    if (result.isFailure()) {
        logger_->error("Failed to shutdown services cleanly: " + result.error());
    }

    isInitialized_ = false;
    logger_->info("Application shut down complete");
    return Result<void>::success();
}

void Application::requestShutdown() {
    shouldShutdown_ = true;
    if (logger_) {
        logger_->info("Shutdown requested");
    }
}

Result<void> Application::createServices() {
    // Create logger first
    logger_ = Logger::create(config_.logging);

    // Create message broker
    messageBroker_ = std::make_shared<LcmBroker>(logger_);

    // Connect logger to LCM broker for error message publishing
    logger_->setLcmBroker(messageBroker_);

    // Create SPI hardware with LCM broker support
    auto spiWithLcm = std::make_unique<Spi>(config_.spi, logger_, messageBroker_);
    deviceController_ = std::make_shared<DeviceController>(std::move(spiWithLcm), logger_);
    dataPublisher_ = std::make_shared<DataPublisher>(messageBroker_, logger_);
    commandSubscriber_ = std::make_shared<CommandSubscriber>(messageBroker_, deviceController_, logger_);

    logger_->debug("All services created successfully");
    return Result<void>::success();
}

Result<void> Application::initializeServices() {
    // Initialize message broker first
    auto messageBrokerResult = messageBroker_->initialize();
    if (messageBrokerResult.isFailure()) {
        return Result<void>::failure("Failed to initialize message broker: " + messageBrokerResult.error());
    }

    // Initialize device controller
    auto deviceControllerResult = deviceController_->initialize();
    if (deviceControllerResult.isFailure()) {
        return Result<void>::failure("Failed to initialize device controller: " + deviceControllerResult.error());
    }

    // Initialize command subscriber
    auto commandSubscriberResult = commandSubscriber_->initialize();
    if (commandSubscriberResult.isFailure()) {
        logger_->warn("Failed to initialize command subscriber: " + commandSubscriberResult.error());
    }

    logger_->debug("All services initialized successfully");
    return Result<void>::success();
}

Result<void> Application::shutdownServices() {
    // Shutdown in reverse order
    if (commandSubscriber_) {
        auto result = commandSubscriber_->shutdown();
        if (result.isFailure()) {
            logger_->warn("Failed to shutdown command subscriber: " + result.error());
        }
    }

    if (deviceController_) {
        auto result = deviceController_->shutdown();
        if (result.isFailure()) {
            logger_->warn("Failed to shutdown device controller: " + result.error());
        }
    }

    if (messageBroker_) {
        auto result = messageBroker_->shutdown();
        if (result.isFailure()) {
            logger_->warn("Failed to shutdown message broker: " + result.error());
        }
    }

    logger_->debug("Services shut down successfully");
    return Result<void>::success();
}

Result<void> Application::processMainLoop() {
    // Process incoming messages
    if (messageBroker_) {
        auto messageResult = messageBroker_->processMessages();
        if (messageResult.isFailure()) {
            logger_->warn("Failed to process messages: " + messageResult.error());
        }
    }

    // Update device controller
    auto deviceResult = deviceController_->processUpdate();
    if (deviceResult.isFailure()) {
        logger_->error("Device controller update failed: " + deviceResult.error());
        return deviceResult;
    }

    // Publish current data
    auto publishResult = publishCurrentData();
    if (publishResult.isFailure()) {
        logger_->warn("Failed to publish data: " + publishResult.error());
    }

    return Result<void>::success();
}

Result<void> Application::publishCurrentData() {
    // Get current sensor data
    auto sensorDataResult = deviceController_->getCurrentSensorData();
    if (sensorDataResult.isFailure()) {
        return Result<void>::failure("Failed to get sensor data: " + sensorDataResult.error());
    }

    const auto& sensorData = sensorDataResult.value();

    // Only publish if timestamp has changed
    if (sensorData.lastUpdate == lastPublishedTimestamp_) {
        return Result<void>::success();
    }

    lastPublishedTimestamp_ = sensorData.lastUpdate;

    // Publish sensor data
    auto sensorResult = dataPublisher_->publishSensorData(sensorData);
    if (sensorResult.isFailure()) {
        logger_->warn("Failed to publish sensor data: " + sensorResult.error());
    }

    // Publish motor states
    for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port) {
        auto motorStateResult = deviceController_->getMotorState(port);
        if (motorStateResult.isSuccess()) {
            auto publishResult = dataPublisher_->publishMotorState(port, motorStateResult.value());
            if (publishResult.isFailure()) {
                logger_->warn("Failed to publish motor " + std::to_string(port) + " state: " + publishResult.error());
            }
        }
    }

    // Publish servo states
    for (PortId port = 0; port < MAX_SERVO_PORTS; ++port) {
        auto servoStateResult = deviceController_->getServoState(port);
        if (servoStateResult.isSuccess()) {
            auto publishResult = dataPublisher_->publishServoState(port, servoStateResult.value());
            if (publishResult.isFailure()) {
                logger_->warn("Failed to publish servo " + std::to_string(port) + " state: " + publishResult.error());
            }
        }
    }

    return Result<void>::success();
}

} // namespace wombat