#include "wombat/Application.h"
#include "wombat/core/Logger.h"
#include <cstdlib>

int main() {
    try {
        wombat::Configuration config{};

        wombat::Application app{config};

        auto initResult = app.initialize();
        if (initResult.isFailure()) {
            // Logger might not be available yet if initialization failed
            auto logger = app.getLogger();
            if (logger) {
                logger->error("Failed to initialize application: " + initResult.error());
            }
            return EXIT_FAILURE;
        }

        auto runResult = app.run();
        if (runResult.isFailure()) {
            auto logger = app.getLogger();
            if (logger) {
                logger->error("Application run failed: " + runResult.error());
            }
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;

    } catch (const std::exception& e) {
        // Try to get logger, but fallback to stderr if not available
        try {
            wombat::Configuration config{};
            auto logger = wombat::Logger::create(config.logging);
            logger->error("Unhandled exception: " + std::string(e.what()));
        } catch (...) {
            // Last resort fallback - logger creation failed
        }
        return EXIT_FAILURE;
    } catch (...) {
        // Try to get logger, but fallback to stderr if not available
        try {
            wombat::Configuration config{};
            auto logger = wombat::Logger::create(config.logging);
            logger->error("Unknown unhandled exception");
        } catch (...) {
            // Last resort fallback - logger creation failed
        }
        return EXIT_FAILURE;
    }
}