#include "wombat/Application.h"
#include "wombat/core/Logger.h"
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>

namespace
{
    bool iequals(const char* a, const char* b)
    {
        while (*a && *b)
        {
            if (std::tolower(static_cast<unsigned char>(*a)) !=
                std::tolower(static_cast<unsigned char>(*b)))
            {
                return false;
            }
            ++a;
            ++b;
        }
        return *a == '\0' && *b == '\0';
    }

    void applyEnvOverrides(wombat::Configuration& config)
    {
        if (const char* level = std::getenv("WOMBAT_LOG_LEVEL"))
        {
            using Level = wombat::Configuration::Logging::Level;
            if (iequals(level, "debug")) config.logging.logLevel = Level::Debug;
            else if (iequals(level, "info")) config.logging.logLevel = Level::Info;
            else if (iequals(level, "warn") || iequals(level, "warning"))
                config.logging.logLevel = Level::Warn;
            else if (iequals(level, "error") || iequals(level, "err"))
                config.logging.logLevel = Level::Error;
        }
    }
}

int main() {
    try {
        wombat::Configuration config{};
        applyEnvOverrides(config);

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
            applyEnvOverrides(config);
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
            applyEnvOverrides(config);
            auto logger = wombat::Logger::create(config.logging);
            logger->error("Unknown unhandled exception");
        } catch (...) {
            // Last resort fallback - logger creation failed
        }
        return EXIT_FAILURE;
    }
}