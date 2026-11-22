// What the reader's logger sends to the UI: a message with a title and a text goes to messages/warn or
// messages/error; a message with one argument stays in the journal.

#include "mocks/MockLogger.h"
#include "wombat/core/Channels.h"
#include "wombat/core/Configuration.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"

#include <raccoon/Transport.h>
#include <raccoon/message_t.hpp>
#include <raccoon/raccoon_ring.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <unistd.h>

using namespace wombat;

namespace
{
    void removeRing(const std::string& channel)
    {
        char path[512]{};
        if (rrb_channel_to_path(channel.c_str(), path, sizeof(path)) == 0) (void)::unlink(path);
    }

    bool waitFor(const std::function<bool()>& done)
    {
        for (int i = 0; i < 200 && !done(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return done();
    }
}

TEST(LoggerMessagesTest, OnlyTitleAndContentReachTheUi)
{
    removeRing(Channels::ERROR_MESSAGES);
    removeRing(Channels::WARN_MESSAGES);

    auto quiet = std::make_shared<test::MockLogger>();
    auto broker = std::make_shared<LcmBroker>(quiet);
    ASSERT_TRUE(broker->initialize().isSuccess());

    // A second transport listens, as botui does.
    raccoon::Transport listener = raccoon::Transport::create();
    std::atomic<int> errors{0};
    std::atomic<int> warnings{0};
    std::string errorTitle, errorContent, warnTitle;
    ASSERT_NE(listener.subscribe<raccoon::message_t>(Channels::ERROR_MESSAGES, [&](const raccoon::message_t& m) {
        errorTitle = m.title;
        errorContent = m.content;
        ++errors;
    }), 0u);
    ASSERT_NE(listener.subscribe<raccoon::message_t>(Channels::WARN_MESSAGES, [&](const raccoon::message_t& m) {
        warnTitle = m.title;
        ++warnings;
    }), 0u);

    Configuration::Logging config;
    auto logger = Logger::create(config, broker);

    // One argument: the journal only.
    logger->warn("only the journal");
    logger->error("only the journal, too");
    // Title and content: the UI as well.
    logger->warn("Slow control loop", "One loop took 64 ms.");
    logger->error("IMU not working", "The IMU delivers only zeros.");

    const auto pump = [&] {
        listener.spin_once(10);
        return errors.load() >= 1 && warnings.load() >= 1;
    };
    ASSERT_TRUE(waitFor(pump));
    for (int i = 0; i < 20; ++i)  // time for anything else that was sent
    {
        listener.spin_once(5);
    }

    EXPECT_EQ(errors.load(), 1);
    EXPECT_EQ(warnings.load(), 1);
    EXPECT_EQ(errorTitle, "IMU not working");
    EXPECT_EQ(errorContent, "The IMU delivers only zeros.");
    EXPECT_EQ(warnTitle, "Slow control loop");

    logger->setLcmBroker(nullptr);
    broker->shutdown();
}
