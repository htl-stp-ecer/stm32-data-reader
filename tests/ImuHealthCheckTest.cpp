#include "wombat/core/ImuHealthCheck.h"

#include <gtest/gtest.h>

using namespace wombat;
using namespace std::chrono_literals;

namespace
{
    ImuHealthCheck::Clock::time_point at(std::chrono::milliseconds t)
    {
        return ImuHealthCheck::Clock::time_point{} + t;
    }

    // What the reader gets from a chip that does not answer. The firmware still derives a linear
    // acceleration of -1 g on z from the zero accelerometer.
    SensorData silentChip()
    {
        SensorData d;
        d.linearAcceleration.z = -1.0f;
        return d;
    }

    SensorData working()
    {
        SensorData d;
        d.accelerometer.z = 9.81f;
        d.temperature = 31.5f;
        d.heading = 120.0f;
        return d;
    }
}

TEST(ImuHealthCheckTest, ZeroTemperatureAndHeadingForTheWholeWindowReportOnce)
{
    ImuHealthCheck check{10s};
    EXPECT_FALSE(check.observe(silentChip(), at(0ms)));
    EXPECT_FALSE(check.observe(silentChip(), at(5s)));
    EXPECT_FALSE(check.observe(silentChip(), at(9999ms)));
    EXPECT_TRUE(check.observe(silentChip(), at(10s)));
    EXPECT_FALSE(check.observe(silentChip(), at(11s))) << "reported only once";
}

TEST(ImuHealthCheckTest, ADataSampleInTheWindowMeansTheImuWorks)
{
    ImuHealthCheck check{10s};
    EXPECT_FALSE(check.observe(silentChip(), at(0ms)));
    EXPECT_FALSE(check.observe(working(), at(2s)));
    EXPECT_FALSE(check.observe(silentChip(), at(20s))) << "a working IMU is not checked again";
}

TEST(ImuHealthCheckTest, TemperatureAloneOrHeadingAloneIsData)
{
    SensorData d;
    d.temperature = 25.0f;
    EXPECT_FALSE(ImuHealthCheck::silent(d));
    d = {};
    d.heading = 12.0f;
    EXPECT_FALSE(ImuHealthCheck::silent(d));
    EXPECT_TRUE(ImuHealthCheck::silent(SensorData{}));
    EXPECT_TRUE(ImuHealthCheck::silent(silentChip()));
}

TEST(ImuHealthCheckTest, TheWindowStartsAtTheFirstSample)
{
    ImuHealthCheck check{10s};
    // The reader may run a long time before the first sample arrives.
    EXPECT_FALSE(check.observe(silentChip(), at(100s)));
    EXPECT_FALSE(check.observe(silentChip(), at(105s)));
    EXPECT_TRUE(check.observe(silentChip(), at(110s)));
}
