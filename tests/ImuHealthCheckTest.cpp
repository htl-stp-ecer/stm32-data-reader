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

    // What the reader gets from an IMU that does not answer: every IMU field zero, also the quaternion
    // (a default SensorData has the identity quaternion).
    SensorData zeros()
    {
        SensorData d;
        d.dmpOrientation = {0.0f, 0.0f, 0.0f, 0.0f};
        return d;
    }

    SensorData working()
    {
        SensorData d;
        d.accelerometer.z = 9.81f;
        d.temperature = 31.5f;
        return d;
    }
}

TEST(ImuHealthCheckTest, AllZeroSamplesForTheWholeWindowReportOnce)
{
    ImuHealthCheck check{10s};
    const SensorData zero = zeros();
    EXPECT_FALSE(check.observe(zero, at(0ms)));
    EXPECT_FALSE(check.observe(zero, at(5s)));
    EXPECT_FALSE(check.observe(zero, at(9999ms)));
    EXPECT_TRUE(check.observe(zero, at(10s)));
    EXPECT_FALSE(check.observe(zero, at(11s))) << "reported only once";
}

TEST(ImuHealthCheckTest, AnyNonZeroSampleInTheWindowMeansTheImuWorks)
{
    ImuHealthCheck check{10s};
    const SensorData zero = zeros();
    EXPECT_FALSE(check.observe(zero, at(0ms)));
    EXPECT_FALSE(check.observe(working(), at(2s)));
    EXPECT_FALSE(check.observe(zero, at(20s))) << "a working IMU is not checked again";
}

TEST(ImuHealthCheckTest, EachImuValueCountsAsData)
{
    SensorData d = zeros();
    d.gyro.x = 0.01f;
    EXPECT_FALSE(ImuHealthCheck::allZero(d));
    d = zeros();
    d.dmpOrientation.w = 1.0f;
    EXPECT_FALSE(ImuHealthCheck::allZero(d));
    d = zeros();
    d.heading = 12.0f;
    EXPECT_FALSE(ImuHealthCheck::allZero(d));
    d = zeros();
    d.temperature = 25.0f;
    EXPECT_FALSE(ImuHealthCheck::allZero(d));
    EXPECT_TRUE(ImuHealthCheck::allZero(zeros()));
    EXPECT_FALSE(ImuHealthCheck::allZero(SensorData{})) << "the identity quaternion is a value";
}

TEST(ImuHealthCheckTest, TheWindowStartsAtTheFirstSample)
{
    ImuHealthCheck check{10s};
    const SensorData zero = zeros();
    // The reader may run a long time before the first sample arrives.
    EXPECT_FALSE(check.observe(zero, at(100s)));
    EXPECT_FALSE(check.observe(zero, at(105s)));
    EXPECT_TRUE(check.observe(zero, at(110s)));
}
