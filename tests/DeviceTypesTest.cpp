#include "wombat/core/DeviceTypes.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace wombat;

// --- Vector3f ---

TEST(Vector3fTest, DefaultIsZero)
{
    Vector3f v{};
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vector3fTest, EqualVectors)
{
    Vector3f a{1.0f, 2.0f, 3.0f};
    Vector3f b{1.0f, 2.0f, 3.0f};
    EXPECT_EQ(a, b);
}

TEST(Vector3fTest, UnequalVectors)
{
    Vector3f a{1.0f, 2.0f, 3.0f};
    Vector3f b{1.0f, 2.0f, 4.0f};
    EXPECT_FALSE(a == b);
}

TEST(Vector3fTest, EpsilonComparison)
{
    Vector3f a{1.0f, 2.0f, 3.0f};
    // Differences smaller than 1e-6 should be equal
    Vector3f b{1.0f + 5e-7f, 2.0f - 5e-7f, 3.0f + 5e-7f};
    EXPECT_EQ(a, b);
}

TEST(Vector3fTest, BeyondEpsilonNotEqual)
{
    Vector3f a{1.0f, 2.0f, 3.0f};
    Vector3f b{1.0f + 2e-6f, 2.0f, 3.0f};
    EXPECT_FALSE(a == b);
}

// --- Quaternionf ---

TEST(QuaternionfTest, DefaultIsIdentity)
{
    Quaternionf q{};
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionfTest, EqualQuaternions)
{
    Quaternionf a{0.5f, 0.5f, 0.5f, 0.5f};
    Quaternionf b{0.5f, 0.5f, 0.5f, 0.5f};
    EXPECT_EQ(a, b);
}

TEST(QuaternionfTest, UnequalQuaternions)
{
    Quaternionf a{1.0f, 0.0f, 0.0f, 0.0f};
    Quaternionf b{0.0f, 1.0f, 0.0f, 0.0f};
    EXPECT_FALSE(a == b);
}

TEST(QuaternionfTest, EpsilonComparison)
{
    Quaternionf a{1.0f, 0.0f, 0.0f, 0.0f};
    Quaternionf b{1.0f + 5e-7f, 0.0f, 0.0f, 0.0f};
    EXPECT_EQ(a, b);
}

// --- ImuAccuracy ---

TEST(ImuAccuracyTest, DefaultIsZero)
{
    ImuAccuracy a{};
    EXPECT_EQ(a.gyro, 0);
    EXPECT_EQ(a.accelerometer, 0);
    EXPECT_EQ(a.linearAcceleration, 0);
    EXPECT_EQ(a.compass, 0);
    EXPECT_EQ(a.quaternion, 0);
}

TEST(ImuAccuracyTest, Equality)
{
    ImuAccuracy a{3, 3, 3, 3, 3};
    ImuAccuracy b{3, 3, 3, 3, 3};
    EXPECT_EQ(a, b);
}

TEST(ImuAccuracyTest, Inequality)
{
    ImuAccuracy a{3, 3, 3, 3, 3};
    ImuAccuracy b{3, 2, 3, 3, 3};
    EXPECT_FALSE(a == b);
}

// --- MotorState defaults ---

TEST(MotorStateTest, DefaultValues)
{
    MotorState m{};
    EXPECT_EQ(m.direction, MotorDirection::Off);
    EXPECT_EQ(m.controlMode, MotorControlMode::Pwm);
    EXPECT_EQ(m.speed, 0u);
    EXPECT_EQ(m.backEmf, 0);
    EXPECT_EQ(m.position, 0);
    EXPECT_FALSE(m.done);
}

// --- ServoState defaults ---

TEST(ServoStateTest, DefaultValues)
{
    ServoState s{};
    EXPECT_EQ(s.mode, ServoMode::Disabled);
    EXPECT_EQ(s.position, 0u);
}

// --- SensorData defaults ---

TEST(SensorDataTest, DefaultValues)
{
    SensorData d{};
    EXPECT_EQ(d.gyro, Vector3f{});
    EXPECT_EQ(d.accelerometer, Vector3f{});
    EXPECT_EQ(d.orientation, Quaternionf{});
    EXPECT_FLOAT_EQ(d.temperature, 0.0f);
    EXPECT_FLOAT_EQ(d.batteryVoltage, 0.0f);
    EXPECT_EQ(d.digitalBits, 0u);
    EXPECT_EQ(d.lastUpdate, 0u);
}