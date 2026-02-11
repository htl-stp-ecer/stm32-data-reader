#include "wombat/core/Result.h"
#include <gtest/gtest.h>
#include <string>

using namespace wombat;

// --- Result<T> ---

TEST(ResultTest, SuccessHoldsValue)
{
    auto r = Result<int>::success(42);
    EXPECT_TRUE(r.isSuccess());
    EXPECT_FALSE(r.isFailure());
    EXPECT_EQ(r.value(), 42);
}

TEST(ResultTest, FailureHoldsError)
{
    auto r = Result<int>::failure("something went wrong");
    EXPECT_TRUE(r.isFailure());
    EXPECT_FALSE(r.isSuccess());
    EXPECT_EQ(r.error(), "something went wrong");
}

TEST(ResultTest, ValueOnFailureThrows)
{
    auto r = Result<int>::failure("bad");
    EXPECT_THROW(r.value(), std::runtime_error);
}

TEST(ResultTest, ErrorOnSuccessThrows)
{
    auto r = Result<int>::success(1);
    EXPECT_THROW(r.error(), std::runtime_error);
}

TEST(ResultTest, ValueOrReturnsValueOnSuccess)
{
    auto r = Result<int>::success(10);
    EXPECT_EQ(r.valueOr(99), 10);
}

TEST(ResultTest, ValueOrReturnsDefaultOnFailure)
{
    auto r = Result<int>::failure("oops");
    EXPECT_EQ(r.valueOr(99), 99);
}

TEST(ResultTest, SuccessWithFloatType)
{
    auto r = Result<float>::success(3.14f);
    EXPECT_TRUE(r.isSuccess());
    EXPECT_FLOAT_EQ(r.value(), 3.14f);
}

TEST(ResultTest, FailureWithFloatType)
{
    auto r = Result<float>::failure("err");
    EXPECT_TRUE(r.isFailure());
    EXPECT_EQ(r.error(), "err");
}

// --- Result<void> ---

TEST(ResultVoidTest, SuccessIsSuccess)
{
    auto r = Result<void>::success();
    EXPECT_TRUE(r.isSuccess());
    EXPECT_FALSE(r.isFailure());
}

TEST(ResultVoidTest, FailureIsFailure)
{
    auto r = Result<void>::failure("fail");
    EXPECT_TRUE(r.isFailure());
    EXPECT_FALSE(r.isSuccess());
    EXPECT_EQ(r.error(), "fail");
}

TEST(ResultVoidTest, ErrorOnSuccessThrows)
{
    auto r = Result<void>::success();
    EXPECT_THROW(r.error(), std::runtime_error);
}