//
// Created by tobias on 9/14/25.
//
#pragma once

#include <stdexcept>
#include <string>
#include <variant>

namespace wombat
{
    template <typename T>
    class Result
    {
    public:
        static Result success(T value)
        {
            return Result{std::move(value)};
        }

        static Result failure(std::string error)
        {
            return Result{std::move(error)};
        }

        [[nodiscard]] bool isSuccess() const noexcept
        {
            return std::holds_alternative<T>(data_);
        }

        [[nodiscard]] bool isFailure() const noexcept
        {
            return std::holds_alternative<std::string>(data_);
        }

        [[nodiscard]] const T& value() const
        {
            if (isFailure())
            {
                throw std::runtime_error("Accessing value of failed Result: " + error());
            }
            return std::get<T>(data_);
        }

        [[nodiscard]] T valueOr(T defaultValue) const
        {
            return isSuccess() ? std::get<T>(data_) : std::move(defaultValue);
        }

        [[nodiscard]] const std::string& error() const
        {
            if (isSuccess())
            {
                throw std::runtime_error("Accessing error of successful Result");
            }
            return std::get<std::string>(data_);
        }

    private:
        explicit Result(T value) : data_{std::move(value)}
        {
        }

        explicit Result(std::string error) : data_{std::move(error)}
        {
        }

        std::variant<T, std::string> data_;
    };

    template <>
    class Result<void>
    {
    public:
        static Result success();

        static Result failure(std::string error);

        [[nodiscard]] bool isSuccess() const noexcept;

        [[nodiscard]] bool isFailure() const noexcept;

        [[nodiscard]] const std::string& error() const;

    private:
        explicit Result(bool success);

        explicit Result(std::string error);

        bool success_;
        std::string error_;
    };
}
