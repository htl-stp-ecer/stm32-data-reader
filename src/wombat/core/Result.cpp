//
// Created by tobias on 9/14/25.
//
#include "wombat/core/Result.h"

namespace wombat {

Result<void> Result<void>::success() {
    return Result{true};
}

Result<void> Result<void>::failure(std::string error) {
    return Result{std::move(error)};
}

bool Result<void>::isSuccess() const noexcept {
    return success_;
}

bool Result<void>::isFailure() const noexcept {
    return !success_;
}

const std::string& Result<void>::error() const {
    if (isSuccess()) {
        throw std::runtime_error("Accessing error of successful Result");
    }
    return error_;
}

Result<void>::Result(bool success) : success_{success} {}

Result<void>::Result(std::string error) : success_{false}, error_{std::move(error)} {}

}