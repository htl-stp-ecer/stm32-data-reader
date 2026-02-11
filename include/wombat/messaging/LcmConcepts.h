#pragma once

#include <concepts>
#include <cstdint>

namespace wombat
{
    template <typename T>
    concept LcmMessage = requires(T t, const T ct, void* buf, const void* cbuf, int offset, int maxlen)
    {
        { t.timestamp } -> std::convertible_to<int64_t>;
        { T::getHash() } -> std::same_as<int64_t>;
        { T::getTypeName() } -> std::convertible_to<const char*>;
        { ct.encode(buf, offset, maxlen) } -> std::same_as<int>;
        { t.decode(cbuf, offset, maxlen) } -> std::same_as<int>;
    };
}