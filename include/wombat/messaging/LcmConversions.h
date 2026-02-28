#pragma once

#include "wombat/core/DeviceTypes.h"
#include <exlcm/vector3f_t.hpp>
#include <exlcm/quaternion_t.hpp>
#include <exlcm/scalar_f_t.hpp>
#include <exlcm/scalar_i32_t.hpp>
#include <exlcm/scalar_i8_t.hpp>
#include <exlcm/orientation_matrix_t.hpp>
#include <chrono>

namespace wombat
{
    inline int64_t currentTimestampUsec()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    inline exlcm::vector3f_t toLcm(const Vector3f& v)
    {
        exlcm::vector3f_t msg{};
        msg.timestamp = currentTimestampUsec();
        msg.x = v.x;
        msg.y = v.y;
        msg.z = v.z;
        return msg;
    }

    inline exlcm::quaternion_t toLcm(const Quaternionf& q)
    {
        exlcm::quaternion_t msg{};
        msg.timestamp = currentTimestampUsec();
        msg.w = q.w;
        msg.x = q.x;
        msg.y = q.y;
        msg.z = q.z;
        return msg;
    }

    inline exlcm::scalar_f_t toLcmScalarF(float value)
    {
        exlcm::scalar_f_t msg{};
        msg.timestamp = currentTimestampUsec();
        msg.value = value;
        return msg;
    }

    inline exlcm::scalar_i32_t toLcmScalarI32(int32_t value)
    {
        exlcm::scalar_i32_t msg{};
        msg.timestamp = currentTimestampUsec();
        msg.value = value;
        return msg;
    }

    inline exlcm::scalar_i8_t toLcmScalarI8(uint8_t value)
    {
        exlcm::scalar_i8_t msg{};
        msg.timestamp = currentTimestampUsec();
        msg.dir = value;
        return msg;
    }

    inline exlcm::orientation_matrix_t toLcmOrientationMatrix(const int8_t m[9])
    {
        exlcm::orientation_matrix_t msg{};
        msg.timestamp = currentTimestampUsec();
        for (int i = 0; i < 9; ++i) msg.m[i] = m[i];
        return msg;
    }
}