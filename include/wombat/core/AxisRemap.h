//
// Body-frame to world-frame axis remapping.
//
// The user configures which body axis maps to each world axis
// (e.g. world_z = body_y for a robot where body Y is "up").
// This remap is applied to ALL sensor data (vectors and quaternion)
// before publishing to LCM, so all downstream consumers receive
// world-frame data with Z = up.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include <array>
#include <cmath>
#include <cstdint>

namespace wombat
{
    struct AxisRemap
    {
        // 3x3 rotation matrix (row-major) body → world.  Default: identity.
        std::array<float, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};

        // Pre-computed quaternion Q_R representing the same rotation.
        Quaternionf qR{1.0f, 0.0f, 0.0f, 0.0f};

        bool identity{true};

        // ----------------------------------------------------------------
        // Build from the int8 orientation matrix sent via LCM.
        // The matrix must be a proper rotation (det = +1) composed of
        // elements in {-1, 0, 1}.
        // ----------------------------------------------------------------
        void setFromMatrix(const int8_t m[9])
        {
            for (int i = 0; i < 9; ++i)
                R[i] = static_cast<float>(m[i]);

            // Compute determinant
            const float det =
                R[0] * (R[4] * R[8] - R[5] * R[7]) -
                R[1] * (R[3] * R[8] - R[5] * R[6]) +
                R[2] * (R[3] * R[7] - R[4] * R[6]);

            // If det == -1, negate second row to make it a proper rotation.
            // This enforces a right-handed world frame.
            if (det < 0.0f)
            {
                R[3] = -R[3];
                R[4] = -R[4];
                R[5] = -R[5];
            }

            identity = (R[0] == 1 && R[4] == 1 && R[8] == 1 &&
                R[1] == 0 && R[2] == 0 && R[3] == 0 &&
                R[5] == 0 && R[6] == 0 && R[7] == 0);

            matrixToQuaternion();
        }

        // ----------------------------------------------------------------
        // Remap a 3-vector: v_world = R * v_body
        // ----------------------------------------------------------------
        Vector3f remapVector(const Vector3f& v) const
        {
            if (identity) return v;
            return {
                R[0] * v.x + R[1] * v.y + R[2] * v.z,
                R[3] * v.x + R[4] * v.y + R[5] * v.z,
                R[6] * v.x + R[7] * v.y + R[8] * v.z
            };
        }

        // ----------------------------------------------------------------
        // Remap a quaternion: Q_world = Q_body * Q_R^{-1}
        //
        // Proof:
        //   Q_body rotates body→earth: v_earth = Q_body·v_body·Q_body^-1
        //   Q_R   rotates body→world:  v_world = Q_R·v_body·Q_R^-1
        //                        i.e.  v_body  = Q_R^-1·v_world·Q_R
        //
        //   v_earth = Q_body · (Q_R^-1·v_world·Q_R) · Q_body^-1
        //           = (Q_body·Q_R^-1) · v_world · (Q_body·Q_R^-1)^-1
        //
        //   So Q_world = Q_body · Q_R^-1 = Q_body · conj(Q_R)
        // ----------------------------------------------------------------
        Quaternionf remapQuaternion(const Quaternionf& q) const
        {
            if (identity) return q;

            // conj(qR) = (qR.w, -qR.x, -qR.y, -qR.z)
            const float cw = qR.w, cx = -qR.x, cy = -qR.y, cz = -qR.z;

            // Hamilton product: q * conj(qR)
            return {
                q.w * cw - q.x * cx - q.y * cy - q.z * cz,
                q.w * cx + q.x * cw + q.y * cz - q.z * cy,
                q.w * cy - q.x * cz + q.y * cw + q.z * cx,
                q.w * cz + q.x * cy - q.y * cx + q.z * cw
            };
        }

    private:
        // Shepperd's method: rotation matrix → unit quaternion
        void matrixToQuaternion()
        {
            const float r00 = R[0], r01 = R[1], r02 = R[2];
            const float r10 = R[3], r11 = R[4], r12 = R[5];
            const float r20 = R[6], r21 = R[7], r22 = R[8];

            const float trace = r00 + r11 + r22;
            float w, x, y, z;

            if (trace > 0.0f)
            {
                const float s = 0.5f / std::sqrt(trace + 1.0f);
                w = 0.25f / s;
                x = (r21 - r12) * s;
                y = (r02 - r20) * s;
                z = (r10 - r01) * s;
            }
            else if (r00 > r11 && r00 > r22)
            {
                const float s = 2.0f * std::sqrt(1.0f + r00 - r11 - r22);
                w = (r21 - r12) / s;
                x = 0.25f * s;
                y = (r01 + r10) / s;
                z = (r02 + r20) / s;
            }
            else if (r11 > r22)
            {
                const float s = 2.0f * std::sqrt(1.0f + r11 - r00 - r22);
                w = (r02 - r20) / s;
                x = (r01 + r10) / s;
                y = 0.25f * s;
                z = (r12 + r21) / s;
            }
            else
            {
                const float s = 2.0f * std::sqrt(1.0f + r22 - r00 - r11);
                w = (r10 - r01) / s;
                x = (r02 + r20) / s;
                y = (r12 + r21) / s;
                z = 0.25f * s;
            }

            // Normalize
            const float n = std::sqrt(w * w + x * x + y * y + z * z);
            qR = {w / n, x / n, y / n, z / n};
        }
    };
} // namespace wombat