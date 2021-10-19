//
// Created by Storm Phoenix on 2021/10/15.
//
#include "Tri2D.h"

namespace MvsTexturing {
    namespace Math {
        Tri2D::Tri2D(math::Vec2f v1, math::Vec2f v2, math::Vec2f v3) :
                v1(v1), v2(v2), v3(v3) {
            math::Matrix2f T;
            T[0] = v1[0] - v3[0]; T[1] = v2[0] - v3[0];
            T[2] = v1[1] - v3[1]; T[3] = v2[1] - v3[1];

            detT = T[0] * T[3] - T[2] * T[1];

            aabb.min_x = std::min(v1[0], std::min(v2[0], v3[0]));
            aabb.min_y = std::min(v1[1], std::min(v2[1], v3[1]));
            aabb.max_x = std::max(v1[0], std::max(v2[0], v3[0]));
            aabb.max_y = std::max(v1[1], std::max(v2[1], v3[1]));
        }
    }
}