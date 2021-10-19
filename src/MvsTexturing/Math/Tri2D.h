//
// Created by Storm Phoenix on 2021/10/15.
//

#ifndef TEXTURINGPIPELINE_TRI2D_H
#define TEXTURINGPIPELINE_TRI2D_H

#include <math/vector.h>
#include <math/matrix.h>

#include "Rect2D.h"

namespace MvsTexturing {
    namespace Math {
        class Tri2D {
        private:
            math::Vec2f v1;
            math::Vec2f v2;
            math::Vec2f v3;
            float detT;

            Rect2D<float> aabb;
        public:
            /** Constructor which calculates the axis aligned bounding box and prepares the calculation of barycentric coordinates. */
            Tri2D(math::Vec2f v1, math::Vec2f v2, math::Vec2f v3);

            /** Determines whether the given point is inside via barycentric coordinates. */
            bool inside(float x, float y) const;

            /** Returns the barycentric coordinates for the given point. */
            math::Vec3f get_barycentric_coords(float x, float y) const;

            /** Returns the area of the triangle. */
            float get_area(void) const;

            /** Returns the axis aligned bounding box. */
            Rect2D<float> get_aabb(void) const;
        };

        inline Rect2D<float>
        Tri2D::get_aabb(void) const {
            return aabb;
        }

        inline math::Vec3f
        Tri2D::get_barycentric_coords(float x, float y) const {
            float const alpha = ((v2[1] - v3[1]) * (x - v3[0]) + (v3[0] - v2[0]) * (y - v3[1])) / detT;
            float const beta = ((v3[1] - v1[1]) * (x - v3[0]) + (v1[0] - v3[0]) * (y - v3[1])) / detT;
            float const gamma = 1.0f - alpha - beta;
            return math::Vec3f(alpha, beta, gamma);
        }

        inline bool
        Tri2D::inside(float x, float y) const {
            float const dx = (x - v3[0]);
            float const dy = (y - v3[1]);

            float const alpha = ((v2[1] - v3[1]) * dx + (v3[0] - v2[0]) * dy) / detT;
            if (alpha < 0.0f || alpha > 1.0f)
                return false;

            float const beta = ((v3[1] - v1[1]) * dx + (v1[0] - v3[0]) * dy) / detT;
            if (beta < 0.0f || beta > 1.0f)
                return false;

            if (alpha + beta > 1.0f)
                return false;

            /* else */
            return true;
        }

        inline float
        Tri2D::get_area(void) const {
            math::Vec2f u = v2 - v1;
            math::Vec2f v = v3 - v1;

            return 0.5f * std::abs(u[0] * v[1] - u[1] * v[0]);
        }
    }
}

#endif //TEXTURINGPIPELINE_TRI2D_H
