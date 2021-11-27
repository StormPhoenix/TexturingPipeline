//
// Created by Storm Phoenix on 2021/11/27.
//

#ifndef TEXTURINGPIPELINE_FACEGROUP_H
#define TEXTURINGPIPELINE_FACEGROUP_H

#include <Eigen/Core>
#include <vector>

namespace MvsTexturing {
    using Vec3 = Eigen::Matrix<double, 1, 3>;
    using Vec2 = Eigen::Matrix<double, 1, 2>;
    using Vec4 = Eigen::Matrix<double, 1, 4>;

    namespace Base {
        class FaceGroup {
        public:
            std::vector<std::size_t> m_face_indices;
            Vec3 m_plane_normal, m_plane_center;
            Vec3 m_x_axis, m_y_axis;

            explicit FaceGroup() {}
        };
    }
}

#endif //TEXTURINGPIPELINE_FACEGROUP_H
