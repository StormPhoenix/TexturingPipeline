//
// Created by Storm Phoenix on 2021/11/16.
//

#ifndef TEXTURINGPIPELINE_MESHSUBDIVISION_H
#define TEXTURINGPIPELINE_MESHSUBDIVISION_H

#include <Eigen/Core>

namespace MvsTexturing {
    namespace MeshSubdivision {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        bool make_mesh_dense(const AttributeMatrix &V, const IndexMatrix &F,
                             AttributeMatrix &out_V, IndexMatrix &out_F, AttributeMatrix &out_FC,
                             AttributeMatrix &out_dense_FC, const std::size_t kMaxFaces = 800000);
    }
}

#endif //TEXTURINGPIPELINE_MESHSUBDIVISION_H
