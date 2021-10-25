//
// Created by Storm Phoenix on 2021/10/11.
//

#ifndef TEXTURINGPIPELINE_MVSTEXTURING_H
#define TEXTURINGPIPELINE_MVSTEXTURING_H

#include <mve/mesh.h>
#include <mve/mesh_info.h>

#include <util/system.h>
#include <util/file_system.h>

#include <IO/IO.h>
#include <Base/View.h>
#include <Base/LabelGraph.h>
#include <Base/TextureAtlas.h>
#include <Utils/Timer.h>
#include <TextureMapper/SceneBuilder.h>
#include <TextureMapper/ViewSelection.h>

namespace MvsTexturing {
    typedef mve::TriangleMesh::Ptr MeshPtr;
    typedef mve::TriangleMesh::ConstPtr MeshConstPtr;
    typedef mve::MeshInfo MeshInfo;

    namespace Base {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
    }
}

#endif //TEXTURINGPIPELINE_MVSTEXTURING_H
