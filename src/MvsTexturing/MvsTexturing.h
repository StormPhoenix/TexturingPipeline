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
#include <Base/SparseTable.h>
#include <Base/TextureAtlas.h>
#include <Utils/Timer.h>
#include <Mapper/SceneBuilder.h>
#include <Mapper/ViewSelection.h>

namespace MvsTexturing {
    typedef mve::TriangleMesh::Ptr MeshPtr;
    typedef mve::TriangleMesh::ConstPtr MeshConstPtr;
    typedef mve::MeshInfo MeshInfo;
    typedef std::vector<Base::TextureView> TextureViewList;
    typedef std::vector<Base::TexturePatch::Ptr> TexturePatchList;
    typedef std::vector<Base::TextureAtlas::Ptr> TextureAtlasList;
    typedef std::vector<std::vector<Base::FaceProjectionInfo>> FaceProjectionInfoList;
    typedef std::vector<std::vector<Base::VertexProjectionInfo>> VertexProjectionInfoList;
    typedef std::vector<Base::EdgeProjectionInfo> EdgeProjectionInfoList;
    typedef Base::SparseTable<std::uint32_t, std::uint16_t, double> DataCosts;

    namespace Base {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
        using Vec3 = Eigen::Matrix<Base::Scalar, 1, 3>;
    }
}

#endif //TEXTURINGPIPELINE_MVSTEXTURING_H
