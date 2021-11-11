//
// Created by Storm Phoenix on 2021/11/10.
//

#ifndef TEXTURINGPIPELINE_SEAMSMOOTHER_H
#define TEXTURINGPIPELINE_SEAMSMOOTHER_H

#include <mve/mesh.h>
#include <mve/mesh_info.h>

#include <Base/LabelGraph.h>
#include <Base/TexturePatch.h>

namespace MvsTexturing {
    namespace SeamSmoother {
        void global_seam_leveling(const Base::LabelGraph &graph,
                                  mve::TriangleMesh::ConstPtr mesh,
                                  const mve::MeshInfo &mesh_info,
                                  const std::vector<std::vector<Base::VertexProjectionInfo>> &vertex_projection_infos,
                                  std::vector<Base::TexturePatch::Ptr> *texture_patches);

        void local_seam_leveling(const Base::LabelGraph &graph, mve::TriangleMesh::ConstPtr mesh,
                                 const std::vector<std::vector<Base::VertexProjectionInfo>> &vertex_projection_infos,
                                 std::vector<Base::TexturePatch::Ptr> *texture_patches);
    }
}

#endif //TEXTURINGPIPELINE_SEAMSMOOTHER_H
