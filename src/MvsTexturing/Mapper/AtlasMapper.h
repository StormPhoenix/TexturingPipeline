//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_ATLASMAPPER_H
#define TEXTURINGPIPELINE_ATLASMAPPER_H

#include <Parameter.h>
#include <Base/View.h>
#include <Base/LabelGraph.h>
#include <Base/TexturePatch.h>
#include <Base/TextureAtlas.h>

#include <MvsTexturing.h>

namespace MvsTexturing {
    namespace AtlasMapper {
        typedef mve::TriangleMesh::Ptr MeshPtr;
        typedef mve::TriangleMesh::ConstPtr MeshConstPtr;
        typedef mve::MeshInfo MeshInfo;

        void generate_texture_patches(const Base::LabelGraph &graph, MeshConstPtr mesh,
                                      const MeshInfo &mesh_info, std::vector<Base::TextureView> *texture_views,
                                      const Parameter &settings,
                                      std::vector<std::vector<Base::VertexProjectionInfo>> *vertex_projection_infos,
                                      std::vector<Base::TexturePatch::Ptr> *texture_patches);

        void generate_texture_atlases(const Parameter &param,
                                      std::vector<Base::TexturePatch::Ptr> *orig_texture_patches,
                                      std::vector<Base::TextureAtlas::Ptr> *texture_atlases,
                                      bool tone_mapping_gamma = false);
    }
}

#endif //TEXTURINGPIPELINE_ATLASMAPPER_H
