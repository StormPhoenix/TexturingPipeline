//
// Created by Storm Phoenix on 2021/10/18.
//

#ifndef TEXTURINGPIPELINE_SCENEBUILDER_H
#define TEXTURINGPIPELINE_SCENEBUILDER_H

#include <vector>
#include <Base/View.h>

namespace MvsTexturing {
    namespace Builder {
        void build_scene(const std::string &in_scene, std::vector<Base::TextureView> *texture_views,
                         const std::string &distortion_img_dir);

        namespace MVE {
            typedef mve::TriangleMesh::Ptr MeshPtr;
            typedef mve::TriangleMesh::ConstPtr MeshConstPtr;

            void prepare_mesh(mve::MeshInfo *mesh_info, MeshPtr mesh);
        }
    }
}

#endif //TEXTURINGPIPELINE_SCENEBUILDER_H
