//
// Created by Storm Phoenix on 2021/10/18.
//

#ifndef TEXTURINGPIPELINE_IO_H
#define TEXTURINGPIPELINE_IO_H

#include <string>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <Base/TextureAtlas.h>

namespace MvsTexturing {
    namespace IO {
        namespace MVE {
            mve::TriangleMesh::Ptr load_ply_mesh (const std::string &filename);

            void save_obj_mesh(const std::string &filename, mve::TriangleMesh::ConstPtr mesh,
                               const std::vector<Base::TextureAtlas::Ptr> &texture_atlases);
        }
    }
}

#endif //TEXTURINGPIPELINE_IO_H
