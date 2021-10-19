//
// Created by Storm Phoenix on 2021/10/18.
//

#ifndef TEXTURINGPIPELINE_IO_H
#define TEXTURINGPIPELINE_IO_H

#include <string>
#include <mve/mesh.h>

namespace MvsTexturing {
    namespace IO {
        namespace MVE {
            mve::TriangleMesh::Ptr load_ply_mesh (const std::string &filename);
        }
    }
}

#endif //TEXTURINGPIPELINE_IO_H
