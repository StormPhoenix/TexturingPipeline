//
// Created by Storm Phoenix on 2021/10/22.
//

#ifndef TEXTURINGPIPELINE_REMESHINGUTILS_H
#define TEXTURINGPIPELINE_REMESHINGUTILS_H

#include <Base/TriMesh.h>
#include <mve/mesh.h>

namespace TextureRemeshing {
    namespace Utils {
        mve::TriangleMesh::Ptr triMesh_to_mveMesh(MeshPolyRefinement::Base::TriMesh &tri_mesh);
    }
}

#endif //TEXTURINGPIPELINE_REMESHINGUTILS_H
