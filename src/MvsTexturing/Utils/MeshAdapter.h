//
// Created by Storm Phoenix on 2021/11/11.
//

#ifndef TEXTURINGPIPELINE_MESHADAPTER_H
#define TEXTURINGPIPELINE_MESHADAPTER_H

#include <Base/TriMesh.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <MvsTexturing.h>

namespace MvsTexturing {
    namespace Utils {
        typedef MeshPolyRefinement::Base::TriMesh TriMesh;

        bool eigenMesh_to_TriMesh(const Base::AttributeMatrix &mesh_vertices,
                                  const Base::IndexMatrix &mesh_faces,
                                  TriMesh &tri_mesh);

        MeshPtr eigenMesh_to_mveMesh(const Base::AttributeMatrix &V,
                                     const Base::IndexMatrix &F,
                                     const Base::AttributeMatrix &FC);

        MeshPtr eigenMesh_to_mveMesh(const Base::AttributeMatrix &V,
                                     const Base::IndexMatrix &F);

        MeshPtr triMesh_to_mveMesh(TriMesh &tri_mesh);

        void mveMesh_to_triMesh(MeshConstPtr mve_mesh, TriMesh &tri_mesh);
    }
}

#endif //TEXTURINGPIPELINE_MESHADAPTER_H
