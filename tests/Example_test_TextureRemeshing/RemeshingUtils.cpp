//
// Created by Storm Phoenix on 2021/10/22.
//

#include <Base/TriMesh.h>
#include <TextureMapper/SceneBuilder.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <memory>

namespace TextureRemeshing {
    namespace Utils {
        mve::TriangleMesh::Ptr triMesh_to_mveMesh(MeshPolyRefinement::Base::TriMesh &tri_mesh) {
            mve::TriangleMesh::Ptr ans = mve::TriangleMesh::create();

            mve::TriangleMesh::VertexList &vertices_list = ans->get_vertices();
            mve::TriangleMesh::FaceList &faces_list = ans->get_faces();

            Eigen::Index n_vertices = tri_mesh.m_vertices.rows();
            vertices_list.resize(n_vertices);
            for (int i = 0; i < n_vertices; i ++) {
                math::Vec3f &v = vertices_list[i];
                v[0] = tri_mesh.m_vertices(i, 0);
                v[1] = tri_mesh.m_vertices(i, 1);
                v[2] = tri_mesh.m_vertices(i, 2);
            }


            Eigen::Index n_faces = tri_mesh.m_faces.rows();
            faces_list.resize(3 * n_faces);
            for (int i = 0; i < n_faces * 3; i += 3) {
                faces_list[i + 0] = tri_mesh.m_faces(i / 3, 0);
                faces_list[i + 1] = tri_mesh.m_faces(i / 3, 1);
                faces_list[i + 2] = tri_mesh.m_faces(i / 3, 2);
            }

            mve::MeshInfo mesh_info(ans);
            MvsTexturing::Builder::MVE::prepare_mesh(&mesh_info, ans);

            return ans;
        }
    }
}