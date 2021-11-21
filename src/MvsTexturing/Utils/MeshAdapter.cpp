//
// Created by Storm Phoenix on 2021/11/11.
//

#include <Base/TriMesh.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include "MvsTexturing.h"

#include <DataIO/IO.h>

namespace MvsTexturing {
    namespace Utils {
        typedef MeshPolyRefinement::Base::TriMesh TriMesh;

        bool eigenMesh_to_TriMesh(const Base::AttributeMatrix &mesh_vertices,
                                  const Base::IndexMatrix &mesh_faces,
                                  TriMesh &tri_mesh) {
            if (mesh_vertices.rows() <= 0 || mesh_faces.rows() <= 0) {
                return false;
            }

            std::vector<double> tmp_vertices(mesh_vertices.rows() * 3);
            for (int i = 0; i < mesh_vertices.rows(); i++) {
                for (int j = 0; j < 3; j++) {
                    tmp_vertices[i * 3 + j] = mesh_vertices(i, j);
                }
            }

            std::vector<std::size_t> tmp_faces(mesh_faces.rows() * 3);
            for (int i = 0; i < mesh_faces.rows(); i++) {
                for (int j = 0; j < 3; j++) {
                    tmp_faces[i * 3 + j] = mesh_faces(i, j);
                }
            }
            return MeshPolyRefinement::IO::read_mesh_from_memory(tmp_vertices, tmp_faces, tri_mesh);

        }

        MeshPtr eigenMesh_to_mveMesh(const Base::AttributeMatrix &V,
                                     const Base::IndexMatrix &F,
                                     const Base::AttributeMatrix &FC) {
            if (V.rows() <= 0 || F.rows() <= 0) {
                return nullptr;
            }

            if (FC.rows() > 0 && FC.rows() != F.rows()) {
                return nullptr;
            }

            mve::TriangleMesh::Ptr ret = mve::TriangleMesh::create();

            mve::TriangleMesh::VertexList &vertices_list = ret->get_vertices();
            mve::TriangleMesh::FaceList &faces_list = ret->get_faces();
            mve::TriangleMesh::ColorList &face_colors_list = ret->get_face_colors();

            // copy vertices
            Eigen::Index n_vertices = V.rows();
            vertices_list.resize(n_vertices);
            for (int i = 0; i < n_vertices; i++) {
                math::Vec3f &v = vertices_list[i];
                v[0] = V(i, 0);
                v[1] = V(i, 1);
                v[2] = V(i, 2);
            }

            // copy faces
            Eigen::Index n_faces = F.rows();
            faces_list.resize(3 * n_faces);
            for (int i = 0; i < n_faces * 3; i += 3) {
                faces_list[i + 0] = F(i / 3, 0);
                faces_list[i + 1] = F(i / 3, 1);
                faces_list[i + 2] = F(i / 3, 2);
            }

            // copy face colors
            Eigen::Index n_face_colors = FC.rows();
            face_colors_list.resize(n_face_colors);
            for (int i = 0; i < n_face_colors; i++) {
                math::Vec4f &f_color = face_colors_list[i];
                for (int c = 0; c < FC.cols(); c++) {
                    f_color[c] = FC(i, c);
                }
            }

            mve::MeshInfo mesh_info(ret);
            MvsTexturing::Builder::MVE::prepare_mesh(&mesh_info, ret);

            return ret;
        }

        MeshPtr eigenMesh_to_mveMesh(const Base::AttributeMatrix &V,
                                     const Base::IndexMatrix &F) {
            Base::AttributeMatrix FC;
            return eigenMesh_to_mveMesh(V, F, FC);
        }

        MeshPtr triMesh_to_mveMesh(TriMesh &tri_mesh) {
            mve::TriangleMesh::Ptr ans = mve::TriangleMesh::create();

            mve::TriangleMesh::VertexList &vertices_list = ans->get_vertices();
            mve::TriangleMesh::FaceList &faces_list = ans->get_faces();

            Eigen::Index n_vertices = tri_mesh.m_vertices.rows();
            vertices_list.resize(n_vertices);
            for (int i = 0; i < n_vertices; i++) {
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

        void mveMesh_to_triMesh(MeshConstPtr mve_mesh, TriMesh &tri_mesh) {
            // copy vertices
            const mve::MeshBase::VertexList &vertices = mve_mesh->get_vertices();
            std::vector<double> tmp_vertices(vertices.size() * 3);
            for (std::size_t i = 0; i < vertices.size(); i++) {
                for (std::size_t j = 0; j < 3; j++) {
                    tmp_vertices[i * 3 + j] = vertices[i][j];
                }
            }

            // copy faces
            const mve::TriangleMesh::FaceList &faces = mve_mesh->get_faces();
            std::vector<std::size_t> tmp_faces(faces.size());
            for (std::size_t i = 0; i < faces.size(); i++) {
                tmp_faces[i] = faces[i];
            }

            MeshPolyRefinement::IO::read_mesh_from_memory(tmp_vertices, tmp_faces, tri_mesh);
        }
    }
}