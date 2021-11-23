#include "MeshSubdivide.h"
#include <igl/edge_flaps.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/is_edge_manifold.h>
#include <algorithm>

namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		MeshSubdivide::MeshSubdivide(Base::TriMesh& mesh) :m_mesh(mesh) {
			if (!igl::is_edge_manifold(m_mesh.m_faces)) {
				std::cerr << "Error when spliting: not edge-manifold" << std::endl;
				exit(-1);
			}
			igl::edge_flaps(m_mesh.m_faces, E, EMAP, EF, EI);
		}

		std::vector<int> MeshSubdivide::get_vertex_face_loop(int vi){
			int start_fi = m_mesh.m_vf_adjacency[vi][0];
			int ccw_start_ei = -1, cw_start_ei = -1;
			int num_face = m_mesh.m_faces.rows();
			for (int i = 0; i < 3; ++i) {
				if (m_mesh.m_faces(start_fi, i) == vi) {
					i = (i + 1) % 3;
					ccw_start_ei = EMAP(start_fi + i * num_face);
					i = (i + 1) % 3;
					cw_start_ei = EMAP(start_fi + i * num_face);
					break;
				}
			}
			
			int cur_fi = start_fi, cur_ei = ccw_start_ei;
			auto search_loop = [&](bool ccw)->std::vector<int> {
				std::vector<int> result;
				do {
					result.push_back(cur_fi);
					int nside = (EF(cur_ei, 0) == cur_fi) ? 1 : 0;
					int nv = EI(cur_ei, nside);
					cur_fi = EF(cur_ei, nside);
					if (cur_fi == -1) {
						break;
					}
					int dir = ccw ? -1 : 1;
					cur_ei = EMAP(cur_fi + ((nv + dir + 3) % 3)*num_face);
				} while (cur_fi != start_fi);
				return result;
			};
			//search in counter clock wise order
			std::vector<int> ccw_result = search_loop(true);
			if (cur_fi == -1) {
				cur_fi = start_fi, cur_ei = cw_start_ei;
				std::vector<int> cw_result = search_loop(false);
				std::reverse(cw_result.begin(), cw_result.end());
				cw_result.pop_back();
				ccw_result.insert(ccw_result.end(), cw_result.begin(), cw_result.end());
				ccw_result.push_back(-1);
			}
			return ccw_result;
			
		}

		std::set<MeshSubdivide::PlaneBoundaryPath> MeshSubdivide::get_vertex_path_set(int vi) {
			std::set<MeshSubdivide::PlaneBoundaryPath> result;

			/*std::vector<int> n_faces = get_vertex_face_loop(vi);			
			bool is_on_boundary = false;
			if (n_faces.back() == -1) {
				is_on_boundary = true;
				n_faces.pop_back();
			}

			for (int i = 0; i < n_faces.size() - 1; ++i) {
				int p0 = m_mesh.m_face_plane_index[n_faces[i]];
				int p1 = m_mesh.m_face_plane_index[n_faces[i + 1]];
				if (p0 != -1 && p1 != -1 &&
					p0 != p1) {
					result.insert(PlaneBoundaryPath(p0,p1));
				}
			}

			if (!is_on_boundary) {
				int i = n_faces.size() - 1;
				int p0 = m_mesh.m_face_plane_index[n_faces[i]];
				int p1 = m_mesh.m_face_plane_index[n_faces[0]];
				if (p0 != -1 && p1 != -1 &&
					p0 != p1) {
					result.insert(PlaneBoundaryPath(p0, p1));
				}
			}*/

			std::unordered_set<int> plane_set = m_mesh.get_vertex_plane_set(vi);
			
			for (auto iter = plane_set.begin(); iter != plane_set.end(); ++iter) {
				for (auto n_iter = std::next(iter); n_iter != plane_set.end(); ++n_iter) {
					result.insert(PlaneBoundaryPath(*iter, *n_iter));
				}
			}
			
			return result;
		}

		bool MeshSubdivide::is_on_same_plane_boundary_path(int v0, int v1) {
			auto path0 = get_vertex_path_set(v0);
			auto path1 = get_vertex_path_set(v1);
			//printf("(%d, %d), (%d, %d)\n", v0, path0.size(), v1, path1.size());
			//path0 = path1;
			std::vector<MeshSubdivide::PlaneBoundaryPath> common_path;
			std::set_intersection(path0.begin(), path0.end(), path1.begin(), path1.end(), std::back_inserter(common_path));
			return common_path.size() > 0;
			/*if (ret) {
				printf("(%d, %d), (%d, %d)\n", v0, path0.size(), v1, path1.size());
			}*/
			
		}

		void MeshSubdivide::check_edge() {
			m_edge_new_vertex_index.resize(E.rows(), -1);
			int origin_num_vertices = m_mesh.m_vertices.rows();
			for (int ei = 0; ei < E.rows(); ++ei) {
				if (EF(ei, 0) == -1 || EF(ei, 1) == -1)
					continue;
				int p0 = m_mesh.m_face_plane_index[EF(ei, 0)];
				int p1 = m_mesh.m_face_plane_index[EF(ei, 1)];
				
				if (p0 == p1 && p0 != -1 &&is_on_same_plane_boundary_path(E(ei,0), E(ei,1))) {
					//need split
					Base::Vec3d mid_point = (m_mesh.m_vertices.row(E(ei, 0)) + m_mesh.m_vertices.row(E(ei, 1)))*0.5;
					m_edge_new_vertex_index[ei] = origin_num_vertices + m_new_vertex_buffer.size();
					m_new_vertex_buffer.push_back(mid_point);
				}
			}
		}

		void MeshSubdivide::split_face() {
			//add new vertices to mesh
			int new_num_vertex = m_new_vertex_buffer.size();
			Eigen::Matrix<double,-1,-1,Eigen::RowMajor> new_vertices = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(m_new_vertex_buffer[0].data(), new_num_vertex, 3);
			m_mesh.m_vertices.conservativeResize(m_mesh.m_vertices.rows() + new_num_vertex, 3);
			m_mesh.m_vertices.bottomRows(new_num_vertex) = new_vertices;

			//split faces
			int num_face = m_mesh.m_faces.rows();
			std::vector<Eigen::RowVector3i> new_faces;
			
			new_faces.reserve(num_face);
			for (int fi = 0; fi < num_face; ++fi) {
				std::vector<int> split_edge;
				for (int i = 0; i < 3; ++i) {
					int ei = EMAP(fi + i * num_face);
					if (m_edge_new_vertex_index[ei] != -1) {
						split_edge.push_back(ei);
					}
				}
				int sz = split_edge.size();
				if (sz == 0) {
					new_faces.push_back(m_mesh.m_faces.row(fi));
					m_face_map.push_back(fi);
				}
				else if (sz == 1) {
					int ei = split_edge[0];
					int i = EF(ei, 0) == fi ? EI(ei, 0) : EI(ei, 1);
					int vi = m_mesh.m_faces(fi, i), nvi = m_mesh.m_faces(fi, (i + 1) % 3), pvi = m_mesh.m_faces(fi, (i + 2) % 3);
					int new_vi = m_edge_new_vertex_index[ei];
					new_faces.push_back(Base::Vec3i(new_vi, vi, nvi));
					new_faces.push_back(Base::Vec3i(new_vi, pvi, vi));
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
				}
				else if (sz == 2) {
					
					int ei0 = split_edge[0], ei1 = split_edge[1];
					int i0 = EF(ei0, 0) == fi ? EI(ei0, 0) : EI(ei0, 1);
					int i1 = EF(ei1, 0) == fi ? EI(ei1, 0) : EI(ei1, 1);
					if ((i0 + 1) % 3 != i1) {
						std::swap(ei0, ei1);
						std::swap(i0, i1);
					}
					int vi0 = m_mesh.m_faces(fi, i0), vi1 = m_mesh.m_faces(fi, i1);
					int vi = m_mesh.m_faces(fi, (i1 + 1) % 3);
					int nvi0 = m_edge_new_vertex_index[ei0], nvi1 = m_edge_new_vertex_index[ei1];
					new_faces.push_back(Base::Vec3i(vi, nvi1, nvi0));
					new_faces.push_back(Base::Vec3i(nvi1, vi0, nvi0));
					new_faces.push_back(Base::Vec3i(nvi0, vi0, vi1));
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
				}
				else if (sz == 3) {
					int ei0 = split_edge[0], ei1 = split_edge[1], ei2 = split_edge[2];
					int i0 = EF(ei0, 0) == fi ? EI(ei0, 0) : EI(ei0, 1);
					int i1 = EF(ei1, 0) == fi ? EI(ei1, 0) : EI(ei1, 1);
					int i2 = EF(ei2, 0) == fi ? EI(ei2, 0) : EI(ei2, 1);
					int vi0 = m_mesh.m_faces(fi, i0), vi1 = m_mesh.m_faces(fi, i1), vi2 = m_mesh.m_faces(fi, i2);
					int nvi0 = m_edge_new_vertex_index[ei0], nvi1 = m_edge_new_vertex_index[ei1], nvi2 = m_edge_new_vertex_index[ei2];
					new_faces.push_back(Base::Vec3i(vi1, nvi0, nvi2));
					new_faces.push_back(Base::Vec3i(nvi0, nvi1, nvi2));
					new_faces.push_back(Base::Vec3i(nvi0, vi2, nvi1));
					new_faces.push_back(Base::Vec3i(vi0, nvi2, nvi1));
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
					m_face_map.push_back(fi);
				}
			}

			m_mesh.m_faces = Eigen::Map<Base::IndexMatrix>(new_faces[0].data(), new_faces.size(), 3);
			
		}

		void MeshSubdivide::update_mesh() {
			if (m_face_map.size() != m_mesh.m_faces.rows()) {
				printf("Error!!! Face map should be consistent with face\n");
				return;
			}

			igl::triangle_triangle_adjacency(m_mesh.m_faces, m_mesh.m_ff_adjacency);

			std::vector<std::vector<int>> VFi;
			igl::vertex_triangle_adjacency(m_mesh.m_vertices.rows(), m_mesh.m_faces, m_mesh.m_vf_adjacency, VFi);

			igl::per_face_normals(m_mesh.m_vertices, m_mesh.m_faces, m_mesh.m_face_normals);

			m_mesh.m_face_labels.resize(0);

			m_mesh.m_face_planar_score.resize(0);

			m_mesh.m_face_plane_index = m_mesh.m_face_plane_index(m_face_map).eval();

			for (Base::PlaneGroup& g : m_mesh.m_plane_groups) {
				g.m_indices.clear();
			}

			for (int fi = 0; fi < m_mesh.m_faces.rows(); ++fi) {
				int pi = m_mesh.m_face_plane_index[fi];
				if (pi != -1) {
					m_mesh.m_plane_groups[pi].m_indices.push_back(fi);
				}
			}


		}


		void MeshSubdivide::split() {
			check_edge();
			split_face();
			update_mesh();
		}
	}
}