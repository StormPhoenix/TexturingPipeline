#include "PlaneMerge.h"
#include "RegionGrowing.h"
#include <igl/edge_flaps.h>
#include <iterator>
#include <DataIO/IO.h>

namespace MeshPolyRefinement {
	namespace PlaneEstimation {
		extern void estimate_plane(const Base::TriMesh&mesh,
			const std::vector<std::size_t>& face_indices,
			Base::Vec3& plane_normal,
			Base::Vec3& plane_center);

		PlaneMerge::PlaneMerge(Base::TriMesh& mesh):m_mesh(mesh) {
			m_cos_threshold = std::cos(15.0*M_PI / 180.0);
			m_line_cos_threshold = std::cos(85.0*M_PI / 180.0);
			m_plane_parent.resize(m_mesh.m_plane_groups.size());
			for (int i = 0; i < m_plane_parent.size(); ++i) {
				m_plane_parent[i] = i;
			}
			init_plane_neighbors();
		}

		void PlaneMerge::init_plane_neighbors(){
			
			m_plane_neighbors.resize(m_mesh.m_plane_groups.size());
			// Eigen::VectorXi EMAP;
			// Eigen::MatrixXi E, EF, EI;
			igl::edge_flaps(m_mesh.m_faces, E, EMAP, EF, EI);
//			printf("Total %d edges\n",E.rows());
//			printf("Max plane index %d, min plane index %d\n", m_mesh.m_face_plane_index.maxCoeff(),m_mesh.m_face_plane_index.minCoeff());
			for (int ei = 0; ei < E.rows(); ++ei) {

				int f0 = EF(ei, 0), f1 = EF(ei, 1); 
				
				// if(f0 != -1 && f1 != -1){
				// 	printf("ei %d, f0 %d, f1 %d, p0 %d, p1 %d\n", ei, f0, f1, 
				// 	m_mesh.m_face_plane_index[f0], m_mesh.m_face_plane_index[f1]);
				// }
				// else{
				// 	printf("ei %d, f0 %d, f1 %d\n", ei, f0, f1);
				// }
				if(f0 != -1 && m_mesh.m_face_plane_index[f0] != -1 &&
					(f1 == -1 || m_mesh.m_face_plane_index[f1] == -1)){
					m_plane_neighbors[m_mesh.m_face_plane_index[f0]].insert(-1);
					continue;
				}

				if(f1 != -1 && m_mesh.m_face_plane_index[f1] != -1 &&
					(f0 == -1 || m_mesh.m_face_plane_index[f0] == -1)){
					m_plane_neighbors[m_mesh.m_face_plane_index[f1]].insert(-1);
					continue;
				}

				if(f0 != -1 && m_mesh.m_face_plane_index[f0] != -1 &&
					f1 != -1 && m_mesh.m_face_plane_index[f1] != -1
					&& m_mesh.m_face_plane_index[f0] != m_mesh.m_face_plane_index[f1]){
					m_plane_neighbors[m_mesh.m_face_plane_index[f0]].insert(m_mesh.m_face_plane_index[f1]);
					m_plane_neighbors[m_mesh.m_face_plane_index[f1]].insert(m_mesh.m_face_plane_index[f0]);
				}
			}
			
			// for(int pi = 0; pi < m_plane_neighbors.size();++pi){
			// 	printf("Neighbor of plane %d:", pi);
			// 	for(auto iter = m_plane_neighbors[pi].begin();
			// 		iter != m_plane_neighbors[pi].end();++iter){
			// 		printf(" %d",*iter);
			// 	}
			// 	printf("\n");
			// }
			
		}

		int PlaneMerge::find_parent(int plane_index) {
			if (plane_index == -1)
				return -1;
			if (m_plane_parent[plane_index] != plane_index) {
				m_plane_parent[plane_index] = find_parent(m_plane_parent[plane_index]);
			}
			return m_plane_parent[plane_index];
		}

		double PlaneMerge::distance(const Base::PlaneGroup&g, const Base::Vec3& pt) const{
			return std::fabs(g.m_plane_normal.dot(pt - g.m_plane_center));
		}

		bool PlaneMerge::is_inner_plane(int pi){
			std::unordered_set<int> pn;
			for(auto iter = m_plane_neighbors[pi].begin();iter != m_plane_neighbors[pi].end(); ++iter){
				pn.insert(find_parent(*iter));
			}
			m_plane_neighbors[pi] = pn;
			return pn.size() == 1;
		}

		bool PlaneMerge::is_parallel_planes(int pi0, int pi1) {
			if (pi0 == -1 || pi1 == -1) {
				std::cout << "invalid plane index" << std::endl;
				return false;
			}
			
			if(is_inner_plane(pi0) || is_inner_plane(pi1)){
				//printf("Find plane surrounded by another plane\n");
				return true;
			}
			 const Base::PlaneGroup& g0 = m_mesh.m_plane_groups[pi0];
			 const Base::PlaneGroup& g1 = m_mesh.m_plane_groups[pi1];
			 //Base::Vec3 bigger_plane_normal = g0.m_indices.size() > g1.m_indices.size() ? g0.m_plane_normal:g1.m_plane_normal;
			 //Base::Vec3 center_line = (g0.m_plane_center - g1.m_plane_center).normalized();
			return g0.m_plane_normal.dot(g1.m_plane_normal) >= m_cos_threshold
				&& distance(m_mesh.m_plane_groups[pi0], m_mesh.m_plane_groups[pi1].m_plane_center) <= 30.0*m_mesh.m_plane_groups[pi0].m_avg_distance
				&& distance(m_mesh.m_plane_groups[pi1], m_mesh.m_plane_groups[pi0].m_plane_center) <= 30.0*m_mesh.m_plane_groups[pi1].m_avg_distance;
		}

		Base::PlaneGroup PlaneMerge::merge_plane(int pi0, int pi1) {
			return PlaneRegion::merge(m_mesh,m_mesh.m_plane_groups[pi0], m_mesh.m_plane_groups[pi1]);
		}

		void PlaneMerge::update_mesh() {
			// for (int fi = 0; fi < m_mesh.m_faces.rows(); ++fi) {
			// 	if (m_mesh.m_face_plane_index[fi] != -1) {
			// 		m_mesh.m_face_plane_index[fi] = find_parent(m_mesh.m_face_plane_index[fi]);
			// 	}
			// }

			// for (int pi = 0; pi < m_mesh.m_plane_groups.size(); ++pi) {
			// 	Base::PlaneGroup& group = m_mesh.m_plane_groups[pi];
			// 	if (group.m_indices.size() > 0) {
			// 		estimate_plane(m_mesh,
			// 			group.m_indices,
			// 			group.m_plane_normal,
			// 			group.m_plane_center);
			// 		group.params.head(3) = group.m_plane_normal;
			// 		group.params[3] = -group.m_plane_normal.dot(group.m_plane_center);
			// 		PlaneRegion::compute_axis(group);
			// 	}
			// }

			// std::vector<Base::PlaneGroup> new_group;
			// for (const Base::PlaneGroup& g : m_mesh.m_plane_groups) {
			// 	if (g.m_indices.size() > 0) {
			// 		int plane_idx = new_group.size();
			// 		for (int fi : g.m_indices) {
			// 			m_mesh.m_face_plane_index[fi] = plane_idx;
			// 		}
			// 		new_group.push_back(g);
			// 	}
			// }
			// m_mesh.m_plane_groups = new_group;

			std::vector<Base::PlaneGroup> tmp(m_mesh.m_plane_groups);

			
			m_mesh.m_plane_groups.clear();
			for (int pi = 0; pi < tmp.size(); ++pi) {
				Base::PlaneGroup& group = tmp[pi];
				if (group.m_indices.size() > 0) {
					estimate_plane(m_mesh,
						group.m_indices,
						group.m_plane_normal,
						group.m_plane_center);
					group.params.head(3) = group.m_plane_normal;
					group.params[3] = -group.m_plane_normal.dot(group.m_plane_center);
					PlaneRegion::compute_axis(group);
					m_mesh.m_plane_groups.push_back(group);
				}
				
			}

//			printf("origin %d planes, after merge %d planes\n", tmp.size(), m_mesh.m_plane_groups.size());
			for(int i = 0; i < m_mesh.m_plane_groups.size();++i){
				//printf("group %d, face number %d\n", i, m_mesh.m_plane_groups[i].m_indices.size());
				for(int fi:m_mesh.m_plane_groups[i].m_indices){
					m_mesh.m_face_plane_index[fi] = i;
				}
			}

			
		}

		void PlaneMerge::merge() {
//			printf("%d planes before merge\n",m_mesh.m_plane_groups.size());
			
			//for (int vi = 0; vi < m_mesh.m_vertices.rows(); ++vi) {
			for (const auto& ef : EF.rowwise()) {
				//auto plane_set = m_mesh.get_vertex_plane_set(vi);
				//if (plane_set.size() == 2) {
					//int pi0 = find_parent(*plane_set.begin());
					//int pi1 = find_parent(*(++plane_set.begin()));
				if(ef[0] != -1 && ef[1] != -1 && 
				m_mesh.m_face_plane_index[ef[0]] != -1 &&
				m_mesh.m_face_plane_index[ef[1]] != -1 &&
				m_mesh.m_face_plane_index[ef[0]] != m_mesh.m_face_plane_index[ef[1]]){
					int pi0 = find_parent(m_mesh.m_face_plane_index[ef[0]]);
					int pi1 = find_parent(m_mesh.m_face_plane_index[ef[1]]);

					if (pi0 != pi1
						&& is_parallel_planes(pi0, pi1)) {
						int pi = m_mesh.m_plane_groups.size();
						Base::PlaneGroup g = merge_plane(pi0, pi1);
						m_mesh.m_plane_groups.push_back(g);
						m_plane_parent.push_back(pi);
						/*printf("merge %d(%d), %d(%d) into %d(%d)\n",
							pi0, m_mesh.m_plane_groups[pi0].m_indices.size(),
							pi1, m_mesh.m_plane_groups[pi1].m_indices.size(),
							pi, m_mesh.m_plane_groups[pi].m_indices.size());*/
						m_mesh.m_plane_groups[pi0].m_indices.clear();
						m_mesh.m_plane_groups[pi1].m_indices.clear();
						m_plane_parent[pi0] = pi;
						m_plane_parent[pi1] = pi;
						auto& neighbor0 = m_plane_neighbors[pi0];
						auto& neighbor1 = m_plane_neighbors[pi1];
						neighbor0.erase(pi1);
						neighbor1.erase(pi0);
						std::unordered_set<int> new_neighbor;
						for(auto iter = neighbor1.begin(); iter != neighbor1.end(); ++iter){
							new_neighbor.insert(*iter);
						}
						for(auto iter = neighbor0.begin(); iter != neighbor0.end(); ++iter){
							new_neighbor.insert(*iter);
						}
						/*notice here may cause memory re-allocation, the reference neighbor0
						 and neighbor1 may be invalid*/
						m_plane_neighbors.push_back(new_neighbor);
						m_plane_neighbors[pi0].clear();
						m_plane_neighbors[pi1].clear();

					}
				}
			}

			update_mesh();
//			printf("%d planes after merge\n",m_mesh.m_plane_groups.size());
		}
	}
}