#include "RegionExpand.h"
#include "PlaneMerge.h"
#include "RegionGrowing.h"
#include <igl/fit_plane.h>
#include <math.h>
#include <queue>
#include <unordered_map>
#include <Visualize/Visualize.h>
#include <DataIO/IO.h>
namespace MeshPolyRefinement {
	namespace PlaneEstimation {

		

		extern void estimate_plane(const Base::TriMesh&mesh,
			const std::vector<std::size_t>& face_indices,
			Base::Vec3& plane_normal,
			Base::Vec3& plane_center);

		inline bool is_label_expanable(Base::FaceSemanticLabel l) {
			if (l == Base::FaceSemanticLabel::ROOF
				|| l == Base::FaceSemanticLabel::BUILDING
				|| l == Base::FaceSemanticLabel::UNSET
				|| l == Base::FaceSemanticLabel::MULTILABELED)
				return true;
			return false;
		}

		void merge_uncredible_planes(Base::TriMesh& mesh){
			double cos_threshold = std::cos(15.0*M_PI / 180.0);
			bool merged = true;
			while(merged){
				merged = false;
				std::vector<bool> is_credible(mesh.m_plane_groups.size(), true);
				std::vector<std::unordered_map<int,int>> plane_boundary_counts(mesh.m_plane_groups.size());
				const auto& FF = mesh.m_ff_adjacency;
				for(int gi = 0; gi < mesh.m_plane_groups.size(); ++gi){
					Base::PlaneGroup& g = mesh.m_plane_groups[gi];
					bool connect_to_non_planar = false;
					if(g.m_indices.size() == 0)
						continue;
					auto &plane_boundary_count = plane_boundary_counts[gi];
					int bound_face_number = 0;
					for(int fi : g.m_indices){
						bool is_bound = false;
						for(int nfi : FF.row(fi)){
							if(nfi == -1 || mesh.m_face_plane_index[nfi] == -1 ){
								connect_to_non_planar = true;
							}
							if (nfi == -1 || 
							mesh.m_face_plane_index[nfi] != mesh.m_face_plane_index[fi]){
								is_bound = true;
							}

							if(nfi != -1 && mesh.m_face_plane_index[nfi] != mesh.m_face_plane_index[fi]
							&& mesh.m_face_plane_index[nfi] != -1
							){
								auto iter = plane_boundary_count.find(mesh.m_face_plane_index[nfi]);
								if(iter == plane_boundary_count.end()){
									plane_boundary_count[mesh.m_face_plane_index[nfi]] = 1;
								}
								else{
									++(iter->second);
								}
							}

						}
						if(is_bound)
							++bound_face_number;
					}
					//printf("plane %d, face number %d, bound face number %d, neighbor %d\n",gi, g.m_indices.size() ,bound_face_number, plane_boundary_count.size());
					// for(auto iter = plane_boundary_count.begin();
					// 	iter != plane_boundary_count.end();
					// 	++iter){
					// 	printf("N(%d), ", iter->first);
					// }
					// printf("\n");
					//Visualize::show_mesh_plane_segments(mesh, gi);
					if(g.m_indices.size() - bound_face_number < 15
					|| (plane_boundary_count.size() == 2 && !connect_to_non_planar)){
						is_credible[gi] = false;
					
					}

				}

				for(int gi = 0; gi < mesh.m_plane_groups.size(); ++gi){
					if(!is_credible[gi]){
						//std::cout << "Found uncredible plane " << gi << std::endl;
						int target_gi = -1, max_bound = -1;
						auto &plane_boundary_count = plane_boundary_counts[gi];
						for(auto iter = plane_boundary_count.begin();
							iter != plane_boundary_count.end(); ++iter){
							if(is_credible[iter->first] && iter->second > max_bound){
								if(mesh.m_plane_groups[iter->first].m_plane_normal.dot(
									mesh.m_plane_groups[gi].m_plane_normal)>=cos_threshold){
									target_gi = iter->first;
									max_bound = iter->second;
								}
								
							}
						}

						if(target_gi != -1){
							merged = true;
							//printf("---Found target plane %d\n",target_gi);
							auto& tg = mesh.m_plane_groups[target_gi];
							auto& g = mesh.m_plane_groups[gi];
							tg.m_indices.insert(tg.m_indices.end(),g.m_indices.begin(),g.m_indices.end());
							for(auto fi : g.m_indices){
								mesh.m_face_plane_index[fi] = target_gi;
							}
							g.m_indices.clear();
						}
					}
				}
			}
			
			
			auto groups = mesh.m_plane_groups;
			mesh.m_plane_groups.clear();
			for(const auto& g:groups){
				if(!g.m_indices.empty()){
					int gi = mesh.m_plane_groups.size();
					mesh.m_plane_groups.push_back(g);
					for(auto fi : g.m_indices){
						mesh.m_face_plane_index[fi] = gi;
					}
				}
			}

		}

		
		
		

		void plane_region_expand(Base::TriMesh& mesh, 
			Base::Scalar dist_ratio,
			Base::Scalar angle) {
			double normal_angle_cos = std::cos(angle * M_PI / 180.0);
			bool expanded = true;
			Eigen::Vector<int, -1> face_plane_index;
			int iteration = 0;
			while (expanded) {
				/*if (iteration > 10)
					break;*/
				expanded = false;
				int count = 0;
				face_plane_index = mesh.m_face_plane_index;
				for (auto &g : mesh.m_plane_groups) {
					std::vector<std::size_t> indices = g.m_indices;
					for (std::size_t fi : g.m_indices) {
						for (int nfi : mesh.m_ff_adjacency.row(fi)) {
							//test if the face(nfi) can be added into g
							if (nfi != -1 && face_plane_index[nfi] == -1 &&
								is_label_expanable(mesh.m_face_labels[nfi]) &&
								std::fabs(g.m_plane_normal.dot(mesh.m_face_normals.row(nfi))) >= normal_angle_cos) {
								
								Base::Scalar max_dist = 0;
								for (int vi : mesh.m_faces.row(nfi)) {
									Base::Scalar dist = point_plane_distance(mesh.m_vertices.row(vi), g.m_plane_normal, g.m_plane_center);
									max_dist = max_dist >= dist ? max_dist : dist;
								}
								//std::cout << max_dist << ", " << g.m_avg_distance << std::endl;
								if (max_dist <= dist_ratio * g.m_avg_distance) {
									//expand
									indices.push_back(nfi);
									face_plane_index[nfi] = mesh.m_face_plane_index[fi];
									expanded = true;
									++count;
								}
							}
						}
					}
					g.m_indices = indices;
				}
				mesh.m_face_plane_index = face_plane_index;
				//std::cout << "Expand " << count << " faces" << std::endl;
				++iteration;
			}

		}

		void plane_region_merge(Base::TriMesh& mesh) {
			PlaneMerge plane_merge(mesh);
			plane_merge.merge();
			//IO::save_mesh_plane_segments("merge-inter.ply",mesh);
			merge_uncredible_planes(mesh);
		}

		void plane_region_refine(Base::TriMesh& mesh){
			std::vector<bool> visited(mesh.m_faces.rows(),false);
			for(int seed = 0; seed < mesh.m_faces.rows();++seed){
				//choose a seed
				if(!visited[seed] && mesh.m_face_plane_index[seed] == -1){
					std::vector<std::size_t> indices;
					std::queue<int> que;
					/*store the adjacent planes and the boundary edge number*/
					std::unordered_map<int,int> plane_boundary_count;
					que.push(seed);
					visited[seed] = true;
					while(!que.empty()){
						int fi = que.front();
						que.pop();
						indices.push_back(fi);
						
						for(int nfi : mesh.m_ff_adjacency.row(fi)){
							if(nfi == -1)
								continue;
							if(!visited[nfi] && mesh.m_face_plane_index[nfi] == -1){
								que.push(nfi);
								visited[nfi] = true;
							}
							else if(mesh.m_face_plane_index[nfi] != -1){
								auto iter = plane_boundary_count.find(mesh.m_face_plane_index[nfi]);
								if(iter == plane_boundary_count.end()){
									plane_boundary_count.insert(std::make_pair(mesh.m_face_plane_index[nfi],1));
								}
								else{
									++(iter->second);
								}
							}

						}
					}
					if(indices.size() <= 15){
						int max_bound_count = 0;
						int pi = -1;
						for(auto iter = plane_boundary_count.begin();
						iter != plane_boundary_count.end(); ++iter){
							if(iter->second > max_bound_count){
								max_bound_count = iter->second;
								pi = iter->first;
							}
						}
						if(pi != -1){
							//add to pi
							mesh.m_plane_groups[pi].m_indices.insert(mesh.m_plane_groups[pi].m_indices.end(),
							indices.begin(),indices.end());
							for(auto fi:indices){
								mesh.m_face_plane_index[fi] = pi;
							}
						}
					}
				}
			}

		}
		
	}
}