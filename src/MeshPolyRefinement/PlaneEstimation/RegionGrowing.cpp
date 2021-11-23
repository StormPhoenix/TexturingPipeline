#include "RegionGrowing.h"

#include <igl/parallel_for.h>
#include <igl/fit_plane.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>

#include <iostream>
#include <iterator>


#include <math.h>
namespace MeshPolyRefinement {
	namespace PlaneEstimation {

		

		void estimate_plane(const Base::TriMesh&mesh, 
			const std::vector<std::size_t>& face_indices,
			Base::Vec3& plane_normal, 
			Base::Vec3& plane_center) {
			Base::AttributeMatrix points(face_indices.size() * 3, 3);//For convenience, every vertex may be counted for multiple times
			for (std::size_t i = 0; i < face_indices.size(); ++i) {
				std::size_t face_index = face_indices[i];
				points.block<3, 3>(i * 3, 0) = mesh.m_vertices(mesh.m_faces.row(face_index), Eigen::all);
			}
			Base::Vec3d N, C;
			igl::fit_plane(points, N, C);
			//plane_normal should contains the current plane
			if (plane_normal.dot(N) >= 0)
				plane_normal = N;
			else
				plane_normal = -N;
			plane_center = C;
		}

		SeedMap::SeedMap(const Base::TriMesh& mesh, Base::Scalar thres)
		:m_planar_score(mesh.m_face_planar_score){

//			std::cout << "Constructing seed map... ..." << std::endl;
			m_seed_order.resize(mesh.m_faces.rows());
			m_thres = thres;
			
			auto init = [&](int seed_index) {
				m_seed_order[seed_index] = seed_index;
			};
			igl::parallel_for(m_seed_order.size(), init);

			struct PlanarScoreCMP {
			private:
				const Eigen::Vector<Base::Scalar, -1>& m_scores;
			public:
				PlanarScoreCMP(const Eigen::Vector<Base::Scalar, -1>& scores) :m_scores(scores) {}

				bool operator()(const key_type&i, const key_type&j) const {
					return m_scores[i] > m_scores[j]; //Decreasing order
				}
			};

			
			std::sort(m_seed_order.begin(), m_seed_order.end(), PlanarScoreCMP(m_planar_score));

			/*for (int index : m_seed_order) {
				std::cout << m_planar_score[index] << std::endl;
			}*/

			//skip those faces whose planar score is too low or is not building/roof/road
			/*auto skip = [&](int seed_index) {
				int face_index = m_seed_order[seed_index];
				const auto& label = mesh.m_face_labels[face_index];
				if (label == Base::FaceSemanticLabel::UNSET ||
					label == Base::FaceSemanticLabel::BUILDING ||
					label == Base::FaceSemanticLabel::ROOF ||
					label == Base::FaceSemanticLabel::ROAD) {
					if (m_planar_score[face_index] < m_thres) {
						m_seed_order[seed_index] = std::size_t(-1);
					}
				}
				else {
					m_seed_order[seed_index] = std::size_t(-1);
				}
			};
			igl::parallel_for(m_seed_order.size(), skip);*/
			for (int seed_index = 0; seed_index < m_seed_order.size(); ++seed_index) {
				int face_index = m_seed_order[seed_index];
				const auto& label = mesh.m_face_labels[face_index];
				if (label == Base::FaceSemanticLabel::UNSET ||
					label == Base::FaceSemanticLabel::BUILDING ||
					label == Base::FaceSemanticLabel::ROOF ||
					label == Base::FaceSemanticLabel::ROAD) {
					if (m_planar_score[face_index] < m_thres) {
						m_seed_order[seed_index] = std::size_t(-1);
					}
				}
				else {
					m_seed_order[seed_index] = std::size_t(-1);
				}
			}

		}

		PlaneRegion::PlaneRegion(const Base::TriMesh& mesh,
			Base::Scalar score_thres,
			Base::Scalar angle,
			Base::Scalar ratio,
			std::size_t min_size)
		: m_avg_distance_ratio(ratio), 
			m_min_region_size(min_size),
			m_mesh(mesh),
			m_min_planar_score(score_thres){
			m_normal_angle_cos = std::cos(angle * M_PI / 180.0);
			m_cur_avg_distance = 0.0;

		}

		bool PlaneRegion::is_part_of_region(const std::size_t index_from,
			const std::size_t index_to,
			const std::vector<std::size_t>& indices) {

			//test sematic
			Base::FaceSemanticLabel label = m_mesh.m_face_labels[index_to];
			if (label != Base::FaceSemanticLabel::UNSET
				&& label != Base::FaceSemanticLabel::BUILDING
				&& label != Base::FaceSemanticLabel::ROOF
				&& label != Base::FaceSemanticLabel::ROAD)
				return false;
			
			//test planar score
			if (m_mesh.m_face_planar_score[index_to] < m_min_planar_score)
				return false;

			if (indices.size() == 0)
				return true;

			//test face normal

			// if (std::fabs(m_cur_plane_normal.dot(m_mesh.m_face_normals.row(index_to))) < m_normal_angle_cos)
			// 	return false;
			if (m_cur_plane_normal.dot(m_mesh.m_face_normals.row(index_to)) < m_normal_angle_cos)
				return false;
			

			//test distance
			m_cur_distance = 0;
			for (int vi : m_mesh.m_faces.row(index_to)) {
				Base::Scalar dist = point_plane_distance(m_mesh.m_vertices.row(vi), m_cur_plane_normal, m_cur_center);
				m_cur_distance = m_cur_distance >= dist ? m_cur_distance : dist;
			}
			if (indices.size() > 4 && m_cur_distance > m_avg_distance_ratio * m_cur_avg_distance)//if the region is too small, we don't test distance condition
				return false;

			return true;
		}

		void PlaneRegion::compute_axis(Base::PlaneGroup& g){
			Base::Scalar d = -g.m_plane_normal.dot(g.m_plane_center);
			Base::Vec3 p = -d*g.m_plane_normal;
			if ((p - g.m_plane_center).norm() < 1e-5) {
				//p is too close to origin
				p.setZero();
				if (g.m_plane_normal[0] != 0) {
					p[0] = -d / g.m_plane_normal[0];
				}
				else if (g.m_plane_normal[1] != 0) {
					p[1] = -d / g.m_plane_normal[1];
				}
				else {
					p[2] = -d / g.m_plane_normal[2];
				}
			}
			g.m_x_axis = (p - g.m_plane_center).normalized();
			g.m_y_axis = (g.m_plane_normal.cross(g.m_x_axis)).normalized();
		}

		Base::PlaneGroup PlaneRegion::merge(const Base::TriMesh& mesh, const Base::PlaneGroup& g0, Base::PlaneGroup& g1){
			Base::PlaneGroup group;
			group.m_indices = g0.m_indices;
			group.m_indices.insert(group.m_indices.end(), g1.m_indices.begin(), g1.m_indices.end());
			//estimate_plane(mesh, group.m_indices, group.m_plane_normal, group.m_plane_center);

			// group.m_plane_normal = ((g0.m_plane_normal + g1.m_plane_normal)*0.5).normalized();
			// group.m_plane_center = (g0.m_plane_center + g1.m_plane_center)*0.5;
			// group.m_avg_distance = (g0.m_avg_distance + g1.m_avg_distance)*0.5;
			group.m_plane_normal = (g0.m_plane_normal * g0.m_indices.size() + g1.m_plane_normal * g1.m_indices.size()).normalized();
			group.m_plane_center = (g0.m_plane_center * g0.m_indices.size() + g1.m_plane_center * g1.m_indices.size())/group.m_indices.size();
			group.m_avg_distance = (g0.m_avg_distance * g0.m_indices.size() + g1.m_avg_distance * g1.m_indices.size())/group.m_indices.size();
			compute_axis(group);
			group.params.head(3) = group.m_plane_normal;
			group.params[3] = -group.m_plane_normal.dot(group.m_plane_center);
			return group;
		}
		bool PlaneRegion::is_valid_region(const std::vector<std::size_t>& indices) {
			if (indices.size() >= m_min_region_size) {
				Base::PlaneGroup group;
				group.m_indices = indices;
				group.m_plane_normal = m_cur_plane_normal;
				group.m_plane_center = m_cur_center;
				group.m_avg_distance = m_cur_avg_distance;
				compute_axis(group);
				group.params.head(3) = group.m_plane_normal;
				group.params[3] = -group.m_plane_normal.dot(group.m_plane_center);
				m_valid_groups.push_back(group);
				return true;
			}
			return false;
		}

		void PlaneRegion::update(const std::vector<std::size_t>& indices) {
			if (indices.size() == 1) {
				m_cur_plane_normal = m_mesh.m_face_normals.row(indices[0]);
				m_cur_center = m_mesh.m_vertices.row(m_mesh.m_faces(indices[0], 0));
				m_cur_distance = m_cur_avg_distance = 0.0;

			}
			else {
				
				estimate_plane(m_mesh, indices, m_cur_plane_normal, m_cur_center);
				m_cur_avg_distance = m_cur_avg_distance *(indices.size() - 1) + m_cur_distance;
				m_cur_avg_distance /= indices.size();
			}
		}

		void region_growing_plane_estimate(
			Base::TriMesh& mesh,
			Base::Scalar score_thres,
			Base::Scalar angle,
			Base::Scalar ratio,
			std::size_t min_size) {
			using Region_growing = CGAL::Shape_detection::Region_growing<
				std::vector<std::size_t> ,
				FaceNeighbor,
				PlaneRegion,
				SeedMap>;

			std::vector<std::vector<std::size_t> > regions;
			FaceNeighbor face_neighbor(mesh);
			PlaneRegion plane_region(mesh, score_thres, angle, ratio, min_size);
			SeedMap seed_map(mesh,score_thres);
//			std::cout << "Seed map constructed" << std::endl;
			std::vector<std::size_t> input_face_range(mesh.m_faces.rows());
			Region_growing region_growing(input_face_range, face_neighbor, plane_region, seed_map);
			region_growing.detect(std::back_inserter(regions));
//			std::cout << "Detect " << regions.size() << ", " << plane_region.m_valid_groups.size() << " planes" << std::endl;
			mesh.m_plane_groups = plane_region.m_valid_groups;
			
			auto construct_map = [&](int group_index) {
				const auto& g = mesh.m_plane_groups[group_index];
				for (std::size_t idx : g.m_indices) {
					mesh.m_face_plane_index[idx] = group_index;
				}
			};

			igl::parallel_for(mesh.m_plane_groups.size(), construct_map);
		}
	}
}