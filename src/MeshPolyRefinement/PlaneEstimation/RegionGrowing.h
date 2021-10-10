#pragma once
/*Use CGAL Region Growing templates, please refer to
https://doc.cgal.org/latest/Shape_detection/index.html#Shape_detection_RegionGrowingMesh_parameters
for more details*/

#ifndef REGION_GROWING_H
#define REGION_GROWING_H

#include <Base/TriMesh.h>

#include <vector>
namespace MeshPolyRefinement {
	namespace PlaneEstimation {
		inline Base::Scalar point_plane_distance(const Base::Vec3& P, const Base::Vec3& N, const Base::Vec3& C) {
			Base::Vec3 v = P - C;
			return fabs(v.dot(N));
		}

		class SeedMap {
		public:
			using key_type = std::size_t;
			using value_type = std::size_t;

			SeedMap(const Base::TriMesh& mesh, Base::Scalar thres);

			value_type operator[](const key_type& key) const {
				return m_seed_order[key];
			}

			friend value_type get(const SeedMap& seed_map, const key_type& key) {
				return seed_map[key];
			}

			const Eigen::Vector<Base::Scalar, -1>& m_planar_score;

		private:
			std::vector<value_type> m_seed_order;
			Base::Scalar m_thres;
		};

		class FaceNeighbor {
		private:
			const Base::IndexMatrix& m_TT;
		public:
			FaceNeighbor(const Base::TriMesh& mesh) :m_TT(mesh.m_ff_adjacency) {}

			void operator()(
				const std::size_t query_index,
				std::vector<std::size_t>& neighbors) const {
				neighbors.clear();
				for (int ni : m_TT.row(query_index)) {
					if (ni >= 0) {
						neighbors.push_back(ni);
					}
				}
			}
		};

		

		class PlaneRegion {
		private:
			const Base::TriMesh& m_mesh;
			
			Base::Vec3 m_cur_plane_normal, m_cur_center;

			//the maximum distance from selected faces to the plane,
			//when they are added into the group, 
			//it's just a statistical variable
			Base::Scalar m_cur_avg_distance;
			Base::Scalar m_cur_distance;
			
			//thresholds
			Base::Scalar m_normal_angle_cos, m_min_planar_score ,m_avg_distance_ratio;//ratio to the m_cur_avg_distance
			std::size_t m_min_region_size;

		public:

			std::vector<Base::PlaneGroup> m_valid_groups;

			static Base::PlaneGroup merge(const Base::TriMesh& mesh, const Base::PlaneGroup& g0, Base::PlaneGroup& g1);

			static void compute_axis(Base::PlaneGroup& g);

			PlaneRegion(const Base::TriMesh& mesh, 
				Base::Scalar score_thres, 
				Base::Scalar angle, 
				Base::Scalar ratio, 
				std::size_t min_size);//the angle is in degree

			bool is_part_of_region(const std::size_t index_from,
				const std::size_t index_to,
				const std::vector<std::size_t>& indices);

			bool is_valid_region(const std::vector<std::size_t>& indices);

			void update(const std::vector<std::size_t>& indices);


		};


		void region_growing_plane_estimate(
			Base::TriMesh& mesh,
			Base::Scalar score_thres,
			Base::Scalar angle,
			Base::Scalar ratio,
			std::size_t min_size);
	}
}
#endif