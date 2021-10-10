#pragma once
#ifndef PLANE_MERGE_H
#define PLANE_MERGE_H

#include <Base/TriMesh.h>
#include <unordered_set>
#include <vector>

namespace MeshPolyRefinement {
	namespace PlaneEstimation {
		class PlaneMerge {
		private:

			Base::TriMesh& m_mesh;

			std::vector<int> m_plane_parent;

			std::vector<std::unordered_set<int> > m_plane_neighbors;

			Eigen::VectorXi EMAP;
			Eigen::MatrixXi E, EF, EI;
			

			double m_cos_threshold;

			double m_line_cos_threshold;

			void init_plane_neighbors();

			int find_parent(int plane_index);

			bool is_parallel_planes(int pi0, int pi1);

			Base::PlaneGroup merge_plane(int pi0, int pi1);

			void update_mesh();

			double distance(const Base::PlaneGroup&g, const Base::Vec3& pt) const;

			bool is_inner_plane(int pi);
			

		public:
			PlaneMerge(Base::TriMesh& mesh);

			void merge();
		
		


		};
	}
}
#endif
