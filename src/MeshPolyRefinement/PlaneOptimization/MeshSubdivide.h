#pragma once
#ifndef MESH_SUBDIVIDE_H
#define MESH_SUBDIVIDE_H

#include <Base/TriMesh.h>
#include <set>

namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		/*Subdivide mesh triangle so that any edge connecting
		two points on the same plane boundary must be on the boundary.*/
		class MeshSubdivide {
		public:
			/*Path line that is made by interseting two planes.*/
			struct PlaneBoundaryPath {
				std::pair<int, int> m_planes;
				PlaneBoundaryPath() {};
				PlaneBoundaryPath(int p0, int p1) {
					if (p0 < p1)
						m_planes = std::make_pair(p0, p1);
					else
						m_planes = std::make_pair(p1, p0);
				}
				PlaneBoundaryPath(const PlaneBoundaryPath& p) {
					m_planes = p.m_planes;
				}

				PlaneBoundaryPath& operator=(const PlaneBoundaryPath& p) {
					if (this != &p) {
						m_planes = p.m_planes;
					}
					return *this;
				}

				bool operator<(const PlaneBoundaryPath& p) const {
					if (m_planes.first != p.m_planes.first) {
						return m_planes.first < p.m_planes.first;
					}
					else {
						return m_planes.second < p.m_planes.second;
					}
				}
			};

			
			MeshSubdivide(Base::TriMesh& mesh);

			void split();

		private:
			//variables
			Base::TriMesh& m_mesh;

			std::vector<std::vector<PlaneBoundaryPath>> m_vertex_paths;//

			std::vector<Base::Vec3d> m_new_vertex_buffer;

			std::vector<int> m_face_map;//birth face of new faces

			std::vector<int> m_edge_new_vertex_index;//if an edge needs to be splited, add midpoint to m_new_vertex_buffer, otherwise -1

			/*edge and face structure*/
			Eigen::VectorXi EMAP;
			Eigen::MatrixXi E, EF, EI;

			//functions

			/*return vertex neighbor faces in loop order, 
			the last element -1 means the vertex on geometry boundary*/
			std::vector<int> get_vertex_face_loop(int vi);

			std::set<PlaneBoundaryPath> get_vertex_path_set(int vi);

			bool is_on_same_plane_boundary_path(int v0, int v1);

			void check_edge();//check all edges and split them if in need

			void split_face();

			void update_mesh();
		};
	}
}
#endif
