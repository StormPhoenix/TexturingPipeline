#ifndef TRIMESH_H
#define TRIMESH_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <unordered_set>
namespace MeshPolyRefinement {
	namespace Base {

		enum FaceSemanticLabel
		{
			UNSET = 0,
			BUILDING = 1,
			ROAD = 2,
			VEGETATION = 3,
			VEHICLE = 4,
			ROOF = 5,
			OTHER = 6,
			MULTILABELED = 7
		};

		typedef double Scalar;
		typedef Eigen::Matrix<double, 1, 3> Vec3d;
		typedef Eigen::Matrix<float, 1, 3> Vec3f;
		typedef Eigen::Matrix<int, 1, 3> Vec3i;
		using Vec3 = Eigen::Matrix<Base::Scalar, 1, 3>;
		using Vec2 = Eigen::Matrix<Base::Scalar, 1, 2>;
		using Vec4 = Eigen::Matrix<Base::Scalar, 1, 4>;
		typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
		typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
		typedef Eigen::Vector<FaceSemanticLabel, -1> FaceSemanticVector;
		

		struct PlaneGroup {
			std::vector<std::size_t> m_indices;
			Base::Vec3 m_plane_normal, m_plane_center;//center is the bary-center(approximately) of vertices
			Base::Scalar m_avg_distance;
			Base::Vec3 m_x_axis, m_y_axis;
			Base::Vec4 params;
		};


		struct TriMesh {
		public:
			//Geometry 
			AttributeMatrix m_vertices, m_vertex_colors, m_face_normals;
			IndexMatrix m_faces, m_ff_adjacency;
			std::vector<std::vector<int>> m_vf_adjacency;

			//semantic && plane group
			FaceSemanticVector m_face_labels;
			std::vector<PlaneGroup> m_plane_groups;
			Eigen::Vector<Base::Scalar, -1> m_face_planar_score;
			Eigen::Vector<int, -1> m_face_plane_index;

			//
			std::unordered_set<int> get_vertex_plane_set(int vi) const {
				std::unordered_set<int> plane_set;
				for (int fi : m_vf_adjacency[vi]) {
					if (m_face_plane_index[fi] != -1) {
						plane_set.insert(m_face_plane_index[fi]);
					}
				}
				return plane_set;
			}
		};

		
	}
}
#endif