#pragma once

#ifndef ENERGY_FUNC_H
#define ENERGY_FUNC_H


#include "Base/TriMesh.h"

#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		//NonFlip means faces shall not change orientation in 3D space or degenerate
		class NonFlipPlaneEnergy : public ceres::FirstOrderFunction {
		public:
			NonFlipPlaneEnergy(const Base::TriMesh& mesh);
			

			virtual ~NonFlipPlaneEnergy();

			virtual bool Evaluate(const double* parameters,
				double* cost,
				double* gradient) const;

			double compute_energy(Eigen::MatrixXd& x);

			void compute_gradient(Eigen::MatrixXd& x, Eigen::MatrixXd& grad) const;

			virtual int NumParameters() const;

			
		private:

			double compute_quadric_energy(const double* parameters) const;//compute quadric plane energy term to total cost energy

			Eigen::VectorXd compute_quadric_gradient(const double* parameters) const;//compute quadric plane gradient

			double compute_quadric_energy_gradient(const double* parameters, Eigen::VectorXd& gradient);

			// void compute_barrier_energy(const double* parameters,
			// 	double* cost) const;//ADD flip-preventing barrier energy term to total cost energy

			// void compute_barrier_gradient(const double* parameters,
			// 	double* gradient) const;//ADD flip-preventing barrier gradient term to total gradient

			double compute_normal_energy(const double* parameters) const;//

			double compute_distance_energy(const double* parameters) const;

			Eigen::VectorXd compute_distance_gradient(const double* parameters) const;

			Eigen::VectorXd compute_normal_gradient(const double* parameters) const;

			Eigen::VectorXd compute_numeric_gradient(const double* parameters,std::function<void(const double*, double*)> func) const;//just to debug barrier gradient

			Base::Vec3 project_gradient_onto_plane(int vertex_index, Base::Vec3 grad) const;

			void build_quadirc_mat();

			void build_face_matrix();//prepare matrices for normal energy

			
			double gamma, exp_n;

			double plane_weight, barrier_weight, normal_weight, distance_weight;

			double angle_cos_thres;
			

			int m_num_vertices, m_num_faces;
			
			Eigen::Matrix4d* m_zero_quadric_mat;

			Eigen::VectorXd m_initial_vertices;// 3 * m_num_vertices

			std::vector<Eigen::Matrix4d*> m_vertex_quad_mats;

			std::vector<std::unordered_set<int>> m_vertex_plane_index;

			std::vector<double> m_initial_face_area;

			const std::vector<Base::PlaneGroup>& m_planes;

			const Eigen::Vector<int, -1>& m_face_plane_index;
			
			const Base::AttributeMatrix& m_initial_face_normals;

			const Base::IndexMatrix& m_faces;

			Eigen::SparseMatrix<double> m_quadric_mat;//Total qudaric mat, size: (4*vn, 4*vn)

			Eigen::MatrixXd m_mask; // size: (fn,3), 0: face on plane, 1: face not on plane

			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> m_target_normal; // size: (fn,3)

			Eigen::SparseMatrix<double,Eigen::RowMajor> m_AB_mask, m_AC_mask; // size: (fn,vn), the mask vector AB and AC of every triangle ABC

			
		};
	}
}
#endif
