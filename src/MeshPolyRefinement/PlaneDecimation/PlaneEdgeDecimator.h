#pragma once
#ifndef PLANE_EDGE_DECIMATE_H
#define PLANE_EDGE_DICIMATE_H
#include "PlaneDecimator.h"

namespace MeshPolyRefinement {
	namespace PlaneDecimation {
		class PlaneEdgeDecimator :public Decimator{
		public:
			PlaneEdgeDecimator(Base::TriMesh& mesh) :Decimator(mesh) {};
		protected:
			virtual void plane_cost_and_placement(const int              /*e*/,
				const Eigen::MatrixXd &/*V*/,
				const Eigen::MatrixXi &/*F*/,
				const Eigen::MatrixXi &/*E*/,
				const Eigen::VectorXi &/*EMAP*/,
				const Eigen::MatrixXi &/*EF*/,
				const Eigen::MatrixXi &/*EI*/,
				double &               /*cost*/,
				Eigen::RowVectorXd &   /*p*/) override;
			
			virtual void remove_degenerate_face(Eigen::MatrixXd& U,
				Eigen::MatrixXi& G,
				Eigen::VectorXi& J,
				Eigen::VectorXi& I) override;

		};
	}
}
#endif