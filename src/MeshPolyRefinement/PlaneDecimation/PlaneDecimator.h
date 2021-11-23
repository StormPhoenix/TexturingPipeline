#pragma once
#ifndef DECIMATOR_H
#define DECIMATOR_H


#include <Base/TriMesh.h>
#include <igl/decimate.h>


namespace MeshPolyRefinement {
	namespace PlaneDecimation {
		/*This class can decimate triangles in the plane. 
		Vertices on the boundary of the plane remain unchanged.*/
		class Decimator {
		protected:
			
			Base::TriMesh& m_mesh;

			int orig_m;//original face number

			//std::unordered_set<int> m_stable_vertices;

			bool is_valid_face(const int fi, 
				const Eigen::MatrixXd& V,
				const Eigen::MatrixXi& F) const;

			

			
			virtual void remove_degenerate_face(Eigen::MatrixXd& U,
				Eigen::MatrixXi& G,
				Eigen::VectorXi& J,
				Eigen::VectorXi& I);
			

			void update_mesh_info(const Eigen::MatrixXd& U, 
				const Eigen::MatrixXi& G,
				const Eigen::VectorXi& J,
				const Eigen::VectorXi& I);

			/*Compute the edge cost and new vertex position.
			The mesh to be decimated should be closed edge-manifold. If the mesh has open bounaries,
			the vertices on boundary should be connected to infinity.
			However, we should guarantee that the input mesh is vertex-manifold.*/
			virtual void plane_cost_and_placement(const int              /*e*/,
				const Eigen::MatrixXd &/*V*/,
				const Eigen::MatrixXi &/*F*/,
				const Eigen::MatrixXi &/*E*/,
				const Eigen::VectorXi &/*EMAP*/,
				const Eigen::MatrixXi &/*EF*/,
				const Eigen::MatrixXi &/*EI*/,
				double &               /*cost*/,
				Eigen::RowVectorXd &   /*p*/);

			/*Check whether collapsing an edge will cause triangle flipping.*/
			virtual bool check_flip(
				const Eigen::MatrixXd &                             ,/*V*/
      			const Eigen::MatrixXi &                             ,/*F*/
      			const Eigen::MatrixXi &                             ,/*E*/
      			const Eigen::VectorXi &                             ,/*EMAP*/
      			const Eigen::MatrixXi &                             ,/*EF*/
      			const Eigen::MatrixXi &                             ,/*EI*/
      			const Eigen::RowVectorXd &   						,/*p*/
      			const int 											,/*e*/
				const std::vector<int>&                             ,
				const std::vector<int>& 
			) ;


			
		public:
			/*The input mesh should be vertex-manifold && edge-manifold
			and it can contain open boundaries.*/
			Decimator(Base::TriMesh& mesh);

			virtual bool decimate();

			void edge_cost_color(Eigen::MatrixXi & E, Eigen::MatrixXd& C);

			

			
		};
	}
}
#endif