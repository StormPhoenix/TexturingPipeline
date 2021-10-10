#pragma once
#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <Base/TriMesh.h>

namespace MeshPolyRefinement {
	namespace Visualize {

		void show_mesh(const Base::TriMesh& mesh);

		void show_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

		void show_mesh_labels(const Base::TriMesh& mesh);

		//void show_mesh_seed_map(const Base::TriMesh& mesh, float thres);

		void show_mesh_planar_score(const Base::TriMesh& mesh, float thres);

		void show_mesh_plane_segments(const Base::TriMesh& mesh);

		void show_mesh_plane_segments(const Base::TriMesh& mesh, int gi);

		void show_mesh_plane_segments(const Base::TriMesh& mesh1, const Base::TriMesh& mesh2);

		void show_mesh_decimation_edge_cost(const Base::TriMesh & mesh, const Eigen::MatrixXi& E, const Eigen::MatrixXd& C);

		void show_mesh_face_feature(const Base::TriMesh& mesh, const std::vector<double>& feature);

		void show_mesh_cluster(const Base::TriMesh& mesh, const std::vector<int>& cluster_ids, int num_clusters);

		void show_polygon(const Base::AttributeMatrix& V);

		void show_plane_polygon(const Base::TriMesh& mesh, const Base::PlaneGroup& g, const std::vector<int>& poly);

		void show_boundary_and_holes(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F,  const std::vector<std::vector<int>>&loops, int bound_loop_id);

		void show_path(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const std::vector<int>& path);

		void show_mesh_constrained_edge(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const Eigen::MatrixXi& E);

		void show_mesh_edge(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const Eigen::MatrixXi& E, const Eigen::MatrixXd& C);
		
	}
	
}
#endif