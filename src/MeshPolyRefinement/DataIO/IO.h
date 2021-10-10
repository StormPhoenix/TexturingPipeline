#pragma once

#ifndef IO_H
#define IO_H


#include "Base/TriMesh.h"
#include <string>
namespace MeshPolyRefinement {
	namespace IO {
		bool read_mesh_from_obj(const std::string& file_name, Base::TriMesh& mesh);

		bool read_mesh_from_ply(const std::string& file_name, Base::TriMesh& mesh);
		
		bool save_mesh_plane_segments(const std::string& file_name, const Base::TriMesh& mesh);
		
		bool read_mesh_plane_parameters(const std::string& file_name, Base::TriMesh& mesh);
		
		bool save_mesh_plane_parameters(const std::string& file_name, const Base::TriMesh& mesh);
		
		bool save_mesh_clusters(const std::string& file_name, const Base::TriMesh& mesh, const std::vector<int>& cluster_ids, int num_clusters);

		bool save_mesh(const std::string& file_name, const Base::TriMesh& mesh);

		bool save_mesh(const std::string& file_name, const Eigen::MatrixXd & V, const Eigen::MatrixXi& F);
	}
}
#endif
