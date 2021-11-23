#pragma once

#ifndef OPTIMIZE_H
#define OPTIMIZE_H

#include <Base/TriMesh.h>
namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		/*
		Split triangles whose two edges lie on the same intersecting line of two planes,
		otherwise causing degenerate triangle.
		*/
		void slice_mesh_faces(Base::TriMesh& mesh, std::vector<int>& valid_faces);

		void update_mesh(Base::TriMesh& mesh);

		void split_mesh_triangle(Base::TriMesh& mesh);
													

		void optimize_mesh(Base::TriMesh& mesh, int max_iter = 500);

		void optimize_mesh_line_search(Base::TriMesh& mesh);

	}
}
#endif
