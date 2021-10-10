#pragma once
#ifndef REPAIR_H
#define REPAIR_H

#include "Base/TriMesh.h"

namespace MeshPolyRefinement {
	namespace IO {
		void merge_close_vertex(Base::AttributeMatrix& vertices, Base::AttributeMatrix& vertex_colors, Base::IndexMatrix& faces);

		void remove_identical_index_faces(Base::IndexMatrix& faces);

		void remove_degenerate_faces(Base::TriMesh& mesh);

		void split_non_manifold_vertex_with_property(Base::AttributeMatrix& vertices, Base::AttributeMatrix& vertex_colors, Base::IndexMatrix& faces);

		void repair_non_manifold_edge(Base::TriMesh& mesh);

		void repair_non_manifold(Base::TriMesh& mesh);//repair non-manifold vertices and edges
	}
}
#endif
