#pragma once
#ifndef REGION_EXPAND_H
#define REGION_EXPAND_H


#include "Base/TriMesh.h"


#include <vector>
namespace MeshPolyRefinement {
	namespace PlaneEstimation {
		void plane_region_expand(Base::TriMesh& mesh, 
			Base::Scalar dist_ratio,
			Base::Scalar angle);


		void plane_region_merge(Base::TriMesh& mesh);
		
		/*Add small non-planar region to adjacent plane group*/
		void plane_region_refine(Base::TriMesh& mesh);
		


	}
}
#endif