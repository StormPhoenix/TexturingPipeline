#ifndef PLANE_REMESH_H
#define PLANE_REMESH_H

#include <Base/TriMesh.h>
namespace MeshPolyRefinement {
    namespace PlaneDecimation {
        class PlaneRemesh{
        protected:
            Base::TriMesh& m_mesh;
        
        public:
            PlaneRemesh(Base::TriMesh& mesh):m_mesh(mesh){}

            virtual void remesh() = 0;
        };

    }
}
#endif