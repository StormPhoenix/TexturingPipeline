#ifndef DELAUNAY_REMESH_2D_H
#define DELAUNAY_REMESH_2D_H
#include "PlaneRemesh.h"

namespace MeshPolyRefinement {
    namespace PlaneDecimation {
        class DelaunayRemesh2D : public PlaneRemesh{
        
        public:

            struct EdgeFace{
                int source;
                int target;
                int face;
                EdgeFace(){source = target = face = -1;}
                EdgeFace(int s, int t, int f) : source(s), target(t),face(f){}
            };


            DelaunayRemesh2D(Base::TriMesh& mesh);

            virtual void remesh() override;

            

        private:

            void update_mesh();

            Base::IndexMatrix remesh_group(int pi);

            void sort_halfedges(
                const Base::AttributeMatrix& V,
                const Base::IndexMatrix& F,
                int pi,
                int source, 
                std::vector<EdgeFace>& halfedges);//assume targets.size() >= 3


            /*Assume all the vertices's z coordinate is 0.0*/
            void delete_triangle_in_holes(const Base::AttributeMatrix& V,
                Base::IndexMatrix& F,
                const std::vector<std::vector<int>> bound_loops);

            std::vector<std::vector<int>> compute_bound_loops(const Base::AttributeMatrix& V,
                const Base::IndexMatrix& F,
                int pi);

            
            
            
        };
    }
}
#endif