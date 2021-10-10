#ifndef PLANE_MANIFOLD_H
#define PLANE_MANIFOLD_H

#include <Base/TriMesh.h>

#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/Properties.h>
#include <CGAL/Simple_cartesian.h>
#include <deque>
namespace MeshPolyRefinement{
    namespace PlaneEstimation{
        class PlaneManifold{
        public:
            PlaneManifold(Base::TriMesh& mesh);
            
			typedef CGAL::Simple_cartesian<Base::Scalar>          K;
			typedef K::Point_3									Point_3;
			typedef CGAL::Surface_mesh<K::Point_3>              Mesh;
			using Vertex_index = Mesh::Vertex_index;
			using Face_index = Mesh::Face_index;

            void make_vertex_manifold();
        private:
            Base::TriMesh& m_mesh;
            
            Mesh cgal_mesh;
            std::vector<Vertex_index> m_v_index_map;
            

            void construct_cgal_mesh();

            void update_mesh();

            std::deque<Face_index> faces_around_vertex(int vi);
        };
    }
}
#endif