#include "PlaneManifold.h"
#include <unordered_set>


namespace MeshPolyRefinement{
    namespace PlaneEstimation{

        PlaneManifold::PlaneManifold(Base::TriMesh& mesh)
        :m_mesh(mesh){
            construct_cgal_mesh();
        }

        void PlaneManifold::construct_cgal_mesh(){
            Base::AttributeMatrix& vertices = m_mesh.m_vertices;
			Base::IndexMatrix& faces = m_mesh.m_faces;
            m_v_index_map.resize(vertices.rows());
            Mesh::Property_map<Face_index, int> face_plane_index_property, face_original_index;
            bool created;
			//boost::tie(face_plane_index_property, created) = cgal_mesh.add_property_map<Face_index, int>("f:face_plane_index", -1);
            boost::tie(face_original_index, created) = cgal_mesh.add_property_map<Face_index, int>("f:face_original_index", -1);
            //add vertex
			for (int i = 0; i < vertices.rows(); ++i) {
				const Base::Vec3 &p = vertices.row(i);
				Vertex_index vi = cgal_mesh.add_vertex(Point_3(p[0], p[1], p[2]));
				m_v_index_map[i] = vi;
			}
            //add face
			for (int i = 0; i < faces.rows(); ++i) {
				const Base::Vec3i &f = faces.row(i);
				Face_index fi = cgal_mesh.add_face(m_v_index_map[f[0]], m_v_index_map[f[1]], m_v_index_map[f[2]]);
				if (fi == Mesh::null_face()) {
					std::cout << "warning --- add null face: " << f << std::endl;
				}
				else {
					
					//face_plane_index_property[fi] = m_mesh.m_face_plane_index[i];
                    face_original_index[fi] = i;
					
				}

			}
        }

        std::deque<PlaneManifold::Face_index> PlaneManifold::faces_around_vertex(int vi){
            std::deque<PlaneManifold::Face_index> ret;
            auto vertex_index = m_v_index_map[vi];
            auto half_edge = cgal_mesh.halfedge(vertex_index);
            auto start_edge = half_edge;
            auto start_face = cgal_mesh.face(half_edge);
            
            bool ccw = true;
            do{
                if(ccw)
                    ret.push_back(cgal_mesh.face(half_edge));
                else
                    ret.push_front(cgal_mesh.face(half_edge));
                if (cgal_mesh.is_border(half_edge)){
                    ccw = false;
                    half_edge = cgal_mesh.opposite(cgal_mesh.next(start_edge));
                }
                else if(ccw){ 
                    half_edge = cgal_mesh.prev(cgal_mesh.opposite(half_edge));
                }
                else{
                    half_edge = cgal_mesh.opposite(cgal_mesh.next(half_edge));
                }

            }while(cgal_mesh.is_valid(half_edge) && half_edge != start_edge);
            if(half_edge != start_edge){
                ret.push_back(PlaneManifold::Face_index());
            }
            return ret;
        }


        void PlaneManifold::update_mesh(){
            //auto face_plane_index_property = cgal_mesh.property_map<PlaneManifold::Face_index,int>("f:face_plane_index").first;
            for(auto& g : m_mesh.m_plane_groups){
                g.m_indices.clear();

            }

            for(int fi = 0; fi < m_mesh.m_faces.rows(); ++fi){
                int pi = m_mesh.m_face_plane_index[fi];
                if(pi != -1){
                    m_mesh.m_plane_groups[pi].m_indices.push_back(fi);
                }
            }
        }

        void PlaneManifold::make_vertex_manifold(){
        
           
            auto face_original_index = cgal_mesh.property_map<PlaneManifold::Face_index,int>("f:face_original_index").first;
            auto& face_plane_index = m_mesh.m_face_plane_index;
            for(int pi = 0; pi < m_mesh.m_plane_groups.size(); ++pi){
                std::unordered_set<int> visited_vi;
                auto& g = m_mesh.m_plane_groups[pi];
                for(auto fi : g.m_indices){
                    for(int vi : m_mesh.m_faces.row(fi)){
                        if(visited_vi.count(vi) == 0){
                            visited_vi.insert(vi);
                            auto cgal_faces = faces_around_vertex(vi);
                            // auto first = cgal_faces.front();
                            // while(cgal_faces.front() != Mesh::null_face() && face_plane_index[face_original_index[cgal_faces.front()]] != pi){
                            //     cgal_faces.push_back(cgal_faces.front());
                            //     cgal_faces.pop_front();
                            //     if(cgal_faces.front() == first)
                            //         break;
                            // }
                            
                            int count = 0;
                            auto prev_face = cgal_faces.back();
                            for(int i = 0; i < cgal_faces.size();++i){
                                auto cur_face = cgal_faces[i];
                                if(prev_face != Mesh::null_face() && face_plane_index[face_original_index[prev_face]] == pi){
                                    if(cur_face == Mesh::null_face() || face_plane_index[face_original_index[cur_face]] != pi){
                                        ++count;
                                    }
                                    
                                }
                                prev_face = cur_face;
                            }
                            if(count > 1){
                                //non-manifold vertex
                                for(const auto face_index : cgal_faces){
                                    if(face_index != Mesh::null_face())
                                        face_plane_index[face_original_index[face_index]] = pi;
                                }
                            }



                        }
                    }
                }
            }
            update_mesh();
        }
    }
}