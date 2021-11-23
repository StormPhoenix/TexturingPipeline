#include "DelaunayRemesh2D.h"
#include <igl/boundary_loop.h>
#include <igl/dijkstra.h>
#include <igl/list_to_matrix.h>
#include <igl/edges.h>
#include <igl/adjacency_list.h>
#include <igl/barycenter.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/sort_vectors_ccw.h>
#include <igl/is_vertex_manifold.h>
#include <math.h>
#include <set>
#include <deque>
#include <Visualize/Visualize.h>
#include <DataIO/IO.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <unordered_map>

#define CDT_USE_BOOST

#include <CDT.h>


namespace MeshPolyRefinement {
    namespace PlaneDecimation {
        DelaunayRemesh2D::DelaunayRemesh2D(Base::TriMesh &mesh)
                : PlaneRemesh(mesh) {}


        void DelaunayRemesh2D::sort_halfedges(const Base::AttributeMatrix &V,
                                              const Base::IndexMatrix &F,
                                              int pi,
                                              int source,
                                              std::vector<EdgeFace> &halfedges) {
            if (halfedges.size() <= 2)
                return;//no need to sort
            const Base::PlaneGroup &g = m_mesh.m_plane_groups[pi];
            Base::Vec3 start_pos = V.row(source);
            std::unordered_map<int, double> angles;

            for (const EdgeFace &e : halfedges) {
                int t = e.source == source ? e.target : e.source;
                Base::Vec3 diff = V.row(t) - start_pos;
                angles[t] = atan2(diff.dot(g.m_y_axis), diff.dot(g.m_x_axis));
            }

            struct AngleCompare {

                int source;
                const std::unordered_map<int, double> &angles;

                AngleCompare(
                        int s,
                        const std::unordered_map<int, double> &A) : source(s), angles(A) {}

                bool operator()(const EdgeFace &e1, const EdgeFace &e2) const {
                    int v1 = e1.source == source ? e1.target : e1.source;
                    int v2 = e2.source == source ? e2.target : e2.source;
                    return angles.at(v1) < angles.at(v2);
                }
            };
            std::sort(halfedges.begin(), halfedges.end(), AngleCompare(source, angles));

            // Eigen::RowVectorXd plane_vectors;
            // plane_vectors.resize(halfedges.size()*3);
            // plane_vectors.setZero();

            // for(int t = 0; t < halfedges.size(); ++t){
            //     Base::Vec3 end = (halfedges[t].source == source?V.row(halfedges[t].target):V.row(halfedges[t].source));
            //     Base::Vec3 diff = end - start_pos;
            //     plane_vectors[t * 3] = diff[0];
            //     plane_vectors[t * 3 + 1] = diff[1];
            //     plane_vectors[t * 3 + 2] = diff[2];

            // }
            // Eigen::VectorXi sort_order;
            // Eigen::RowVectorXd plane_normal;
            // plane_normal = g.m_plane_normal;
            // igl::sort_vectors_ccw(plane_vectors,plane_normal,sort_order);

            // //std::vector<int> new_targets(edge_face.size()), new_faces(edge_face.size());
            // std::vector<EdgeFace> new_halfedges(halfedges.size());
            // for(int t = 0; t < halfedges.size(); ++t){
            //     new_halfedges[sort_order[t]] = halfedges[t];

            // }
            // halfedges = new_halfedges;
            //check if in-coming and out-going halfedges are list at regular intervals
            // bool last_is_in_coming = halfedges[0].target == source;
            // for(int i = 1; i < halfedges.size(); ++i) {
            //     if(last_is_in_coming && halfedges[i].source != source){
            //         printf("Two in-coming halfedges\n");
            //         Eigen::MatrixXi E(halfedges.size(),2);
            //         Eigen::MatrixXd C(halfedges.size(),3);
            //         for(int j = 0; j < halfedges.size(); ++j){
            //             E.row(j) = Eigen::RowVector2i(halfedges[j].source,halfedges[j].target);
            //             if(halfedges[j].source == source){
            //                 C.row(j) = Eigen::RowVector3d(1.0,1.0,1.0);
            //             }
            //             else{
            //                 C.row(j) = Eigen::RowVector3d(1.0,0.0,0.0);
            //             }
            //         }
            //         Visualize::show_mesh_edge(V,F,E,C);
            //         exit(-1);
            //     }

            //     if(!last_is_in_coming && halfedges[i].target != source){

            //         printf("Two out-going halfedges\n");
            //         Eigen::MatrixXi E(halfedges.size(),2);
            //         Eigen::MatrixXd C(halfedges.size(),3);
            //         for(int j = 0; j < halfedges.size(); ++j){
            //             E.row(j) = Eigen::RowVector2i(halfedges[j].source,halfedges[j].target);
            //             if(halfedges[j].source == source){
            //                 C.row(j) = Eigen::RowVector3d(1.0,1.0,1.0);
            //             }
            //             else{
            //                 C.row(j) = Eigen::RowVector3d(1.0,0.0,0.0);
            //             }
            //         }
            //         Visualize::show_mesh_edge(V,F,E,C);
            //         exit(-1);
            //     }
            //     last_is_in_coming = !last_is_in_coming;
            // }


        }

        std::vector<std::vector<int>> DelaunayRemesh2D::compute_bound_loops(const Base::AttributeMatrix &V,
                                                                            const Base::IndexMatrix &F,
                                                                            int pi) {
            //typedef std::pair<int, int> HalfEdge;

            /*
            Key: the Source vertex index of boundary halfedges
            Value: the Target vertex index of boundary halfedges
            */
            std::unordered_map<int, std::vector<EdgeFace>> v_halfedge;

            std::vector<int> v_halfedge_number(V.rows(), 0);

            Base::IndexMatrix FF;
            igl::triangle_triangle_adjacency(F, FF);
            for (int fi = 0; fi < FF.rows(); ++fi) {
                for (int i = 0; i < 3; ++i) {
                    if (FF(fi, i) == -1) {
                        //Found boundary edge
                        int source = F(fi, i), target = F(fi, (i + 1) % 3);
                        if (source == target) {
                            printf("Found same vertex indices in a triangle\n");
                        }
                        v_halfedge[source].emplace_back(source, target, fi);
                        v_halfedge[target].emplace_back(source, target, fi);
                        ++(v_halfedge_number[source]);
                        ++(v_halfedge_number[target]);


                    }
                }
            }

            for (auto iter = v_halfedge.begin(); iter != v_halfedge.end(); ++iter) {
                sort_halfedges(V, F, pi, iter->first, iter->second);
            }

            std::vector<std::vector<int> > loops;
            while (!v_halfedge.empty()) {
                int start = v_halfedge.begin()->first;
                int source = start;
                int prev_vertex = -1;
                std::vector<int> loop;
                do {
                    //add to loop
                    loop.push_back(source);
                    //printf("loop insert %d\n",source);
                    //choose halfedge

                    int choose = -1;
                    if (prev_vertex == -1) {
                        for (choose = 0; choose < v_halfedge[source].size(); ++choose) {
                            if (v_halfedge[source][choose].face != -1
                                && v_halfedge[source][choose].source == source) {
                                break;//Found out-going halfedge
                            }
                        }
                    } else {
                        for (int ei = 0; ei < v_halfedge[source].size(); ++ei) {
                            if (v_halfedge[source][ei].source == prev_vertex
                                && v_halfedge[source][ei].target == source) {
                                //Found in-coming halfedge
                                choose = (ei + 1) % v_halfedge[source].size();
                                while (v_halfedge[source][choose].face == -1
                                       || v_halfedge[source][choose].source != source) {
                                    //printf("warning !!! unexpected use of out-going halfedge\n");
                                    choose = (choose + 1) % v_halfedge[source].size();
                                }


                            }
                        }
                    }
                    if (choose == -1) {
                        printf("Cannot find out halfedge, prev_vertex: %d, source %d\n", prev_vertex, source);
                    }
                    int target = v_halfedge[source][choose].target;
                    v_halfedge[source][choose].face = -1;//change face to -1, which means the halfedge has been used

                    --(v_halfedge_number[source]);
                    --(v_halfedge_number[target]);
                    if (v_halfedge_number[source] == 0) {
                        v_halfedge.erase(source);
                    }
                    if (v_halfedge_number[target] == 0) {
                        v_halfedge.erase(target);//now the target should be the loop start
                    }
                    prev_vertex = source;
                    source = target;


                } while (source != start);
                loops.push_back(loop);
            }
            return loops;


        }

        bool check_constrained_loops(const CDT::EdgeUSet &edges) {
            std::unordered_map<int, std::pair<int, int>> vertex_neighbor;
            for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
                int v1 = iter->v1(), v2 = iter->v2();
                auto it = vertex_neighbor.find(v1);
                if (it == vertex_neighbor.end()) {
                    vertex_neighbor.insert(std::make_pair(v1, std::make_pair(v2, -1)));
                } else {
                    if (it->second.second == -1) {
                        it->second.second = v2;
                    } else if (it->second.second == v2) {
                        printf("Found two same edges\n");
                        return false;
                    } else {
                        printf("Found a vertex that's connected to more than 2 neighbors\n");
                        return false;
                    }
                }
                std::swap(v1, v2);
                it = vertex_neighbor.find(v1);
                if (it == vertex_neighbor.end()) {
                    vertex_neighbor.insert(std::make_pair(v1, std::make_pair(v2, -1)));
                } else {
                    if (it->second.second == -1) {
                        it->second.second = v2;
                    } else if (it->second.second == v2) {
                        printf("Found two same edges\n");
                        return false;
                    } else {
                        printf("Found a vertex that's connected to more than 2 neighbors\n");
                        return false;
                    }
                }

            }

            for (auto iter = vertex_neighbor.begin(); iter != vertex_neighbor.end(); ++iter) {
                if (iter->second.first == -1 || iter->second.second == -1) {
                    printf("Found vertex that has fewer than 2 neighbor(s)\n");
                    return false;
                }
            }
            std::vector<std::vector<int>> loops;
            while (!vertex_neighbor.empty()) {
                std::vector<int> loop;
                int seed = vertex_neighbor.begin()->first;
                int cur_vertex = seed, prev_vertex = -1;
                do {
                    loop.push_back(cur_vertex);
                    const auto &neighbor_pair = vertex_neighbor[cur_vertex];
                    int next = neighbor_pair.first == prev_vertex ? neighbor_pair.second : neighbor_pair.first;
                    vertex_neighbor.erase(cur_vertex);
                    prev_vertex = cur_vertex;
                    cur_vertex = next;
                } while (cur_vertex != seed);
                loops.push_back(loop);
            }
            printf("Check complete, found %ld loops\n", loops.size());
            return true;
        }

        using Triangulation = CDT::Triangulation<double>;

        void CDT_to_mesh(const Triangulation &cdt,
                         Base::AttributeMatrix &V,
                         Base::IndexMatrix &F) {
            std::vector<std::vector<int>> f_list;
            int vn = cdt.vertices.size() - 3;

            V.resize(vn, 3);
            for (int i = 0; i < vn; ++i) {
                const auto &v_cdt = cdt.vertices[i + 3];
                V.row(i) = Eigen::RowVector<Base::Scalar, 3>(v_cdt.pos.x, v_cdt.pos.y, 0.0);
            }


            for (const auto &f_cdt : cdt.triangles) {
                //F.row(n++) = Eigen::RowVector3i(f_cdt.vertices[0],f_cdt.vertices[1],f_cdt.vertices[2]);
                if (f_cdt.vertices[0] > 2 && f_cdt.vertices[1] > 2 && f_cdt.vertices[2] > 2) {
                    f_list.push_back(
                            {int(f_cdt.vertices[0]) - 3, int(f_cdt.vertices[1]) - 3, int(f_cdt.vertices[2]) - 3});
                }
            }
            igl::list_to_matrix(f_list, F);

        }


        void DelaunayRemesh2D::delete_triangle_in_holes(const Base::AttributeMatrix &V,
                                                        Base::IndexMatrix &F,
                                                        const std::vector<std::vector<int>> bound_loops) {
            typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
            typedef K::Point_2 Point;

            std::vector<std::vector<Point>> polylines;
            for (const auto &loop : bound_loops) {
                std::vector<Point> poly;
                for (int vi : loop) {
                    poly.emplace_back(V(vi, 0), V(vi, 1));
                }
                polylines.push_back(poly);
            }

            Base::AttributeMatrix face_center;// #F by 2
            igl::barycenter(V.leftCols(2), F, face_center);

            std::unordered_set<int> candidate_faces;
            for (int fi = 0; fi < F.rows(); ++fi) {
                Point c(face_center(fi, 0), face_center(fi, 1));
                for (const auto &pts : polylines) {
                    if (CGAL::bounded_side_2(pts.begin(), pts.end(), c, K()) == CGAL::ON_BOUNDED_SIDE) {
                        if (candidate_faces.count(fi) > 0) {
                            candidate_faces.erase(fi);
                        } else {
                            candidate_faces.insert(fi);
                        }
                    }
                }

            }
            std::vector<int> valid_faces;
            valid_faces.insert(valid_faces.end(), candidate_faces.begin(), candidate_faces.end());
            F = F(valid_faces, Eigen::all).eval();

        }

        Base::IndexMatrix DelaunayRemesh2D::remesh_group(int pi) {
            const Base::PlaneGroup &g = m_mesh.m_plane_groups[pi];
            std::vector<int> v_index_map(m_mesh.m_vertices.rows(), -1);
            std::unordered_set<int> v_index_set;
            std::vector<int> v_ref;
            Base::IndexMatrix F = m_mesh.m_faces(g.m_indices, Eigen::all).eval();
            for (int r = 0; r < F.rows(); ++r) {
                for (int c = 0; c < 3; ++c) {
                    v_index_set.insert(F(r, c));
                }
            }
            v_ref.insert(v_ref.end(), v_index_set.begin(), v_index_set.end());

            for (int i = 0; i < v_ref.size(); ++i) {
                v_index_map[v_ref[i]] = i;
            }

            Base::AttributeMatrix V = m_mesh.m_vertices(v_ref, Eigen::all).eval();
            for (int r = 0; r < F.rows(); ++r) {
                for (int c = 0; c < 3; ++c) {
                    F(r, c) = v_index_map[F(r, c)];
                }
            }


            std::vector<std::vector<int>> bound_loops = compute_bound_loops(V, F, pi);

            Eigen::Vector<bool, -1> B;
            //IO::save_mesh("debug_group.obj",V,F);

            // if(!igl::is_vertex_manifold(F, B)){
            //     //Visualize::show_boundary_and_holes(V,F,bound_loops,0);

            //     printf("Found non vertex manifold plane group %d\n", pi);
            //     IO::save_mesh("debug_group_non-manifold.obj",V,F);
            //     //IO::save_mesh("non-vertex-manifold.ply",V,F);
            //     Visualize::show_boundary_and_holes(V,F,bound_loops,0);
            // }





            Triangulation cdt = Triangulation(CDT::FindingClosestPoint::BoostRTree);
            std::vector<CDT::V2d<double>> v2d;
            for (const auto &v3d: V.rowwise()) {
                auto diff = v3d - g.m_plane_center;
                v2d.push_back(CDT::V2d<double>::make(diff.dot(g.m_x_axis), diff.dot(g.m_y_axis)));

            }
            cdt.insertVertices(v2d);
            //printf("cdt %d vertices and %d triangles after insert vertices\n", cdt.vertices.size(),cdt.triangles.size());

            // if(bound_loops.size() > 1)
            //     Visualize::show_boundary_and_holes(V,F,bound_loops,0);


            std::vector<CDT::Edge> constrain_edge;
            for (const auto &loop: bound_loops) {
                for (int i = 0; i < loop.size() - 1; ++i) {
                    constrain_edge.emplace_back(loop[i], loop[i + 1]);
                }
                constrain_edge.emplace_back(loop[loop.size() - 1], loop[0]);
            }
            cdt.insertEdges(constrain_edge);


            CDT_to_mesh(cdt, V, F);
            // if(bound_loops.size() > 1) {
            //     auto f_tmp = F;
            //     for(int r = 0; r < F.rows();++r){
            // 	for(int c = 0; c < 3; ++c){
            // 		f_tmp(r,c) = v_ref[f_tmp(r,c)];
            // 	}
            //     IO::save_mesh("debug_group_triangulation.obj",m_mesh.m_vertices,f_tmp);
            //     }

            // }
            delete_triangle_in_holes(V, F, bound_loops);

            for (int r = 0; r < F.rows(); ++r) {
                for (int c = 0; c < 3; ++c) {
                    F(r, c) = v_ref[F(r, c)];
                }
            }
            // if(bound_loops.size() > 1)
            //     IO::save_mesh("debug_group_remesh.obj",m_mesh.m_vertices,F);

            return F;


        }

        void DelaunayRemesh2D::update_mesh() {

            m_mesh.m_vertex_colors.resize(0, 0);
            m_mesh.m_face_labels.resize(0);
            m_mesh.m_face_planar_score.resize(0);

            igl::triangle_triangle_adjacency(m_mesh.m_faces, m_mesh.m_ff_adjacency);

            std::vector<std::vector<int> > VFi;
            igl::vertex_triangle_adjacency(m_mesh.m_vertices.rows(), m_mesh.m_faces, m_mesh.m_vf_adjacency, VFi);

            igl::per_face_normals(m_mesh.m_vertices, m_mesh.m_faces, m_mesh.m_face_normals);


            for (auto &g: m_mesh.m_plane_groups) {
                g.m_indices.clear();
            }

            for (int fi = 0; fi < m_mesh.m_faces.rows(); ++fi) {
                if (m_mesh.m_face_plane_index[fi] != -1) {
                    m_mesh.m_plane_groups[m_mesh.m_face_plane_index[fi]].m_indices.push_back(fi);
                }
            }


        }

        void DelaunayRemesh2D::remesh() {
            // auto groups = m_mesh.m_plane_groups;
            // m_mesh.m_plane_groups.clear();
            // for(int i = 0; i < groups.size();++i){
            //     m_mesh.m_plane_groups.push_back(groups[i]);
            //     printf("group %d, size %d\n",i,groups[i].m_indices.size());
            //     Visualize::show_mesh_plane_segments(m_mesh);
            //     m_mesh.m_plane_groups.clear();
            // }
            // m_mesh.m_plane_groups = groups;
            // return;
            // auto f = remesh_group(20);
            // IO::save_mesh("debug_group_remesh.obj",m_mesh.m_vertices,f);
            // return;
            std::vector<Base::IndexMatrix> remesh_faces;
            int group_face_number = 0;
            for (int pi = 0; pi < m_mesh.m_plane_groups.size(); ++pi) {
                remesh_faces.push_back(remesh_group(pi));
                group_face_number += remesh_faces.back().rows();
            }

            std::vector<int> non_group_face_indices;
            for (int fi = 0; fi < m_mesh.m_faces.rows(); ++fi) {
                if (m_mesh.m_face_plane_index[fi] == -1) {
                    non_group_face_indices.push_back(fi);
                }
            }
            m_mesh.m_faces = m_mesh.m_faces(non_group_face_indices, Eigen::all).eval();
            m_mesh.m_face_plane_index = m_mesh.m_face_plane_index(non_group_face_indices).eval();

            int total_face_number = non_group_face_indices.size() + group_face_number;
            m_mesh.m_faces.conservativeResize(total_face_number, 3);
            m_mesh.m_face_plane_index.conservativeResize(total_face_number);

            int start_r = non_group_face_indices.size();
            for (int pi = 0; pi < m_mesh.m_plane_groups.size(); ++pi) {
                const auto &face = remesh_faces[pi];
                m_mesh.m_faces.block(start_r, 0, face.rows(), 3) = face;
                m_mesh.m_face_plane_index.segment(start_r, face.rows()) = Eigen::VectorXi::Constant(face.rows(), pi);
                start_r += face.rows();
            }

            update_mesh();
        }
    }
}
