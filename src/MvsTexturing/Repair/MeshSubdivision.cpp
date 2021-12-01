//
// Created by Storm Phoenix on 2021/11/16.
//

#include "MeshSubdivision.h"
#include <map>
#include <vector>

namespace MvsTexturing {
    namespace MeshSubdivision {
        namespace __inner__ {

            struct ColorGenerator {
                std::size_t rgb[3];

                ColorGenerator() {
                    rgb[0] = rgb[1] = rgb[2] = 0;
                }

                void operator()() {
                    rgb[2] = (rgb[2] + 1) % 256;
                    if (rgb[2] == 0) {
                        rgb[1] = (rgb[1] + 1) % 256;
                        if (rgb[1] == 0) {
                            rgb[0] = (rgb[0] + 1) % 256;
                        }
                    }
                }
            };

            struct Point {
                Scalar p[3];

                Point() {
                    p[0] = p[1] = p[2] = 0;
                }

                Point(Scalar x, Scalar y, Scalar z) {
                    p[0] = x;
                    p[1] = y;
                    p[2] = z;
                }

                Point operator+(const Point &other) const {
                    return Point((x() + other.x()), (y() + other.y()), (z() + other.z()));
                }

                Point operator/(double val) const {
                    return Point(x() / val, y() / val, z() / val);
                }

                Scalar x() const {
                    return p[0];
                }

                Scalar y() const {
                    return p[1];
                }

                Scalar z() const {
                    return p[2];
                }

                double dist(const Point &other) const {
                    return std::sqrt(std::pow(std::abs(x() - other.x()), 2) +
                                     std::pow(std::abs(y() - other.y()), 2) +
                                     std::pow(std::abs(z() - other.z()), 2));
                }
            };

            struct Edge {
                std::size_t v0_idx, v1_idx;

                Edge() : v0_idx(0), v1_idx(0) {}

                Edge(std::size_t v0, std::size_t v1) : v0_idx(v0), v1_idx(v1) {
                    if (v0_idx > v1_idx) {
                        std::swap(v0_idx, v1_idx);
                    }
                }

                bool operator<(const struct Edge &other) const {
                    if (v0_idx < other.v0_idx) {
                        return true;
                    } else if (v0_idx == other.v0_idx) {
                        return v1_idx < other.v1_idx;
                    } else {
                        return false;
                    }
                }

                bool operator==(const struct Edge &other) const {
                    return (v0_idx == other.v0_idx) && (v1_idx == other.v1_idx);
                }

                void operator=(const Edge &other) {
                    v0_idx = other.v0_idx;
                    v1_idx = other.v1_idx;
                }
            };

            // store edge length
            struct EdgeLen {
                std::size_t edge_idx;
                double len;

                EdgeLen() : edge_idx(0), len(-1) {}

                EdgeLen(std::size_t e, double l) : edge_idx(e), len(l) {}
            };

            // edge length compare function
            bool edge_len_cmp(struct EdgeLen &a, struct EdgeLen &b) {
                return a.len < b.len;
            }

            struct Face {
                std::size_t v[3];
                std::size_t rgba[4];

                explicit Face() {
                    v[0] = v[1] = v[2] = 0;
                    rgba[0] = rgba[1] = rgba[2] = rgba[3] = 0;
                }

                explicit Face(std::size_t v0, std::size_t v1, std::size_t v2, const std::size_t *color) {
                    v[0] = v0;
                    v[1] = v1;
                    v[2] = v2;

                    rgba[0] = color[0];
                    rgba[1] = color[1];
                    rgba[2] = color[2];
                    rgba[3] = color[3];
                }

                bool contains_edge(const struct Edge &e) const {
                    return (e == Edge(v[0], v[1])) || (e == Edge(v[0], v[2])) ||
                           (e == Edge(v[1], v[2]));
                }

                bool opposite_vertex(const struct Edge &e, std::size_t &ret_v) const {
                    if (Edge(v[0], v[1]) == e) {
                        ret_v = v[2];
                    } else if (Edge(v[0], v[2]) == e) {
                        ret_v = v[1];
                    } else if (Edge(v[1], v[2]) == e) {
                        ret_v = v[0];
                    } else {
                        return false;
                    }
                    return true;
                }

                bool opposite_edge(const std::size_t v_, struct Edge &e) const {
                    if (v_ == v[0]) {
                        e = Edge(v[1], v[2]);
                    } else if (v_ == v[1]) {
                        e = Edge(v[0], v[2]);
                    } else if (v_ == v[2]) {
                        e = Edge(v[0], v[1]);
                    } else {
                        return false;
                    }
                    return false;
                }

                bool split(const struct Edge &split_edge, const std::size_t split_v_id,
                           struct Face &f1, struct Face &f2) const {
                    if (!contains_edge(split_edge)) {
                        return false;
                    }

                    int opp_v_indices, pre_v_indices, next_v_indices;
                    opposite_vertex_id(split_edge, opp_v_indices);
                    next_v_indices = (opp_v_indices + 1) % 3;
                    pre_v_indices = (opp_v_indices + 2) % 3;

                    f1 = Face(split_v_id, v[opp_v_indices], v[next_v_indices], rgba);
                    f2 = Face(v[pre_v_indices], v[opp_v_indices], split_v_id, rgba);
                    return true;
                }

                void operator=(const Face &f) {
                    v[0] = f.v[0];
                    v[1] = f.v[1];
                    v[2] = f.v[2];

                    rgba[0] = f.rgba[0];
                    rgba[1] = f.rgba[1];
                    rgba[2] = f.rgba[2];
                    rgba[3] = f.rgba[3];
                }

            private:
                bool opposite_vertex_id(const struct Edge &e, int &v_id) const {
                    if (Edge(v[0], v[1]) == e) {
                        v_id = 2;
                    } else if (Edge(v[0], v[2]) == e) {
                        v_id = 1;
                    } else if (Edge(v[1], v[2]) == e) {
                        v_id = 0;
                    } else {
                        return false;
                    }
                    return true;
                }
            };

            struct EdgeFace {
                struct Edge e;
                std::vector<std::size_t> faces;

                EdgeFace(Edge e_) : e(e_) {}

                void replace_face(std::size_t old_face, std::size_t new_face) {
                    for (int i = 0; i < faces.size(); i++) {
                        if (faces[i] == old_face) {
                            faces[i] = new_face;
                            break;
                        }
                    }
                }

                void replace_faces(std::vector<std::size_t> &new_faces) {
                    faces.clear();
                    for (int i = 0; i < new_faces.size(); i++) {
                        faces.push_back(new_faces[i]);
                    }
                }
            };
        }

        bool stop_criteria(std::vector<__inner__::Face> &faces, std::vector<__inner__::EdgeLen> &edge_heap,
                           const std::size_t kMaxFaces) {
            if (faces.size() > kMaxFaces) {
                return true;
            }
            // TODO 加密程度
//        const double Length_Threshold = 0.09;
            const double Length_Threshold = 0.03;
            if (edge_heap[0].len < Length_Threshold) {
                return true;
            }
            return false;
        }

        bool make_mesh_dense(const AttributeMatrix &V, const IndexMatrix &F,
                             AttributeMatrix &out_V, IndexMatrix &out_F,
                             AttributeMatrix &out_FC, AttributeMatrix &out_dense_FC,
                             const std::size_t kMaxFaces) {
            std::vector<__inner__::Face> faces;
            std::vector<__inner__::Point> vertices;
            std::map<__inner__::Edge, std::size_t> edge_id_map;
            std::vector<__inner__::EdgeLen> edge_len_heap;
            std::vector<__inner__::EdgeFace> ef_adjacency;

            const std::size_t n_faces = F.rows();
            if (n_faces >= (255 * 255 * 255)) {
                return false;
            }

            if (F.rows() <= 0 || V.rows() <= 0) {
                return false;
            }

            // render color for each face
            {
                out_FC.resize(F.rows(), 4);
                __inner__::ColorGenerator generator;
                for (int i = 0; i < F.rows(); i++) {
                    for (int c = 0; c < 3; c++) {
                        out_FC(i, c) = Scalar(generator.rgb[c]);
                    }
                    out_FC(i, 3) = 255;
                    generator();
                }
            }

            // build edge-face adjacency
            {
                vertices.resize(V.rows());
                for (int i = 0; i < V.rows(); ++i) {
                    vertices[i] = __inner__::Point(V(i, 0), V(i, 1), V(i, 2));
                }

                // init vars: ef_adjacency and edge_id_map
                faces.resize(F.rows());
                for (int r = 0; r < F.rows(); r++) {
                    std::size_t f_color[4] = {std::size_t(out_FC(r, 0)), std::size_t(out_FC(r, 1)),
                                              std::size_t(out_FC(r, 2)), std::size_t(out_FC(r, 3))};
                    faces[r] = (__inner__::Face(F(r, 0), F(r, 1), F(r, 2), f_color));

                    std::size_t face_id = r;
                    for (int i = 0; i < 3; ++i) {
                        __inner__::Edge e(F(r, i), F(r, (i + 1) % 3));
                        auto it = edge_id_map.find(e);
                        if (it != edge_id_map.end()) {
                            ef_adjacency[it->second].faces.push_back(face_id);
                        } else {
                            ef_adjacency.push_back({e});
                            ef_adjacency.back().faces.push_back(face_id);
                            edge_id_map[e] = ef_adjacency.size() - 1;
                        }
                    }
                }

                edge_len_heap.resize(ef_adjacency.size());
                for (int i = 0; i < ef_adjacency.size(); ++i) {
                    __inner__::Point p0 = vertices[ef_adjacency[i].e.v0_idx];
                    __inner__::Point p1 = vertices[ef_adjacency[i].e.v1_idx];

                    double len = p0.dist(p1);
                    edge_len_heap[i] = __inner__::EdgeLen(i, len);
                }
                std::make_heap(edge_len_heap.begin(), edge_len_heap.end(), __inner__::edge_len_cmp);
            }

            // split mesh
            while (!stop_criteria(faces, edge_len_heap, kMaxFaces)) {
                std::size_t split_edge_id = edge_len_heap[0].edge_idx;
                std::pop_heap(edge_len_heap.begin(), edge_len_heap.end(), __inner__::edge_len_cmp);

                __inner__::Point p0 = vertices[ef_adjacency[split_edge_id].e.v0_idx];
                __inner__::Point p1 = vertices[ef_adjacency[split_edge_id].e.v1_idx];
                __inner__::Point mid_p = (p0 + p1) / 2;

                // add spilt vertex
                vertices.push_back(mid_p);
                std::size_t newly_vertex_id = vertices.size() - 1;

                const __inner__::Edge split_edge = ef_adjacency[split_edge_id].e;

                __inner__::Edge seg1(newly_vertex_id, split_edge.v0_idx);
                __inner__::Edge seg2(newly_vertex_id, split_edge.v1_idx);

                std::vector<std::size_t> seg1_adjacency_faces;
                std::vector<std::size_t> seg2_adjacency_faces;

                int n_adj_face = ef_adjacency[split_edge_id].faces.size();
                for (int i = 0; i < n_adj_face; i++) {
                    {
                        // get opposite vertex id
                        std::size_t f_id = ef_adjacency[split_edge_id].faces[i];
                        std::size_t opposite_vertex_id;
                        if (!faces[f_id].opposite_vertex(split_edge, opposite_vertex_id)) {
                            throw std::runtime_error("edge not belong to the face. ");
                        }

                        // add spilt faces
                        __inner__::Face f1, f2;
                        if (!faces[f_id].split(split_edge, newly_vertex_id, f1, f2)) {
                            throw std::runtime_error("face can't be spilt. ");
                        }
                        faces[f_id] = f1;
                        faces.push_back(f2);

                        // record new face ids
                        std::size_t split_face1_id = f_id;
                        std::size_t split_face2_id = faces.size() - 1;

                        if (f1.contains_edge(seg1)) {
                            seg1_adjacency_faces.push_back(split_face1_id);
                            seg2_adjacency_faces.push_back(split_face2_id);
                        } else if (f1.contains_edge(seg2)) {
                            seg1_adjacency_faces.push_back(split_face2_id);
                            seg2_adjacency_faces.push_back(split_face1_id);
                        } else {
                            throw std::runtime_error("split face not contains split edges. ");
                        }

                        // add new edge
                        __inner__::Edge new_edge(newly_vertex_id, opposite_vertex_id);
                        ef_adjacency.push_back(__inner__::EdgeFace(new_edge));
                        ef_adjacency.back().faces.push_back(split_face1_id);
                        ef_adjacency.back().faces.push_back(split_face2_id);

                        std::size_t new_edge_id = ef_adjacency.size() - 1;
                        edge_id_map[new_edge] = new_edge_id;

                        double len = vertices[newly_vertex_id].dist(vertices[opposite_vertex_id]);
                        if (i == 0) {
                            edge_len_heap.back() = __inner__::EdgeLen(new_edge_id, len);
                        } else {
                            edge_len_heap.push_back(__inner__::EdgeLen(new_edge_id, len));
                        }
                        std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), __inner__::edge_len_cmp);

                        // modify edge - face adjacency
                        __inner__::Edge f1_opposite_edge;
                        f1.opposite_edge(newly_vertex_id, f1_opposite_edge);
                        std::size_t f1_oppo_edge_id = edge_id_map[f1_opposite_edge];
                        ef_adjacency[f1_oppo_edge_id].replace_face(f_id, split_face1_id);

                        __inner__::Edge f2_opposite_edge;
                        f2.opposite_edge(newly_vertex_id, f2_opposite_edge);
                        std::size_t f2_oppo_edge_id = edge_id_map[f2_opposite_edge];
                        ef_adjacency[f2_oppo_edge_id].replace_face(f_id, split_face2_id);
                        // TODO make len heap
                    }
                }

                // add split edges
                {
                    ef_adjacency[split_edge_id] = seg1;
                    ef_adjacency[split_edge_id].replace_faces(seg1_adjacency_faces);
                    std::map<__inner__::Edge, std::size_t>::iterator it = edge_id_map.find(split_edge);
                    if (it == edge_id_map.end()) {
                        throw std::runtime_error("split edge not exist. ");
                    }
                    edge_id_map.erase(it);
                    edge_id_map[seg1] = split_edge_id;
                    edge_len_heap.push_back({split_edge_id, vertices[seg1.v0_idx].dist(vertices[seg1.v1_idx])});
                    std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), __inner__::edge_len_cmp);

                    ef_adjacency.push_back(seg2);
                    edge_id_map[seg2] = ef_adjacency.size() - 1;
                    ef_adjacency.back().replace_faces(seg2_adjacency_faces);
                    edge_len_heap.push_back(
                            {ef_adjacency.size() - 1, vertices[seg2.v0_idx].dist(vertices[seg2.v1_idx])});
                    std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), __inner__::edge_len_cmp);
                }
            }

            // copy vertices
            AttributeMatrix eigen_vertices(vertices.size(), 3);
            for (int r = 0; r < vertices.size(); r++) {
                for (int c = 0; c < 3; c++) {
                    eigen_vertices(r, c) = vertices[r].p[c];
                }
            }
            out_V = eigen_vertices;

            // copy faces
            IndexMatrix eigen_faces(faces.size(), 3);
            AttributeMatrix eigen_fcolors(faces.size(), 4);
            for (int r = 0; r < faces.size(); r++) {
                for (int c = 0; c < 3; c++) {
                    eigen_faces(r, c) = faces[r].v[c];
                }

                for (int c = 0; c < 4; c++) {
                    eigen_fcolors(r, c) = faces[r].rgba[c];
                }
            }
            out_F = eigen_faces;
            out_dense_FC = eigen_fcolors;

            return true;
        }
    }
}
