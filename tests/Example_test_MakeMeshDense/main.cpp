//
// Created by Storm Phoenix on 2021/10/25.
//
#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/program_options.hpp>

#include <IO/IO.h>
#include <MvsTexturing.h>

namespace bpo = boost::program_options;

void parse_args(bpo::variables_map &vm, int argc, char **argv) {
    bpo::options_description opts("Example test MeshRemeshing options");
    opts.add_options()
            ("help", "produce help message")
            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
            ("output_mesh", bpo::value<std::string>()->default_value("res.ply"), "path of output simplified mesh");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        throw std::runtime_error("undefine options in command lines.\n");
    }
}

using namespace MvsTexturing;

namespace MakeDense {
    struct point {
        Base::Scalar p[3];

        point() {
            p[0] = p[1] = p[2] = 0;
        }

        point(Base::Scalar x, Base::Scalar y, Base::Scalar z) {
            p[0] = x;
            p[1] = y;
            p[2] = z;
        }

        point operator+(const point &other) const {
            return point((x() + other.x()), (y() + other.y()), (z() + other.z()));
        }

        point operator/(double val) const {
            return point(x() / val, y() / val, z() / val);
        }

        Base::Scalar x() const {
            return p[0];
        }

        Base::Scalar y() const {
            return p[1];
        }

        Base::Scalar z() const {
            return p[2];
        }

        double dist(const point &other) const {
            return std::sqrt(std::pow(std::abs(x() - other.x()), 2) +
                             std::pow(std::abs(y() - other.y()), 2) +
                             std::pow(std::abs(z() - other.z()), 2));
        }
    };

    struct edge {
        std::size_t v0_idx, v1_idx;

        edge() : v0_idx(0), v1_idx(0) {}

        edge(std::size_t v0, std::size_t v1) : v0_idx(v0), v1_idx(v1) {
            if (v0_idx > v1_idx) {
                std::swap(v0_idx, v1_idx);
            }
        }

        bool operator<(const struct edge &other) const {
            if (v0_idx < other.v0_idx) {
                return true;
            } else if (v0_idx == other.v0_idx) {
                return v1_idx < other.v1_idx;
            } else {
                return false;
            }
        }

        bool operator==(const struct edge &other) const {
            return (v0_idx == other.v0_idx) && (v1_idx == other.v1_idx);
        }

        void operator=(const edge &other) {
            v0_idx = other.v0_idx;
            v1_idx = other.v1_idx;
        }
    };

    struct edge_len {
        std::size_t edge_idx;
        double len;

        edge_len() : edge_idx(0), len(-1) {}

        edge_len(std::size_t e, double l) : edge_idx(e), len(l) {}
    };

    bool edge_len_cmp(struct edge_len &a, struct edge_len &b) {
        return a.len < b.len;
    }

    struct face {
        std::size_t v[3];

        face() {
            v[0] = v[1] = v[2] = 0;
        }

        face(std::size_t v0, std::size_t v1, std::size_t v2) {
            v[0] = v0;
            v[1] = v1;
            v[2] = v2;
        }

        bool contains_edge(const struct edge &e) const {
            return (e == edge(v[0], v[1])) || (e == edge(v[0], v[2])) ||
                   (e == edge(v[1], v[2]));
        }

        bool opposite_vertex(const struct edge &e, std::size_t &ret_v) const {
            if (edge(v[0], v[1]) == e) {
                ret_v = v[2];
            } else if (edge(v[0], v[2]) == e) {
                ret_v = v[1];
            } else if (edge(v[1], v[2]) == e) {
                ret_v = v[0];
            } else {
                return false;
            }
            return true;
        }

        bool opposite_edge(const std::size_t v_, struct edge &e) const {
            if (v_ == v[0]) {
                e = edge(v[1], v[2]);
            } else if (v_ == v[1]) {
                e = edge(v[0], v[2]);
            } else if (v_ == v[2]) {
                e = edge(v[0], v[1]);
            } else {
                return false;
            }
            return false;
        }

        bool split(const struct edge &split_edge, const std::size_t split_v_id,
                   struct face &f1, struct face &f2) const {
            if (!contains_edge(split_edge)) {
                return false;
            }

            int opp_v_indices, pre_v_indices, next_v_indices;
            opposite_vertex_id(split_edge, opp_v_indices);
            next_v_indices = (opp_v_indices + 1) % 3;
            pre_v_indices = (opp_v_indices + 2) % 3;

            f1 = face(split_v_id, v[opp_v_indices], v[next_v_indices]);
            f2 = face(v[pre_v_indices], v[opp_v_indices], split_v_id);
            return true;
        }

        void operator=(const face &f) {
            v[0] = f.v[0];
            v[1] = f.v[1];
            v[2] = f.v[2];
        }

    private:
        bool opposite_vertex_id(const struct edge &e, int &v_id) const {
            if (edge(v[0], v[1]) == e) {
                v_id = 2;
            } else if (edge(v[0], v[2]) == e) {
                v_id = 1;
            } else if (edge(v[1], v[2]) == e) {
                v_id = 0;
            } else {
                return false;
            }
            return true;
        }
    };

    struct edge_face {
        struct edge e;
        std::vector<std::size_t> faces;

        edge_face(edge e_) : e(e_) {}

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

    bool stop_criteria(std::vector<face> &faces, std::vector<edge_len> &edge_heap) {
        const double Length_Threshold = 0.02;
        if (edge_heap[0].len < Length_Threshold) {
            return true;
        }
        return false;
    }

    void make_mesh_dense(const Base::AttributeMatrix &V, const Base::IndexMatrix &F,
                         Base::AttributeMatrix &out_V, Base::IndexMatrix &out_F) {
        // TODO 修改后的 mesh 与未修改 mesh 之间的联系

        std::vector<face> faces;
        std::vector<point> vertices;
        std::map<edge, std::size_t> edge_id_map;
        std::vector<edge_len> edge_len_heap;
        std::vector<edge_face> ef_adjacency;

        if (F.rows() <= 0 || V.rows() <= 0) {
            return;
        }

        // build edge-face adjacency
        {
            vertices.resize(V.rows());
            for (int i = 0; i < V.rows(); ++i) {
                vertices[i] = point(V(i, 0), V(i, 1), V(i, 2));
            }

            faces.resize(F.rows());
            for (int r = 0; r < F.rows(); r++) {
                faces[r] = (face(F(r, 0), F(r, 1), F(r, 2)));

                std::size_t face_id = r;
                for (int i = 0; i < 3; ++i) {
                    edge e(F(r, i), F(r, (i + 1) % 3));
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
                point p0 = vertices[ef_adjacency[i].e.v0_idx];
                point p1 = vertices[ef_adjacency[i].e.v1_idx];

                double len = p0.dist(p1);
                edge_len_heap[i] = edge_len(i, len);
            }
            std::make_heap(edge_len_heap.begin(), edge_len_heap.end(), edge_len_cmp);
        }

        // split mesh
        while (!stop_criteria(faces, edge_len_heap)) {
            std::size_t split_edge_id = edge_len_heap[0].edge_idx;
            std::pop_heap(edge_len_heap.begin(), edge_len_heap.end(), edge_len_cmp);

            point p0 = vertices[ef_adjacency[split_edge_id].e.v0_idx];
            point p1 = vertices[ef_adjacency[split_edge_id].e.v1_idx];
            point mid_p = (p0 + p1) / 2;

            // add spilt vertex
            vertices.push_back(mid_p);
            std::size_t newly_vertex_id = vertices.size() - 1;

            const edge split_edge = ef_adjacency[split_edge_id].e;

            edge seg1(newly_vertex_id, split_edge.v0_idx);
            edge seg2(newly_vertex_id, split_edge.v1_idx);

            std::vector<std::size_t> seg1_adjacency_faces;
            std::vector<std::size_t> seg2_adjacency_faces;

            int n_adj_face = ef_adjacency[split_edge_id].faces.size();
            for (int i = 0; i < n_adj_face; i++) {
                {
                    std::size_t f_id = ef_adjacency[split_edge_id].faces[i];
                    std::size_t opposite_vertex_id;
                    if (!faces[f_id].opposite_vertex(split_edge, opposite_vertex_id)) {
                        throw std::runtime_error("edge not belong to the face. ");
                    }

                    // add spilt faces
                    face f1, f2;
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
                    edge new_edge(newly_vertex_id, opposite_vertex_id);
                    ef_adjacency.push_back(edge_face(new_edge));
                    ef_adjacency.back().faces.push_back(split_face1_id);
                    ef_adjacency.back().faces.push_back(split_face2_id);

                    std::size_t new_edge_id = ef_adjacency.size() - 1;
                    edge_id_map[new_edge] = new_edge_id;

                    double len = vertices[newly_vertex_id].dist(vertices[opposite_vertex_id]);
                    if (i == 0) {
                        edge_len_heap.back() = edge_len(new_edge_id, len);
                    } else {
                        edge_len_heap.push_back(edge_len(new_edge_id, len));
                    }
                    std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), edge_len_cmp);

                    // modify edge - face adjacency
                    edge f1_opposite_edge;
                    f1.opposite_edge(newly_vertex_id, f1_opposite_edge);
                    std::size_t f1_oppo_edge_id = edge_id_map[f1_opposite_edge];
                    ef_adjacency[f1_oppo_edge_id].replace_face(f_id, split_face1_id);

                    edge f2_opposite_edge;
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
                std::map<edge, std::size_t>::iterator it = edge_id_map.find(split_edge);
                if (it == edge_id_map.end()) {
                    throw std::runtime_error("splite edge not exist. ");
                }
                edge_id_map.erase(it);
                edge_id_map[seg1] = split_edge_id;
                edge_len_heap.push_back({split_edge_id, vertices[seg1.v0_idx].dist(vertices[seg1.v1_idx])});
                std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), edge_len_cmp);

                ef_adjacency.push_back(seg2);
                edge_id_map[seg2] = ef_adjacency.size() - 1;
                ef_adjacency.back().replace_faces(seg2_adjacency_faces);
                edge_len_heap.push_back({ef_adjacency.size() - 1, vertices[seg2.v0_idx].dist(vertices[seg2.v1_idx])});
                std::push_heap(edge_len_heap.begin(), edge_len_heap.end(), edge_len_cmp);
            }
        }

        Base::AttributeMatrix eigen_vertices(vertices.size(), 3);
        for (int r = 0; r < vertices.size(); r++) {
            for (int c = 0; c < 3; c++) {
                eigen_vertices(r, c) = vertices[r].p[c];
            }
        }
        out_V = eigen_vertices;

        Base::IndexMatrix eigen_faces(faces.size(), 3);
        for (int r = 0; r < faces.size(); r++) {
            for (int c = 0; c < 3; c++) {
                eigen_faces(r, c) = faces[r].v[c];
            }
        }
        out_F = eigen_faces;

        std::cout << "MakeDense result model - faces: " << out_V.rows() << " vertices: " << out_F.rows() << std::endl;

        return;
    }
}

int main(int argc, char **argv) {
    bpo::variables_map vm;
    parse_args(vm, argc, argv);

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();

    using namespace MvsTexturing;
    Base::AttributeMatrix V, N, T;
    Base::IndexMatrix F, FN, FT;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> material_map;
//    MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, V, N, T, F, FN, FT, face_materials, material_map);
    MvsTexturing::IO::load_mesh_from_ply(in_mesh_path, V, F);

    Base::AttributeMatrix out_V;
    Base::IndexMatrix out_F;

    std::cout << "MakeDense origin model - faces: " << V.rows() << " vertices: " << F.rows() << std::endl;
    MakeDense::make_mesh_dense(V, F, out_V, out_F);

    MvsTexturing::IO::save_mesh(out_mesh_path, out_V, out_F);

    return 0;
}