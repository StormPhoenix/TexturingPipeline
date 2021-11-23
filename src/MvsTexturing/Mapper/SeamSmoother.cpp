//
// Created by Storm Phoenix on 2021/10/17.
//

#include <set>
#include <map>
#include <iostream>

#include <util/timer.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <math/accum.h>
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>

#include "Base/LabelGraph.h"
#include "Base/TexturePatch.h"

#include "MvsTexturing.h"

namespace MvsTexturing {
    namespace SeamSmoother {
        typedef Eigen::SparseMatrix<float> SpMat;

        struct MeshEdge {
            std::size_t v1;
            std::size_t v2;
        };

        math::Vec3f sample_edge(Base::TexturePatch::ConstPtr texture_patch, math::Vec2f p1, math::Vec2f p2) {
            math::Vec2f p12 = p2 - p1;
            std::size_t num_samples = std::max(p12.norm(), 1.0f) * 2.0f;
            math::Accum<math::Vec3f> color_accum(math::Vec3f(0.0f));
            /* Sample the edge with linear weights. */
            for (std::size_t s = 0; s < num_samples; ++s) {
                float fraction = static_cast<float>(s) / (num_samples - 1);
                math::Vec2f sample_point = p1 + p12 * fraction;
                math::Vec3f color(texture_patch->get_pixel_value(sample_point));
                color_accum.add(color, 1.0f - fraction);
            }

            return color_accum.normalized();
        }

        void find_mesh_edge_projections(const VertexProjectionInfoList &vertex_projection_infos,
                                        MeshEdge mesh_edge,
                                        EdgeProjectionInfoList *edge_projection_infos) {
            std::vector<Base::VertexProjectionInfo> const &v1_projection_infos = vertex_projection_infos[mesh_edge.v1];
            std::vector<Base::VertexProjectionInfo> const &v2_projection_infos = vertex_projection_infos[mesh_edge.v2];

            /* Use a set to eliminate duplicates which may occur if the mesh is degenerated. */
            std::set<Base::EdgeProjectionInfo> edge_projection_infos_set;

            for (Base::VertexProjectionInfo v1_projection_info : v1_projection_infos) {
                for (Base::VertexProjectionInfo v2_projection_info : v2_projection_infos) {
                    if (v1_projection_info.texture_patch_id != v2_projection_info.texture_patch_id) continue;

                    for (std::size_t face_id1 : v1_projection_info.faces) {
                        for (std::size_t face_id2 : v2_projection_info.faces) {
                            if (face_id1 != face_id2) continue;

                            std::size_t texture_patch_id = v1_projection_info.texture_patch_id;
                            math::Vec2f p1 = v1_projection_info.projection;
                            math::Vec2f p2 = v2_projection_info.projection;

                            Base::EdgeProjectionInfo edge_projection_info = {texture_patch_id, p1, p2};
                            edge_projection_infos_set.insert(edge_projection_info);
                        }
                    }
                }
            }

            edge_projection_infos->insert(edge_projection_infos->end(), edge_projection_infos_set.begin(),
                                          edge_projection_infos_set.end());
        }

        math::Vec3f calculate_difference(const VertexProjectionInfoList &vertex_projection_infos,
                                         MeshConstPtr &mesh, const TexturePatchList &texture_patches,
                                         const std::vector<MeshEdge> &seam_edges, int label1, int label2) {

            assert(label1 != 0 && label2 != 0 && label1 < label2);
            assert(!seam_edges.empty());

            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();

            math::Accum<math::Vec3f> color1_accum(math::Vec3f(0.0f));
            math::Accum<math::Vec3f> color2_accum(math::Vec3f(0.0f));

            for (MeshEdge const &seam_edge : seam_edges) {
                math::Vec3f v1 = vertices[seam_edge.v1];
                math::Vec3f v2 = vertices[seam_edge.v2];
                float length = (v2 - v1).norm();

                assert(length != 0.0f);

                std::vector<Base::EdgeProjectionInfo> edge_projection_infos;
                find_mesh_edge_projections(vertex_projection_infos, seam_edge, &edge_projection_infos);

                std::size_t num_samples = 0;

                for (Base::EdgeProjectionInfo const &edge_projection_info : edge_projection_infos) {
                    Base::TexturePatch::Ptr texture_patch = texture_patches[edge_projection_info.texture_patch_id];
                    const int texture_patch_label = texture_patch->get_label();
                    if (texture_patch_label == label1 || texture_patch_label == label2) {
                        if (texture_patch_label == label1)
                            color1_accum.add(
                                    sample_edge(texture_patch, edge_projection_info.p1, edge_projection_info.p2),
                                    length);

                        if (texture_patch_label == label2)
                            color2_accum.add(
                                    sample_edge(texture_patch, edge_projection_info.p1, edge_projection_info.p2),
                                    length);

                        num_samples++;
                    }
                }
                assert(num_samples == 2);
            }

            math::Vec3f color1 = color1_accum.normalized();
            math::Vec3f color2 = color2_accum.normalized();

            /* The order is essential. */
            math::Vec3f difference = color2 - color1;

            assert(!std::isnan(difference[0]));
            assert(!std::isnan(difference[1]));
            assert(!std::isnan(difference[2]));

            return difference;
        }

        void
        find_seam_edges_for_vertex_label_combination(const Base::LabelGraph &graph, MeshConstPtr &mesh,
                                                     MeshInfo const &mesh_info, std::size_t vertex,
                                                     std::size_t label1, std::size_t label2,
                                                     std::vector<MeshEdge> *seam_edges) {

            assert(label1 != 0 && label2 != 0 && label1 < label2);

            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();

            std::vector<std::size_t> const &adj_verts = mesh_info[vertex].verts;
            for (std::size_t i = 0; i < adj_verts.size(); ++i) {
                std::size_t adj_vertex = adj_verts[i];
                if (vertex == adj_vertex) continue;

                std::vector<std::size_t> edge_faces;
                mesh_info.get_faces_for_edge(vertex, adj_vertex, &edge_faces);

                for (std::size_t j = 0; j < edge_faces.size(); ++j) {
                    for (std::size_t k = j + 1; k < edge_faces.size(); ++k) {

                        std::size_t face_label1 = graph.get_label(edge_faces[j]);
                        std::size_t face_label2 = graph.get_label(edge_faces[k]);
                        if (!(face_label1 < face_label2)) std::swap(face_label1, face_label2);

                        if (face_label1 != label1 || face_label2 != label2) continue;

                        math::Vec3f v1 = vertices[vertex];
                        math::Vec3f v2 = vertices[adj_vertex];
                        float length = (v2 - v1).norm();

                        /* Ignore zero length edges. */
                        if (length == 0.0f) continue;

                        MeshEdge seam_edge = {vertex, adj_vertex};
                        seam_edges->push_back(seam_edge);
                    }
                }
            }
        }

        void global_seam_leveling(const Base::LabelGraph &graph,
                                  mve::TriangleMesh::ConstPtr mesh,
                                  const MeshInfo &mesh_info,
                                  const VertexProjectionInfoList &vertex_projection_infos,
                                  TexturePatchList *texture_patches) {

            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();
            std::size_t const num_vertices = vertices.size();

            std::vector<std::map<std::size_t, std::size_t> > vertlabel2row;
            vertlabel2row.resize(num_vertices);

            std::vector<std::vector<std::size_t> > labels;
            labels.resize(num_vertices);

            /* Assign each vertex for each label a new index(row) within the solution vector x. */
            std::size_t x_row = 0;
            for (std::size_t i = 0; i < num_vertices; ++i) {
                std::set<std::size_t> label_set;

                std::vector<std::size_t> faces = mesh_info[i].faces;
                std::set<std::size_t>::iterator it = label_set.begin();
                for (std::size_t j = 0; j < faces.size(); ++j) {
                    std::size_t label = graph.get_label(faces[j]);
                    label_set.insert(it, label);
                }

                for (it = label_set.begin(); it != label_set.end(); ++it) {
                    std::size_t label = *it;
                    if (label == 0) continue;
                    vertlabel2row[i][label] = x_row;
                    labels[i].push_back(label);
                    ++x_row;
                }
            }
            std::size_t x_rows = x_row;
            assert(x_rows < static_cast<std::size_t>(std::numeric_limits<int>::max()));

            float const lambda = 0.1f;

            /* Fill the Tikhonov matrix Gamma(regularization constraints). */
            std::size_t Gamma_row = 0;
            std::vector<Eigen::Triplet<float, int> > coefficients_Gamma;
            coefficients_Gamma.reserve(2 * num_vertices);
            for (std::size_t i = 0; i < num_vertices; ++i) {
                for (std::size_t j = 0; j < labels[i].size(); ++j) {
                    std::vector<std::size_t> const &adj_verts = mesh_info[i].verts;
                    for (std::size_t k = 0; k < adj_verts.size(); ++k) {
                        std::size_t adj_vertex = adj_verts[k];
                        for (std::size_t l = 0; l < labels[adj_vertex].size(); ++l) {
                            std::size_t label = labels[i][j];
                            std::size_t adj_vertex_label = labels[adj_vertex][l];
                            if (i < adj_vertex && label == adj_vertex_label) {
                                Eigen::Triplet<float, int> t1(Gamma_row, vertlabel2row[i][label], lambda);
                                Eigen::Triplet<float, int> t2(Gamma_row, vertlabel2row[adj_vertex][adj_vertex_label],
                                                              -lambda);
                                coefficients_Gamma.push_back(t1);
                                coefficients_Gamma.push_back(t2);
                                Gamma_row++;
                            }
                        }
                    }
                }
            }
            std::size_t Gamma_rows = Gamma_row;
            assert(Gamma_rows < static_cast<std::size_t>(std::numeric_limits<int>::max()));

            SpMat Gamma(Gamma_rows, x_rows);
            Gamma.setFromTriplets(coefficients_Gamma.begin(), coefficients_Gamma.end());

            /* Fill the matrix A and the coefficients for the Vector b of the linear equation system. */
            std::vector<Eigen::Triplet<float, int> > coefficients_A;
            std::vector<math::Vec3f> coefficients_b;
            std::size_t A_row = 0;
            for (std::size_t i = 0; i < num_vertices; ++i) {
                for (std::size_t j = 0; j < labels[i].size(); ++j) {
                    for (std::size_t k = 0; k < labels[i].size(); ++k) {
                        std::size_t label1 = labels[i][j];
                        std::size_t label2 = labels[i][k];
                        if (label1 < label2) {

                            std::vector<MeshEdge> seam_edges;
                            find_seam_edges_for_vertex_label_combination(graph, mesh, mesh_info, i, label1, label2,
                                                                         &seam_edges);

                            if (seam_edges.empty()) continue;

                            Eigen::Triplet<float, int> t1(A_row, vertlabel2row[i][label1], 1.0f);
                            Eigen::Triplet<float, int> t2(A_row, vertlabel2row[i][label2], -1.0f);
                            coefficients_A.push_back(t1);
                            coefficients_A.push_back(t2);

                            coefficients_b.push_back(
                                    calculate_difference(vertex_projection_infos, mesh, *texture_patches, seam_edges,
                                                         label1, label2));

                            ++A_row;
                        }
                    }
                }
            }

            std::size_t A_rows = A_row;
            assert(A_rows < static_cast<std::size_t>(std::numeric_limits<int>::max()));

            SpMat A(A_rows, x_rows);
            A.setFromTriplets(coefficients_A.begin(), coefficients_A.end());

            SpMat Lhs = A.transpose() * A + Gamma.transpose() * Gamma;
            /* Only keep lower triangle (CG only uses the lower), prune the rest and compress matrix. */
            Lhs.prune([](const int &row, const int &col, const float &value) -> bool {
                return col <= row && value != 0.0f;
            }); // value != 0.0f is only to suppress a compiler warning

            std::vector<std::map<std::size_t, math::Vec3f> > adjust_values(num_vertices);
            std::cout << "\n\tLhs dimensionality: " << Lhs.rows() << " x " << Lhs.cols() << std::endl;

            util::WallTimer timer;
            std::cout << "\tCalculating adjustments:" << std::endl;
#pragma omp parallel for
            for (std::size_t channel = 0; channel < 3; ++channel) {
                /* Prepare solver. */
                Eigen::ConjugateGradient<SpMat, Eigen::Lower> cg;
                cg.setMaxIterations(1000);
                cg.setTolerance(0.0001);
                cg.compute(Lhs);

                /* Prepare right hand side. */
                Eigen::VectorXf b(A_rows);
                for (std::size_t i = 0; i < coefficients_b.size(); ++i) {
                    b[i] = coefficients_b[i][channel];
                }
                Eigen::VectorXf Rhs = SpMat(A.transpose()) * b;

                /* Solve for x. */
                Eigen::VectorXf x(x_rows);
                x = cg.solve(Rhs);

                /* Subtract mean because system is underconstrained and we seek the solution with minimal adjustments. */
                x = x.array() - x.mean();

#pragma omp critical
                std::cout << "\t\tColor channel " << channel << ": CG took "
                          << cg.iterations() << " iterations. Residual is " << cg.error() << std::endl;

#pragma omp critical
                for (std::size_t i = 0; i < num_vertices; ++i) {
                    for (std::size_t j = 0; j < labels[i].size(); ++j) {
                        std::size_t label = labels[i][j];
                        adjust_values[i][label][channel] = x[vertlabel2row[i][label]];
                    }
                }
            }
            std::cout << "\t\tTook " << timer.get_elapsed_sec() << " seconds" << std::endl;

            mve::TriangleMesh::FaceList const &mesh_faces = mesh->get_faces();

#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches->size(); ++i) {
                Base::TexturePatch::Ptr texture_patch = texture_patches->at(i);

                int label = texture_patch->get_label();
                std::vector<std::size_t> const &faces = texture_patch->get_faces();
                std::vector<math::Vec3f> patch_adjust_values(faces.size() * 3, math::Vec3f(0.0f));

                /* Only adjust texture_patches originating form input images. */
                if (label == 0) {
                    texture_patch->adjust_colors(patch_adjust_values);
                    continue;
                };

                for (std::size_t j = 0; j < faces.size(); ++j) {
                    for (std::size_t k = 0; k < 3; ++k) {
                        std::size_t face_pos = faces[j] * 3 + k;
                        std::size_t vertex = mesh_faces[face_pos];
                        patch_adjust_values[j * 3 + k] = adjust_values[vertex].find(label)->second;
                    }
                }

                texture_patch->adjust_colors(patch_adjust_values);
            }
        }

        math::Vec3f mean_color_of_edge_point(const EdgeProjectionInfoList &edge_projection_infos,
                                 const TexturePatchList &texture_patches, float t) {

            assert(0.0f <= t && t <= 1.0f);
            math::Accum<math::Vec3f> color_accum(math::Vec3f(0.0f));

            for (const Base::EdgeProjectionInfo &edge_projection_info : edge_projection_infos) {
                Base::TexturePatch::Ptr texture_patch = texture_patches[edge_projection_info.texture_patch_id];
                if (texture_patch->get_label() == 0) continue;
                math::Vec2f pixel = edge_projection_info.p1 * t + (1.0f - t) * edge_projection_info.p2;
                math::Vec3f color = texture_patch->get_pixel_value(pixel);
                color_accum.add(color, 1.0f);
            }

            math::Vec3f mean_color = color_accum.normalized();
            return mean_color;
        }

        void
        draw_line(math::Vec2f p1, math::Vec2f p2,
                  std::vector<math::Vec3f> const &edge_color, Base::TexturePatch::Ptr texture_patch) {
            /* http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm */

            int x0 = std::floor(p1[0] + 0.5f);
            int y0 = std::floor(p1[1] + 0.5f);
            int const x1 = std::floor(p2[0] + 0.5f);
            int const y1 = std::floor(p2[1] + 0.5f);

            float tdx = static_cast<float>(x1 - x0);
            float tdy = static_cast<float>(y1 - y0);
            float length = std::sqrt(tdx * tdx + tdy * tdy);

            int const dx = std::abs(x1 - x0);
            int const dy = std::abs(y1 - y0);
            int const sx = x0 < x1 ? 1 : -1;
            int const sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            int x = x0;
            int y = y0;
            while (true) {
                math::Vec2i pixel(x, y);

                tdx = static_cast<float>(x1 - x);
                tdy = static_cast<float>(y1 - y);

                /* If the length is zero we sample the midpoint of the projected edge. */
                float t = (length != 0.0f) ? std::sqrt(tdx * tdx + tdy * tdy) / length : 0.5f;

                math::Vec3f color;
                if (t < 1.0f && edge_color.size() > 1) {
                    std::size_t idx = std::floor(t * (edge_color.size() - 1));
                    color = (1.0f - t) * edge_color[idx] + t * edge_color[idx + 1];
                } else {
                    color = edge_color.back();
                }

                texture_patch->set_pixel_value(pixel, color);
                if (x == x1 && y == y1)
                    break;

                int const e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y += sy;
                }
            }
        }

        struct Pixel {
            math::Vec2i pos;
            math::Vec3f const *color;
        };

        struct Line {
            math::Vec2i from;
            math::Vec2i to;
            std::vector<math::Vec3f> const *color;
        };

        void find_seam_edges(const Base::LabelGraph &graph, mve::TriangleMesh::ConstPtr mesh,
                        std::vector<MeshEdge> *seam_edges) {
            mve::TriangleMesh::FaceList const &faces = mesh->get_faces();

            seam_edges->clear();

            // Is it possible that a single edge is part of more than three faces whichs' label is non zero???

            for (std::size_t node = 0; node < graph.num_nodes(); ++node) {
                std::vector<std::size_t> const &adj_nodes = graph.get_adj_nodes(node);
                for (std::size_t adj_node : adj_nodes) {
                    /* Add each edge only once. */
                    if (node > adj_node) continue;

                    int label1 = graph.get_label(node);
                    int label2 = graph.get_label(adj_node);
                    /* Add only seam edges. */
                    if (label1 == label2) continue;

                    /* Find shared edge of the faces. */
                    std::vector<std::size_t> shared_edge;
                    for (int i = 0; i < 3; ++i) {
                        std::size_t v1 = faces[3 * node + i];

                        for (int j = 0; j < 3; j++) {
                            std::size_t v2 = faces[3 * adj_node + j];

                            if (v1 == v2) shared_edge.push_back(v1);
                        }
                    }

                    assert(shared_edge.size() == 2);
                    std::size_t v1 = shared_edge[0];
                    std::size_t v2 = shared_edge[1];

                    assert(v1 != v2);
                    if (v1 > v2) std::swap(v1, v2);

                    MeshEdge seam_edge = {v1, v2};
                    seam_edges->push_back(seam_edge);
                }
            }
        }

#define STRIP_SIZE 20

        void local_seam_leveling(const Base::LabelGraph &graph, mve::TriangleMesh::ConstPtr mesh,
                                 const VertexProjectionInfoList &vertex_projection_infos,
                                 TexturePatchList *texture_patches) {

            std::size_t const num_vertices = vertex_projection_infos.size();
            std::vector<math::Vec3f> vertex_colors(num_vertices);
            std::vector<std::vector<math::Vec3f> > edge_colors;

            // edge_projection_infos
            // 保存所有的 seam edge
            // seam edge 连接着两个 face，两个 face 有不同的 label
            // edge_projection_infos 将每个 face 在不同 label 上的投影坐标 uv 保存下来
            // 这一步骤：记录所有 seam edge 在 texture patch 的投影信息
            std::vector<std::vector<Base::EdgeProjectionInfo>> edge_projection_infos;
            {
                std::vector<MeshEdge> seam_edges;
                // 在 mesh 上依据 face 联通性和打上的 label 检查 seam_edges
                find_seam_edges(graph, mesh, &seam_edges);
                edge_colors.resize(seam_edges.size());
                edge_projection_infos.resize(seam_edges.size());
                for (std::size_t i = 0; i < seam_edges.size(); ++i) {
                    MeshEdge const &seam_edge = seam_edges[i];
                    find_mesh_edge_projections(vertex_projection_infos, seam_edge,
                                               &edge_projection_infos[i]);
                }
            }

            // lines 每个 texture patch 保存多个对应的 seam edge，seam edge 上采样许多 sample point，以及这些
            //      sample point 计算得到的 mean color
            // lines 按照 texture patch id 分类，每个分类下保存投射到的所有 edge 上采样点的 mean color
            std::vector<std::vector<Line> > lines(texture_patches->size());

            // pixels 按照 texture patch id 分类，每个分类下保存能投射到所有 vertex 的 mean color
            std::vector<std::vector<Pixel> > pixels(texture_patches->size());

            /* Sample edge colors. */
            for (std::size_t i = 0; i < edge_projection_infos.size(); ++i) {
                /* Determine sampling (ensure at least two samples per edge). */
                // TODO 临时修改
//        float max_length = 1;
                float max_length = 4;
                for (Base::EdgeProjectionInfo const &edge_projection_info : edge_projection_infos[i]) {
                    float length = (edge_projection_info.p1 - edge_projection_info.p2).norm();
                    max_length = std::max(max_length, length);
                }

                std::vector<math::Vec3f> &edge_color = edge_colors[i];
                edge_color.resize(std::ceil(max_length * 2.0f));
                // 在 seam edge 按照等距间隔采样点，每个点的色值在不同的 patch 上做平均
                for (std::size_t j = 0; j < edge_color.size(); ++j) {
                    float t = static_cast<float>(j) / (edge_color.size() - 1);
                    edge_color[j] = mean_color_of_edge_point(edge_projection_infos[i], *texture_patches, t);
                }

                for (Base::EdgeProjectionInfo const &edge_projection_info : edge_projection_infos[i]) {
                    Line line;
                    line.from = edge_projection_info.p1 + math::Vec2f(0.5f, 0.5f);
                    line.to = edge_projection_info.p2 + math::Vec2f(0.5f, 0.5f);
                    line.color = &edge_colors[i];
                    lines[edge_projection_info.texture_patch_id].push_back(line);
                }
            }

            /* Sample vertex colors. */
            // 上文计算了 seam edge 之间采样点的 mean color
            // 这里把 seam edge 两端点 vertex 的 mean color 给弄出来
            for (std::size_t i = 0; i < vertex_colors.size(); ++i) {
                std::vector<Base::VertexProjectionInfo> const &projection_infos = vertex_projection_infos[i];
                if (projection_infos.size() <= 1) continue;

                math::Accum<math::Vec3f> color_accum(math::Vec3f(0.0f));
                for (Base::VertexProjectionInfo const &projection_info : projection_infos) {
                    Base::TexturePatch::Ptr texture_patch = texture_patches->at(projection_info.texture_patch_id);
                    if (texture_patch->get_label() == 0) continue;
                    math::Vec3f color = texture_patch->get_pixel_value(projection_info.projection);
                    color_accum.add(color, 1.0f);
                }
                if (color_accum.w == 0.0f) continue;

                vertex_colors[i] = color_accum.normalized();

                for (Base::VertexProjectionInfo const &projection_info : projection_infos) {
                    Pixel pixel;
                    pixel.pos = math::Vec2i(projection_info.projection + math::Vec2f(0.5f, 0.5f));
                    pixel.color = &vertex_colors[i];
                    pixels[projection_info.texture_patch_id].push_back(pixel);
                }
            }

#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches->size(); ++i) {
                Base::TexturePatch::Ptr texture_patch = texture_patches->at(i);
                // 复制一份 texture patch 做备份
                mve::FloatImage::Ptr image = texture_patch->get_image()->duplicate();

                /* Apply colors. */
                for (Pixel const &pixel : pixels[i]) {
                    texture_patch->set_pixel_value(pixel.pos, *pixel.color);
                }

                for (Line const &line : lines[i]) {
                    draw_line(line.from, line.to, *line.color, texture_patch);
                }

                /* Only alter a small strip of texture patches originating from input images. */
                if (texture_patch->get_label() != 0) {
                    texture_patch->prepare_blending_mask(STRIP_SIZE);
                }

                texture_patch->blend(image);
                texture_patch->release_blending_mask();
            }
        }
    }
}