//
// Created by Storm Phoenix on 2021/10/17.
//
#include <set>
#include <map>
#include <list>
#include <iostream>
#include <fstream>

#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <mve/image_tools.h>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <util/timer.h>

#include "Parameter.h"
#include "Base/View.h"
#include "Base/LabelGraph.h"
#include "Base/TexturePatch.h"
#include "Base/TextureAtlas.h"

#include "MvsTexturing.h"

namespace MvsTexturing {
    namespace AtlasMapper {
#define MAX_HOLE_NUM_FACES 100
#define MAX_HOLE_PATCH_SIZE 100

#define MAX_TEXTURE_SIZE (8 * 1024)
#define PREF_TEXTURE_SIZE (4 * 1024)
#define MIN_TEXTURE_SIZE (256)

        template<typename T>
        T clamp_nan_low(T const &v, T const &lo, T const &hi) {
            return (v > lo) ? ((v < hi) ? v : hi) : lo;
        }

        template<typename T>
        T clamp_nan_hi(T const &v, T const &lo, T const &hi) {
            return (v < hi) ? ((v > lo) ? v : lo) : hi;
        }

        template<typename T>
        T clamp(T const &v, T const &lo, T const &hi) {
            return (v < lo) ? lo : ((v > hi) ? hi : v);
        }

        /** Struct representing a TexturePatch candidate
        * - final texture patches are obtained by merging candiates.
        **/
        struct TexturePatchCandidate {
            Math::Rect2D<int> bounding_box;
            Base::TexturePatch::Ptr texture_patch;
        };

        bool fill_hole(std::vector<std::size_t> const &hole, const Base::LabelGraph &graph,
                       MeshConstPtr mesh, const MeshInfo &mesh_info,
                       VertexProjectionInfoList *vertex_projection_infos,
                       TexturePatchList *texture_patches) {

            mve::TriangleMesh::FaceList const &mesh_faces = mesh->get_faces();
            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();

            std::map<std::size_t, std::set<std::size_t> > tmp;
            for (std::size_t const face_id : hole) {
                std::size_t const v0 = mesh_faces[face_id * 3 + 0];
                std::size_t const v1 = mesh_faces[face_id * 3 + 1];
                std::size_t const v2 = mesh_faces[face_id * 3 + 2];

                tmp[v0].insert(face_id);
                tmp[v1].insert(face_id);
                tmp[v2].insert(face_id);
            }

            std::size_t const num_vertices = tmp.size();
            /* Only fill small holes. */
            if (num_vertices > MAX_HOLE_NUM_FACES) return false;

            /* Calculate 2D parameterization using the technique from libremesh/patch2d,
             * which was published as source code accompanying the following paper:
             *
             * Isotropic Surface Remeshing
             * Simon Fuhrmann, Jens Ackermann, Thomas Kalbe, Michael Goesele
             */

            std::size_t seed = -1;
            std::vector<bool> is_border(num_vertices, false);
            std::vector<std::vector<std::size_t> > adj_verts_via_border(num_vertices);
            /* Index structures to map from local <-> global vertex id. */
            std::map<std::size_t, std::size_t> g2l;
            std::vector<std::size_t> l2g(num_vertices);
            /* Index structure to determine column in matrix/vector. */
            std::vector<std::size_t> idx(num_vertices);

            std::size_t num_border_vertices = 0;

            bool disk_topology = true;
            std::map<std::size_t, std::set<std::size_t> >::iterator it = tmp.begin();
            for (std::size_t j = 0; j < num_vertices; ++j, ++it) {
                std::size_t vertex_id = it->first;
                g2l[vertex_id] = j;
                l2g[j] = vertex_id;

                /* Check topology in original mesh. */
                if (mesh_info[vertex_id].vclass != mve::MeshInfo::VERTEX_CLASS_SIMPLE) {
                    /* Complex/Border vertex in original mesh */
                    disk_topology = false;
                    break;
                }

                /* Check new topology and determine if vertex is now at the border. */
                std::vector<std::size_t> const &adj_faces = mesh_info[vertex_id].faces;
                std::set<std::size_t> const &adj_hole_faces = it->second;
                std::vector<std::pair<std::size_t, std::size_t> > fan;
                for (std::size_t k = 0; k < adj_faces.size(); ++k) {
                    std::size_t adj_face = adj_faces[k];
                    if (graph.get_label(adj_faces[k]) == 0 &&
                        adj_hole_faces.find(adj_face) != adj_hole_faces.end()) {
                        std::size_t curr = adj_faces[k];
                        std::size_t next = adj_faces[(k + 1) % adj_faces.size()];
                        std::pair<std::size_t, std::size_t> pair(curr, next);
                        fan.push_back(pair);
                    }
                }

                std::size_t gaps = 0;
                for (std::size_t k = 0; k < fan.size(); k++) {
                    std::size_t curr = fan[k].first;
                    std::size_t next = fan[(k + 1) % fan.size()].first;
                    if (fan[k].second != next) {
                        ++gaps;

                        for (std::size_t l = 0; l < 3; ++l) {
                            if (mesh_faces[curr * 3 + l] == vertex_id) {
                                std::size_t second = mesh_faces[curr * 3 + (l + 2) % 3];
                                adj_verts_via_border[j].push_back(second);
                            }
                            if (mesh_faces[next * 3 + l] == vertex_id) {
                                std::size_t first = mesh_faces[next * 3 + (l + 1) % 3];
                                adj_verts_via_border[j].push_back(first);
                            }
                        }
                    }
                }

                is_border[j] = gaps == 1;

                /* Check if vertex is now complex. */
                if (gaps > 1) {
                    /* Complex vertex in hole */
                    disk_topology = false;
                    break;
                }

                if (is_border[j]) {
                    idx[j] = num_border_vertices++;
                    seed = vertex_id;
                } else {
                    idx[j] = j - num_border_vertices;
                }
            }
            tmp.clear();

            /* No disk or genus zero topology */
            if (!disk_topology || num_border_vertices == 0) return false;

            std::vector<std::size_t> border;
            border.reserve(num_border_vertices);
            std::size_t prev = seed;
            std::size_t curr = seed;
            while (prev == seed || curr != seed) {
                std::size_t next = std::numeric_limits<std::size_t>::max();
                std::vector<std::size_t> const &adj_verts = adj_verts_via_border[g2l[curr]];
                for (std::size_t adj_vert : adj_verts) {
                    assert(is_border[g2l[adj_vert]]);
                    if (adj_vert != prev && adj_vert != curr) {
                        next = adj_vert;
                        break;
                    }
                }
                if (next != std::numeric_limits<std::size_t>::max()) {
                    prev = curr;
                    curr = next;
                    border.push_back(next);
                } else {
                    /* No new border vertex */
                    border.clear();
                    break;
                }

                /* Loop within border */
                if (border.size() > num_border_vertices) break;
            }

            if (border.size() != num_border_vertices) return false;

            float total_length = 0.0f;
            float total_projection_length = 0.0f;
            for (std::size_t j = 0; j < border.size(); ++j) {
                std::size_t vi0 = border[j];
                std::size_t vi1 = border[(j + 1) % border.size()];
                std::vector<Base::VertexProjectionInfo> vpi0, vpi1;
#pragma omp critical (vpis)
                {
                    vpi0 = vertex_projection_infos->at(vi0);
                    vpi1 = vertex_projection_infos->at(vi1);
                }

                /* According to the previous checks (vertex class within the origial
                 * mesh and boundary) there has to be at least one projection
                 * of each border vertex in a common texture patch. */
                math::Vec2f vp0(NAN), vp1(NAN);
                for (const Base::VertexProjectionInfo &info0 : vpi0) {
                    for (const Base::VertexProjectionInfo &info1 : vpi1) {
                        if (info0.texture_patch_id == info1.texture_patch_id) {
                            vp0 = info0.projection;
                            vp1 = info1.projection;
                            break;
                        }
                    }
                }
                assert(!std::isnan(vp0[0]) && !std::isnan(vp0[1]));
                assert(!std::isnan(vp1[0]) && !std::isnan(vp1[1]));

                total_projection_length += (vp0 - vp1).norm();
                math::Vec3f const &v0 = vertices[vi0];
                math::Vec3f const &v1 = vertices[vi1];
                total_length += (v0 - v1).norm();
            }
            float radius = total_projection_length / (2.0f * MATH_PI);

            if (total_length < std::numeric_limits<float>::epsilon()) return false;

            std::vector<math::Vec2f> projections(num_vertices);
            {
                float length = 0.0f;
                for (std::size_t j = 0; j < border.size(); ++j) {
                    float angle = 2.0f * MATH_PI * (length / total_length);
                    projections[g2l[border[j]]] = math::Vec2f(std::cos(angle), std::sin(angle));
                    math::Vec3f const &v0 = vertices[border[j]];
                    math::Vec3f const &v1 = vertices[border[(j + 1) % border.size()]];
                    length += (v0 - v1).norm();
                }
            }

            typedef Eigen::Triplet<float, int> Triplet;
            std::vector<Triplet> coeff;
            std::size_t matrix_size = num_vertices - border.size();

            Eigen::VectorXf xx(matrix_size), xy(matrix_size);

            if (matrix_size != 0) {
                Eigen::VectorXf bx(matrix_size);
                Eigen::VectorXf by(matrix_size);
                for (std::size_t j = 0; j < num_vertices; ++j) {
                    if (is_border[j]) continue;

                    std::size_t const vertex_id = l2g[j];

                    /* Calculate "Mean Value Coordinates" as proposed by Michael S. Floater */
                    std::map<std::size_t, float> weights;

                    std::vector<std::size_t> const &adj_faces = mesh_info[vertex_id].faces;
                    for (std::size_t adj_face : adj_faces) {
                        std::size_t v0 = mesh_faces[adj_face * 3 + 0];
                        std::size_t v1 = mesh_faces[adj_face * 3 + 1];
                        std::size_t v2 = mesh_faces[adj_face * 3 + 2];
                        if (v1 == vertex_id) std::swap(v1, v0);
                        if (v2 == vertex_id) std::swap(v2, v0);

                        math::Vec3f v01 = vertices[v1] - vertices[v0];
                        float v01n = v01.norm();
                        math::Vec3f v02 = vertices[v2] - vertices[v0];
                        float v02n = v02.norm();

                        /* Ensure numerical stability */
                        if (v01n * v02n < std::numeric_limits<float>::epsilon()) return false;

                        float calpha = v01.dot(v02) / (v01n * v02n);
                        float alpha = std::acos(clamp(calpha, -1.0f, 1.0f));
                        weights[g2l[v1]] += std::tan(alpha / 2.0f) / (v01n / 2.0f);
                        weights[g2l[v2]] += std::tan(alpha / 2.0f) / (v02n / 2.0f);
                    }

                    std::map<std::size_t, float>::iterator it;
                    float sum = 0.0f;
                    for (it = weights.begin(); it != weights.end(); ++it)
                        sum += it->second;
                    if (sum < std::numeric_limits<float>::epsilon()) return false;
                    for (it = weights.begin(); it != weights.end(); ++it)
                        it->second /= sum;

                    bx[idx[j]] = 0.0f;
                    by[idx[j]] = 0.0f;
                    for (it = weights.begin(); it != weights.end(); ++it) {
                        if (is_border[it->first]) {
                            math::Vec2f projection = projections[it->first];
                            bx[idx[j]] += projection[0] * it->second;
                            by[idx[j]] += projection[1] * it->second;
                        } else {
                            coeff.push_back(Triplet(idx[j], idx[it->first], -it->second));
                        }
                    }
                }

                for (std::size_t j = 0; j < matrix_size; ++j) {
                    coeff.push_back(Triplet(j, j, 1.0f));
                }

                typedef Eigen::SparseMatrix<float> SpMat;
                SpMat A(matrix_size, matrix_size);
                A.setFromTriplets(coeff.begin(), coeff.end());

                Eigen::SparseLU<SpMat> solver;
                solver.analyzePattern(A);
                solver.factorize(A);
                xx = solver.solve(bx);
                xy = solver.solve(by);
            }

            float const max_hole_patch_size = MAX_HOLE_PATCH_SIZE;
            int image_size = std::min(std::floor(radius * 1.1f) * 2.0f, max_hole_patch_size);
            /* Ensure a minimum scale of one */
            image_size += 2 * (1 + Base::texture_patch_border);
            int scale = image_size / 2 - Base::texture_patch_border;
            for (std::size_t j = 0, k = 0; j < num_vertices; ++j) {
                if (is_border[j]) {
                    projections[j] = projections[j] * scale + image_size / 2;
                } else {
                    projections[j] = math::Vec2f(xx[k], xy[k]) * scale + image_size / 2;
                    ++k;
                }
            }

            std::vector<math::Vec2f> texcoords;
            texcoords.reserve(hole.size());
            for (std::size_t const face_id : hole) {
                for (std::size_t j = 0; j < 3; ++j) {
                    std::size_t const vertex_id = mesh_faces[face_id * 3 + j];
                    math::Vec2f const &projection = projections[g2l[vertex_id]];
                    texcoords.push_back(projection);
                }
            }
            mve::FloatImage::Ptr image = mve::FloatImage::create(image_size, image_size, 3);
            //DEBUG image->fill_color(*math::Vec4uc(0, 255, 0, 255));
            Base::TexturePatch::Ptr texture_patch = Base::TexturePatch::create(0, hole, texcoords, image);
            std::size_t texture_patch_id;
#pragma omp critical
            {
                texture_patches->push_back(texture_patch);
                texture_patch_id = texture_patches->size() - 1;
            }

            for (std::size_t j = 0; j < num_vertices; ++j) {
                std::size_t const vertex_id = l2g[j];
                std::vector<std::size_t> const &adj_faces = mesh_info[vertex_id].faces;
                std::vector<std::size_t> faces;
                faces.reserve(adj_faces.size());
                for (std::size_t adj_face : adj_faces) {
                    if (graph.get_label(adj_face) == 0) {
                        faces.push_back(adj_face);
                    }
                }
                Base::VertexProjectionInfo info = {texture_patch_id, projections[j], faces};
#pragma omp critical (vpis)
                vertex_projection_infos->at(vertex_id).push_back(info);
            }

            return true;
        }

        void
        merge_vertex_projection_infos(VertexProjectionInfoList *vertex_projection_infos) {
            /* Merge vertex infos within the same texture patch. */
            // vertex_projection_infos 里每个 vertex 连接的 faces 每个可能都有不同的 texture patch id
            //  现在按照 texture patch id 将不同 faces 聚合起来
#pragma omp parallel for
            for (std::size_t i = 0; i < vertex_projection_infos->size(); ++i) {
                std::vector<Base::VertexProjectionInfo> &infos = vertex_projection_infos->at(i);

                std::map<std::size_t, Base::VertexProjectionInfo> info_map;
                std::map<std::size_t, Base::VertexProjectionInfo>::iterator it;

                for (const Base::VertexProjectionInfo &info : infos) {
                    std::size_t texture_patch_id = info.texture_patch_id;
                    if ((it = info_map.find(texture_patch_id)) == info_map.end()) {
                        info_map[texture_patch_id] = info;
                    } else {
                        it->second.faces.insert(it->second.faces.end(),
                                                info.faces.begin(), info.faces.end());
                    }
                }

                infos.clear();
                infos.reserve(info_map.size());
                for (it = info_map.begin(); it != info_map.end(); ++it) {
                    infos.push_back(it->second);
                }
            }
        }

        /** Create a TexturePatchCandidate by calculating the faces' bounding box
        * projected into the view,
        *  relative texture coordinates and extacting the texture views relevant part
        */
        TexturePatchCandidate
        generate_candidate(int label, const Base::TextureView &texture_view,
                           const std::vector<std::size_t> &faces, MeshConstPtr mesh,
                           const Parameter &param) {

            mve::ByteImage::Ptr view_image = texture_view.get_image();
            int min_x = view_image->width(), min_y = view_image->height();
            int max_x = 0, max_y = 0;

            mve::TriangleMesh::FaceList const &mesh_faces = mesh->get_faces();
            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();

            std::vector<math::Vec2f> texcoords;
            for (std::size_t i = 0; i < faces.size(); ++i) {
                for (std::size_t j = 0; j < 3; ++j) {
                    math::Vec3f vertex = vertices[mesh_faces[faces[i] * 3 + j]];
                    math::Vec2f pixel = texture_view.get_pixel_coords(vertex);

                    texcoords.push_back(pixel);

                    min_x = std::min(static_cast<int>(std::floor(pixel[0])), min_x);
                    min_y = std::min(static_cast<int>(std::floor(pixel[1])), min_y);
                    max_x = std::max(static_cast<int>(std::ceil(pixel[0])), max_x);
                    max_y = std::max(static_cast<int>(std::ceil(pixel[1])), max_y);
                }
            }

            /* Check for valid projections/erroneous labeling files. */
            assert(min_x >= 0);
            assert(min_y >= 0);
            assert(max_x < view_image->width());
            assert(max_y < view_image->height());

            int width = max_x - min_x + 1;
            int height = max_y - min_y + 1;

            /* Add border and adjust min accordingly. */
            width += 2 * Base::texture_patch_border;
            height += 2 * Base::texture_patch_border;
            min_x -= Base::texture_patch_border;
            min_y -= Base::texture_patch_border;

            /* Calculate the relative texcoords. */
            math::Vec2f min(min_x, min_y);
            for (std::size_t i = 0; i < texcoords.size(); ++i) {
                texcoords[i] = texcoords[i] - min;
            }

            mve::ByteImage::Ptr byte_image;
            byte_image = mve::image::crop(view_image, width, height, min_x, min_y, *math::Vec3uc(255, 0, 255));
            mve::FloatImage::Ptr image = mve::image::byte_to_float_image(byte_image);

            if (param.tone_mapping == Tone_Mapping_Gamma) {
                // 涉及到颜色矫正？
                mve::image::gamma_correct(image, 2.2f);
            }

            TexturePatchCandidate texture_patch_candidate =
                    {Math::Rect2D<int>(min_x, min_y, max_x, max_y),
                     Base::TexturePatch::create(label, faces, texcoords, image)};
            return texture_patch_candidate;
        }

        void generate_texture_patches(const Base::LabelGraph &graph, MeshConstPtr mesh,
                                      const MeshInfo &mesh_info, TextureViewList *texture_views,
                                      const Parameter &param,
                                      VertexProjectionInfoList *vertex_projection_infos,
                                      TexturePatchList *texture_patches) {

            util::WallTimer timer;

            mve::TriangleMesh::FaceList const &mesh_faces = mesh->get_faces();
            mve::TriangleMesh::VertexList const &vertices = mesh->get_vertices();
            vertex_projection_infos->resize(vertices.size());

            std::size_t num_patches = 0;

            std::cout << "\tRunning... " << std::flush;
#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_views->size(); ++i) {

                std::vector<std::vector<std::size_t> > subgraphs;
                int const label = i + 1;
                // 获取具有相同 label 并且相互联通的子区域
                graph.get_subgraphs(label, &subgraphs);

                Base::TextureView *texture_view = &texture_views->at(i);
                texture_view->load_image();
                std::list<TexturePatchCandidate> candidates;
                for (std::size_t j = 0; j < subgraphs.size(); ++j) {
                    // generate_candidate：
                    //  生产 patch 区域，包括
                    //  （label,
                    //  faces,
                    //  texcoords(相对于 rect 左下角的 offset),
                    //  在image 上的 rect 范围）
                    candidates.push_back(generate_candidate(label, *texture_view, subgraphs[j], mesh, param));
                }
                texture_view->release_image();

                /* Merge candidates which contain the same image content. */
                // 候选 texture patch 如果和其他的 texture patch 完全被覆盖，则消除这个 patch
                std::list<TexturePatchCandidate>::iterator it, sit;
                for (it = candidates.begin(); it != candidates.end(); ++it) {
                    for (sit = candidates.begin(); sit != candidates.end();) {
                        Math::Rect2D<int> bounding_box = sit->bounding_box;
                        if (it != sit && bounding_box.is_inside(&it->bounding_box)) {
                            Base::TexturePatch::Faces &faces = it->texture_patch->get_faces();
                            Base::TexturePatch::Faces &ofaces = sit->texture_patch->get_faces();
                            faces.insert(faces.end(), ofaces.begin(), ofaces.end());

                            Base::TexturePatch::Texcoords &texcoords = it->texture_patch->get_texcoords();
                            Base::TexturePatch::Texcoords &otexcoords = sit->texture_patch->get_texcoords();
                            math::Vec2f offset;
                            offset[0] = sit->bounding_box.min_x - it->bounding_box.min_x;
                            offset[1] = sit->bounding_box.min_y - it->bounding_box.min_y;
                            for (std::size_t i = 0; i < otexcoords.size(); ++i) {
                                // 重新修改 texture patch 上的 UV 坐标
                                texcoords.push_back(otexcoords[i] + offset);
                            }

                            sit = candidates.erase(sit);
                        } else {
                            ++sit;
                        }
                    }
                }

                // 1. 合并案冗余的候选区域后，生成最后的 texture patch
                // 2. 对每一个 vertex 都生产对应的 (texture patch id, projection uv, face_id) 组合
                it = candidates.begin();
                for (; it != candidates.end(); ++it) {
                    std::size_t texture_patch_id;

#pragma omp critical
                    {
                        texture_patches->push_back(it->texture_patch);
                        texture_patch_id = num_patches++;
                    }

                    std::vector<std::size_t> const &faces = it->texture_patch->get_faces();
                    std::vector<math::Vec2f> const &texcoords = it->texture_patch->get_texcoords();
                    for (std::size_t i = 0; i < faces.size(); ++i) {
                        std::size_t const face_id = faces[i];
                        std::size_t const face_pos = face_id * 3;
                        for (std::size_t j = 0; j < 3; ++j) {
                            std::size_t const vertex_id = mesh_faces[face_pos + j];
                            math::Vec2f const projection = texcoords[i * 3 + j];

                            // vertex
                            // ：属于哪个 texture patch 的（texture_patch_id）
                            // ：投影坐标是多少
                            // ：属于哪个 face（face_id）
                            Base::VertexProjectionInfo info = {texture_patch_id, projection, {face_id}};

                            // 考虑到不同的 face 会共用同一个 vertex，这个信息要保存下来
                            // TODO 考虑到融合失败的情况，很有可能是看起来共面的 face 没有共用同一个定点
#pragma omp critical
                            vertex_projection_infos->at(vertex_id).push_back(info);
                        }
                    }
                }
            }

            merge_vertex_projection_infos(vertex_projection_infos);

            // 补洞？有一些 faces 是看不见的，这里用来补洞
            // TODO 后续可能要在这里做一些操作
            {
                std::vector<std::size_t> unseen_faces;
                std::vector<std::vector<std::size_t> > subgraphs;
                graph.get_subgraphs(0, &subgraphs);

#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < subgraphs.size(); ++i) {
                    std::vector<std::size_t> const &subgraph = subgraphs[i];

                    bool success = false;
                    if (!param.skip_hole_filling) {
                        success = fill_hole(subgraph, graph, mesh, mesh_info,
                                            vertex_projection_infos, texture_patches);
                    }

                    if (success) {
                        num_patches += 1;
                    } else {
                        if (param.keep_unseen_faces) {
#pragma omp critical
                            unseen_faces.insert(unseen_faces.end(),
                                                subgraph.begin(), subgraph.end());
                        }
                    }
                }

                if (!unseen_faces.empty()) {
                    mve::FloatImage::Ptr image = mve::FloatImage::create(3, 3, 3);
                    std::vector<math::Vec2f> texcoords;
                    for (std::size_t i = 0; i < unseen_faces.size(); ++i) {
                        math::Vec2f projections[] = {{2.0f, 1.0f},
                                                     {1.0f, 1.0f},
                                                     {1.0f, 2.0f}};
                        texcoords.insert(texcoords.end(), &projections[0], &projections[3]);
                    }
                    Base::TexturePatch::Ptr texture_patch
                            = Base::TexturePatch::create(0, unseen_faces, texcoords, image);
                    texture_patches->push_back(texture_patch);
                    std::size_t texture_patch_id = texture_patches->size() - 1;

                    for (std::size_t i = 0; i < unseen_faces.size(); ++i) {
                        std::size_t const face_id = unseen_faces[i];
                        std::size_t const face_pos = face_id * 3;
                        for (std::size_t j = 0; j < 3; ++j) {
                            std::size_t const vertex_id = mesh_faces[face_pos + j];
                            math::Vec2f const projection = texcoords[i * 3 + j];

                            Base::VertexProjectionInfo info = {texture_patch_id, projection, {face_id}};

                            vertex_projection_infos->at(vertex_id).push_back(info);
                        }
                    }
                }
            }

            // 再次合并
            merge_vertex_projection_infos(vertex_projection_infos);

            std::cout << "done. (Took " << timer.get_elapsed_sec() << "s)" << std::endl;
            std::cout << "\t" << num_patches << " texture patches." << std::endl;
        }

        /**
  * Heuristic to calculate an appropriate texture atlas size.
  * @warning asserts that no texture patch exceeds the dimensions
  * of the maximal possible texture atlas size.
  */
        unsigned int calculate_texture_size(const std::list<Base::TexturePatch::ConstPtr> &texture_patches) {
            unsigned int size = MAX_TEXTURE_SIZE;

            while (true) {
                unsigned int total_area = 0;
                unsigned int max_width = 0;
                unsigned int max_height = 0;
                unsigned int padding = size >> 7;

                for (Base::TexturePatch::ConstPtr texture_patch : texture_patches) {
                    unsigned int width = texture_patch->get_width() + 2 * padding;
                    unsigned int height = texture_patch->get_height() + 2 * padding;

                    max_width = std::max(max_width, width);
                    max_height = std::max(max_height, height);

                    unsigned int area = width * height;
                    unsigned int waste = area - texture_patch->get_size();

                    /* Only consider patches where the information dominates padding. */
                    if (static_cast<double>(waste) / texture_patch->get_size() > 1.0) {
                        /* Since the patches are sorted by size we can assume that only
                         * few further patches will contribute to the size and break. */
                        break;
                    }

                    total_area += area;
                }

                assert(max_width < MAX_TEXTURE_SIZE);
                assert(max_height < MAX_TEXTURE_SIZE);
                if (size > PREF_TEXTURE_SIZE &&
                    max_width < PREF_TEXTURE_SIZE &&
                    max_height < PREF_TEXTURE_SIZE &&
                    total_area / (PREF_TEXTURE_SIZE * PREF_TEXTURE_SIZE) < 8) {
                    size = PREF_TEXTURE_SIZE;
                    continue;
                }

                if (size <= MIN_TEXTURE_SIZE) {
                    return MIN_TEXTURE_SIZE;
                }

                if (max_height < size / 2 && max_width < size / 2 &&
                    static_cast<double>(total_area) / (size * size) < 0.2) {
                    size = size / 2;
                    continue;
                }

                return size;
            }
        }

        bool comp(Base::TexturePatch::ConstPtr first, Base::TexturePatch::ConstPtr second) {
            return first->get_size() > second->get_size();
        }

        void generate_texture_atlases(TexturePatchList *orig_texture_patches,
                                      TextureAtlasList *texture_atlases,
                                      bool tone_mapping_gamma) {
            std::list<Base::TexturePatch::ConstPtr> texture_patches;
            while (!orig_texture_patches->empty()) {
                Base::TexturePatch::Ptr texture_patch = orig_texture_patches->back();
                orig_texture_patches->pop_back();

                if (tone_mapping_gamma) {
                    mve::image::gamma_correct(texture_patch->get_image(), 1.0f / 2.2f);
                }

                texture_patches.push_back(texture_patch);
            }

            std::cout << "\tSorting texture patches... " << std::flush;
            /* Improve the bin-packing algorithm efficiency by sorting texture patches
             * in descending order of size. */
            texture_patches.sort(comp);
            std::cout << "done." << std::endl;

            std::size_t const total_num_patches = texture_patches.size();
            std::size_t remaining_patches = texture_patches.size();
            std::ofstream tty("/dev/tty", std::ios_base::out);

#pragma omp parallel
            {
#pragma omp single
                {
                    while (!texture_patches.empty()) {
                        unsigned int texture_size = calculate_texture_size(texture_patches);

                        texture_atlases->push_back(Base::TextureAtlas::create(texture_size));
                        Base::TextureAtlas::Ptr texture_atlas = texture_atlases->back();

                        /* Try to insert each of the texture patches into the texture atlas. */
                        std::list<Base::TexturePatch::ConstPtr>::iterator it = texture_patches.begin();
                        for (; it != texture_patches.end();) {
                            std::size_t done_patches = total_num_patches - remaining_patches;
                            int precent = static_cast<float>(done_patches)
                                          / total_num_patches * 100.0f;
                            if (total_num_patches > 100
                                && done_patches % (total_num_patches / 100) == 0) {

                                tty << "\r\tWorking on atlas " << texture_atlases->size() << " "
                                    << precent << "%... " << std::flush;
                            }

                            if (texture_atlas->insert(*it)) {
                                it = texture_patches.erase(it);
                                remaining_patches -= 1;
                            } else {
                                ++it;
                            }
                        }

#pragma omp task
                        texture_atlas->finalize();
                    }

                    std::cout << "\r\tWorking on atlas " << texture_atlases->size()
                              << " 100%... done." << std::endl;
                    util::WallTimer timer;
                    std::cout << "\tFinalizing texture atlases... " << std::flush;
#pragma omp taskwait
                    std::cout << "done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;

                    /* End of single region */
                }
                /* End of parallel region. */
            }
        }

    }
}