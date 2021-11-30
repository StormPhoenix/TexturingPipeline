//
// Created by Storm Phoenix on 2021/10/22.
//

#include <memory>
#include <map>
#include <set>
#include <queue>

#include <Base/TriMesh.h>
#include <Base/TexturePatch.h>
#include <Mapper/SceneBuilder.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <mve/image.h>
#include <mve/image_tools.h>

#include <common.h>

namespace TextureRemeshing {
    namespace Utils {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
        using TriMesh = MeshPolyRefinement::Base::TriMesh;

        mve::TriangleMesh::Ptr triMesh_to_mveMesh(MeshPolyRefinement::Base::TriMesh &tri_mesh) {
            mve::TriangleMesh::Ptr ans = mve::TriangleMesh::create();

            mve::TriangleMesh::VertexList &vertices_list = ans->get_vertices();
            mve::TriangleMesh::FaceList &faces_list = ans->get_faces();

            Eigen::Index n_vertices = tri_mesh.m_vertices.rows();
            vertices_list.resize(n_vertices);
            for (int i = 0; i < n_vertices; i++) {
                math::Vec3f &v = vertices_list[i];
                v[0] = tri_mesh.m_vertices(i, 0);
                v[1] = tri_mesh.m_vertices(i, 1);
                v[2] = tri_mesh.m_vertices(i, 2);
            }


            Eigen::Index n_faces = tri_mesh.m_faces.rows();
            faces_list.resize(3 * n_faces);
            for (int i = 0; i < n_faces * 3; i += 3) {
                faces_list[i + 0] = tri_mesh.m_faces(i / 3, 0);
                faces_list[i + 1] = tri_mesh.m_faces(i / 3, 1);
                faces_list[i + 2] = tri_mesh.m_faces(i / 3, 2);
            }

            mve::MeshInfo mesh_info(ans);
            MvsTexturing::Builder::MVE::prepare_mesh(&mesh_info, ans);

            return ans;
        }

        bool create_plane_patches(const TriMesh &mesh, const AttributeMatrix &global_texcoords,
                                  const IndexMatrix &global_texcoord_ids,
                                  const std::vector<std::string> &face_materials,
                                  const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                  std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                  std::size_t padding_pixels = 10,
                                  const std::size_t plane_density = 300) {
            bool using_materials = (!face_materials.empty()) && (!material_image_map.empty());
            padding_pixels = std::max(padding_pixels, std::size_t(2));
            struct TexCoord {
                Scalar u, v;

                TexCoord() : u(0.), v(0.) {}

                TexCoord(Scalar u_, Scalar v_) : u(u_), v(v_) {}

                Scalar operator[](int index) const {
                    if (index == 0) {
                        return u;
                    } else if (index == 1) {
                        return v;
                    }
                    throw std::runtime_error("Texcoord index out of range. ");
                }

                Scalar &operator[](int index) {
                    if (index == 0) {
                        return u;
                    } else if (index == 1) {
                        return v;
                    }
                    throw std::runtime_error("Texcoord index out of range. ");
                }
            };
            float gauss_mat[9] = {1.0f, 2.0f, 1.0f, 2.0f, 4.0f, 2.0f, 1.0f, 2.0f, 1.0f};
            std::for_each(gauss_mat, gauss_mat + 9, [](float &v) { v /= 16.0f; });

            using namespace MeshPolyRefinement;
            for (std::size_t g_idx = 0; g_idx < mesh.m_plane_groups.size(); g_idx++) {
                const Base::PlaneGroup &group = mesh.m_plane_groups[g_idx];

                // Compute 3D uv
                Base::Scalar max_dx, min_dx = 0;
                Base::Scalar max_dy, min_dy = 0;
                bool d_init = false;
                const std::size_t n_faces = group.m_indices.size();
                std::vector<math::Vec2f> texcoords;
                texcoords.resize(n_faces * 3);

                for (std::size_t f_i = 0; f_i < n_faces; f_i++) {
                    std::size_t face_idx = group.m_indices[f_i];
                    Base::AttributeMatrix points_3 = mesh.m_vertices(mesh.m_faces.row(face_idx), Eigen::all);

//            std::cout << "Debug points 3: " << points_3 << std::endl;
                    Base::Vec3 d0 = points_3.row(0) - group.m_plane_center;
                    Base::Vec3 d1 = points_3.row(1) - group.m_plane_center;
                    Base::Vec3 d2 = points_3.row(2) - group.m_plane_center;

                    Base::Scalar dx0 = d0.dot(group.m_x_axis);

                    Base::Scalar dx1 = d1.dot(group.m_x_axis);
                    Base::Scalar dx2 = d2.dot(group.m_x_axis);

                    Base::Scalar dy0 = d0.dot(group.m_y_axis);
                    Base::Scalar dy1 = d1.dot(group.m_y_axis);
                    Base::Scalar dy2 = d2.dot(group.m_y_axis);

                    if (!d_init) {
                        d_init = true;
                        max_dx = std::max(dx0, std::max(dx1, dx2));
                        min_dx = std::min(dx0, std::min(dx1, dx2));

                        max_dy = std::max(dy0, std::max(dy1, dy2));
                        min_dy = std::min(dy0, std::min(dy1, dy2));
                    } else {
                        max_dx = std::max(max_dx, std::max(dx0, std::max(dx1, dx2)));
                        min_dx = std::min(min_dx, std::min(dx0, std::min(dx1, dx2)));

                        max_dy = std::max(max_dy, std::max(dy0, std::max(dy1, dy2)));
                        min_dy = std::min(min_dy, std::min(dy0, std::min(dy1, dy2)));
                    }

                    texcoords[f_i * 3 + 0] = {dx0, dy0};
                    texcoords[f_i * 3 + 1] = {dx1, dy1};
                    texcoords[f_i * 3 + 2] = {dx2, dy2};
                }

                Base::Scalar d_width = max_dx - min_dx;
                Base::Scalar d_height = max_dy - min_dy;

                // Create images
                const std::size_t image_width = d_width * plane_density + 2 * padding_pixels;
                const std::size_t image_height = d_height * plane_density + 2 * padding_pixels;
                mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

                float random_color[3];
                Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
                random_color[0] = double(color(0));
                random_color[1] = double(color(1));
                random_color[2] = double(color(2));

                patch_image->fill_color(random_color);

                // Re-scale texture coords
                double padding = double(padding_pixels) / plane_density;
#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < texcoords.size(); i++) {
                    texcoords[i][0] = (texcoords[i][0] - min_dx + padding) * plane_density;
                    texcoords[i][1] = (texcoords[i][1] - min_dy + padding) * plane_density;
                }

                if (using_materials) {
                    // Copy src images
                    mve::ByteImage::Ptr patch_mask = mve::ByteImage::create(image_width, image_height, 1);
                    patch_mask->fill(0);
                    for (std::size_t i = 0; i < texcoords.size(); i += 3) {
                        std::size_t f_i = i / 3;
                        std::size_t f_idx = group.m_indices[f_i];

                        if (f_idx >= face_materials.size() ||
                            material_image_map.find(face_materials[f_idx]) == material_image_map.end()) {
                            continue;
                        }

                        mve::ByteImage::ConstPtr src_image = material_image_map.find(face_materials[f_idx])->second;
                        const int src_width = src_image->width();
                        const int src_height = src_image->height();

                        TexCoord src_v1;
                        src_v1[0] = global_texcoords(global_texcoord_ids(f_idx, 0), 0) * Scalar(src_width);
                        src_v1[1] = global_texcoords(global_texcoord_ids(f_idx, 0), 1) * Scalar(src_height);

                        TexCoord src_v2;
                        src_v2[0] = global_texcoords(global_texcoord_ids(f_idx, 1), 0) * Scalar(src_width);
                        src_v2[1] = global_texcoords(global_texcoord_ids(f_idx, 1), 1) * Scalar(src_height);

                        TexCoord src_v3;
                        src_v3[0] = global_texcoords(global_texcoord_ids(f_idx, 2), 0) * Scalar(src_width);
                        src_v3[1] = global_texcoords(global_texcoord_ids(f_idx, 2), 1) * Scalar(src_height);

                        using namespace MvsTexturing;
                        math::Vec2f dest_v1 = texcoords[i];
                        math::Vec2f dest_v2 = texcoords[i + 1];
                        math::Vec2f dest_v3 = texcoords[i + 2];
                        Math::Tri2D tri(dest_v1, dest_v2, dest_v3);
                        float area = tri.get_area();
                        if (area < std::numeric_limits<float>::epsilon()) { continue; }

                        Math::Rect2D<float> aabb = tri.get_aabb();
                        int const min_tri_x = static_cast<int>(std::floor(aabb.min_x));
                        int const min_tri_y = static_cast<int>(std::floor(aabb.min_y));
                        int const max_tri_x = static_cast<int>(std::ceil(aabb.max_x));
                        int const max_tri_y = static_cast<int>(std::ceil(aabb.max_y));

#pragma omp parallel for schedule(dynamic)
                        for (std::size_t x = min_tri_x; x <= max_tri_x; x++) {
                            for (std::size_t y = min_tri_y; y <= max_tri_y; y++) {
                                math::Vec3f bcoords = tri.get_barycentric_coords(x, y);
                                if (bcoords.minimum() >= 0.0f) {
                                    math::Vec2f src_coord = {
                                            src_v1[0] * bcoords[0] + src_v2[0] * bcoords[1] + src_v3[0] * bcoords[2],
                                            src_v1[1] * bcoords[0] + src_v2[1] * bcoords[1] + src_v3[1] * bcoords[2]
                                    };
//                                    unsigned char src_color = src_image->at(src_coord[0], src_coord[1], c);
                                    for (int c = 0; c < 3; c++) {
                                        float normalize = 0.f;
                                        float src_color = 0.f;
                                        for (int kernel_offset_j = -1; kernel_offset_j <= 1; kernel_offset_j++) {
                                            for (int kernel_offset_i = -1; kernel_offset_i <= 1; kernel_offset_i++) {
                                                const int gx = int(src_coord[0]) + kernel_offset_i;
                                                const int gy = int(src_coord[1]) + kernel_offset_j;

                                                if (gx >= 0 && gx < src_image->width() &&
                                                    gy >= 0 && gy < src_image->height()) {
                                                    float weight = gauss_mat[(kernel_offset_j + 1) * 3 + (kernel_offset_i + 1)];
                                                    normalize += weight;
                                                    src_color += (src_image->at(gx, gy, c)) * weight;
                                                }
                                            }
                                        }
                                        src_color = (src_color / normalize);
                                        patch_image->at(x, y, c) = std::min(1.0f,
                                                                            std::max(0.0f,
                                                                                     ((float) src_color) / 255.0f));
                                    }
                                    patch_mask->at(x, y, 0) = 255;
                                } else {
                                    continue;
                                }
                            }
                        }
                    }

                    {
                        std::set<std::pair<int, int>> border_pixels;
                        // dialect texture patch
                        for (int epoch = 0; epoch < padding_pixels / 2; epoch++) {
                            for (int x = 0; x < patch_mask->width(); x++) {
                                for (int y = 0; y < patch_mask->height(); y++) {
                                    if (patch_mask->at(x, y, 0) == 255) {
                                        continue;
                                    }

                                    for (int j = -1; j <= 1; ++j) {
                                        for (int i = -1; i <= 1; ++i) {
                                            int nx = x + i;
                                            int ny = y + j;

                                            if (0 <= nx && nx < patch_mask->width() &&
                                                0 <= ny && ny < patch_mask->height() &&
                                                patch_mask->at(nx, ny, 0) == 255) {
                                                border_pixels.insert(std::pair<int, int>(x, y));
                                            }
                                        }
                                    }
                                }
                            }

                            for (auto it = border_pixels.begin(); it != border_pixels.end(); it++) {
                                const int x = it->first;
                                const int y = it->second;

                                for (int c = 0; c < 3; c++) {
                                    float normalize = 0.f;
                                    float value = 0.f;
                                    for (int j = -1; j <= 1; ++j) {
                                        for (int i = -1; i <= 1; ++i) {
                                            const int nx = x + i;
                                            const int ny = y + j;

                                            if (0 <= nx && nx < image_width &&
                                                0 <= ny && ny < image_height &&
                                                patch_mask->at(nx, ny, 0) == 255) {

                                                float weight = gauss_mat[(j + 1) * 3 + (i + 1)];
                                                normalize += weight;

                                                value += (patch_image->at(nx, ny, c) * 255.0f) * weight;
                                            }
                                        }
                                    }
                                    patch_image->at(x, y, c) = (value / normalize) / 255.0f;
                                }
                                patch_mask->at(x, y, 0) = 255;
                            }
                            border_pixels.clear();
                        }
                    }
                    patch_mask.reset();
                }

                // Create texture patch
                MvsTexturing::Base::TexturePatch::Ptr patch =
                        MvsTexturing::Base::TexturePatch::create(0, group.m_indices, texcoords, patch_image);
                texture_patches->push_back(patch);
            }

#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches->size(); ++i) {
                MvsTexturing::Base::TexturePatch::Ptr texture_patch = texture_patches->operator[](i);
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }

            return true;
        }

        bool create_plane_patches(const TriMesh &mesh, const AttributeMatrix &global_texcoords,
                                  const IndexMatrix &global_texcoord_ids,
                                  std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                  std::size_t padding_pixels = 10,
                                  const std::size_t plane_density = 300) {
            std::vector<std::string> _nop_vec;
            std::map<std::string, mve::ByteImage::Ptr> _nop_map;
            return create_plane_patches(mesh, global_texcoords, global_texcoord_ids, _nop_vec, _nop_map,
                                        texture_patches, padding_pixels, plane_density);
        }

        bool has_common_texture_coordinates(const std::size_t face_index, const std::size_t adj_face_index,
                                            const AttributeMatrix &texture_coords,
                                            const IndexMatrix &texture_coord_indices) {
            int n_common = 0;
            for (int c = 0; c < 3; c++) {
                int texture_coordinate_index = texture_coord_indices(face_index, c);
                for (int adj_c = 0; adj_c < 3; adj_c++) {
                    int adj_texture_coordinate_index = texture_coord_indices(adj_face_index, adj_c);

                    if (texture_coordinate_index == adj_texture_coordinate_index) {
                        n_common++;
                    } else {
                        double texture_coord[2] = {texture_coords(texture_coordinate_index, 0),
                                                   texture_coords(texture_coordinate_index, 1)};

                        double adj_texture_coord[2] = {texture_coords(adj_texture_coordinate_index, 0),
                                                       texture_coords(adj_texture_coordinate_index, 1)};

                        if ((texture_coord[0] == adj_texture_coord[0]) &&
                            (texture_coord[1] == adj_texture_coord[1])) {
                            n_common++;
                        }

                        LOG_TRACE("texture coordinate diff: ({0} - {1})",
                                  texture_coord[0] - adj_texture_coord[0], texture_coord[1] - adj_texture_coord[1]);
                    }
                }
            }

            return n_common > 0;
        }

        bool has_common_texture_materials(const std::size_t face_index, const std::size_t adj_face_index,
                                          const std::vector<std::string> &face_materials) {
            return face_materials[face_index] == face_materials[adj_face_index];
        }

        bool is_connected_in_texture(const std::size_t face_index, const std::size_t adj_face_index,
                                     const std::vector<std::string> &face_materials,
                                     const AttributeMatrix &texture_coords,
                                     const IndexMatrix &texture_coord_indices) {
            return has_common_texture_coordinates(face_index, adj_face_index, texture_coords, texture_coord_indices) &&
                   has_common_texture_materials(face_index, adj_face_index, face_materials);
        }

        void divide_faces_by_texture(const IndexMatrix &ff_adjacency, std::set<std::size_t> &face_group,
                                     const std::vector<std::string> &face_materials,
                                     const AttributeMatrix &texture_coords,
                                     const IndexMatrix &texture_coord_indices,
                                     std::vector<std::vector<std::size_t>> &result) {
            // classification in texture level
            // divide faces by material and texture coordinates
            while (!face_group.empty()) {
                std::set<std::size_t> visited;
                std::queue<std::size_t> q;

                q.push((*face_group.begin()));
                visited.insert((*face_group.begin()));

                result.push_back(std::vector<std::size_t>());
                while (!q.empty()) {
                    std::size_t f_index = q.front();
                    result.back().push_back(f_index);
                    q.pop();

                    for (int c = 0; c < 3; c++) {
                        int adj_f_index = ff_adjacency(f_index, c);

                        if (adj_f_index < 0) {
                            continue;
                        }

                        if (face_group.find(adj_f_index) == face_group.end()) {
                            continue;
                        }

                        if (visited.find(adj_f_index) != visited.end()) {
                            continue;
                        }

                        if (!is_connected_in_texture(f_index, adj_f_index, face_materials, texture_coords,
                                                     texture_coord_indices)) {
                            continue;
                        }

                        q.push(adj_f_index);
                        visited.insert(adj_f_index);
                    }
                }

                for (auto f_index : result.back()) {
                    face_group.erase(f_index);
                }
            }

            /*
            std::map<std::string, std::size_t> category_id;
            for (const std::size_t face_index : face_group) {
                if (face_index >= face_materials.size()) {
                    LOG_ERROR(" - divide_face_group_by_texture() error: face_index out of range");
                }

                std::string material_name = face_materials[face_index];
                if (category_id.find(material_name) != category_id.end()) {
                    result[category_id[material_name]].push_back(face_index);
                } else {
                    result.push_back(std::vector<std::size_t>());
                    category_id[material_name] = result.size() - 1;
                    result.back().push_back(face_index);
                }
            }
             */
        }

        bool create_irregular_patches(const MeshPolyRefinement::Base::TriMesh &mesh,
                                      const AttributeMatrix &global_texcoords,
                                      const IndexMatrix &global_texcoord_ids,
                                      std::vector<std::set<std::size_t>> &none_plane_group_faces,
                                      const std::vector<std::string> &face_materials,
                                      const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                      std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                      const std::size_t kPaddingPixels, const std::size_t kPlaneDensity) {
            for (auto &face_group : none_plane_group_faces) {
                std::vector<std::vector<std::size_t>> same_material_patch;
                divide_faces_by_texture(mesh.m_ff_adjacency, face_group, face_materials,
                                        global_texcoords, global_texcoord_ids, same_material_patch);

                for (const auto &patch_faces : same_material_patch) {
                    const std::string material_name = face_materials[patch_faces[0]];
                    mve::ByteImage::ConstPtr material = material_image_map.find(material_name)->second;
                    const int material_width = material->width();
                    const int material_height = material->height();

                    int min_x = material_width, min_y = material_height;
                    int max_x = 0, max_y = 0;

                    std::vector<math::Vec2f> patch_texture_coords;
                    for (std::size_t face_index : patch_faces) {
                        math::Vec2f src_v1;
                        src_v1[0] = global_texcoords(global_texcoord_ids(face_index, 0), 0) * double(material_width);
                        src_v1[1] = global_texcoords(global_texcoord_ids(face_index, 0), 1) * double(material_height);
                        min_x = std::min(min_x, static_cast<int>(src_v1[0]));
                        min_y = std::min(min_y, static_cast<int>(src_v1[1]));
                        max_x = std::max(max_x, static_cast<int>(src_v1[0]));
                        max_y = std::max(max_y, static_cast<int>(src_v1[1]));
                        patch_texture_coords.push_back(src_v1);

                        math::Vec2f src_v2;
                        src_v2[0] = global_texcoords(global_texcoord_ids(face_index, 1), 0) * double(material_width);
                        src_v2[1] = global_texcoords(global_texcoord_ids(face_index, 1), 1) * double(material_height);
                        min_x = std::min(min_x, static_cast<int>(src_v2[0]));
                        min_y = std::min(min_y, static_cast<int>(src_v2[1]));
                        max_x = std::max(max_x, static_cast<int>(src_v2[0]));
                        max_y = std::max(max_y, static_cast<int>(src_v2[1]));
                        patch_texture_coords.push_back(src_v2);

                        math::Vec2f src_v3;
                        src_v3[0] = global_texcoords(global_texcoord_ids(face_index, 2), 0) * double(material_width);
                        src_v3[1] = global_texcoords(global_texcoord_ids(face_index, 2), 1) * double(material_height);
                        min_x = std::min(min_x, static_cast<int>(src_v3[0]));
                        min_y = std::min(min_y, static_cast<int>(src_v3[1]));
                        max_x = std::max(max_x, static_cast<int>(src_v3[0]));
                        max_y = std::max(max_y, static_cast<int>(src_v3[1]));
                        patch_texture_coords.push_back(src_v3);
                    }

                    if (min_x < 0 || min_y < 0 || max_x >= material_width ||
                        max_y >= material_height) {
                        LOG_ERROR(" - create_irregular_patches() error: texture coords out of range.");
                        return false;
                    }

                    const int patch_width = max_x - min_x + 1 + 2 * kPaddingPixels;
                    const int patch_height = max_y - min_y + 1 + 2 * kPaddingPixels;
                    min_x -= kPaddingPixels;
                    min_y -= kPaddingPixels;
                    mve::ByteImage::Ptr byte_image = mve::image::crop(material, patch_width, patch_height,
                                                                      min_x, min_y, *math::Vec3uc(255, 0, 255));
                    mve::FloatImage::Ptr patch_image = mve::image::byte_to_float_image(byte_image);

                    math::Vec2f min(min_x, min_y);
                    for (std::size_t i = 0; i < patch_texture_coords.size(); ++i) {
                        patch_texture_coords[i] = patch_texture_coords[i] - min;
                    }

                    using namespace MvsTexturing;
                    texture_patches->push_back(
                            Base::TexturePatch::create(0, patch_faces, patch_texture_coords, patch_image));
                }
            }

            return true;
        }
    }
}