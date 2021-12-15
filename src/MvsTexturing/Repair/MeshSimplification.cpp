//
// Created by Storm Phoenix on 2021/11/17.
//

#include <set>
#include <vector>
#include <queue>

#include <Eigen/Core>
#include <Parameter.h>
#include <Base/TexturePatch.h>

#include <igl/fit_plane.h>
#include <igl/parallel_for.h>
#include <igl/triangle_triangle_adjacency.h>
#include <mve/image_tools.h>
#include <common.h>

#include "Mapper/AtlasMapper.h"
#include "MeshSimplification.h"

namespace MvsTexturing {
    namespace MeshRepair {
        namespace __inner__ {
            struct Face {
                std::size_t v[3];

                explicit Face() {
                    v[0] = v[1] = v[2] = 0;
                }

                explicit Face(std::size_t v0, std::size_t v1, std::size_t v2) {
                    if (v0 == v1 || v0 == v2 || v1 == v2) {
                        throw std::runtime_error("face vertex repeated. ");
                    }

                    v[0] = v0;
                    v[1] = v1;
                    v[2] = v2;

                    if (v[0] > v[1]) {
                        std::swap(v[0], v[1]);
                    }

                    if (v[0] > v[2]) {
                        std::swap(v[0], v[2]);
                    }

                    if (v[1] > v[2]) {
                        std::swap(v[1], v[2]);
                    }
                }

                void operator=(const Face &f) {
                    v[0] = f.v[0];
                    v[1] = f.v[1];
                    v[2] = f.v[2];
                }

                bool operator<(const struct Face &other) const {
                    if (v[0] < other.v[0]) {
                        return true;
                    } else if (v[0] == other.v[0]) {
                        if (v[1] < other.v[1]) {
                            return true;
                        } else if (v[1] == other.v[1]) {
                            return v[2] < other.v[2];
                        } else {
                            return false;
                        }
                    } else {
                        return false;
                    }
                }
            };

            enum SplitType {
                NOT_SPLIT,
                VERTIAL_SPLIT,
                HORIZONTAL_SPLIT
            };
        }

        using TexturePatch = MvsTexturing::Base::TexturePatch;

        __inner__::SplitType need_split(TexturePatch::Ptr patch) {
            using namespace __inner__;
            if (patch == nullptr) {
                return NOT_SPLIT;
            }

            int max_size = std::max(patch->get_width(), patch->get_height());
            if (max_size <= AtlasMapper::kMaxTexturePatchSize) {
                return NOT_SPLIT;
            }

            if (patch->get_width() > patch->get_height()) {
                return HORIZONTAL_SPLIT;
            } else {
                return VERTIAL_SPLIT;
            }
        }

        void split_bigger_patch(TexturePatch::Ptr bigger_patch, std::vector<TexturePatch::Ptr> &result) {
            using namespace __inner__;

            std::vector<TexturePatch::Ptr> patches, tmp_patches;
            patches.push_back(bigger_patch);

            while ((!patches.empty()) || (!tmp_patches.empty())) {
                for (int i = patches.size() - 1; i >= 0; i--) {
                    TexturePatch::Ptr patch = patches[i];
                    patches.pop_back();

                    bool split_result = false;
                    SplitType split_type = need_split(patch);
                    if (split_type == VERTIAL_SPLIT) {
                        split_result = patch->split_vertical(tmp_patches);
                    } else if (split_type == HORIZONTAL_SPLIT) {
                        split_result = patch->split_horizontal(tmp_patches);
                    }

                    if (split_type == NOT_SPLIT) {
                        result.push_back(patch);
                        continue;
                    } else if (!split_result) {
                        // big texture patch but little triangles
                        if (patch->get_width() < AtlasMapper::kMaxTextureMapSize &&
                            patch->get_height() < AtlasMapper::kMaxTextureMapSize) {
                            result.push_back(patch);
                            continue;
                        } else {
                            float scale_factor = float(AtlasMapper::kMaxTextureMapSize - 10) /
                                                 (std::max(patch->get_width(), patch->get_height()));

                            patch->rescale(scale_factor);
                            result.push_back(patch);
                            continue;
                        }
                    }
                }
                patches.swap(tmp_patches);
            }
        }

        bool create_plane_patches_on_sparse_mesh(
                const Parameter &param, const AttributeMatrix &sparse_mesh_vertices,
                const IndexMatrix &sparse_mesh_faces, const std::vector<FaceGroup> &sparse_planar_groups,

                const AttributeMatrix &dense_mesh_vertices, const IndexMatrix &dense_mesh_faces,
                const std::vector<math::Vec2f> &dense_mesh_face_texture_coords,
                const std::vector<FloatImageConstPtr> &dense_mesh_face_materials,

                const FacesSubdivisions &faces_subdivision, std::vector<TexturePatch::Ptr> *final_texture_patches,
                std::size_t padding_pixels, std::size_t plane_density) {
            padding_pixels = std::max(padding_pixels, std::size_t(2));

            if ((dense_mesh_faces.rows() * 3) != dense_mesh_face_texture_coords.size()) {
                // dense mesh data error
                return false;
            }

            if (dense_mesh_faces.rows() != dense_mesh_face_materials.size()) {
                // dense mesh data error
                return false;
            }

            if (final_texture_patches == nullptr) {
                // dense mesh error
                return false;
            }

            float gauss_mat[9] = {1.0f, 2.0f, 1.0f, 2.0f, 4.0f, 2.0f, 1.0f, 2.0f, 1.0f};
            std::for_each(gauss_mat, gauss_mat + 9, [](float &v) { v /= 16.0f; });

            for (int g_idx = 0; g_idx < sparse_planar_groups.size(); g_idx++) {
                const FaceGroup &face_group = sparse_planar_groups[g_idx];

                // compute 3D uv
                double max_dx, min_dx = 0;
                double max_dy, min_dy = 0;
                bool d_init = false;

                // init sparse texture coords
                const std::size_t n_group_faces = face_group.m_face_indices.size();
                std::vector<math::Vec2f> sparse_mesh_texture_coords;
                sparse_mesh_texture_coords.resize(n_group_faces * 3);

                // init dense texture coords
                std::vector<math::Vec2f> dense_mesh_texture_coords;
                std::vector<std::size_t> dense_mesh_face_ids;

                for (int f_i = 0; f_i < n_group_faces; f_i++) {
                    const std::size_t sparse_face_idx = face_group.m_face_indices[f_i];
                    AttributeMatrix sparse_points3 = sparse_mesh_vertices(sparse_mesh_faces.row(sparse_face_idx),
                                                                          Eigen::all);

                    Vec3 sparse_d0 = sparse_points3.row(0) - face_group.m_plane_center;
                    Vec3 sparse_d1 = sparse_points3.row(1) - face_group.m_plane_center;
                    Vec3 sparse_d2 = sparse_points3.row(2) - face_group.m_plane_center;

                    Scalar sparse_dx0 = sparse_d0.dot(face_group.m_x_axis);
                    Scalar sparse_dx1 = sparse_d1.dot(face_group.m_x_axis);
                    Scalar sparse_dx2 = sparse_d2.dot(face_group.m_x_axis);

                    Scalar sparse_dy0 = sparse_d0.dot(face_group.m_y_axis);
                    Scalar sparse_dy1 = sparse_d1.dot(face_group.m_y_axis);
                    Scalar sparse_dy2 = sparse_d2.dot(face_group.m_y_axis);

                    if (!d_init) {
                        d_init = true;
                        max_dx = std::max(sparse_dx0, std::max(sparse_dx1, sparse_dx2));
                        min_dx = std::min(sparse_dx0, std::min(sparse_dx1, sparse_dx2));

                        max_dy = std::max(sparse_dy0, std::max(sparse_dy1, sparse_dy2));
                        min_dy = std::min(sparse_dy0, std::min(sparse_dy1, sparse_dy2));
                    } else {
                        max_dx = std::max(max_dx, std::max(sparse_dx0, std::max(sparse_dx1, sparse_dx2)));
                        min_dx = std::min(min_dx, std::min(sparse_dx0, std::min(sparse_dx1, sparse_dx2)));

                        max_dy = std::max(max_dy, std::max(sparse_dy0, std::max(sparse_dy1, sparse_dy2)));
                        min_dy = std::min(min_dy, std::min(sparse_dy0, std::min(sparse_dy1, sparse_dy2)));
                    }

                    sparse_mesh_texture_coords[f_i * 3 + 0] = {sparse_dx0, sparse_dy0};
                    sparse_mesh_texture_coords[f_i * 3 + 1] = {sparse_dx1, sparse_dy1};
                    sparse_mesh_texture_coords[f_i * 3 + 2] = {sparse_dx2, sparse_dy2};

                    {
                        // TODO calculate dense model face texture coords
                        const std::vector<std::size_t> &subdivision_faces = faces_subdivision[sparse_face_idx];
                        for (auto it = subdivision_faces.begin(); it != subdivision_faces.end(); it++) {
                            std::size_t sub_face_idx = (*it);
                            AttributeMatrix dense_points3 = dense_mesh_vertices(dense_mesh_faces.row(sub_face_idx),
                                                                                Eigen::all);

                            Vec3 dense_d0 = dense_points3.row(0) - face_group.m_plane_center;
                            Vec3 dense_d1 = dense_points3.row(1) - face_group.m_plane_center;
                            Vec3 dense_d2 = dense_points3.row(2) - face_group.m_plane_center;

                            Scalar dense_dx0 = dense_d0.dot(face_group.m_x_axis);
                            Scalar dense_dx1 = dense_d1.dot(face_group.m_x_axis);
                            Scalar dense_dx2 = dense_d2.dot(face_group.m_x_axis);

                            Scalar dense_dy0 = dense_d0.dot(face_group.m_y_axis);
                            Scalar dense_dy1 = dense_d1.dot(face_group.m_y_axis);
                            Scalar dense_dy2 = dense_d2.dot(face_group.m_y_axis);

                            dense_mesh_texture_coords.push_back({dense_dx0, dense_dy0});
                            dense_mesh_texture_coords.push_back({dense_dx1, dense_dy1});
                            dense_mesh_texture_coords.push_back({dense_dx2, dense_dy2});

                            dense_mesh_face_ids.push_back(sub_face_idx);
                        }
                    }
                }

                Scalar d_width = max_dx - min_dx;
                Scalar d_height = max_dy - min_dy;

                // Create images
                const std::size_t image_width = d_width * plane_density + 2 * padding_pixels;
                const std::size_t image_height = d_height * plane_density + 2 * padding_pixels;
                mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

                float random_color[3];
                if (param.debug_mode) {
                    random_color[0] = double(1.0);
                    random_color[1] = double(0.0);
                    random_color[2] = double(0.0);
                } else {
                    random_color[0] = double(0);
                    random_color[1] = double(0);
                    random_color[2] = double(0);
                }

                patch_image->fill_color(random_color);

                // Re-scale texture coords
                double padding = double(padding_pixels) / plane_density;
#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < sparse_mesh_texture_coords.size(); i++) {
                    sparse_mesh_texture_coords[i][0] =
                            (sparse_mesh_texture_coords[i][0] - min_dx + padding) * plane_density;
                    sparse_mesh_texture_coords[i][1] =
                            (sparse_mesh_texture_coords[i][1] - min_dy + padding) * plane_density;
                }

#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < dense_mesh_texture_coords.size(); i++) {
                    dense_mesh_texture_coords[i][0] =
                            (dense_mesh_texture_coords[i][0] - min_dx + padding) * plane_density;
                    dense_mesh_texture_coords[i][1] =
                            (dense_mesh_texture_coords[i][1] - min_dy + padding) * plane_density;
                }

                if (dense_mesh_face_materials.size() > 0) {
                    // using material ...
                    // copy src images ...
                    mve::ByteImage::Ptr patch_mask = mve::ByteImage::create(image_width, image_height, 1);
                    patch_mask->fill(0);

                    for (std::size_t i = 0; i < dense_mesh_texture_coords.size(); i += 3) {
                        std::size_t sub_f_i = i / 3;
                        std::size_t sub_f_idx = dense_mesh_face_ids[sub_f_i];

                        if (sub_f_idx >= dense_mesh_face_materials.size()) {
                            continue;
                        }

                        FloatImageConstPtr src_dense_img = dense_mesh_face_materials[sub_f_idx];
                        const int src_width = src_dense_img->width();
                        const int src_height = src_dense_img->height();

                        math::Vec2f src_uv1 = dense_mesh_face_texture_coords[sub_f_idx * 3 + 0];
                        math::Vec2f src_uv2 = dense_mesh_face_texture_coords[sub_f_idx * 3 + 1];
                        math::Vec2f src_uv3 = dense_mesh_face_texture_coords[sub_f_idx * 3 + 2];

                        math::Vec2f dest_uv1 = dense_mesh_texture_coords[i];
                        math::Vec2f dest_uv2 = dense_mesh_texture_coords[i + 1];
                        math::Vec2f dest_uv3 = dense_mesh_texture_coords[i + 2];
                        using namespace MvsTexturing;
                        Math::Tri2D tri(dest_uv1, dest_uv2, dest_uv3);
                        const float area = tri.get_area();
                        if (area < std::numeric_limits<float>::epsilon()) {
                            continue;
                        }

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
                                            src_uv1[0] * bcoords[0] + src_uv2[0] * bcoords[1] + src_uv3[0] * bcoords[2],
                                            src_uv1[1] * bcoords[0] + src_uv2[1] * bcoords[1] + src_uv3[1] * bcoords[2]
                                    };

                                    for (int c = 0; c < 3; c++) {
                                        float normalize = 0.f;
                                        float src_color = 0.f;

                                        src_color = src_dense_img->at(int(src_coord[0]), int(src_coord[1]), c);
                                        patch_image->at(x, y, c) = std::min(1.0f, std::max(0.0f, src_color));
                                    }
                                    patch_mask->at(x, y, 0) = 255;
                                } else {
                                    continue;
                                }
                            }
                        }
                    }

                    {
                        // filter patch image
                        mve::FloatImage::Ptr filter_image =
                                mve::FloatImage::create(patch_image->width(), patch_image->height(), 3);
                        filter_image->fill(0);

#pragma omp parallel for schedule(dynamic)
                        for (int x = 0; x < patch_mask->width(); x++) {
                            for (int y = 0; y < patch_mask->height(); y++) {
                                if (patch_mask->at(x, y, 0) == 0) {
                                    continue;
                                }

                                for (int c = 0; c < 3; c++) {
                                    float normalize = 0.f;
                                    float filter_color = 0.f;

                                    for (int kernel_offset_j = -1; kernel_offset_j <= 1; kernel_offset_j++) {
                                        for (int kernel_offset_i = -1; kernel_offset_i <= 1; kernel_offset_i++) {
                                            const int nx = x + kernel_offset_i;
                                            const int ny = y + kernel_offset_j;

                                            if (nx >= 0 && nx < patch_mask->width() &&
                                                ny >= 0 && ny < patch_mask->height() &&
                                                patch_mask->at(nx, ny, 0) == 255) {
                                                float weight = gauss_mat[(kernel_offset_j + 1) * 3 +
                                                                         (kernel_offset_i + 1)];
                                                normalize += weight;
                                                filter_color += (patch_image->at(nx, ny, c)) * 255 * weight;
                                            }
                                        }
                                    }

                                    filter_color = filter_color / normalize;
                                    filter_image->at(x, y, c) =
                                            std::min(1.0f, std::max(0.0f, ((float) filter_color) / 255.0f));
                                }
                            }
                        }

#pragma omp parallel for schedule(dynamic)
                        for (int x = 0; x < patch_image->width(); x++) {
                            for (int y = 0; y < patch_image->height(); y++) {
                                if (patch_mask->at(x, y, 0) == 0) {
                                    continue;
                                }

                                for (int c = 0; c < 3; c++) {
                                    patch_image->at(x, y, c) = filter_image->at(x, y, c);
                                }
                            }
                        }
                        filter_image.reset();
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
                        MvsTexturing::Base::TexturePatch::create(0, face_group.m_face_indices,
                                                                 sparse_mesh_texture_coords,
                                                                 patch_image);

                final_texture_patches->push_back(patch);
            }

            LOG_INFO(" - begin split bigger patches ... ");
            {
                // split bigger patches
                std::vector<TexturePatch::Ptr> tmp_patches;
                for (int i = final_texture_patches->size() - 1; i >= 0; i--) {
                    TexturePatch::Ptr patch = (*final_texture_patches)[i];
                    final_texture_patches->pop_back();
                    split_bigger_patch(patch, tmp_patches);
                }

                LOG_DEBUG(" - add all split patches ... ");
                for (int i = tmp_patches.size() - 1; i >= 0; i--) {
                    TexturePatch::Ptr patch = tmp_patches[i];
                    final_texture_patches->push_back(patch);
                }
            }

#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < final_texture_patches->size(); ++i) {
                MvsTexturing::Base::TexturePatch::Ptr texture_patch = final_texture_patches->operator[](i);
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }

            return true;
        }

        bool has_common_texture_coordinates(const std::size_t face_index, const std::size_t adj_face_index,
                                            const std::vector<math::Vec2f> &texture_coords) {
            int n_common = 0;
            for (int c = 0; c < 3; c++) {
                double texture_coord[2] = {texture_coords[face_index * 3 + c][0],
                                           texture_coords[face_index * 3 + c][1]};

                for (int adj_c = 0; adj_c < 3; adj_c++) {
                    double adj_texture_coord[2] = {texture_coords[adj_face_index * 3 + adj_c][0],
                                                   texture_coords[adj_face_index * 3 + adj_c][1]};

                    {
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
                                          const std::vector<FloatImageConstPtr> &face_materials) {
            return face_materials[face_index] == face_materials[adj_face_index];
        }

        bool is_connected_in_texture(const std::size_t face_index, const std::size_t adj_face_index,
                                     const std::vector<FloatImageConstPtr> &face_materials,
                                     const std::vector<math::Vec2f> &texture_coords) {
            return has_common_texture_coordinates(face_index, adj_face_index, texture_coords) &&
                   has_common_texture_materials(face_index, adj_face_index, face_materials);
        }

        void divide_faces_by_texture(const IndexMatrix &sparse_ff_adjacency, std::set<std::size_t> &sparse_patch,
                                     const std::vector<FloatImageConstPtr> &dense_face_materials,
                                     const std::vector<math::Vec2f> &dense_texture_coords,
                                     const std::vector<std::vector<std::size_t>> &face_subdivisions,
                                     std::vector<std::vector<std::size_t>> &result) {
            // classification in texture level
            // divide faces by material and texture coordinates
            while (!sparse_patch.empty()) {
                std::set<std::size_t> visited;
                std::queue<std::size_t> q;

                q.push((*sparse_patch.begin()));
                visited.insert((*sparse_patch.begin()));

                result.push_back(std::vector<std::size_t>());
                while (!q.empty()) {
                    std::size_t f_index = q.front();
                    std::size_t dense_f_index = face_subdivisions[f_index][0];
                    result.back().push_back(f_index);
                    q.pop();

                    for (int c = 0; c < 3; c++) {
                        int sparse_adj_f_index = sparse_ff_adjacency(f_index, c);
//                    int adj_f_index =

                        if (sparse_adj_f_index < 0) {
                            continue;
                        }

                        if (sparse_patch.find(sparse_adj_f_index) == sparse_patch.end()) {
                            continue;
                        }

                        if (visited.find(sparse_adj_f_index) != visited.end()) {
                            continue;
                        }

                        int dense_adj_f_index = face_subdivisions[sparse_adj_f_index][0];
                        if (!is_connected_in_texture(dense_f_index, dense_adj_f_index, dense_face_materials,
                                                     dense_texture_coords)) {
                            continue;
                        }

                        q.push(sparse_adj_f_index);
                        visited.insert(sparse_adj_f_index);
                    }
                }

                for (auto f_index : result.back()) {
                    sparse_patch.erase(f_index);
                }
            }
        }

        bool create_irregular_patches_on_sparse_mesh(
                const IndexMatrix &spares_ff_adjacency,
                std::vector<std::set<std::size_t>> &sparse_irregular_patches,

                const std::vector<math::Vec2f> &dense_texture_coords,
                const std::vector<FloatImageConstPtr> &dense_face_materials,

                const std::vector<std::vector<std::size_t>> &face_subdivisions,
                std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                const std::size_t kPaddingPixels, const std::size_t kPlaneDensity) {

            if (texture_patches == nullptr) {
                LOG_ERROR(" - create_irregular_patches_on_sparse_mesh() : variable texture_patches is null");
                return false;
            }
            const int kExistPatches = texture_patches->size();
            for (std::set<std::size_t> &sparse_patch : sparse_irregular_patches) {
                std::vector<std::vector<std::size_t>> same_material_patch;
                divide_faces_by_texture(spares_ff_adjacency, sparse_patch, dense_face_materials,
                                        dense_texture_coords, face_subdivisions, same_material_patch);

                for (const auto &sparse_patch_faces : same_material_patch) {
                    FloatImageConstPtr material = dense_face_materials[sparse_patch_faces[0]];
                    const int material_width = material->width();
                    const int material_height = material->height();

                    int min_x = material_width, min_y = material_height;
                    int max_x = 0, max_y = 0;

                    std::vector<math::Vec2f> sparse_patch_texture_coords;
                    for (std::size_t sparse_face_index : sparse_patch_faces) {
                        std::size_t dense_face_index = face_subdivisions[sparse_face_index][0];

                        math::Vec2f src_v1 = dense_texture_coords[dense_face_index * 3 + 0];
                        math::Vec2f src_v2 = dense_texture_coords[dense_face_index * 3 + 1];
                        math::Vec2f src_v3 = dense_texture_coords[dense_face_index * 3 + 2];

                        {
                            min_x = std::min(min_x, static_cast<int>(src_v1[0]));
                            min_y = std::min(min_y, static_cast<int>(src_v1[1]));
                            max_x = std::max(max_x, static_cast<int>(src_v1[0]));
                            max_y = std::max(max_y, static_cast<int>(src_v1[1]));
                            sparse_patch_texture_coords.push_back(src_v1);

                            min_x = std::min(min_x, static_cast<int>(src_v2[0]));
                            min_y = std::min(min_y, static_cast<int>(src_v2[1]));
                            max_x = std::max(max_x, static_cast<int>(src_v2[0]));
                            max_y = std::max(max_y, static_cast<int>(src_v2[1]));
                            sparse_patch_texture_coords.push_back(src_v2);

                            min_x = std::min(min_x, static_cast<int>(src_v3[0]));
                            min_y = std::min(min_y, static_cast<int>(src_v3[1]));
                            max_x = std::max(max_x, static_cast<int>(src_v3[0]));
                            max_y = std::max(max_y, static_cast<int>(src_v3[1]));
                            sparse_patch_texture_coords.push_back(src_v3);
                        }

                    }

                    if (min_x < 0 || min_y < 0 || max_x >= material_width ||
                        max_y >= material_height) {
                        LOG_ERROR(
                                " - create_irregular_patches_on_sparse_mesh() error, texture coords out of range, min_x: {}, min_y: {}, max_x: {}, max_y: {}, width: {}, height: {} ",
                                min_x, min_y, max_x, max_y, material_width, material_height);
                        return false;
                    }

                    const int patch_width = max_x - min_x + 1 + 2 * kPaddingPixels;
                    const int patch_height = max_y - min_y + 1 + 2 * kPaddingPixels;
                    min_x -= kPaddingPixels;
                    min_y -= kPaddingPixels;
                    mve::FloatImage::Ptr patch_image = mve::image::crop(material, patch_width, patch_height,
                                                                        min_x, min_y, *math::Vec3f(1.0, 0, 1.0));
                    math::Vec2f min(min_x, min_y);
                    for (std::size_t i = 0; i < sparse_patch_texture_coords.size(); ++i) {
                        sparse_patch_texture_coords[i] = sparse_patch_texture_coords[i] - min;
                    }

                    using namespace MvsTexturing;
                    texture_patches->push_back(
                            Base::TexturePatch::create(0, sparse_patch_faces, sparse_patch_texture_coords,
                                                       patch_image));
                }
            }

#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = kExistPatches; i < texture_patches->size(); ++i) {
                MvsTexturing::Base::TexturePatch::Ptr texture_patch = texture_patches->operator[](i);
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }

            return true;
        }

        void find_connected_face_group(const IndexMatrix &ff_adjacency, const std::set<std::size_t> &faces_set,
                                       const int face_index, std::set<std::size_t> &face_group) {
            /**
            * first level classification
            * divide faces by off-plane criteria
            */

            if (face_index == -1 ||
                (faces_set.find(std::size_t(face_index)) == faces_set.end())) {
                return;
            }

            std::queue<std::size_t> q;
            std::set<std::size_t> visited;
            q.push(face_index);
            while (!q.empty()) {
                int next_f_index = q.front();
                q.pop();

                face_group.insert(std::size_t(next_f_index));
                // find adjacent face index
                for (int i = 0; i < 3; i++) {
                    int adj_f_index = ff_adjacency(next_f_index, i);
                    if (adj_f_index == -1) {
                        continue;
                    }

                    if (visited.find(adj_f_index) != visited.end()) {
                        continue;
                    }

                    if (faces_set.find(adj_f_index) == faces_set.end()) {
                        continue;
                    }

                    q.push(adj_f_index);
                    visited.insert(adj_f_index);
                }
            }
        };

        bool create_irregular_patches_on_sparse_mesh(
                const AttributeMatrix &sparse_mesh_vertices,
                const IndexMatrix &sparse_mesh_faces,
                std::set<std::size_t> &irregular_patch_faces,
                const std::vector<math::Vec2f> &dense_texture_coords,
                const std::vector<FloatImageConstPtr> &dense_face_materials,
                const FacesSubdivisions &faces_subdivision,
                std::vector<TexturePatch::Ptr> *final_texture_patches,
                const std::size_t kPaddingPixels, const std::size_t kPlaneDensity) {

            IndexMatrix ff_adjacency;
            igl::triangle_triangle_adjacency(sparse_mesh_faces, ff_adjacency);

            std::vector<std::set<std::size_t>> irregular_patches;
            while (!irregular_patch_faces.empty()) {
                std::size_t face_index = (*irregular_patch_faces.begin());
                irregular_patches.push_back(std::set<std::size_t>());

                // find connected faces
                find_connected_face_group(ff_adjacency, irregular_patch_faces,
                                          face_index, irregular_patches.back());
                for (auto f_index : irregular_patches.back()) {
                    irregular_patch_faces.erase(f_index);
                }
            }

            return create_irregular_patches_on_sparse_mesh(ff_adjacency, irregular_patches, dense_texture_coords,
                                                           dense_face_materials, faces_subdivision,
                                                           final_texture_patches,
                                                           kPaddingPixels, kPlaneDensity);
        }

        bool fit_face_group_plane(const AttributeMatrix &vertices, const IndexMatrix &faces, FaceGroup &group) {
            if (group.m_face_indices.size() == 0) {
                return false;
            }

            AttributeMatrix points(group.m_face_indices.size() * 3, 3);
            for (std::size_t i = 0; i < group.m_face_indices.size(); i++) {
                std::size_t face_index = group.m_face_indices[i];
                points.block<3, 3>(i * 3, 0) = vertices(faces.row(face_index), Eigen::all);
            }

            Vec3 N, C;
            igl::fit_plane(points, N, C);

            group.m_plane_center = C;
            group.m_plane_normal = N;

            Scalar d = -group.m_plane_normal.dot(group.m_plane_center);
            Vec3 p = -d * group.m_plane_normal;
            if ((p - group.m_plane_center).norm() < 1e-5) {
                //p is too close to origin
                p.setZero();
                if (group.m_plane_normal[0] != 0) {
                    p[0] = -d / group.m_plane_normal[0];
                } else if (group.m_plane_normal[1] != 0) {
                    p[1] = -d / group.m_plane_normal[1];
                } else {
                    p[2] = -d / group.m_plane_normal[2];
                }
            }
            group.m_x_axis = (p - group.m_plane_center).normalized();
            group.m_y_axis = (group.m_plane_normal.cross(group.m_x_axis)).normalized();


            return true;
        }

        bool remove_duplicate_faces(IndexMatrix &faces) {
            AttributeMatrix face_colors;
            return remove_duplicate_faces(faces, face_colors);
        }

        bool remove_duplicate_faces(IndexMatrix &faces, AttributeMatrix &face_colors) {
            if (faces.cols() != 3 || faces.rows() == 0) {
                return false;
            }

            bool process_face_colors = true;
            if (face_colors.rows() != faces.rows()) {
                process_face_colors = false;
            }
            std::size_t color_channels = face_colors.cols();

            std::set<__inner__::Face> unique_face_set;
            std::vector<std::size_t> tmp_faces;
            std::vector<double> tmp_face_colors;

            for (std::size_t r = 0; r < faces.rows(); r++) {
                const __inner__::Face f = __inner__::Face(faces(r, 0), faces(r, 1), faces(r, 2));
                if (unique_face_set.find(f) == unique_face_set.end()) {
                    unique_face_set.insert(f);
                    for (int c = 0; c < 3; c++) {
                        tmp_faces.push_back(faces(r, c));
                    }

                    if (process_face_colors) {
                        for (int c = 0; c < color_channels; c++) {
                            tmp_face_colors.push_back(face_colors(r, c));
                        }
                    }
                }
            }

            faces.resize(tmp_faces.size() / 3, 3);
            if (process_face_colors) {
                face_colors.resize(tmp_face_colors.size() / color_channels, color_channels);
            }

            for (std::size_t r = 0; r < tmp_faces.size() / 3; r++) {
                for (int c = 0; c < 3; c++) {
                    faces(r, c) = tmp_faces[r * 3 + c];
                }

                if (process_face_colors) {
                    for (int c = 0; c < color_channels; c++) {
                        face_colors(r, c) = tmp_face_colors[r * color_channels + c];
                    }
                }
            }

            return true;
        }
    }
}