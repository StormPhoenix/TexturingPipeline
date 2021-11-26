//
// Created by Storm Phoenix on 2021/11/17.
//

#include <set>
#include <vector>

#include <Base/TexturePatch.h>
#include <Eigen/Core>

#include <igl/parallel_for.h>
#include <igl/fit_plane.h>

#include "MeshSimplification.h"

namespace MeshSimplification {
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

    }

    /**
     * // spare model
     * @param sparse_mesh_V
     * @param sparse_mesh_F
     *
     * // dense model
     * @param dense_mesh_V
     * @param dense_mesh_F
     * @param dense_mesh_T [0, 1]
     * @param dense_mesh_Ft
     * @param dense_mesh_F_material
     * @param dense_mesh_materials
     *
     * // mappings between sparse and dense model
     * @param faces_subdivision
     *
     * // generated spare model texture patches
     * @param texture_patches
     * @return
     */
    bool simplify_mesh_texture(const Mesh &sparse_mesh, const Mesh &dense_mesh,
                               const std::vector<ImagePtr> &dense_mesh_materials,
                               const FacesSubdivisions &faces_subdivision,
                               const FaceGroups &sparse_mesh_face_groups,
                               TexturePatchArray *sparse_texture_patches,
                               std::size_t padding_pixels,
                               const std::size_t plane_density) {
        padding_pixels = std::max(padding_pixels, std::size_t(2));

        if (dense_mesh.m_faces.rows() != dense_mesh.m_face_texture_coord_ids.rows()) {
            // dense mesh data error
            return false;
        }

        if (dense_mesh.m_faces.rows() != dense_mesh.m_face_material_ids.rows()) {
            // dense mesh data error
            return false;
        }

        if (sparse_texture_patches == nullptr) {
            // dense mesh error
            return false;
        }

        float gauss_mat[9] = {1.0f, 2.0f, 1.0f, 2.0f, 4.0f, 2.0f, 1.0f, 2.0f, 1.0f};
        std::for_each(gauss_mat, gauss_mat + 9, [](float &v) { v /= 16.0f; });

        for (int g_idx = 0; g_idx < sparse_mesh_face_groups.size(); g_idx++) {
            const FaceGroup &face_group = sparse_mesh_face_groups[g_idx];

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
                AttributeMatrix sparse_points3 = sparse_mesh.m_vertices(sparse_mesh.m_faces.row(sparse_face_idx),
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
                        AttributeMatrix dense_points3 = dense_mesh.m_vertices(dense_mesh.m_faces.row(sub_face_idx),
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
            Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
            random_color[0] = double(color(0));
            random_color[1] = double(color(1));
            random_color[2] = double(color(2));

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

            if (dense_mesh_materials.size() > 0) {
                // using material ...
                // copy src images ...
                mve::ByteImage::Ptr patch_mask = mve::ByteImage::create(image_width, image_height, 1);
                patch_mask->fill(0);

                for (std::size_t i = 0; i < dense_mesh_texture_coords.size(); i += 3) {
                    // TODO delete
                    // 代码不好写，之前 texture coords 数组的顺序和 group.m_indices 是有绑定关系的
                    // 而 dense model 的纹理和 dense_texture_coord 的绑定关系消失。
                    std::size_t sub_f_i = i / 3;
                    std::size_t sub_f_idx = dense_mesh_face_ids[sub_f_i];

                    if (sub_f_idx >= dense_mesh_materials.size()) {
                        continue;
                    }

                    // TODO delete do copy operation
                    mve::ByteImage::ConstPtr src_dense_img = dense_mesh_materials[sub_f_idx];
                    const int src_width = src_dense_img->width();
                    const int src_height = src_dense_img->height();

                    math::Vec2f src_uv1;
                    src_uv1[0] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 0), 0)
                                 * Scalar(src_width);
                    src_uv1[1] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 0), 1)
                                 * Scalar(src_height);

                    math::Vec2f src_uv2;
                    src_uv2[0] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 1), 0)
                                 * Scalar(src_width);
                    src_uv2[1] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 1), 1)
                                 * Scalar(src_height);

                    math::Vec2f src_uv3;
                    src_uv3[0] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 2), 0)
                                 * Scalar(src_width);
                    src_uv3[1] = dense_mesh.m_texture_coords(dense_mesh.m_face_texture_coord_ids(sub_f_idx, 2), 1)
                                 * Scalar(src_height);

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

                                    for (int kernel_offset_j = -1; kernel_offset_j <= 1; kernel_offset_j++) {
                                        for (int kernel_offset_i = -1; kernel_offset_i <= 1; kernel_offset_i++) {
                                            const int gx = int(src_coord[0]) + kernel_offset_i;
                                            const int gy = int(src_coord[1]) + kernel_offset_j;

                                            if (gx >= 0 && gx < src_dense_img->width() &&
                                                gy >= 0 && gy < src_dense_img->height()) {
                                                float weight = gauss_mat[(kernel_offset_j + 1) * 3 +
                                                                         (kernel_offset_i + 1)];
                                                normalize += weight;
                                                src_color += (src_dense_img->at(gx, gy, c)) * weight;
                                            }
                                        }
                                    }
                                    src_color = (src_color / normalize);
                                    patch_image->at(x, y, c) = std::min(1.0f, std::max(0.0f,
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
                    MvsTexturing::Base::TexturePatch::create(0, face_group.m_face_indices,
                                                             sparse_mesh_texture_coords, patch_image);
            sparse_texture_patches->push_back(patch);
        }

#pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < sparse_texture_patches->size(); ++i) {
            MvsTexturing::Base::TexturePatch::Ptr texture_patch = sparse_texture_patches->operator[](i);
            std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
            texture_patch->adjust_colors(patch_adjust_values);
        }

        return true;
    }

    bool simplify_mesh_texture(const AttributeMatrix &sparse_mesh_vertices,
                               const IndexMatrix &sparse_mesh_faces,
                               const std::vector<FaceGroup> &sparse_planar_groups,

                               const AttributeMatrix &dense_mesh_vertices,
                               const IndexMatrix &dense_mesh_faces,
                               const std::vector<math::Vec2f> &dense_mesh_face_texture_coords,
                               const std::vector<FloatImageConstPtr> &dense_mesh_face_materials,

                               const FacesSubdivisions &faces_subdivision,
                               std::vector<TexturePatch::Ptr> *ret_sparse_mesh_texture_patches,
                               std::size_t padding_pixels,
                               std::size_t plane_density) {
        padding_pixels = std::max(padding_pixels, std::size_t(2));

        if ((dense_mesh_faces.rows() * 3) != dense_mesh_face_texture_coords.size()) {
            // dense mesh data error
            return false;
        }

        if (dense_mesh_faces.rows() != dense_mesh_face_materials.size()) {
            // dense mesh data error
            return false;
        }

        if (ret_sparse_mesh_texture_patches == nullptr) {
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
            Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
            random_color[0] = double(color(0));
            random_color[1] = double(color(1));
            random_color[2] = double(color(2));

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
                    // TODO delete
                    // 代码不好写，之前 texture coords 数组的顺序和 group.m_indices 是有绑定关系的
                    // 而 dense model 的纹理和 dense_texture_coord 的绑定关系消失。
                    std::size_t sub_f_i = i / 3;
                    std::size_t sub_f_idx = dense_mesh_face_ids[sub_f_i];

                    if (sub_f_idx >= dense_mesh_face_materials.size()) {
                        continue;
                    }

                    // TODO delete do copy operation
//                    mve::ByteImage::ConstPtr src_dense_img = dense_mesh_face_materials[sub_f_idx];
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

                                    for (int kernel_offset_j = -1; kernel_offset_j <= 1; kernel_offset_j++) {
                                        for (int kernel_offset_i = -1; kernel_offset_i <= 1; kernel_offset_i++) {
                                            const int gx = int(src_coord[0]) + kernel_offset_i;
                                            const int gy = int(src_coord[1]) + kernel_offset_j;

                                            if (gx >= 0 && gx < src_dense_img->width() &&
                                                gy >= 0 && gy < src_dense_img->height()) {
                                                float weight = gauss_mat[(kernel_offset_j + 1) * 3 +
                                                                         (kernel_offset_i + 1)];
                                                normalize += weight;
                                                src_color += (src_dense_img->at(gx, gy, c)) * 255 * weight;
                                            }
                                        }
                                    }
                                    src_color = (src_color / normalize);
                                    patch_image->at(x, y, c) = std::min(1.0f, std::max(0.0f,
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
                    MvsTexturing::Base::TexturePatch::create(0, face_group.m_face_indices, sparse_mesh_texture_coords,
                                                             patch_image);
            ret_sparse_mesh_texture_patches->push_back(patch);
        }

#pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < ret_sparse_mesh_texture_patches->size(); ++i) {
            MvsTexturing::Base::TexturePatch::Ptr texture_patch = ret_sparse_mesh_texture_patches->operator[](i);
            std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
            texture_patch->adjust_colors(patch_adjust_values);
        }

        return true;
    }

    bool fit_face_group_plane(const AttributeMatrix &vertices, const IndexMatrix &faces,
                              FaceGroup &group) {
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