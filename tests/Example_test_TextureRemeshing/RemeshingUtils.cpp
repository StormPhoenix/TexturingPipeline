//
// Created by Storm Phoenix on 2021/10/22.
//

#include <memory>
#include <map>

#include <Base/TriMesh.h>
#include <Base/TexturePatch.h>
#include <TextureMapper/SceneBuilder.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>

namespace TextureRemeshing {
    namespace Utils {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

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

        bool remeshing_from_plane_groups(const MeshPolyRefinement::Base::TriMesh &mesh,
                                         const std::vector<math::Vec2f> &global_texcoords,
                                         const std::vector<std::size_t> &global_texcoord_ids,
                                         const std::vector<std::string> &face_materials,
                                         const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                         std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                         const std::size_t padding_pixels = 10,
                                         const std::size_t plane_density = 300) {
            using namespace MeshPolyRefinement;
            for (std::size_t g_idx = 0; g_idx < mesh.m_plane_groups.size(); g_idx++) {
                const Base::PlaneGroup &group = mesh.m_plane_groups[g_idx];

                // Compute 3D uv
                Base::Scalar max_dx, min_dx = 0;
                Base::Scalar max_dy, min_dy = 0;
                const std::size_t n_faces = group.m_indices.size();
                std::vector<math::Vec2f> texcoords;
                texcoords.resize(n_faces * 3);

//#pragma omp parallel for schedule(dynamic)
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

                    max_dx = std::max(max_dx, std::max(dx0, std::max(dx1, dx2)));
                    min_dx = std::min(min_dx, std::min(dx0, std::min(dx1, dx2)));

                    max_dy = std::max(max_dy, std::max(dy0, std::max(dy1, dy2)));
                    min_dy = std::min(min_dy, std::min(dy0, std::min(dy1, dy2)));

                    texcoords[f_i * 3 + 0] = {dx0, dy0};
                    texcoords[f_i * 3 + 1] = {dx1, dy1};
                    texcoords[f_i * 3 + 2] = {dx2, dy2};
                }

                Base::Scalar d_width = max_dx - min_dx;
                Base::Scalar d_height = max_dy - min_dy;

                // Create images
                std::size_t image_width = d_width * plane_density + 2 * padding_pixels;
                std::size_t image_height = d_height * plane_density + 2 * padding_pixels;
                mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

                // TODO delete
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
                    if ((texcoords[i][0] - min_dx + padding) < 0 || (texcoords[i][1] - min_dy + padding) < 0) {
                        std::cout << "Debug coord: \n\t" << texcoords[i][0] << ", " << texcoords[i][1] << std::endl;
                        std::cout << "\t" << min_dx << ", " << min_dy << std::endl;
                        std::cout << "\t" << texcoords[i][0] - min_dx << ", " << texcoords[i][1] - min_dy << std::endl;
                        std::cout << "\t" << texcoords[i][0] - min_dx + padding << ", "
                                  << texcoords[i][1] - min_dy + padding << std::endl;
                    }

                    texcoords[i][0] = (texcoords[i][0] - min_dx + padding) * plane_density;
                    texcoords[i][1] = (texcoords[i][1] - min_dy + padding) * plane_density;

                    if (texcoords[i][0] < 0 || texcoords[i][1] < 0) {
                        std::cout << "Debug: vt0 " << texcoords[i][0] << " vt1 " << texcoords[i][1] << std::endl;
                    }
                }

                // Copy src images
                for (std::size_t i = 0; i < texcoords.size(); i += 3) {
                    std::size_t f_i = i / 3;
                    std::size_t f_idx = group.m_indices[f_i];

                    mve::ByteImage::ConstPtr src_image = material_image_map.find(face_materials[f_idx])->second;
                    const int src_width = src_image->width();
                    const int src_height = src_image->height();

                    math::Vec2f src_v1 = global_texcoords[global_texcoord_ids[f_idx * 3 + 0]];
                    src_v1[0] *= src_width;
                    src_v1[1] *= src_height;

                    math::Vec2f src_v2 = global_texcoords[global_texcoord_ids[f_idx * 3 + 1]];
                    src_v2[0] *= src_width;
                    src_v2[1] *= src_height;

                    math::Vec2f src_v3 = global_texcoords[global_texcoord_ids[f_idx * 3 + 2]];
                    src_v3[0] *= src_width;
                    src_v3[1] *= src_height;

                    using namespace MvsTexturing;

                    math::Vec2f v1 = texcoords[i];
                    math::Vec2f v2 = texcoords[i + 1];
                    math::Vec2f v3 = texcoords[i + 2];
                    Math::Tri2D tri(v1, v2, v3);
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
                                for (int c = 0; c < 3; c++) {
                                    unsigned char src_color = src_image->at(src_coord[0], src_coord[1], c);
                                    patch_image->at(x, y, c) = std::min(1.0f,
                                                                        std::max(0.0f, ((float) src_color) / 255.0f));
                                }
                            } else {
                                continue;
                            }
                        }
                    }
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

        bool remeshing_from_plane_groups(const MeshPolyRefinement::Base::TriMesh &mesh,
                                         const AttributeMatrix &global_texcoords,
                                         const IndexMatrix &global_texcoord_ids,
                                         const std::vector<std::string> &face_materials,
                                         const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                         std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                         const std::size_t padding_pixels = 10,
                                         const std::size_t plane_density = 300) {
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

            using namespace MeshPolyRefinement;
            for (std::size_t g_idx = 0; g_idx < mesh.m_plane_groups.size(); g_idx++) {
                const Base::PlaneGroup &group = mesh.m_plane_groups[g_idx];

                // Compute 3D uv
                Base::Scalar max_dx, min_dx = 0;
                Base::Scalar max_dy, min_dy = 0;
                const std::size_t n_faces = group.m_indices.size();
                std::vector<math::Vec2f> texcoords;
                texcoords.resize(n_faces * 3);

//#pragma omp parallel for schedule(dynamic)
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

                    max_dx = std::max(max_dx, std::max(dx0, std::max(dx1, dx2)));
                    min_dx = std::min(min_dx, std::min(dx0, std::min(dx1, dx2)));

                    max_dy = std::max(max_dy, std::max(dy0, std::max(dy1, dy2)));
                    min_dy = std::min(min_dy, std::min(dy0, std::min(dy1, dy2)));

                    texcoords[f_i * 3 + 0] = {dx0, dy0};
                    texcoords[f_i * 3 + 1] = {dx1, dy1};
                    texcoords[f_i * 3 + 2] = {dx2, dy2};
                }

                Base::Scalar d_width = max_dx - min_dx;
                Base::Scalar d_height = max_dy - min_dy;

                // Create images
                std::size_t image_width = d_width * plane_density + 2 * padding_pixels;
                std::size_t image_height = d_height * plane_density + 2 * padding_pixels;
                mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

                // TODO delete
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
                    if ((texcoords[i][0] - min_dx + padding) < 0 || (texcoords[i][1] - min_dy + padding) < 0) {
                        std::cout << "Debug coord: \n\t" << texcoords[i][0] << ", " << texcoords[i][1] << std::endl;
                        std::cout << "\t" << min_dx << ", " << min_dy << std::endl;
                        std::cout << "\t" << texcoords[i][0] - min_dx << ", " << texcoords[i][1] - min_dy << std::endl;
                        std::cout << "\t" << texcoords[i][0] - min_dx + padding << ", "
                                  << texcoords[i][1] - min_dy + padding << std::endl;
                    }

                    texcoords[i][0] = (texcoords[i][0] - min_dx + padding) * plane_density;
                    texcoords[i][1] = (texcoords[i][1] - min_dy + padding) * plane_density;

                    if (texcoords[i][0] < 0 || texcoords[i][1] < 0) {
                        std::cout << "Debug: vt0 " << texcoords[i][0] << " vt1 " << texcoords[i][1] << std::endl;
                    }
                }

                // Copy src images
                for (std::size_t i = 0; i < texcoords.size(); i += 3) {
                    std::size_t f_i = i / 3;
                    std::size_t f_idx = group.m_indices[f_i];

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

                    math::Vec2f v1 = texcoords[i];
                    math::Vec2f v2 = texcoords[i + 1];
                    math::Vec2f v3 = texcoords[i + 2];
                    Math::Tri2D tri(v1, v2, v3);
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
                                for (int c = 0; c < 3; c++) {
                                    unsigned char src_color = src_image->at(src_coord[0], src_coord[1], c);
                                    patch_image->at(x, y, c) = std::min(1.0f,
                                                                        std::max(0.0f, ((float) src_color) / 255.0f));
                                }
                            } else {
                                continue;
                            }
                        }
                    }
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
    }
}