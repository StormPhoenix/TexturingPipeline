//
// Created by Storm Phoenix on 2021/10/22.
//

#include <memory>
#include <map>
#include <set>

#include <Base/TriMesh.h>
#include <Base/TexturePatch.h>
#include <Mapper/SceneBuilder.h>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <mve/image.h>
#include <mve/image_io.h>
#include <mve/image_tools.h>
#include <util/timer.h>

#include <sstream>

namespace PlaneTexMerge {
    namespace Utils {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        math::Vec3f find_vertex_coord(const MeshPolyRefinement::Base::TriMesh &mesh,
                                      const std::size_t f_idx,
                                      math::Vec3f bcoords) {
            AttributeMatrix p0 = mesh.m_vertices.row(mesh.m_faces(f_idx, 0));
            AttributeMatrix p1 = mesh.m_vertices.row(mesh.m_faces(f_idx, 1));
            AttributeMatrix p2 = mesh.m_vertices.row(mesh.m_faces(f_idx, 2));
            AttributeMatrix p = (p0 * bcoords[0] + p1 * bcoords[1] + p2 * bcoords[2]);

            math::Vec3f vertex;
            vertex[0] = p(0, 0);
            vertex[1] = p(0, 1);
            vertex[2] = p(0, 2);
            return vertex;
        }

        // TODO for debugger
        /*
         * 至少要过滤掉大量 camera
         */
        void Debug_projection_images(const std::size_t Group_Id,
                                     const MeshPolyRefinement::Base::TriMesh &mesh,
                                     std::vector<MvsTexturing::Base::TextureView> &camera_views,
                                     const std::vector<math::Vec2f> &tex_coords,
                                     const std::size_t image_width,
                                     const std::size_t image_height) {
            const MeshPolyRefinement::Base::PlaneGroup &group = mesh.m_plane_groups[Group_Id];

            for (std::size_t camera_id = 0; camera_id < camera_views.size(); camera_id++) {
                using namespace MvsTexturing;
                Base::TextureView &camera = camera_views[camera_id];
                camera.load_image();

                mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);
                patch_image->fill(0);

                for (std::size_t i = 0; i < tex_coords.size(); i += 3) {
                    std::size_t f_i = i / 3;
                    std::size_t f_idx = group.m_indices[f_i];

                    math::Vec2f dest_v1 = tex_coords[i];
                    math::Vec2f dest_v2 = tex_coords[i + 1];
                    math::Vec2f dest_v3 = tex_coords[i + 2];
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
                                math::Vec3f p = find_vertex_coord(mesh, f_idx, bcoords);
                                math::Vec2f pixel = camera.get_pixel_coords(p);

                                if (pixel[0] >= 0 && pixel[0] < camera.get_width() &&
                                    pixel[1] >= 0 && pixel[1] < camera.get_height()) {
                                    math::Vec3f src_color = camera.get_pixel_values(pixel);
                                    for (int c = 0; c < 3; c++) {
                                        patch_image->at(x, y, c) = src_color[c];
                                    }
                                }
                            } else {
                                continue;
                            }
                        }
                    }
                }

                {
                    // Save patch images
                    std::stringstream ss;
                    ss << "./Output_PlaneMerge/Debug_Group_" << Group_Id << "_Camera_" << camera_id << "_.png";

                    std::string filename;
                    ss >> filename;

                    mve::image::save_png_file(mve::image::float_to_byte_image(patch_image), filename);
                }
                patch_image.reset();
                camera.release_image();
            }
        }

        bool generate_plane_projection_patch(const MeshPolyRefinement::Base::TriMesh &mesh,
                                             std::vector<MvsTexturing::Base::TextureView> &camera_views,
                                             std::vector<MvsTexturing::Base::TexturePatch> *texture_patch,
                                             std::size_t Padding_Pixels = 10,
                                             const std::size_t Plane_Density = 300) {
            // TODO Debug
            const int Debug_Group = 9;

            Padding_Pixels = std::max(Padding_Pixels, std::size_t(4));
            for (std::size_t group_idx = 0; group_idx < mesh.m_plane_groups.size(); group_idx++) {
                using namespace MeshPolyRefinement;
                const Base::PlaneGroup &group = mesh.m_plane_groups[group_idx];

                // Compute 3D uv
                Base::Scalar max_dx, min_dx = 0;
                Base::Scalar max_dy, min_dy = 0;
                bool d_init = false;
                const std::size_t n_faces = group.m_indices.size();

                std::vector<math::Vec2f> tex_coords;
                tex_coords.resize(n_faces * 3);

                for (std::size_t f_i = 0; f_i < n_faces; f_i++) {
                    std::size_t face_idx = group.m_indices[f_i];
                    Base::AttributeMatrix points_3 = mesh.m_vertices(mesh.m_faces.row(face_idx), Eigen::all);

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

                    tex_coords[f_i * 3 + 0] = {dx0, dy0};
                    tex_coords[f_i * 3 + 1] = {dx1, dy1};
                    tex_coords[f_i * 3 + 2] = {dx2, dy2};
                }

                Base::Scalar d_width = max_dx - min_dx;
                Base::Scalar d_height = max_dy - min_dy;

                // Create images
                const std::size_t image_width = d_width * Plane_Density + 2 * Padding_Pixels;
                const std::size_t image_height = d_height * Plane_Density + 2 * Padding_Pixels;

                mve::FloatImage::Ptr patch_image = nullptr;
                {
                    patch_image = mve::FloatImage::create(image_width, image_height, 3);
                    float random_color[3];
                    Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
                    random_color[0] = double(color(0));
                    random_color[1] = double(color(1));
                    random_color[2] = double(color(2));
                    patch_image->fill_color(random_color);
                }

                {
                    // Re-scale texture coords
                    const double Padding = double(Padding_Pixels) / Plane_Density;
#pragma omp parallel for schedule(dynamic)
                    for (std::size_t i = 0; i < tex_coords.size(); i++) {
                        tex_coords[i][0] = (tex_coords[i][0] - min_dx + Padding) * Plane_Density;
                        tex_coords[i][1] = (tex_coords[i][1] - min_dy + Padding) * Plane_Density;
                    }
                }

                // TODO Debug
                {
                    if (group_idx == Debug_Group) {
                        util::WallTimer timer;
                        std::cout << "\tDebug generate plane patches...";
                        Debug_projection_images(group_idx, mesh, camera_views, tex_coords, image_width, image_height);
                        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
                    }
                }

                {
                    for (std::size_t i = 0; i < tex_coords.size(); i += 3) {
                        std::size_t f_i = i / 3;

                        using namespace MvsTexturing;
                        math::Vec2f dest_v1 = tex_coords[i];
                        math::Vec2f dest_v2 = tex_coords[i + 1];
                        math::Vec2f dest_v3 = tex_coords[i + 2];
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
                                    for (int c = 0; c < 3; c++) {
                                        float src_color = 0.f;
                                        patch_image->at(x, y, c) = src_color;
                                    }
                                } else {
                                    continue;
                                }
                            }
                        }
                    }
                }

                {
                    // Save patch images
                    std::stringstream ss;
                    ss << "./Output_PlaneMerge/Group_" << group_idx << "_.png";

                    std::string filename;
                    ss >> filename;

                    mve::image::save_png_file(mve::image::float_to_byte_image(patch_image), filename);
                }
            }

            return true;
        }
    }
}