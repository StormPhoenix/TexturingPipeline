//
// Created by Storm Phoenix on 2021/10/11.
//

#include <set>
#include <vector>
#include <iostream>

#include <mve/mesh.h>
#include <mve/image_color.h>
#include <acc/bvh_tree.h>

#include "Base/View.h"
#include "Utils/Timer.h"
#include "Utils/Settings.h"
#include "Utils/ProgressCounter.h"
#include "PhotoConsistencyCheck.h"

namespace MvsTexturing {
    namespace ViewSelection {
        typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;
        typedef std::vector<std::vector<Base::FaceProjectionInfo>> FaceProjectionInfos;

        void calculate_face_projection_infos(mve::TriangleMesh::ConstPtr mesh, BVHTree &bvh_tree,
                                             std::vector<Base::TextureView> *texture_views,
                                             const MvsTexturing::Settings &settings,
                                             FaceProjectionInfos *face_projection_infos) {
            std::vector<unsigned int> const &faces = mesh->get_faces();
            std::vector<math::Vec3f> const &vertices = mesh->get_vertices();
            mve::TriangleMesh::NormalList const &face_normals = mesh->get_face_normals();

            std::size_t const num_views = texture_views->size();

            util::WallTimer timer;
            using namespace Utils;
            ProgressCounter view_counter("\tCalculating face qualities", num_views);
#pragma omp parallel
            {
                std::vector<std::pair<std::size_t, Base::FaceProjectionInfo> > projected_face_view_infos;

#pragma omp for schedule(dynamic)
                for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
                    view_counter.progress<SIMPLE>();

                    Base::TextureView *texture_view = &texture_views->at(j);
                    texture_view->load_image();
                    texture_view->generate_validity_mask();

                    if (settings.data_term == DATA_TERM_GMI) {
                        texture_view->generate_gradient_magnitude();
                        texture_view->erode_validity_mask();
                    }

                    math::Vec3f const &view_pos = texture_view->get_pos();
                    math::Vec3f const &viewing_direction = texture_view->get_viewing_direction();

                    for (std::size_t i = 0; i < faces.size(); i += 3) {
                        std::size_t face_id = i / 3;

                        math::Vec3f const &v1 = vertices[faces[i]];
                        math::Vec3f const &v2 = vertices[faces[i + 1]];
                        math::Vec3f const &v3 = vertices[faces[i + 2]];
                        math::Vec3f const &face_normal = face_normals[face_id];
                        math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

                        /* Check visibility and compute quality */
                        math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
                        math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();

                        /* Backface and basic frustum culling */
                        float viewing_angle = face_to_view_vec.dot(face_normal);
                        if (viewing_angle < 0.0f || viewing_direction.dot(view_to_face_vec) < 0.0f)
                            continue;

                        if (std::acos(viewing_angle) >
                            MATH_DEG2RAD(settings.viewing_angle_threshold))//MATH_DEG2RAD(75.0f))
                            continue;

                        /* Projects into the valid part of the TextureView? */
                        if (!texture_view->inside(v1, v2, v3))
                            continue;

                        if (settings.geometric_visibility_test) {
                            /* Viewing rays do not collide? */
                            bool visible = true;
                            math::Vec3f const *samples[] = {&v1, &v2, &v3};
                            // TODO: random monte carlo samples...

                            for (std::size_t k = 0; k < sizeof(samples) / sizeof(samples[0]); ++k) {
                                BVHTree::Ray ray;
                                ray.origin = *samples[k];
                                ray.dir = view_pos - ray.origin;
                                ray.tmax = ray.dir.norm();
                                ray.tmin = ray.tmax * 0.0001f;
                                ray.dir.normalize();

                                BVHTree::Hit hit;
                                if (bvh_tree.intersect(ray, &hit)) {
                                    visible = false;
                                    break;
                                }
                            }
                            if (!visible) continue;
                        }

                        float total_angle = 0.5 * (viewing_angle + viewing_direction.dot(view_to_face_vec));
                        Base::FaceProjectionInfo info = {j, 0.0f, math::Vec3f(0.0f, 0.0f, 0.0f),
//                                           total_angle};
                                                         viewing_angle};

                        /* Calculate quality. */
                        texture_view->get_face_info(v1, v2, v3, &info, settings);

                        if (info.quality == 0.0) continue;

                        /* Change color space. */
                        mve::image::color_rgb_to_ycbcr(*(info.mean_color));

                        std::pair<std::size_t, Base::FaceProjectionInfo> pair(face_id, info);
                        projected_face_view_infos.push_back(pair);
                    }

                    texture_view->release_image();
                    texture_view->release_validity_mask();
                    if (settings.data_term == DATA_TERM_GMI) {
                        texture_view->release_gradient_magnitude();
                    }
                    view_counter.inc();
                }

                //std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());

#pragma omp critical
                {
                    for (std::size_t i = projected_face_view_infos.size(); 0 < i; --i) {
                        std::size_t face_id = projected_face_view_infos[i - 1].first;
                        const Base::FaceProjectionInfo &info = projected_face_view_infos[i - 1].second;
                        face_projection_infos->at(face_id).push_back(info);
                    }
                    projected_face_view_infos.clear();
                }
            }
        }

        void compute_face_camera_photometric(mve::TriangleMesh::ConstPtr mesh,
                                             std::vector<Base::TextureView> &texture_views,
                                             std::vector<std::set<std::size_t>> &face_visibility_sets,
                                             const Settings &settings) {
            const std::vector<unsigned int> &faces = mesh->get_faces();
            const std::vector<math::Vec3f> &vertices = mesh->get_vertices();
            int n_views = texture_views.size();

            // Build bvh tree
            BVHTree bvh_tree(faces, vertices);

            // Photo-consistency check
            const std::size_t num_faces = faces.size() / 3;
            FaceProjectionInfos face_projection_infos(num_faces);
            calculate_face_projection_infos(mesh, bvh_tree, &texture_views, settings, &face_projection_infos);

            // TODO temporay comment
            std::vector<std::set<std::size_t>> face_photometrics(num_faces);
            for (std::size_t i = 0; i < face_projection_infos.size(); i++) {
                std::vector<Base::FaceProjectionInfo> &infos = face_projection_infos.at(i);
                PhotoMetric::photo_consistency_check(&infos, &(face_photometrics.at(i)), settings);
            }

            for (int j = 0; j < n_views; j++) {
                const Base::TextureView *texture_view = &(texture_views.at(j));
#pragma omp parallel for schedule(dynamic)
                for (int i = 0; i < faces.size(); i += 3) {
                    std::size_t face_id = i / 3;
                    if (face_photometrics[face_id].find(j) == face_photometrics[face_id].end()) {
                        continue;
                    }

                    const math::Vec3f &v0 = vertices[faces[i]];
                    const math::Vec3f &v1 = vertices[faces[i + 1]];
                    const math::Vec3f &v2 = vertices[faces[i + 2]];

                    // projection area check
                    if (!texture_view->inside(v0, v1, v2)) {
                        // TODO 没有投射到，但应该考虑给 image 加一个 padding，不然在某些特殊情况会出现黑色边边
                        continue;
                    }

                    // visibility check
                    const math::Vec3f *samples[] = {&v0, &v1, &v2};
                    const math::Vec3f &view_position = texture_view->get_pos();

                    bool visible = true;
                    for (int k = 0; k < 3; k++) {
                        BVHTree::Ray ray;
                        ray.origin = *samples[k];
                        ray.dir = view_position - ray.origin;
                        ray.tmax = ray.dir.norm();
                        ray.tmin = ray.tmax * 0.0001f;
                        ray.dir.normalize();

                        BVHTree::Hit hit;
                        if (bvh_tree.intersect(ray, &hit)) {
                            visible = false;
                            break;
                        }
                    }

                    if (!visible) {
                        continue;
                    } else {
                        face_visibility_sets[face_id].insert(j);
                    }
                }
            }
        }
    }
}