//
// Created by Storm Phoenix on 2021/10/11.
//

#include <set>
#include <list>
#include <vector>
#include <iostream>
#include <algorithm>

#include <mve/mesh.h>
#include <mve/image_color.h>
#include <acc/bvh_tree.h>

#include <mapmap/full.h>

#define _USE_OPENMP

#include <LBP.h>

#include <Base/TriMesh.h>
#include <DataIO/IO.h>
#include <PlaneEstimation/RegionGrowing.h>
#include <PlaneEstimation/RegionExpand.h>
#include <set>
#include <vector>

#include <common.h>

#include "Base/View.h"
#include "Base/FaceGroup.h"
#include "Base/LabelGraph.h"
#include "Base/SparseTable.h"
#include "Mapper/FaceOutlierDetection.h"
#include "Utils/Timer.h"
#include "Utils/MeshAdapter.h"
#include "Math/Histogram.h"
#include "Parameter.h"

#include "MvsTexturing.h"

#define Degree_15_Cosine 0.965925
#define Degree_40_Cosine 0.766044
#define Degree_60_Cosine 0.5
#define Degree_75_Cosine 0.2588

namespace MvsTexturing {
    namespace ViewSelection {
        typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;
        using FaceGroup = MvsTexturing::Base::PlanarAreaFaceGroup;


        namespace __inner__ {
            enum OcclusionLevel {
                Zero_Point_Occlude,
                One_Point_Occlude,
                Two_Point_Occlude,
                Three_Point_Occlude
            };

            OcclusionLevel check_occlusion_level(
                    const Parameter &param, MeshConstPtr mesh, const BVHTree &bvh_tree,
                    const Base::TextureView &texture_view, std::size_t face_id) {

                const std::vector<unsigned int> &faces = mesh->get_faces();
                const std::vector<math::Vec3f> &vertices = mesh->get_vertices();
                const mve::TriangleMesh::NormalList &face_normals = mesh->get_face_normals();

                const math::Vec3f &view_pos = texture_view.get_pos();
                const math::Vec3f &viewing_direction = texture_view.get_viewing_direction();

                const math::Vec3f &v1 = vertices[faces[face_id * 3]];
                const math::Vec3f &v2 = vertices[faces[face_id * 3 + 1]];
                const math::Vec3f &v3 = vertices[faces[face_id * 3 + 2]];
                const math::Vec3f &face_normal = face_normals[face_id];
                const math::Vec3f face_center = (v1 + v2 + v3) / 3.0f;

                math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
                math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();

                float viewing_angle = face_to_view_vec.dot(face_normal);
                if (viewing_angle < 0.0f || viewing_direction.dot(view_to_face_vec) < 0.0f) {
                    return Three_Point_Occlude;
                }

                if (!texture_view.inside(v1, v2, v3)) {
                    return Three_Point_Occlude;
                }

                int n_occlude_points = 0;
                math::Vec3f const *samples[] = {&v1, &v2, &v3};
                for (std::size_t k = 0; k < sizeof(samples) / sizeof(samples[0]); ++k) {
                    BVHTree::Ray ray;
                    ray.origin = *samples[k];
                    ray.dir = view_pos - ray.origin;
                    ray.tmax = ray.dir.norm();
                    ray.tmin = ray.tmax * 0.0001f;
                    ray.dir.normalize();

                    BVHTree::Hit hit;
                    if (bvh_tree.intersect(ray, &hit)) {
                        n_occlude_points++;
                    }
                }

                switch (n_occlude_points) {
                    case 0:
                        return Zero_Point_Occlude;
                    case 1:
                        return One_Point_Occlude;
                    case 2:
                        return Two_Point_Occlude;
                    default:
                        return Three_Point_Occlude;
                }
            }

            struct CameraScore {
                double score;
                int camera_id;
                bool is_full_overlap;
                double meanCosAngle;
                double overlapRate;

                explicit CameraScore() : score(0.), camera_id(-1), is_full_overlap(false),
                                         meanCosAngle(0), overlapRate(0) {}

                explicit CameraScore(double s, int c, bool overlap) :
                        score(s), camera_id(c), is_full_overlap(overlap),
                        meanCosAngle(0), overlapRate(0) {}
            };

            bool func_camera_score_cmp(const struct CameraScore &a, const struct CameraScore &b) {
                return a.score < b.score;
            }

            struct FaceQuality {
                // camera id, range in [0, camera_num - 1]
                std::uint16_t view_id;
                float quality = 0.0f;
                double color_gauss_value = 0.0f;

                explicit FaceQuality(std::uint16_t vid) : view_id(vid) {}

                explicit FaceQuality(std::uint16_t vid, float q, double g) :
                        view_id(vid), quality(q), color_gauss_value(g) {}

                bool operator<(const FaceQuality &other) const {
                    return view_id < other.view_id;
                }
            };
        }

        struct CameraEvaluation {
            std::size_t cameraID;
            double meanCosAngle;
            int nOverlap;
            double overlapRate;
            double orthoCosAngle;
            double perFacePixelDensity;
        };

        using FacesVisibility = std::vector<std::set<__inner__::FaceQuality>>;

        void calculate_face_projection_infos(mve::SubMesh::Ptr mesh, const BVHTree &bvhTree,
                                             const Parameter &param, TextureViewList &textureViews,
                                             FaceProjectionInfoList *faceLabelInfos) {
            const std::vector<std::size_t> &faceIDs = mesh->getFaceIDs();
            const std::vector<unsigned int> &faces = mesh->getFaces();
            const std::vector<math::Vec3f> &vertices = mesh->getVertices();
            const mve::TriangleMesh::NormalList &face_normals = mesh->getFaceNormals();

            std::size_t const num_views = textureViews.size();

            util::WallTimer timer;
#pragma omp parallel
            {
                std::vector<std::pair<std::size_t, Base::FaceProjectionInfo> > projected_face_view_infos;

#pragma omp for schedule(dynamic)
                for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
                    Base::TextureView *texture_view = &(textureViews.at(j));
                    texture_view->load_image();
                    texture_view->generate_validity_mask();

                    /* TODO old version
                    if (param.data_term == "gmi") {
                        texture_view->generate_gradient_magnitude();
                        texture_view->erode_validity_mask();
                    }
                     */

                    /* TODO delete compute gradient by default */
                    texture_view->generate_gradient_magnitude();
                    if (param.data_term == "gmi") {
                        texture_view->erode_validity_mask();
                    }

                    math::Vec3f const &view_pos = texture_view->get_pos();
                    math::Vec3f const &viewing_direction = texture_view->get_viewing_direction();

                    for (std::size_t faceID: faceIDs) {
                        std::size_t vertexIndex = faceID * 3;

                        math::Vec3f const &v1 = vertices[faces[vertexIndex]];
                        math::Vec3f const &v2 = vertices[faces[vertexIndex + 1]];
                        math::Vec3f const &v3 = vertices[faces[vertexIndex + 2]];

                        math::Vec3f const &face_normal = face_normals[faceID];
                        math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

                        /* Check visibility and compute quality */
                        math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
                        math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();

                        /* Backface and basic frustum culling */
                        float viewing_angle = face_to_view_vec.dot(face_normal);
                        if (viewing_angle < 0.0f || viewing_direction.dot(view_to_face_vec) < 0.0f)
                            continue;

                        if (std::acos(viewing_angle) >
                            MATH_DEG2RAD(param.viewing_angle_threshold))//MATH_DEG2RAD(75.0f))
                            continue;

                        /* Projects into the valid part of the TextureView? */
                        if (!texture_view->inside(v1, v2, v3))
                            continue;

                        if (!param.skip_geometric_visibility_test) {
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
                                if (bvhTree.intersect(ray, &hit)) {
                                    visible = false;
                                    break;
                                }
                            }
                            if (!visible) {
                                continue;
                            }
                        }

                        float total_angle = 0.5 * (viewing_angle + viewing_direction.dot(view_to_face_vec));
                        Base::FaceProjectionInfo info = Base::FaceProjectionInfo(j, 0.0f,
                                                                                 math::Vec3f(0.0f, 0.0f, 0.0f),
                                                                                 viewing_angle);

                        /* Calculate quality. */
                        texture_view->get_face_info(v1, v2, v3, &info, param);

                        if (info.quality == 0.0) continue;

                        /* Change color space. */
                        mve::image::color_rgb_to_ycbcr(*(info.mean_color));

                        std::pair<std::size_t, Base::FaceProjectionInfo> pair(faceID, info);
                        projected_face_view_infos.push_back(pair);
                    }

                    texture_view->release_image();
                    texture_view->release_validity_mask();
                    if (param.data_term == Data_Term_GMI) {
                        texture_view->release_gradient_magnitude();
                    }
                }

                std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());

#pragma omp critical
                {
                    for (std::size_t i = projected_face_view_infos.size(); 0 < i; --i) {
                        std::size_t face_id = projected_face_view_infos[i - 1].first;
                        const Base::FaceProjectionInfo &info = projected_face_view_infos[i - 1].second;
                        faceLabelInfos->at(mesh->subFaceMapped(face_id)).push_back(info);
                    }
                    projected_face_view_infos.clear();
                }
            }
        }

        void calculate_face_projection_infos(MeshConstPtr mesh,
                                             const BVHTree &bvh_tree,
                                             const Parameter &param,
                                             TextureViewList &texture_views,
                                             FaceProjectionInfoList *face_projection_infos) {
            std::vector<unsigned int> const &faces = mesh->get_faces();
            std::vector<math::Vec3f> const &vertices = mesh->get_vertices();
            mve::TriangleMesh::NormalList const &face_normals = mesh->get_face_normals();

            std::size_t const num_views = texture_views.size();

            util::WallTimer timer;
#pragma omp parallel
            {
                std::vector<std::pair<std::size_t, Base::FaceProjectionInfo> > projected_face_view_infos;

#pragma omp for schedule(dynamic)
                for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
                    Base::TextureView *texture_view = &(texture_views.at(j));
                    texture_view->load_image();
                    texture_view->generate_validity_mask();

                    /* old version code
                    if (param.data_term == "gmi") {
                        texture_view->generate_gradient_magnitude();
                        texture_view->erode_validity_mask();
                    }
                     */

                    texture_view->generate_gradient_magnitude();
                    if (param.data_term == "gmi") {
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
                            MATH_DEG2RAD(param.viewing_angle_threshold))//MATH_DEG2RAD(75.0f))
                            continue;

                        /* Projects into the valid part of the TextureView? */
                        if (!texture_view->inside(v1, v2, v3))
                            continue;

                        if (!param.skip_geometric_visibility_test) {
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
                            if (!visible) {
                                continue;
                            }
                        }

                        float total_angle = 0.5 * (viewing_angle + viewing_direction.dot(view_to_face_vec));
                        Base::FaceProjectionInfo info = Base::FaceProjectionInfo(j, 0.0f,
                                                                                 math::Vec3f(0.0f, 0.0f, 0.0f),
                                                                                 viewing_angle);

                        /* Calculate quality. */
                        texture_view->get_face_info(v1, v2, v3, &info, param);

                        if (info.quality == 0.0) continue;

                        /* Change color space. */
                        mve::image::color_rgb_to_ycbcr(*(info.mean_color));

                        std::pair<std::size_t, Base::FaceProjectionInfo> pair(face_id, info);
                        projected_face_view_infos.push_back(pair);
                    }

                    texture_view->release_image();
                    texture_view->release_validity_mask();
                    if (param.data_term == Data_Term_GMI) {
                        texture_view->release_gradient_magnitude();
                    }
                }

                std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());

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

        void calculate_face_visibility(MeshConstPtr mesh, const BVHTree &bvh_tree,
                                       TextureViewList &texture_views, const Parameter &param,
                                       FacesVisibility &face_visibilities) {
            const std::vector<unsigned int> &faces = mesh->get_faces();
            const std::vector<math::Vec3f> &vertices = mesh->get_vertices();

            const std::size_t n_faces = mesh->get_faces().size() / 3;
            const std::size_t n_views = texture_views.size();

            FaceProjectionInfoList face_proj_info_list(n_faces);
            calculate_face_projection_infos(mesh, bvh_tree, param, texture_views, &face_proj_info_list);

            // camera outlier detection
            std::vector<std::set<std::size_t>> face_outliers;
            for (std::size_t i = 0; i < face_proj_info_list.size(); i++) {
                face_outliers.push_back(std::set<std::size_t>());
                std::vector<Base::FaceProjectionInfo> &infos = face_proj_info_list.at(i);

                OutlierDetection::detect_outliers(infos, param, &(face_outliers.at(i)));
            }

#pragma omp parallel for schedule(dynamic)
            for (int face_id = 0; face_id < n_faces; face_id++) {
                const std::vector<Base::FaceProjectionInfo> &face_infos = face_proj_info_list[face_id];
                for (int proj_id = 0; proj_id < face_infos.size(); proj_id++) {
                    const Base::FaceProjectionInfo &info = face_infos[proj_id];
                    if (param.outlier_removal != Outlier_Removal_None) {
                        if (face_outliers[face_id].find(info.view_id) != face_outliers[face_id].end()) {
                            continue;
                        }
                    }

//                        face_visibilities[face_id].insert(info.view_id);
                    face_visibilities[face_id].insert(
                            __inner__::FaceQuality(info.view_id, info.quality, info.color_gauss_value));
                }
            }
        }

        double computeFaceViewCosine(long long face_id, const Base::TextureView &texture_view, MeshConstPtr mesh) {
            const math::Vec3f &view_pos = texture_view.get_pos();
            const std::vector<unsigned int> &faces = mesh->get_faces();
            const std::vector<math::Vec3f> &vertices = mesh->get_vertices();
            const mve::TriangleMesh::NormalList &face_normals = mesh->get_face_normals();

            const math::Vec3f &v1 = vertices[faces[face_id * 3]];
            const math::Vec3f &v2 = vertices[faces[face_id * 3 + 1]];
            const math::Vec3f &v3 = vertices[faces[face_id * 3 + 2]];
            const math::Vec3f face_center = (v1 + v2 + v3) / 3.0f;

            const math::Vec3f &face_normal = face_normals[face_id];

            const math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();
            return face_to_view_vec.dot(face_normal);
        }

        void setFaceDefaultLabel(MeshConstPtr mesh, Base::LabelGraph &graph,
                                 std::vector<std::set<__inner__::FaceQuality>> &faceVisibilities,
                                 TextureViewList &textureViews, const BVHTree &bvhTree,
                                 const Parameter &param) {
            // list all un-labeled face
            std::set<std::size_t> un_labeled_faces;
            for (std::size_t i = 0; i < graph.num_nodes(); i++) {
                if (graph.get_label(i) == 0) {
                    un_labeled_faces.insert(i);
                }
            }

            {
                // set label from adjacent faces
                bool decreased = true;
                while (decreased) {
                    decreased = false;
                    for (auto it = un_labeled_faces.begin(); it != un_labeled_faces.end();) {
                        // try to set label from adjacent labels
                        std::size_t un_labeled_face_id = (*it);
                        if (graph.get_label(un_labeled_face_id) != 0) {
                            it++;
                            continue;
                        }

                        const std::set<__inner__::FaceQuality> &face_camera_visibility = faceVisibilities[un_labeled_face_id];
                        bool is_labeled = false;
                        std::size_t chosen_label = 0;
                        double face_view_score = -1.0;

                        const std::vector<std::size_t> &adj_faces = graph.get_adj_nodes(un_labeled_face_id);
                        for (auto it_adj = adj_faces.begin(); it_adj != adj_faces.end(); it_adj++) {
                            const std::size_t adj_label = graph.get_label((*it_adj));
                            if (adj_label == 0) {
                                continue;
                            }

                            __inner__::FaceQuality adj_label_quality = __inner__::FaceQuality(adj_label - 1);
                            if (face_camera_visibility.find(adj_label_quality) ==
                                face_camera_visibility.end()) {
                                continue;
                            }

                            // calculate angle score
                            {
                                const Base::TextureView &texture_view = textureViews[adj_label - 1];
                                double cosine_angle = computeFaceViewCosine(un_labeled_face_id, texture_view, mesh);

                                if (face_view_score < Degree_15_Cosine) {
                                    if (cosine_angle > 0 && cosine_angle > face_view_score) {
                                        face_view_score = cosine_angle;
                                        is_labeled = true;
                                        chosen_label = adj_label;
                                    }
                                }
                            }
                        }

                        if (is_labeled) {
                            decreased = true;
                            graph.set_label(un_labeled_face_id, chosen_label);
                            un_labeled_faces.erase(it++);
                        } else {
                            it++;
                        }
                    }
                }
            }

            {
                // set default label
                if (un_labeled_faces.size() > 0) {
                    for (auto it = un_labeled_faces.begin(); it != un_labeled_faces.end();) {
                        std::size_t un_labeled_face_id = (*it);
                        if (graph.get_label(un_labeled_face_id) != 0) {
                            it++;
                            continue;
                        }

                        double face_view_score = -1.0;
                        std::size_t chosen_label = 0;
                        const std::set<__inner__::FaceQuality> &face_camera_visibility = faceVisibilities[un_labeled_face_id];
                        for (std::set<__inner__::FaceQuality>::const_iterator it_camera = face_camera_visibility.begin();
                             it_camera != face_camera_visibility.end(); it_camera++) {
                            const Base::TextureView &texture_view = textureViews[(*it_camera).view_id];
                            double cosine_angle = computeFaceViewCosine(un_labeled_face_id, texture_view, mesh);

                            if (cosine_angle > 0 && cosine_angle > face_view_score) {
                                face_view_score = cosine_angle;
                                chosen_label = (*it_camera).view_id + 1;
                            }
                        }

                        if (chosen_label != 0) {
                            graph.set_label(un_labeled_face_id, chosen_label);
                            un_labeled_faces.erase(it++);
                        } else {
                            it++;
                        }
                    }
                }
            }

            {
                // check self occlusion case
                if (un_labeled_faces.size() > 0) {
                    for (auto it = un_labeled_faces.begin(); it != un_labeled_faces.end();) {
                        // try to set label from adjacent labels
                        std::size_t un_labeled_face_id = (*it);
                        if (graph.get_label(un_labeled_face_id) != 0) {
                            it++;
                            continue;
                        }

                        bool is_labeled = false;
                        std::size_t chosen_label = 0;
                        double face_view_score = -1.0;

                        const std::vector<std::size_t> &adj_faces = graph.get_adj_nodes(un_labeled_face_id);
                        for (auto it_adj = adj_faces.begin(); it_adj != adj_faces.end(); it_adj++) {
                            const std::size_t adj_label = graph.get_label((*it_adj));
                            if (adj_label == 0) {
                                continue;
                            }

                            const std::size_t adj_camera_id = adj_label - 1;
                            const Base::TextureView &texture_view = textureViews[adj_camera_id];

                            __inner__::OcclusionLevel occlusion_level = __inner__::check_occlusion_level(param, mesh,
                                                                                                         bvhTree,
                                                                                                         texture_view,
                                                                                                         un_labeled_face_id);
                            if (occlusion_level == __inner__::Three_Point_Occlude) {
                                continue;
                            }

                            // calculate angle score
                            {
                                double cosine_angle = computeFaceViewCosine(un_labeled_face_id, texture_view, mesh);

                                if (face_view_score < Degree_15_Cosine) {
                                    if (cosine_angle > 0 && cosine_angle > face_view_score) {
                                        face_view_score = cosine_angle;
                                        is_labeled = true;
                                        chosen_label = adj_label;
                                    }
                                }
                            }
                        }

                        if (is_labeled) {
                            graph.set_label(un_labeled_face_id, chosen_label);
                            un_labeled_faces.erase(it++);
                        } else {
                            it++;
                        }
                    }
                }
            }
        }

        namespace RegionGrowing {
            struct OverlapRateResult {
                std::set<unsigned int> overlappedFaces;

                OverlapRateResult() {}
            };

            bool overlapCriteria(unsigned int faceID, MeshConstPtr mesh,
                                 unsigned int cameraID, TextureViewList &textureViews) {
                const Base::TextureView &textureView = textureViews[cameraID];
                const math::Vec3f &cameraDirection = textureView.get_viewing_direction();

                const mve::TriangleMesh::NormalList &faceNormals = mesh->get_face_normals();
                const math::Vec3f &faceNormal = faceNormals[faceID];

                // computeFaceViewCosine

                double normalCameraDirCos = faceNormal.dot((-cameraDirection));
                double faceViewCos = computeFaceViewCosine(faceID, textureView, mesh);

                double threshold = Degree_60_Cosine;

                if (normalCameraDirCos <= 0 || faceViewCos <= 0) {
                    return false;
                } else if (normalCameraDirCos >= threshold && faceViewCos > threshold) {
                    return true;
                } else {
                    return false;
                }

                // TODO multiple ways ... consider neighborhood (or deformation) of faces
            }

            void computeFaceCameraOverlapRate(
                    std::set<unsigned int> &faces, MeshConstPtr mesh,
                    std::map<unsigned int, OverlapRateResult> &cameras, TextureViewList &textureViews,
                    std::map<unsigned int, std::set<unsigned int>> &faceCameraMap,
                    const std::vector<std::set<__inner__::FaceQuality>> &faceVisibilities) {
                for (std::set<unsigned int>::iterator it = faces.begin(); it != faces.end(); it++) {
                    unsigned int faceID = (*it);
                    const std::set<__inner__::FaceQuality> &visibleCameras = faceVisibilities[faceID];
                    for (std::set<__inner__::FaceQuality>::const_iterator cit = visibleCameras.begin();
                         cit != visibleCameras.end(); cit++) {
                        unsigned int cameraID = cit->view_id;
                        bool isOverlap = overlapCriteria(faceID, mesh, cameraID, textureViews);
                        if (isOverlap) {
                            cameras[cameraID].overlappedFaces.insert(faceID);
                            faceCameraMap[faceID].insert(cameraID);
                        } else {
                            continue;
                        }
                    }
                }
            }

            void tryOverlap(std::set<unsigned int> &faces, std::map<unsigned int, OverlapRateResult> &cameras,
                            std::map<unsigned int, std::set<unsigned int>> &faceCameraMap,
                            Base::LabelGraph &graph, unsigned int cameraId) {
                if (cameras.find(cameraId) == cameras.end()) {
                    LOG_DEBUG("tryOverlap - cameraID NOT EXISTS");
                    return;
                }

                const OverlapRateResult &result = cameras[cameraId];
                for (std::set<unsigned int>::iterator it = result.overlappedFaces.begin();
                     it != result.overlappedFaces.end(); it++) {
                    unsigned int faceID = (*it);
                    faces.erase(faceID);

                    std::set<unsigned int> &cameraIDs = faceCameraMap[faceID];
                    for (std::set<unsigned int>::const_iterator it = cameraIDs.begin(); it != cameraIDs.end(); it++) {
                        unsigned int relCameraID = (*it);
                        if (relCameraID != cameraId) {
                            cameras[relCameraID].overlappedFaces.erase(faceID);
                        }
                    }

                    graph.set_label(faceID, cameraId + 1);
                }
                cameras.erase(cameraId);
            }

            unsigned int getMaxOverlapRateCamera(std::map<unsigned int, OverlapRateResult> &restCameras) {
                unsigned int ans = 0;
                unsigned int maxOverlapFaces = 0;
                bool isFirstInit = true;

                for (std::map<unsigned int, OverlapRateResult>::iterator it = restCameras.begin();
                     it != restCameras.end(); it++) {

                    int nOverlapFaces = it->second.overlappedFaces.size();
                    if (isFirstInit) {
                        maxOverlapFaces = nOverlapFaces;
                        ans = it->first;
                        isFirstInit = false;
                    } else {
                        if (maxOverlapFaces < nOverlapFaces) {
                            maxOverlapFaces = nOverlapFaces;
                            ans = it->first;
                        } else {
                            continue;
                        }
                    }
                }
                return ans;
            }

            /**
             * Do not consider the case when adjLabel equal to zero
             */
            unsigned int compareDifferentLabel(std::size_t label, std::vector<std::size_t> &adjLabels) {
                unsigned int ans = 0;
                for (std::size_t adjLabel: adjLabels) {
                    if (adjLabel != 0 && adjLabel != label) {
                        ans++;
                    }
                }
                return ans;
            }

            /**
             * 1. label 必定不为 0
             * 2. adjLabels 中不为 0 的个数为 3
             */
            std::size_t threeNeighborCompare(std::size_t label, std::vector<std::size_t> &adjLabels) {
                unsigned int nDifferent = compareDifferentLabel(label, adjLabels);
                if (nDifferent == 3) {
                    for (std::size_t adjLabel: adjLabels) {
                        if (adjLabel != 0) {
                            return adjLabel;
                        }
                    }
                    return label;
                } else if (nDifferent == 2) {
                    std::size_t tmp[2];
                    int tmp_i = 0;
                    for (std::size_t adjLabel: adjLabels) {
                        if (adjLabel == 0 || adjLabel == label) {
                            continue;
                        }

                        tmp[tmp_i] = adjLabel;
                        tmp_i++;
                        if (tmp_i == 2) {
                            break;
                        }
                    }
                    if (tmp[0] == tmp[1]) {
                        return tmp[0];
                    } else {
                        return label;
                    }
                } else {
                    return label;
                }
            }

            std::size_t twoNeighborCompare(std::size_t label, std::vector<std::size_t> &adjLabels) {
                unsigned int nDifferent = compareDifferentLabel(label, adjLabels);
                if (nDifferent == 2) {
                    std::size_t tmp[2];
                    int tmp_i = 0;
                    for (std::size_t adjLabel: adjLabels) {
                        if (adjLabel == 0) {
                            continue;
                        }

                        tmp[tmp_i] = adjLabel;
                        tmp_i++;
                        if (tmp_i == 2) {
                            break;
                        }
                    }
                    if (tmp[0] == tmp[1]) {
                        return tmp[0];
                    } else {
                        // TODO chose randomly, haven't take visibility of geometry into consideration
                        return tmp[1];
                    }
                } else {
                    return label;
                }
            }

            std::size_t greedyBasedLocalLabelTransform(std::size_t label, std::vector<std::size_t> &adjLabels) {
                if (label == 0) {
                    return 0;
                }

                // TODO haven't take visibility of label and geometry into consideration
                unsigned int nNeibor = 0;
                for (std::size_t adjLabel: adjLabels) {
                    if (adjLabel != 0) {
                        nNeibor++;
                    }
                }

                if (nNeibor == 3) {
                    return threeNeighborCompare(label, adjLabels);
                } else if (nNeibor == 2) {
                    return twoNeighborCompare(label, adjLabels);
                } else if (nNeibor == 1) {
                    return label;
                } else if (nNeibor == 0) {
                    return label;
                } else if (nNeibor > 3) {
                    return label;
                }
            }

            void greedyBasedRegionGrowing(Base::LabelGraph &graph) {
                auto getFaceLabelFunc = [&graph](unsigned int faceID) -> std::size_t {
                    return graph.get_label(faceID);
                };

                auto getAdjacentFaceIDsFunc = [&graph](
                        unsigned int faceID, std::vector<unsigned int> &result) -> void {
                    const std::vector<std::size_t> &nodes = graph.get_adj_nodes(faceID);
                    for (std::size_t val: nodes) {
                        result.push_back(val);
                    }
                };

                auto getAdjacentFaceLabelsFunc = [&graph](
                        std::vector<unsigned int> &adjFaceIDs, std::vector<std::size_t> &adjFaceLabels) -> void {
                    for (unsigned int adjFaceID: adjFaceIDs) {
                        adjFaceLabels.push_back(graph.get_label(adjFaceID));
                    }
                };

                bool keepIteration = true;
                unsigned int nFaces = graph.num_nodes();

                std::vector<std::size_t> faceLabels;
                faceLabels.resize(nFaces);
#pragma omp parallel for schedule(dynamic)
                for (unsigned int i = 0; i < nFaces; i++) {
                    faceLabels[i] = 0;
                }

                int maxIteration = 50;
                int nIteration = 0;

                while (keepIteration && nIteration < maxIteration) {
                    keepIteration = false;
                    nIteration++;

#pragma omp parallel for schedule(dynamic)
                    for (unsigned int faceID = 0; faceID < nFaces; faceID++) {
                        std::size_t currentFaceLabel = getFaceLabelFunc(faceID);
                        if (currentFaceLabel == 0) {
                            continue;
                        }

                        std::vector<unsigned int> adjFaceIDs;
                        getAdjacentFaceIDsFunc(faceID, adjFaceIDs);
                        if (adjFaceIDs.size() <= 0) {
                            continue;
                        }

                        std::vector<std::size_t> adjFaceLabels;
                        getAdjacentFaceLabelsFunc(adjFaceIDs, adjFaceLabels);

                        std::size_t transformedLabel = greedyBasedLocalLabelTransform(currentFaceLabel, adjFaceLabels);

                        if (transformedLabel != currentFaceLabel) {
                            faceLabels[faceID] = transformedLabel;
                        }
                    }

#pragma omp parallel for schedule(dynamic)
                    for (unsigned int i = 0; i < nFaces; i++) {
                        std::size_t transformedLabel = faceLabels[i];
                        if (transformedLabel == 0) {
                            continue;
                        }

                        if (transformedLabel != graph.get_label(i)) {
                            graph.set_label(i, transformedLabel);
                            keepIteration = true;
                        }
                    }
                }
            }

            void solve_RegionGrowing_problem(MeshConstPtr mesh, const BVHTree &bvhTree, const Parameter &param,
                                             TextureViewList &textureViews, Base::LabelGraph &graph) {
                // calculate face-camera visibility
                std::vector<std::set<__inner__::FaceQuality>> faceVisibilities(mesh->get_faces().size() / 3);
                calculate_face_visibility(mesh, bvhTree, textureViews, param, faceVisibilities);
                LOG_DEBUG("RegionGrowing - Face visibilities computation DONE");

                // compute initial face-camera mapping based overlap rate
                const unsigned int nFaces = mesh->get_faces().size() / 3;
                const unsigned int nCameras = textureViews.size();

                unsigned int nRestFaces = nFaces;
                unsigned int nRestCameras = nCameras;

                std::set<unsigned int> restFaces;
                std::map<unsigned int, std::set<unsigned int>> faceCameraMap;
                {
                    for (unsigned int i = 0; i < nFaces; i++) {
                        restFaces.insert(i);
                        faceCameraMap[i] = std::set<unsigned int>();
                    }
                }

                std::map<unsigned int, OverlapRateResult> restCameras;
                {
                    for (unsigned int i = 0; i < nCameras; i++) {
                        restCameras[i] = OverlapRateResult();
                    }
                }

                computeFaceCameraOverlapRate(restFaces, mesh, restCameras, textureViews,
                                             faceCameraMap, faceVisibilities);
                LOG_DEBUG("RegionGrowing - Camera overlap rate computation DONE");

                while (nRestFaces > 0 && nRestCameras > 0) {
                    std::size_t maxOverlapRateIndex = getMaxOverlapRateCamera(restCameras);
                    tryOverlap(restFaces, restCameras, faceCameraMap, graph, maxOverlapRateIndex);

                    nRestFaces = restFaces.size();
                    nRestCameras = restCameras.size();
                }
                LOG_DEBUG("RegionGrowing - nRestFaces: {}, nRestCameras: {}", nRestFaces, nRestCameras);

                greedyBasedRegionGrowing(graph);

                setFaceDefaultLabel(mesh, graph, faceVisibilities, textureViews, bvhTree, param);
            }
        }

        namespace Mrf {
            void postprocess_face_infos(const Parameter &param, const TextureViewList &textureViews,
                                        FaceProjectionInfoList *face_projection_infos,
                                        DataCosts *data_costs) {

#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {

                    std::vector<Base::FaceProjectionInfo> &infos = face_projection_infos->at(i);
                    if (param.outlier_removal != Outlier_Removal_None) {
                        OutlierDetection::detect_outliers(infos, param);

                        infos.erase(std::remove_if(infos.begin(), infos.end(),
                                                   [](const Base::FaceProjectionInfo &info) -> bool {
                                                       return info.quality == 0.0f;
                                                   }),
                                    infos.end());
                    }
                    std::sort(infos.begin(), infos.end());
                }

                /* Determine the function for the normlization. */
                float max_quality = 0.0f;
                for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {
                    for (const Base::FaceProjectionInfo &info: face_projection_infos->at(i)) {
                        max_quality = std::max(max_quality, info.quality);
                    }
                }

                Math::Histogram hist_qualities(0.0f, max_quality, 10000);
                for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {
                    for (const Base::FaceProjectionInfo &info: face_projection_infos->at(i)) {
                        hist_qualities.add_value(info.quality);
                    }
                }

                float percentile = hist_qualities.get_approx_percentile(0.995f);

                /* Calculate the costs. */
                for (std::uint32_t i = 0; i < face_projection_infos->size(); ++i) {
                    for (const Base::FaceProjectionInfo &info: face_projection_infos->at(i)) {

                        /* Clamp to percentile and normalize. */
                        float normalized_quality = std::min(1.0f, info.quality / percentile);
                        float data_cost = (1.0f - normalized_quality);
                        data_costs->set_value(i, info.view_id, data_cost);
                    }

                    /* Ensure that all memory is freeed. */
                    face_projection_infos->at(i) = std::vector<Base::FaceProjectionInfo>();
                }

                std::cout << "\tMaximum quality of a face within an image: " << max_quality << std::endl;
                std::cout << "\tClamping qualities to " << percentile << " within normalization." << std::endl;
            }

            void postprocess_face_infos_for_submesh(const Parameter &param, const TextureViewList &textureViews,
                                                    mve::SubMesh::Ptr subMeshPtr,
                                                    FaceProjectionInfoList *faceLabelInfos,
                                                    DataCosts *subDataCosts) {
                const std::vector<std::size_t> &subFaceIDs = subMeshPtr->getFaceIDs();

#pragma omp parallel for schedule(dynamic)
                for (std::size_t faceID: subFaceIDs) {
//                    subMeshPtr->subFaceMapped(faceID)
                    std::vector<Base::FaceProjectionInfo> &infos = faceLabelInfos->at(faceID);
                    if (param.outlier_removal != Outlier_Removal_None) {
                        OutlierDetection::detect_outliers(infos, param);

                        infos.erase(std::remove_if(infos.begin(), infos.end(),
                                                   [](const Base::FaceProjectionInfo &info) -> bool {
                                                       return info.quality == 0.0f;
                                                   }),
                                    infos.end());
                    }
                    std::sort(infos.begin(), infos.end());
                }

                /* Determine the function for the normlization. */
                float max_quality = 0.0f;
                for (std::size_t faceID: subFaceIDs) {
                    for (const Base::FaceProjectionInfo &info: faceLabelInfos->at(faceID)) {
                        max_quality = std::max(max_quality, info.quality);
                    }
                }

                Math::Histogram hist_qualities(0.0f, max_quality, 10000);
                for (std::size_t faceID: subFaceIDs) {
                    for (const Base::FaceProjectionInfo &info: faceLabelInfos->at(faceID)) {
                        hist_qualities.add_value(info.quality);
                    }
                }

                float percentile = hist_qualities.get_approx_percentile(0.995f);

                /* Calculate the costs. */
//                for (std::uint32_t i = 0; i < face_projection_infos->size(); ++i) {
                for (std::size_t faceID: subFaceIDs) {
                    for (const Base::FaceProjectionInfo &info: faceLabelInfos->at(faceID)) {

                        /* Clamp to percentile and normalize. */
                        float normalized_quality = std::min(1.0f, info.quality / percentile);
                        float data_cost = (1.0f - normalized_quality);
                        subDataCosts->set_value(subMeshPtr->subFaceMapped(faceID), info.view_id, data_cost);
                    }

                    /* Ensure that all memory is freeed. */
                    faceLabelInfos->at(faceID) = std::vector<Base::FaceProjectionInfo>();
                }
            }

            void calculate_data_costs(MeshConstPtr mesh, const BVHTree &bvh_tree,
                                      TextureViewList &texture_views, const Parameter &param,
                                      DataCosts *data_costs) {
                std::size_t const num_faces = mesh->get_faces().size() / 3;
                std::size_t const num_views = texture_views.size();

                if (num_faces > std::numeric_limits<std::uint32_t>::max())
                    throw std::runtime_error("Exeeded maximal number of faces");
                if (num_views > std::numeric_limits<std::uint16_t>::max())
                    throw std::runtime_error("Exeeded maximal number of views");

                FaceProjectionInfoList face_projection_infos(num_faces);
                calculate_face_projection_infos(mesh, bvh_tree, param, texture_views, &face_projection_infos);
                postprocess_face_infos(param, texture_views, &face_projection_infos, data_costs);
            }

            SEACAVE::LBPInference::EnergyType
            SmoothnessPotts(SEACAVE::LBPInference::NodeID, SEACAVE::LBPInference::NodeID,
                            SEACAVE::LBPInference::LabelID l1, SEACAVE::LBPInference::LabelID l2) {
                return l1 == l2 && l1 != 0 && l2 != 0 ? SEACAVE::LBPInference::EnergyType(0)
                                                      : SEACAVE::LBPInference::EnergyType(
                                SEACAVE::LBPInference::MaxEnergy);
            }

            void solve_mrf_problem_using_MapMapCpu(const DataCosts &data_costs, Base::LabelGraph &graph) {
                using uint_t = unsigned int;
                using cost_t = float;
                constexpr uint_t simd_w = mapmap::sys_max_simd_width<cost_t>();
                using unary_t = mapmap::UnaryTable<cost_t, simd_w>;
                using pairwise_t = mapmap::PairwisePotts<cost_t, simd_w>;

                /* Construct graph */
                mapmap::Graph<cost_t> mgraph(graph.num_nodes());

                for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
                    if (data_costs.col(i).empty()) continue;

                    std::vector<std::size_t> adj_faces = graph.get_adj_nodes(i);
                    for (std::size_t j = 0; j < adj_faces.size(); ++j) {
                        std::size_t adj_face = adj_faces[j];
                        if (data_costs.col(adj_face).empty()) continue;

                        /* Uni directional */
                        if (i < adj_face) {
                            /// i: face index
                            /// adj_face:
                            mgraph.add_edge(i, adj_face, 1.0f);
                        }
                    }
                }
                mgraph.update_components();

                mapmap::LabelSet<cost_t, simd_w> label_set(graph.num_nodes(), false);
                for (std::size_t i = 0; i < data_costs.cols(); ++i) {
                    DataCosts::Column const &data_costs_for_node = data_costs.col(i);

                    std::vector<mapmap::_iv_st<cost_t, simd_w> > labels;
                    if (data_costs_for_node.empty()) {
                        labels.push_back(0);
                    } else {
                        labels.resize(data_costs_for_node.size());
                        for (std::size_t j = 0; j < data_costs_for_node.size(); ++j) {
                            labels[j] = data_costs_for_node[j].first + 1;
                        }
                    }

                    label_set.set_label_set_for_node(i, labels);
                }

                std::vector<unary_t> unaries;
                unaries.reserve(data_costs.cols());
                pairwise_t pairwise(1.0f);
                for (std::size_t i = 0; i < data_costs.cols(); ++i) {
                    DataCosts::Column const &data_costs_for_node = data_costs.col(i);

                    std::vector<mapmap::_s_t<cost_t, simd_w> > costs;
                    if (data_costs_for_node.empty()) {
                        costs.push_back(1.0f);
                    } else {
                        costs.resize(data_costs_for_node.size());
                        for (std::size_t j = 0; j < data_costs_for_node.size(); ++j) {
                            float cost = data_costs_for_node[j].second;
                            costs[j] = cost;
                        }
                    }

                    unaries.emplace_back(i, &label_set);
                    unaries.back().set_costs(costs);
                }

//                mapmap::StopWhenReturnsDiminish <cost_t, simd_w> terminate(1, 6.0);
                mapmap::StopWhenReturnsDiminish<cost_t, simd_w> terminate(10, 0.01);
                std::vector<mapmap::_iv_st<cost_t, simd_w> > solution;

                auto display = [](const mapmap::luint_t time_ms,
                                  const mapmap::_iv_st<cost_t, simd_w> objective) {
                    // TODO comment for now
                    std::cout << "\t\t" << time_ms / 1000 << "\t" << objective << std::endl;
                };

                /* Create mapMAP solver object. */
                mapmap::mapMAP<cost_t, simd_w> solver;
                solver.set_graph(&mgraph);
                solver.set_label_set(&label_set);
                for (std::size_t i = 0; i < graph.num_nodes(); ++i)
                    solver.set_unary(i, &unaries[i]);
                solver.set_pairwise(&pairwise);
                solver.set_logging_callback(display);
                solver.set_termination_criterion(&terminate);

                /* Pass configuration arguments (optional) for solve. */
                mapmap::mapMAP_control ctr;
                ctr.use_multilevel = true;
                ctr.use_spanning_tree = true;
                ctr.use_acyclic = true;
                ctr.spanning_tree_multilevel_after_n_iterations = 5;
                ctr.force_acyclic = true;
                ctr.min_acyclic_iterations = 5;
                ctr.relax_acyclic_maximal = true;
                ctr.tree_algorithm = mapmap::LOCK_FREE_TREE_SAMPLER;

                /* Set false for non-deterministic (but faster) mapMAP execution. */
                ctr.sample_deterministic = true;
                ctr.initial_seed = 548923723;

//                std::cout << "\tOptimizing:\n\t\tTime[s]\tEnergy" << std::endl;
                solver.optimize(solution, ctr);

                /* Label 0 is undefined. */
                std::size_t num_labels = data_costs.rows() + 1;
                std::size_t undefined = 0;
                /* Extract resulting labeling from solver. */
                for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
                    int label = label_set.label_from_offset(i, solution[i]);
                    if (label < 0 || num_labels <= static_cast<std::size_t>(label)) {
                        throw std::runtime_error("Incorrect labeling");
                    }
                    if (label == 0) undefined += 1;
                    graph.set_label(i, static_cast<std::size_t>(label));
                }
                // TODO comment for now
//                std::cout << '\t' << undefined << " faces have not been seen" << std::endl;
            }

            void solve_mrf_problem_using_openMVS(const DataCosts &data_costs, Base::LabelGraph &graph) {
                using LBPInference = SEACAVE::LBPInference;
                typedef SEACAVE::LBPInference::NodeID NodeID;
                typedef SEACAVE::LBPInference::LabelID LabelID;

                double fRatioDataSmoothness = 0.1f;
                const LBPInference::EnergyType MaxEnergy(fRatioDataSmoothness * LBPInference::MaxEnergy);

                SEACAVE::LBPInference inference;
                inference.SetNumNodes(data_costs.cols());
                inference.SetSmoothCost(SmoothnessPotts);

                // set face neighbors
                for (std::size_t i = 0; i < graph.num_nodes(); i++) {
                    if (data_costs.col(i).empty()) {
                        continue;
                    }

                    std::vector<std::size_t> adj_faces = graph.get_adj_nodes(i);
                    for (std::size_t j = 0; j < adj_faces.size(); ++j) {
                        std::size_t adj_face = adj_faces[j];
                        if (data_costs.col(adj_face).empty()) {
                            continue;
                        }

                        // uni-directional
                        if (i < adj_face) {
                            inference.SetNeighbors(i, adj_face);
                        }
                    }
                }
                /*
                for (std::size_t i = 0; i < mesh->get_faces().size(); i += 3) {
                    std::size_t face_id = i / 3;
                    std::size_t v0 = mesh->get_faces()[i];
                    std::size_t v1 = mesh->get_faces()[i + 1];
                    std::size_t v2 = mesh->get_faces()[i + 2];

                    std::vector<std::size_t> adj_faces;
                    mesh_info.get_faces_for_edge(v0, v1, &adj_faces);
                    mesh_info.get_faces_for_edge(v1, v2, &adj_faces);
                    mesh_info.get_faces_for_edge(v2, v0, &adj_faces);

                    for (std::size_t j = 0; j < adj_faces.size(); j++) {
                        std::size_t adj_face_id = adj_faces[j];
                        if (face_id < adj_face_id) {
                            inference.SetNeighbors(face_id, adj_face_id);
                        }
                    }
                }*/

                // set data cost
                for (NodeID nodeId = 0; nodeId < inference.GetNumNodes(); nodeId++) {
                    inference.SetDataCost(LabelID(0), NodeID(nodeId), MaxEnergy);
                }

                for (std::size_t i = 0; i < data_costs.cols(); i++) {
                    const DataCosts::Column &data_term = data_costs.col(i);
                    if (!data_term.empty()) {
                        NodeID nodeId(i);
                        for (std::size_t j = 0; j < data_term.size(); j++) {
                            LabelID labelId(data_term[j].first + 1);
                            const LBPInference::EnergyType energy_cost = LBPInference::EnergyType(
                                    data_term[j].second * MaxEnergy);
                            inference.SetDataCost(labelId, nodeId, energy_cost);
                        }
                    }
                }

                // Optimize
                inference.Optimize();

                // GetLabel
                for (NodeID nodeId = 0; nodeId < inference.GetNumNodes(); nodeId++) {
                    graph.set_label(static_cast<std::size_t>(nodeId),
                                    static_cast<std::size_t>(inference.GetLabel(nodeId)));
                }
            }

            void solve_mrf_problem(const DataCosts &data_costs, Base::LabelGraph &graph, const Parameter &param) {
                if (param.mrf_call_lib == Mrf_Call_Lib_MapMap) {
                    solve_mrf_problem_using_MapMapCpu(data_costs, graph);
                } else {
                    solve_mrf_problem_using_openMVS(data_costs, graph);
                }
            }

            void setFaceDefaultLabel_V2(MeshConstPtr mesh, const mve::MeshInfo &meshInfo, Base::LabelGraph &graph,
                                        TextureViewList &textureViews, const BVHTree &bvhTree,
                                        const Parameter &param) {
                // list all un-labeled face
                std::vector<std::size_t> unLabeledFaces;
                for (std::size_t i = 0; i < graph.num_nodes(); i++) {
                    if (graph.get_label(i) == 0) {
                        unLabeledFaces.push_back(i);
                    }
                }

                mve::SubMesh::Ptr subMesh = mve::SubMesh::create(mesh, unLabeledFaces);
                std::size_t nSubFaces = subMesh->getFaceCount();
                DataCosts subDataCosts(nSubFaces, textureViews.size());
                Base::LabelGraph subGraph(nSubFaces);
                MvsTexturing::Builder::MVE::build_adjacency_graph(subMesh, meshInfo, &subGraph);

                FaceProjectionInfoList subFaceLabelInfos(nSubFaces);
                calculate_face_projection_infos(subMesh, bvhTree, param, textureViews, &subFaceLabelInfos);
                postprocess_face_infos(param, textureViews, &subFaceLabelInfos, &subDataCosts);
                solve_mrf_problem_using_MapMapCpu(subDataCosts, subGraph);

                for (std::size_t i = 0; i < subGraph.num_nodes(); i ++) {
                    auto mappedFaceID = subMesh->getFaceIDs()[i];
                    graph.set_label(mappedFaceID, subGraph.get_label(i));
                }
            }

            void
            solveMultipleMrfProblem(MeshPtr mesh, MeshInfo &meshInfo, const Parameter &param, const BVHTree &bvhTree,
                                    Base::LabelGraph &graph, TextureViewList &textureViews,
                                    std::vector<FaceGroup> &areas) {
                using namespace MvsTexturing;
                namespace VS = ViewSelection;
                typedef Base::SparseTable<std::uint32_t, std::uint16_t, double> DataCosts;

                std::size_t nTotalFaces = mesh->get_faces().size() / 3;

                // Build sub-meshes
                std::vector<mve::SubMesh::Ptr> subMeshes;
                for (FaceGroup &area: areas) {
                    std::vector<std::size_t> faceIDs;
                    for (std::size_t faceID: area.m_face_indices) {
                        faceIDs.push_back(faceID);
                    }
                    subMeshes.push_back(mve::SubMesh::create(mesh, faceIDs));
                }

                // Calculate all face-label relationships
                FaceProjectionInfoList faceLabelInfos(nTotalFaces);
                calculate_face_projection_infos(mesh, bvhTree, param, textureViews, &faceLabelInfos);

                // Texturing for each sub-mesh
                for (mve::SubMesh::Ptr subMesh: subMeshes) {
                    std::size_t nSubFaces = subMesh->getFaceCount();
                    DataCosts subDataCosts(nSubFaces, textureViews.size());
                    Base::LabelGraph subGraph(nSubFaces);
                    MvsTexturing::Builder::MVE::build_adjacency_graph(subMesh, meshInfo, &subGraph);

                    // Calculate data cost
                    postprocess_face_infos_for_submesh(param, textureViews, subMesh, &faceLabelInfos, &subDataCosts);
//                    solve_mrf_problem(subDataCosts, subGraph, param);
                    solve_mrf_problem_using_openMVS(subDataCosts, subGraph);

                    for (std::size_t i = 0; i < subGraph.num_nodes(); i++) {
                        auto mappedFaceID = subMesh->getFaceIDs()[i];
                        graph.set_label(mappedFaceID, subGraph.get_label(i));
                    }
                }

                Mrf::setFaceDefaultLabel_V2(mesh, meshInfo, graph, textureViews, bvhTree, param);
            }
        }

        namespace Projection {
            using PlaneGroup = MeshPolyRefinement::Base::PlaneGroup;
            using TriMesh = MeshPolyRefinement::Base::TriMesh;
            using LabelGraph = MvsTexturing::Base::LabelGraph;
//            using FacesVisibility = std::vector<std::set<std::size_t>>;

            void camera_project_to_plane(const MeshConstPtr mesh, const mve::MeshInfo &mesh_info,
                                         const PlanarAreaFaceGroup &group, const FacesVisibility &f_visibility,
                                         const int select_camera_id, LabelGraph &graph, bool &is_last_camera,
                                         int *nAppliedFace = nullptr, std::set<int> *usedFaces = nullptr) {
                /**
                 * 相机 select_camera_id 向平面 group 中所有面片进行投影：
                 * 1. 如果 faceID 已被打上其他相机标签，则不投影；
                 * 2. 如果 faceID 尚打上相机标签，则投影相机 select_camera_id
                 *
                 * 对所有在相机 select_camera_id 这一轮投影中被投中对面片，全部做一次区域扩张
                 * 此时投影只 check 了 visibility 选项，万一选择的相机非常倾斜呢？那岂不是投射的都是模糊的
                 */
                is_last_camera = true;
                std::set<std::size_t> applied_face;
                for (std::size_t i = 0; i < group.m_face_indices.size(); i++) {
                    std::size_t faceID = group.m_face_indices[i];
                    if (graph.get_label(faceID) == 0) {
                        // haven't set label
                        const std::set<__inner__::FaceQuality> &vis = f_visibility[faceID];
                        if (vis.find(__inner__::FaceQuality(select_camera_id - 1)) != vis.end()) {
                            graph.set_label(faceID, select_camera_id);
                            if (nAppliedFace != nullptr) {
                                (*nAppliedFace)++;
                            }
                            applied_face.insert(faceID);
                            if (usedFaces != nullptr) {
                                usedFaces->insert(faceID);
                            }
                        } else {
                            is_last_camera = false;
                        }
                    } else {
                        // TODO delete
                        // nothing to do
                    }
                }

                // min-cuts
                for (auto it = applied_face.begin(); it != applied_face.end(); it++) {
                    std::size_t faceID = (*it);
                    __inner__::FaceQuality curFaceLabel = __inner__::FaceQuality(select_camera_id - 1);

                    // find adjacent faces
                    std::vector<std::size_t> adjFaceIDs;
                    {
                        std::set<std::size_t> adj_faces_set;
                        for (int i = 0; i < 3; i++) {
                            std::size_t vid = mesh->get_faces()[faceID * 3 + i];
                            const mve::MeshInfo::AdjacentFaces &adj_faces = mesh_info[vid].faces;
                            for (int j = 0; j < adj_faces.size(); j++) {
                                adj_faces_set.insert(adj_faces[j]);
                            }
                        }

                        for (auto it = adj_faces_set.begin(); it != adj_faces_set.end(); it++) {
                            adjFaceIDs.push_back((*it));
                        }
                    }

                    for (std::size_t adjFaceID: adjFaceIDs) {
                        if (adjFaceID == faceID) {
                            continue;
                        }
                        std::size_t adjLabel = graph.get_label(adjFaceID);
                        if (adjLabel != 0 && adjLabel != select_camera_id) {
                            // Get visible faces
                            const std::set<__inner__::FaceQuality> &adjVisLabel = f_visibility[adjFaceID];
                            if (adjVisLabel.find(curFaceLabel) != adjVisLabel.end()) {
                                // TODO expand by default
//                                graph.set_label(adjFaceID, select_camera_id);

                                // can expand
                                __inner__::FaceQuality adj_quality = __inner__::FaceQuality(adjLabel - 1);
                                double adj_gauss_value = adjVisLabel.find(adj_quality)->color_gauss_value;
                                double cur_gauss_value = adjVisLabel.find(curFaceLabel)->color_gauss_value;

                                if (adj_gauss_value * 1 < cur_gauss_value) {
                                    graph.set_label(adjFaceID, select_camera_id);
                                }
                            } else {
                                // can't expand
                                // TODO
                            }
                        }
                    }
                }
            }

            double getFacePixels(const MeshConstPtr mesh, const TextureView &textureView, std::size_t faceID) {
                std::vector<unsigned int> const &faces = mesh->get_faces();
                std::vector<math::Vec3f> const &vertices = mesh->get_vertices();

                math::Vec3f const &v1 = vertices[faces[faceID * 3]];
                math::Vec3f const &v2 = vertices[faces[faceID * 3 + 1]];
                math::Vec3f const &v3 = vertices[faces[faceID * 3 + 2]];

                math::Vec2f p1 = textureView.get_pixel_coords(v1);
                math::Vec2f p2 = textureView.get_pixel_coords(v2);
                math::Vec2f p3 = textureView.get_pixel_coords(v3);

                Math::Tri2D tri(p1, p2, p3);
                return tri.get_area();
            }

            CameraEvaluation evaluate_camera(const MeshConstPtr mesh, const TextureViewList &textureViews,
                                             const FacesVisibility &facesVisibility, const PlanarAreaFaceGroup &group,
                                             int cameraID, std::set<int> *usedFaceIDs = nullptr) {
                CameraEvaluation ans;
                ans.nOverlap = 0;
                ans.meanCosAngle = 0.f;
                ans.perFacePixelDensity = 0.f;

                // Plane normal
                math::Vec3f planeNormal;
                planeNormal[0] = group.m_plane_normal(0, 0);
                planeNormal[1] = group.m_plane_normal(0, 1);
                planeNormal[2] = group.m_plane_normal(0, 2);
                planeNormal = planeNormal.normalized();

                // Camera data
                const Base::TextureView &textureView = textureViews[cameraID - 1];
                double meanCosAngle = 0.f;
                double meanFacePixels = 0.f;
                int nTotalFace = 0;

                for (std::size_t i = 0; i < group.m_face_indices.size(); i++) {
                    long long faceID = group.m_face_indices[i];
                    const std::set<__inner__::FaceQuality> &visibility = facesVisibility[faceID];
                    if (usedFaceIDs != nullptr && usedFaceIDs->find(faceID) != usedFaceIDs->end()) {
                        continue;
                    }

                    if (visibility.find(__inner__::FaceQuality(cameraID - 1)) != visibility.end()) {
                        ans.nOverlap++;
                        double faceViewCos = computeFaceViewCosine(faceID, textureView, mesh);
                        meanCosAngle += std::max(double(0.), faceViewCos);
                        meanFacePixels += getFacePixels(mesh, textureView, faceID);
                    }
                    nTotalFace++;
                }

                if (nTotalFace == 0) {
                    ans.overlapRate = 0;
                } else {
                    ans.overlapRate = double(ans.nOverlap) / double(nTotalFace);
                    ans.meanCosAngle = meanCosAngle / double(ans.nOverlap);
                    ans.perFacePixelDensity = meanFacePixels / double(ans.nOverlap);
                }

                ans.orthoCosAngle = planeNormal.dot(-textureView.get_viewing_direction().normalized());
                ans.cameraID = cameraID;
                return ans;
            }

            void project_to_plane_V4(const MeshConstPtr mesh, const mve::MeshInfo &mesh_info,
                                     const PlanarAreaFaceGroup &group,
                                     const TextureViewList &texture_views, const FacesVisibility &faces_visibility,
                                     LabelGraph &graph) {
                /**
                 * Filter visible cameras
                 * For each cameraID, cameraID \in [1, nMaxCameras]
                 */
                std::set<int> cameraIDs;
                for (int cameraID = 1; cameraID <= texture_views.size(); cameraID++) {
                    // Filter visible cameras
                    for (std::size_t i = 0; i < group.m_face_indices.size(); i++) {
                        long long faceID = group.m_face_indices[i];
                        const std::set<__inner__::FaceQuality> &visibility = faces_visibility[faceID];

                        if (visibility.find(__inner__::FaceQuality(cameraID - 1)) != visibility.end()) {
                            cameraIDs.insert(cameraID);
                            break;
                        }
                    }
                }

                // Var cameraHeap compare function
                auto cameraEvalDensityCmpFunc = [](const struct CameraEvaluation &a,
                                                   const struct CameraEvaluation &b) -> bool {
                    return a.perFacePixelDensity > b.perFacePixelDensity;
                };


                std::set<int> usedFaceIDs;
                std::vector<CameraEvaluation> cameraEvals;
                for (int cameraID: cameraIDs) {
                    cameraEvals.push_back(evaluate_camera(mesh, texture_views, faces_visibility,
                                                          group, cameraID, &usedFaceIDs));
                }
                std::sort(cameraEvals.begin(), cameraEvals.end(), cameraEvalDensityCmpFunc);

                const unsigned int thresholdKNN = 10;
                std::list<std::size_t> cameraEvalIndices;
                int nextCameraIndex = 0;
                for (; (nextCameraIndex < thresholdKNN) && (nextCameraIndex < cameraEvals.size()); nextCameraIndex++) {
                    cameraEvalIndices.push_back(nextCameraIndex);
                }

                /**
                 * Sort camera by "perFacePixelDensity" attribute, use KNN method
                 */
                bool isPlaneOverlap = false;
                int nRestCameras = cameraEvals.size();
                while (!isPlaneOverlap && nRestCameras > 0) {
                    // Select the most suitable camera
                    int suitableCameraID = 0;
                    int maxOverlapCount = 0;
                    std::list<std::size_t>::iterator cit = cameraEvalIndices.begin();
                    for (std::list<std::size_t>::iterator it = cameraEvalIndices.begin();
                         it != cameraEvalIndices.end(); it++) {
                        if (maxOverlapCount < cameraEvals[(*it)].nOverlap) {
                            maxOverlapCount = cameraEvals[(*it)].nOverlap;
                            suitableCameraID = cameraEvals[(*it)].cameraID;
                            cit = it;
                        }
                    }

                    if (suitableCameraID == 0) {
                        break;
                    }

                    int nAppliedFace = 0;
                    camera_project_to_plane(mesh, mesh_info, group, faces_visibility,
                                            suitableCameraID, graph, isPlaneOverlap, &nAppliedFace, &usedFaceIDs);
                    cameraEvalIndices.erase(cit);

                    if (nextCameraIndex < cameraEvals.size()) {
                        cameraEvalIndices.push_back(nextCameraIndex);
                        nextCameraIndex += 1;
                    }

                    nRestCameras --;
                }
            }

            /**
             * 求解基于平面区域的网格纹理投影问题，参数 planar_groups 代表平面区域面片集合
             * @param planar_groups 网格数据(mesh)中构成平面区域的面片集合
             */
            void solve_projection_problem(MeshConstPtr mesh, const mve::MeshInfo &mesh_info,
                                          const BVHTree &bvh_tree,
                                          const std::vector<PlanarAreaFaceGroup> &planar_groups,
                                          Base::LabelGraph &graph, TextureViewList &texture_views,
                                          const Parameter &param) {
                // visibility detection
                std::vector<std::set<__inner__::FaceQuality>> face_visibilities(mesh->get_faces().size() / 3);
                calculate_face_visibility(mesh, bvh_tree, texture_views, param, face_visibilities);

#pragma omp parallel for schedule(dynamic)
                for (std::size_t group_id = 0; group_id < planar_groups.size(); group_id++) {
                    const PlanarAreaFaceGroup &group = planar_groups[group_id];
                    project_to_plane_V4(mesh, mesh_info, group, texture_views, face_visibilities, graph);
                }

                Mrf::setFaceDefaultLabel_V2(mesh, mesh_info, graph, texture_views, bvh_tree, param);
                setFaceDefaultLabel(mesh, graph, face_visibilities, texture_views, bvh_tree, param);
            }

            void solve_OptimalPerFace(mve::TriangleMesh::ConstPtr mesh, const mve::MeshInfo &mesh_info,
                                      const acc::BVHTree<unsigned int, math::Vec3f> &bvh_tree,
                                      Base::LabelGraph &graph, std::vector<Base::TextureView> &texture_views,
                                      const Parameter &param) {
                // visibility detection
                std::vector<std::set<__inner__::FaceQuality>> face_visibilities(mesh->get_faces().size() / 3);
                calculate_face_visibility(mesh, bvh_tree, texture_views, param, face_visibilities);

                for (int faceID = 0; faceID < face_visibilities.size(); faceID++) {
                    std::set<__inner__::FaceQuality> &qualities = face_visibilities[faceID];

                    double maxQuality = 0;
                    int viewID = -1;
                    for (std::set<__inner__::FaceQuality>::iterator it = qualities.begin();
                         it != qualities.end(); it++) {
                        if (maxQuality < it->quality) {
                            maxQuality = it->quality;
                            viewID = it->view_id;
                        }
                    }

                    if (viewID != -1) {
                        graph.set_label(faceID, (viewID + 1));
                    } else {
                        graph.set_label(faceID, 0);
                    }
                }
            }

            /**
             * 求解基于平面区域的网格纹理投影问题，由于没有平面区域信息参数，所以函数内部会计算网格平面性信息
             */
            void solve_projection_problem(MeshConstPtr mesh, const mve::MeshInfo &mesh_info,
                                          const BVHTree &bvh_tree, Base::LabelGraph &graph,
                                          TextureViewList &texture_views,
                                          const Parameter &param) {
                // detect face groups
                std::vector<PlanarAreaFaceGroup> planar_groups;
                {
                    TriMesh tri_mesh;
                    Utils::mveMesh_to_triMesh(mesh, tri_mesh);
                    {
                        using namespace MeshPolyRefinement;
                        // detect planes on mesh using region-growing
                        PlaneEstimation::region_growing_plane_estimate(tri_mesh, param.planar_score,
                                                                       param.angle_threshold,
                                                                       param.ratio_threshold, param.min_plane_size);

                        // expand plane segment regions
                        PlaneEstimation::plane_region_expand(tri_mesh, 50.0, 45.0);

                        // filter small non-plane regions
                        PlaneEstimation::plane_region_refine(tri_mesh);

                        // merge parallel adjacent plane segments
                        PlaneEstimation::plane_region_merge(tri_mesh);

                        for (std::size_t group_id = 0; group_id < tri_mesh.m_plane_groups.size(); group_id++) {
                            PlaneGroup &group = tri_mesh.m_plane_groups[group_id];
                            planar_groups.push_back(PlanarAreaFaceGroup());

                            for (std::size_t f_index: group.m_indices) {
                                planar_groups.back().m_face_indices.push_back(f_index);
                            }
                            planar_groups.back().m_plane_center = group.m_plane_center;
                            planar_groups.back().m_plane_normal = group.m_plane_normal;

//                            检查平面法线信息
//                            LOG_DEBUG("Check planar groups: ({}, {}, {})", planar_groups.back().m_plane_normal(0, 0),
//                                      planar_groups.back().m_plane_normal(0, 1),
//                                      planar_groups.back().m_plane_normal(0, 2));
                        }
                    }
                }

                solve_projection_problem(mesh, mesh_info, bvh_tree, planar_groups, graph, texture_views, param);
            }
        }
    }
}