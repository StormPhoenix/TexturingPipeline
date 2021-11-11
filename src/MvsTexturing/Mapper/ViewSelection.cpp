//
// Created by Storm Phoenix on 2021/10/11.
//

#include <set>
#include <vector>
#include <iostream>

#include <mve/mesh.h>
#include <mve/image_color.h>
#include <acc/bvh_tree.h>

#include <mapmap/full.h>
#define _USE_OPENMP
#include <LBP.h>

#include "Base/View.h"
#include "Base/LabelGraph.h"
#include "Base/SparseTable.h"
#include "Utils/Timer.h"
#include "Utils/Utils.h"
#include "Math/Histogram.h"
#include "Parameter.h"

#include "MvsTexturing.h"

namespace MvsTexturing {
    namespace ViewSelection {
        typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

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

                    if (param.data_term == "gmi") {
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
                            if (!visible) continue;
                        }

                        float total_angle = 0.5 * (viewing_angle + viewing_direction.dot(view_to_face_vec));
                        Base::FaceProjectionInfo info = {j, 0.0f, math::Vec3f(0.0f, 0.0f, 0.0f),
//                                           total_angle};
                                                         viewing_angle};

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

        void compute_face_camera_photometric(MeshConstPtr mesh,
                                             TextureViewList &texture_views,
                                             std::vector<std::set<std::size_t>> &face_visibility_sets,
                                             const Parameter &param) {
            const std::vector<unsigned int> &faces = mesh->get_faces();
            const std::vector<math::Vec3f> &vertices = mesh->get_vertices();
            int n_views = texture_views.size();

            // Build bvh tree
            BVHTree bvh_tree(faces, vertices);

            // Photo-consistency check
            const std::size_t num_faces = faces.size() / 3;
            FaceProjectionInfoList face_projection_infos(num_faces);
            calculate_face_projection_infos(mesh, bvh_tree, param, texture_views, &face_projection_infos);

            // TODO temporay comment
            /*
            std::vector<std::set<std::size_t>> face_photometrics(num_faces);
            for (std::size_t i = 0; i < face_projection_infos.size(); i++) {
                std::vector<Base::FaceProjectionInfo> &infos = face_projection_infos.at(i);
                PhotoMetric::photo_consistency_check(&infos, &(face_photometrics.at(i)), param);
            }
             */

            for (int j = 0; j < n_views; j++) {
                const Base::TextureView *texture_view = &(texture_views.at(j));
#pragma omp parallel for schedule(dynamic)
                for (int i = 0; i < faces.size(); i += 3) {
                    std::size_t face_id = i / 3;
                    // TODO temporay comment
                    /*
                    if (face_photometrics[face_id].find(j) == face_photometrics[face_id].end()) {
                        continue;
                    }
                     */

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

        namespace Projection {
        }

        namespace Mrf {
            bool photometric_outlier_detection(std::vector<Base::FaceProjectionInfo> *infos, const Parameter &param) {
                if (infos->size() == 0) return true;

                /* Configuration variables. */

                double const gauss_rejection_threshold = 6e-3;

                /* If all covariances drop below this we stop outlier detection. */
                double const minimal_covariance = 5e-4;

                int const outlier_detection_iterations = 10;
                int const minimal_num_inliers = 4;

                float outlier_removal_factor = std::numeric_limits<float>::signaling_NaN();
                if (param.outlier_removal == Outlier_Removal_None) {
                    return true;
                } else if (param.outlier_removal == Outlier_Removal_Gauss_Clamping) {
                    outlier_removal_factor = 1.0f;
                } else if (param.outlier_removal == Outlier_Removal_Gauss_Damping) {
                    outlier_removal_factor = 0.2f;
                }

                Eigen::MatrixX3d inliers(infos->size(), 3);
                std::vector<std::uint32_t> is_inlier(infos->size(), 1);
                for (std::size_t row = 0; row < infos->size(); ++row) {
                    inliers.row(row) = Utils::mve_to_eigen(infos->at(row).mean_color).cast<double>();
                }

                Eigen::RowVector3d var_mean;
                Eigen::Matrix3d covariance;
                Eigen::Matrix3d covariance_inv;

                for (int i = 0; i < outlier_detection_iterations; ++i) {
                    if (inliers.rows() < minimal_num_inliers) {
                        return false;
                    }

                    /* Calculate the inliers' mean color and color covariance. */
                    var_mean = inliers.colwise().mean();
                    Eigen::MatrixX3d centered = inliers.rowwise() - var_mean;
                    covariance = (centered.adjoint() * centered) / double(inliers.rows() - 1);

                    /* If all covariances are very small we stop outlier detection
                     * and only keep the inliers (set quality of outliers to zero). */
                    if (covariance.array().abs().maxCoeff() < minimal_covariance) {
                        for (std::size_t row = 0; row < infos->size(); ++row) {
                            if (!is_inlier[row]) infos->at(row).quality = 0.0f;
                        }
                        return true;
                    }

                    /* Invert the covariance. FullPivLU is not the fastest way but
                     * it gives feedback about numerical stability during inversion. */
                    Eigen::FullPivLU<Eigen::Matrix3d> lu(covariance);
                    if (!lu.isInvertible()) {
                        return false;
                    }
                    covariance_inv = lu.inverse();

                    /* Compute new number of inliers (all views with a gauss value above a threshold). */
                    for (std::size_t row = 0; row < infos->size(); ++row) {
                        Eigen::RowVector3d color = Utils::mve_to_eigen(infos->at(row).mean_color).cast<double>();
                        double gauss_value = Utils::multi_gauss_unnormalized(color, var_mean, covariance_inv);
                        is_inlier[row] = (gauss_value >= gauss_rejection_threshold ? 1 : 0);
                    }
                    /* Resize Eigen matrix accordingly and fill with new inliers. */
                    inliers.resize(std::accumulate(is_inlier.begin(), is_inlier.end(), 0), Eigen::NoChange);
                    for (std::size_t row = 0, inlier_row = 0; row < infos->size(); ++row) {
                        if (is_inlier[row]) {
                            inliers.row(inlier_row++) = Utils::mve_to_eigen(infos->at(row).mean_color).cast<double>();
                        }
                    }
                }

                covariance_inv *= outlier_removal_factor;
                for (Base::FaceProjectionInfo &info : *infos) {
                    Eigen::RowVector3d color = Utils::mve_to_eigen(info.mean_color).cast<double>();
                    double gauss_value = Utils::multi_gauss_unnormalized(color, var_mean, covariance_inv);
                    assert(0.0 <= gauss_value && gauss_value <= 1.0);

                    if (param.outlier_removal == Outlier_Removal_None) {
                        return true;
                    } else if (param.outlier_removal == Outlier_Removal_Gauss_Clamping) {
                        if (gauss_value < gauss_rejection_threshold) {
                            info.quality = 0.0f;
                        }
                    } else if (param.outlier_removal == Outlier_Removal_Gauss_Damping) {
                        info.quality *= gauss_value;
                    }
                }
                return true;
            }

            void postprocess_face_infos(const Parameter &param,
                                        FaceProjectionInfoList *face_projection_infos,
                                        DataCosts *data_costs) {

#pragma omp parallel for schedule(dynamic)
                for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {

                    std::vector<Base::FaceProjectionInfo> &infos = face_projection_infos->at(i);
                    if (param.outlier_removal != Outlier_Removal_None) {
                        photometric_outlier_detection(&infos, param);

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
                    for (const Base::FaceProjectionInfo &info : face_projection_infos->at(i)) {
                        max_quality = std::max(max_quality, info.quality);
                    }
                }

                Math::Histogram hist_qualities(0.0f, max_quality, 10000);
                for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {
                    for (const Base::FaceProjectionInfo &info : face_projection_infos->at(i)) {
                        hist_qualities.add_value(info.quality);
                    }
                }

                float percentile = hist_qualities.get_approx_percentile(0.995f);

                /* Calculate the costs. */
                for (std::uint32_t i = 0; i < face_projection_infos->size(); ++i) {
                    for (const Base::FaceProjectionInfo &info : face_projection_infos->at(i)) {

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

            void calculate_data_costs(MeshConstPtr mesh,
                                      const BVHTree &bvh_tree,
                                      TextureViewList &texture_views,
                                      const Parameter &param, DataCosts *data_costs) {

                std::size_t const num_faces = mesh->get_faces().size() / 3;
                std::size_t const num_views = texture_views.size();

                if (num_faces > std::numeric_limits<std::uint32_t>::max())
                    throw std::runtime_error("Exeeded maximal number of faces");
                if (num_views > std::numeric_limits<std::uint16_t>::max())
                    throw std::runtime_error("Exeeded maximal number of views");

                FaceProjectionInfoList face_projection_infos(num_faces);
                calculate_face_projection_infos(mesh, bvh_tree, param, texture_views, &face_projection_infos);
                postprocess_face_infos(param, &face_projection_infos, data_costs);
            }

            void select_views_using_MapMapCpu(const DataCosts &data_costs, Base::LabelGraph &graph) {
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

                mapmap::StopWhenReturnsDiminish<cost_t, simd_w> terminate(5, 0.01);
                std::vector<mapmap::_iv_st<cost_t, simd_w> > solution;

                auto display = [](const mapmap::luint_t time_ms,
                                  const mapmap::_iv_st<cost_t, simd_w> objective) {
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

                std::cout << "\tOptimizing:\n\t\tTime[s]\tEnergy" << std::endl;
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
                std::cout << '\t' << undefined << " faces have not been seen" << std::endl;
            }

            SEACAVE::LBPInference::EnergyType
            SmoothnessPotts(SEACAVE::LBPInference::NodeID, SEACAVE::LBPInference::NodeID,
                            SEACAVE::LBPInference::LabelID l1,
                            SEACAVE::LBPInference::LabelID l2) {
                return l1 == l2 && l1 != 0 && l2 != 0 ? SEACAVE::LBPInference::EnergyType(0)
                                                      : SEACAVE::LBPInference::EnergyType(
                                SEACAVE::LBPInference::MaxEnergy);
            }

            void select_view_using_openMVS(const DataCosts &data_costs, Base::LabelGraph &graph) {
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

            void select_views(const DataCosts &data_costs, Base::LabelGraph &graph, const Parameter &param) {
                if (param.mrf_call_lib == Mrf_Call_Lib_MapMap) {
                    select_views_using_MapMapCpu(data_costs, graph);
                } else {
                    select_view_using_openMVS(data_costs, graph);
                }
            }
        }
    }
}