//
// Created by Storm Phoenix on 2021/10/20.
//

#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "Base/View.h"
#include "Utils/Utils.h"
#include "Utils/Settings.h"

namespace MvsTexturing {
    namespace PhotoMetric {
        bool photo_consistency_check(std::vector<Base::FaceProjectionInfo> *infos,
                                     std::set<std::size_t> *face_visibility, const Settings &settings) {
            if (infos->size() == 0) return true;

            /* Configuration variables. */
//    double const gauss_rejection_threshold = 6e-3;
//    double const gauss_rejection_threshold = 9e-3;
            double const gauss_rejection_threshold = 1e-2;
//    double const gauss_rejection_threshold = 6e-2;
//    double const gauss_rejection_threshold = 1e-1;

            /* If all covariances drop below this we stop outlier detection. */
            double const minimal_covariance = 5e-4;

            int const outlier_detection_iterations = 10;
            int const minimal_num_inliers = 4;

            float outlier_removal_factor = std::numeric_limits<float>::signaling_NaN();
            switch (settings.outlier_removal) {
                case OUTLIER_REMOVAL_NONE:
                    return true;
                case OUTLIER_REMOVAL_GAUSS_CLAMPING:
                    outlier_removal_factor = 1.0f;
                    break;
                case OUTLIER_REMOVAL_GAUSS_DAMPING:
                    outlier_removal_factor = 0.2f;
                    break;
            }

            // Dataset: mean-color
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
                // RGB mean color
                var_mean = inliers.colwise().mean();
                // Normalized
                Eigen::MatrixX3d centered = inliers.rowwise() - var_mean;
                covariance = (centered.adjoint() * centered) / double(inliers.rows() - 1);

                /* If all covariances are very small we stop outlier detection
                 * and only keep the inliers (set quality of outliers to zero). */
                if (covariance.array().abs().maxCoeff() < minimal_covariance) {
                    for (std::size_t row = 0; row < infos->size(); ++row) {
                        if (!is_inlier[row]) {
                            infos->at(row).quality = 0.0f;
                        } else {
                            face_visibility->insert(infos->at(row).view_id);
                        }
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
                switch (settings.outlier_removal) {
                    case OUTLIER_REMOVAL_NONE:
                        return true;
                    case OUTLIER_REMOVAL_GAUSS_DAMPING:
                        info.quality *= gauss_value;
                        break;
                    case OUTLIER_REMOVAL_GAUSS_CLAMPING:
                        if (gauss_value < gauss_rejection_threshold) {
                            info.quality = 0.0f;
                        } else {
                            face_visibility->insert(info.view_id);
                        }
                        break;
                }
            }
            return true;
        }
    }
}