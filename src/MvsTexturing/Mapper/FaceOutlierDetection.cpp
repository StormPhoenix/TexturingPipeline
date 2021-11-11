//
// Created by Storm Phoenix on 2021/11/5.
//

#include "Parameter.h"
#include "MvsTexturing.h"
#include "Utils/Utils.h"
#include "Mapper/FaceOutlierDetection.h"

namespace MvsTexturing {
    namespace OutlierDetection {
        typedef std::vector<Base::FaceProjectionInfo> FaceProjectionInfoArray;

        bool detect_photometric_outliers(FaceProjectionInfoArray &infos,
                                         const Parameter &param,
                                         std::set<std::size_t> *outliers) {
            if (infos.size() == 0) {
                return true;
            }

            double const gauss_rejection_threshold = 6e-3;
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

            Eigen::MatrixX3d inliers(infos.size(), 3);
            std::vector<std::uint32_t> is_inlier(infos.size(), 1);
            for (std::size_t row = 0; row < infos.size(); ++row) {
                inliers.row(row) = Utils::mve_to_eigen(infos.at(row).mean_color).cast<double>();
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
                    for (std::size_t row = 0; row < infos.size(); ++row) {
                        if (!is_inlier[row]) {
                            infos.at(row).quality = 0.0f;
                            if (outliers != nullptr) {
                                outliers->insert(infos.at(row).view_id);
                            }
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
                for (std::size_t row = 0; row < infos.size(); ++row) {
                    Eigen::RowVector3d color = Utils::mve_to_eigen(infos.at(row).mean_color).cast<double>();
                    double gauss_value = Utils::multi_gauss_unnormalized(color, var_mean, covariance_inv);
                    is_inlier[row] = (gauss_value >= gauss_rejection_threshold ? 1 : 0);
                }
                /* Resize Eigen matrix accordingly and fill with new inliers. */
                inliers.resize(std::accumulate(is_inlier.begin(), is_inlier.end(), 0), Eigen::NoChange);
                for (std::size_t row = 0, inlier_row = 0; row < infos.size(); row++) {
                    if (is_inlier[row]) {
                        inliers.row(inlier_row++) = Utils::mve_to_eigen(infos.at(row).mean_color).cast<double>();
                    }
                }
            }

            covariance_inv *= outlier_removal_factor;
            for (Base::FaceProjectionInfo &info : infos) {
                Eigen::RowVector3d color = Utils::mve_to_eigen(info.mean_color).cast<double>();
                double gauss_value = Utils::multi_gauss_unnormalized(color, var_mean, covariance_inv);
                assert(0.0 <= gauss_value && gauss_value <= 1.0);

                if (param.outlier_removal == Outlier_Removal_None) {
                    return true;
                } else if (param.outlier_removal == Outlier_Removal_Gauss_Clamping) {
                    if (gauss_value < gauss_rejection_threshold) {
                        info.quality = 0.0f;
                        if (outliers != nullptr) {
                            outliers->insert(info.view_id);
                        }
                    }
                } else if (param.outlier_removal == Outlier_Removal_Gauss_Damping) {
                    if (gauss_value < gauss_rejection_threshold) {
                        if (outliers != nullptr) {
                            outliers->insert(info.view_id);
                        }
                    }
                    info.quality *= gauss_value;
                }
            }
            return true;
        }
    }
}