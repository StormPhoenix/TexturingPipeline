//
// Created by Storm Phoenix on 2021/10/15.
//

#include <Eigen/Core>

namespace MvsTexturing {
    namespace Math {
        template<typename T, int N>
        const T multi_gauss_unnormalized(const Eigen::Matrix<T, 1, N> &X, const Eigen::Matrix<T, 1, N> &mu,
                                         const Eigen::Matrix<T, N, N> &inv_covariance) {
            Eigen::Matrix<T, 1, N> mean_X = X - mu;
            return std::exp(T(-0.5) * mean_X * inv_covariance * mean_X.adjoint());
        }
    }
}