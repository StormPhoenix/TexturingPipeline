//
// Created by Storm Phoenix on 2021/10/20.
//

#ifndef TEXTURINGPIPELINE_UTILS_H
#define TEXTURINGPIPELINE_UTILS_H

#include <string>
#include <vector>
#include <fstream>
#include <cstring>
#include <cerrno>
#include <cassert>

#include <Eigen/Core>

#include <util/exception.h>
#include <util/file_system.h>

#include <math/vector.h>
#include <math/matrix.h>
#include <math/functions.h>

#include <mve/mesh.h>

namespace MvsTexturing {
    namespace Utils {
        template<typename T, int M, int N>
        Eigen::Matrix<T, M, N> mve_to_eigen(math::Matrix<T, M, N> const &mat) {
            Eigen::Matrix<T, M, N> ret;
            for (int m = 0; m < M; ++m)
                for (int n = 0; n < N; ++n)
                    ret(m, n) = mat(m, n);
            return ret;
        }

        template<typename T, int N>
        Eigen::Matrix<T, 1, N> mve_to_eigen(math::Vector<T, N> const &vec) {
            Eigen::Matrix<T, 1, N> ret;
            for (int n = 0; n < N; ++n)
                ret(0, n) = vec(n);
            return ret;
        }

        template<typename T, int N>
        T const multi_gauss_unnormalized(Eigen::Matrix<T, 1, N> const &X, Eigen::Matrix<T, 1, N> const &mu,
                                 Eigen::Matrix<T, N, N> const &covariance_inv) {

            Eigen::Matrix<T, 1, N> mean_removed = X - mu;
            return std::exp(T(-0.5) * mean_removed * covariance_inv * mean_removed.adjoint());
        }

        /** Return the number suffix for n, e.g. 3 -> "rd". */
        inline
        std::string number_suffix(int n) {
            if (n % 100 == 11 || n % 100 == 12 || n % 100 == 13)
                return "th";
            if (n % 10 == 1)
                return "st";
            if (n % 10 == 2)
                return "nd";
            if (n % 10 == 3)
                return "rd";
            return "th";
        }

        template<typename T>
        void write_vector_to_csv(std::string const &filename, std::vector<T> const &vector, std::string const &header) {
            std::ofstream out(filename.c_str());
            if (!out.good())
                throw util::FileException(filename, std::strerror(errno));

            out << "Index, " << header << std::endl;
            for (std::size_t i = 0; i < vector.size(); i++) {
                out << i << ", " << vector[i] << std::endl;
            }
            out.close();
        }

        template<typename T>
        void vector_to_file(std::string const &filename, std::vector<T> const &vector) {
            std::ofstream out(filename.c_str(), std::ios::binary);
            if (!out.good())
                throw util::FileException(filename, std::strerror(errno));

            out.write(reinterpret_cast<const char *>(&vector[0]), vector.size() * sizeof(T));
            out.close();
        }

        template<typename T>
        std::vector<T> vector_from_file(std::string const &filename) {
            std::ifstream in(filename.c_str(), std::ios::binary);
            if (!in.good())
                throw util::FileException(filename, std::strerror(errno));
            in.seekg(0, in.end);
            const size_t filesize = in.tellg();
            in.seekg(0, in.beg);
            const size_t num_elements = filesize / sizeof(T);
            std::vector<T> vector(num_elements);
            in.read(reinterpret_cast<char *>(&vector[0]), num_elements * sizeof(T));
            in.close();
            return vector;
        }

        inline void write_string_to_file(std::string const &filename, std::string const &string) {
            std::ofstream out(filename.c_str());
            if (!out.good()) {
                throw util::FileException(filename, std::strerror(errno));
            }

            out << string;
            out.close();
        }

        inline math::Vec4f get_jet_color(float value) {
            assert(0.0f <= value && value <= 1.0f);
            float mvalue = 4 * value;
            float red = math::clamp(std::min(mvalue - 1.5f, -mvalue + 4.5f));
            float green = math::clamp(std::min(mvalue - 0.5f, -mvalue + 3.5f));
            float blue = math::clamp(std::min(mvalue + 0.5f, -mvalue + 2.5f));
            return math::Vec4f(red, green, blue, 1.0f);
        }
    }
}
#endif //TEXTURINGPIPELINE_UTILS_H
