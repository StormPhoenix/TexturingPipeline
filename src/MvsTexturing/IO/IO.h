//
// Created by Storm Phoenix on 2021/10/18.
//

#ifndef TEXTURINGPIPELINE_IO_H
#define TEXTURINGPIPELINE_IO_H

#include <string>
#include <map>
#include <mve/mesh.h>
#include <mve/mesh_info.h>
#include <Base/TextureAtlas.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <mve/mesh_io_obj.h>

namespace MvsTexturing {
    namespace IO {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        void load_mesh_from_obj(std::string const &filename, std::vector<math::Vec3f> &V,
                                std::vector<math::Vec3f> &N, std::vector<math::Vec2f> &T,
                                std::vector<std::size_t> &F, std::vector<std::size_t> &FN,
                                std::vector<std::size_t> &FT, std::vector<std::string> &face_materials,
                                std::map<std::string, std::string> &material_map);

        bool load_mesh_from_obj(const std::string &filename, AttributeMatrix &V, AttributeMatrix &N,
                                AttributeMatrix &T, IndexMatrix &F, IndexMatrix &FN, IndexMatrix &FT,
                                std::vector<std::string> &face_materials,
                                 std::map<std::string, std::string> &material_map);

        bool load_mesh_from_ply(const std::string &filename, AttributeMatrix &V, IndexMatrix &F);

        bool save_mesh(const std::string &file_name, const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

        namespace MVE {
            mve::TriangleMesh::Ptr load_ply_mesh(const std::string &filename);

            void save_obj_mesh(const std::string &filename, mve::TriangleMesh::ConstPtr mesh,
                               const std::vector<Base::TextureAtlas::Ptr> &texture_atlases);
        }
    }
}

#endif //TEXTURINGPIPELINE_IO_H
