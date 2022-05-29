//
// Created by Storm Phoenix on 2021/11/17.
//

#ifndef TEXTURINGPIPELINE_MESHSIMPLIFICATION_H
#define TEXTURINGPIPELINE_MESHSIMPLIFICATION_H

#include <mve/image.h>
#include <Base/FaceGroup.h>
#include <Parameter.h>

namespace MvsTexturing {
    namespace MeshRepair {
        const int kPlaneDensity = 400;

        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        using Vec3 = Eigen::Matrix<Scalar, 1, 3>;
        using Vec2 = Eigen::Matrix<Scalar, 1, 2>;
        using Vec4 = Eigen::Matrix<Scalar, 1, 4>;
        using FaceGroup = MvsTexturing::Base::PlanarAreaFaceGroup;

        class Mesh {
        public:
            Mesh() {}

        public:
            AttributeMatrix m_vertices;
            IndexMatrix m_faces;

            AttributeMatrix m_texture_coords;
            IndexMatrix m_face_texture_coord_ids;

            IndexMatrix m_face_material_ids;
        };

        typedef MvsTexturing::Base::TexturePatch::Ptr TexturePatchPtr;
        typedef std::vector<TexturePatchPtr> TexturePatchArray;
        typedef std::vector<FaceGroup> FaceGroups;
        typedef std::vector<std::vector<std::size_t>> FacesSubdivisions;

        typedef mve::ByteImage::Ptr ImagePtr;

        using FloatImageConstPtr = mve::FloatImage::ConstPtr;
        using TexturePatch = MvsTexturing::Base::TexturePatch;

        bool create_plane_patches_on_sparse_mesh(
                const Parameter &param, const AttributeMatrix &sparse_mesh_vertices,
                const IndexMatrix &sparse_mesh_faces, const std::vector<FaceGroup> &sparse_planar_groups,

                const AttributeMatrix &dense_mesh_vertices, const IndexMatrix &dense_mesh_faces,
                const std::vector<math::Vec2f> &dense_mesh_face_texture_coords,
                const std::vector<FloatImageConstPtr> &dense_mesh_face_materials,

                const FacesSubdivisions &faces_subdivision, std::vector<TexturePatch::Ptr> *final_texture_patches,
                std::size_t kPaddingPixels = Base::TexturePatch::kTexturePatchPadding,
                std::size_t plane_density = kPlaneDensity);

        bool create_irregular_patches_on_sparse_mesh(
                const AttributeMatrix &sparse_mesh_vertices, const IndexMatrix &sparse_mesh_faces,
                std::set<std::size_t> &irregular_patch_faces,
                const std::vector<math::Vec2f> &dense_texture_coords,
                const std::vector<FloatImageConstPtr> &dense_face_materials,
                const FacesSubdivisions &faces_subdivision,
                std::vector<TexturePatch::Ptr> *final_texture_patches,
                const std::size_t kPaddingPixels = Base::TexturePatch::kTexturePatchPadding,
                const std::size_t plane_density = kPlaneDensity);

        bool fit_face_group_plane(const AttributeMatrix &vertices, const IndexMatrix &faces,
                                  FaceGroup &group);

        bool remove_duplicate_faces(IndexMatrix &faces);

        bool remove_duplicate_faces(IndexMatrix &faces, AttributeMatrix &face_colors);
    }
}

#endif //TEXTURINGPIPELINE_MESHSIMPLIFICATION_H
