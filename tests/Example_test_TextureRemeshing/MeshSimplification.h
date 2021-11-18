//
// Created by Storm Phoenix on 2021/11/17.
//

#ifndef TEXTURINGPIPELINE_MESHSIMPLIFICATION_H
#define TEXTURINGPIPELINE_MESHSIMPLIFICATION_H

#include <mve/image.h>

namespace MeshSimplification {

    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
    typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

    using Vec3 = Eigen::Matrix<Scalar, 1, 3>;
    using Vec2 = Eigen::Matrix<Scalar, 1, 2>;
    using Vec4 = Eigen::Matrix<Scalar, 1, 4>;

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

    class FaceGroup {
    public:
        std::vector<std::size_t> m_faces;
        Vec3 m_plane_normal, m_plane_center;
        Vec3 m_x_axis, m_y_axis;
    };

    typedef MvsTexturing::Base::TexturePatch::Ptr TexturePatchPtr;
    typedef std::vector<TexturePatchPtr> TexturePatchArray;
    typedef std::vector<FaceGroup> FaceGroups;
    typedef std::vector<std::vector<std::size_t>> FacesSubdivisions;

    typedef mve::ByteImage::Ptr ImagePtr;

    bool simplify_mesh_texture(const Mesh &sparse_mesh, const Mesh &dense_mesh,
                               const std::vector<ImagePtr> &dense_mesh_materials,
                               const FacesSubdivisions &faces_subdivision,
                               const FaceGroups &sparse_mesh_face_groups,
                               TexturePatchArray *sparse_texture_patches,
                               std::size_t padding_pixels = 10,
                               const std::size_t plane_density = 300);
}

#endif //TEXTURINGPIPELINE_MESHSIMPLIFICATION_H
