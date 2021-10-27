//
// Created by Storm Phoenix on 2021/10/22.
//

#ifndef TEXTURINGPIPELINE_REMESHINGUTILS_H
#define TEXTURINGPIPELINE_REMESHINGUTILS_H

#include <Base/TriMesh.h>
#include <mve/mesh.h>

namespace TextureRemeshing {
    namespace Utils {
        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        mve::TriangleMesh::Ptr triMesh_to_mveMesh(MeshPolyRefinement::Base::TriMesh &tri_mesh);

        bool remeshing_from_plane_groups(const MeshPolyRefinement::Base::TriMesh &mesh,
                                         const std::vector<math::Vec2f> &global_texcoords,
                                         const std::vector<std::size_t> &global_texcoord_ids,
                                         const std::vector<std::string> &face_materials,
                                         const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                         std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                         const std::size_t padding_pixels = 10,
                                         const std::size_t plane_density = 300);

        bool remeshing_from_plane_groups(const MeshPolyRefinement::Base::TriMesh &mesh,
                                         const AttributeMatrix &global_texcoords,
                                         const IndexMatrix &global_texcoord_ids,
                                         const std::vector<std::string> &face_materials,
                                         const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                         std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                         const std::size_t padding_pixels = 10,
                                         const std::size_t plane_density = 300);
    }
}

#endif //TEXTURINGPIPELINE_REMESHINGUTILS_H
