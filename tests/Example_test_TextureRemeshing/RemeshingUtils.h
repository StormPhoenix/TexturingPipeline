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

        bool create_plane_patches(const MeshPolyRefinement::Base::TriMesh &mesh,
                                  const AttributeMatrix &global_texcoords,
                                  const IndexMatrix &global_texcoord_ids,
                                  std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                  std::size_t padding_pixels = 10,
                                  const std::size_t plane_density = 300);

        bool create_plane_patches(const MeshPolyRefinement::Base::TriMesh &mesh,
                                  const AttributeMatrix &global_texcoords,
                                  const IndexMatrix &global_texcoord_ids,
                                  const std::vector<std::string> &face_materials,
                                  const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                  std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                  const std::size_t padding_pixels = 10,
                                  const std::size_t plane_density = 300);

        bool create_irregular_patches(const MeshPolyRefinement::Base::TriMesh &mesh,
                                      const AttributeMatrix &global_texcoords,
                                      const IndexMatrix &global_texcoord_ids,
                                      std::vector<std::set<std::size_t>> &none_plane_group_faces,
                                      const std::vector<std::string> &face_materials,
                                      const std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                      std::vector<MvsTexturing::Base::TexturePatch::Ptr> *texture_patches,
                                      const std::size_t kPaddingPixels = 10,
                                      const std::size_t kPlaneDensity = 300);
    }
}

#endif //TEXTURINGPIPELINE_REMESHINGUTILS_H
