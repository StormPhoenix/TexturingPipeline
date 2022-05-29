//
// Created by Storm Phoenix on 2021/10/11.
//

#ifndef TEXTURINGPIPELINE_VIEWSELECTION_H
#define TEXTURINGPIPELINE_VIEWSELECTION_H

#include <mve/mesh.h>
#include <acc/bvh_tree.h>

#include <MvsTexturing.h>
#include <Base/View.h>
#include <Base/FaceGroup.h>
#include <Base/SparseTable.h>

#include <set>
#include <vector>

namespace MvsTexturing {
    namespace ViewSelection {

        namespace RegionGrowing {
            void solve_RegionGrowing_problem(mve::TriangleMesh::ConstPtr mesh,
                                             const acc::BVHTree<unsigned int, math::Vec3f> &bvhTree,
                                             const Parameter &param, std::vector<Base::TextureView> &textureViews,
                                             Base::LabelGraph &graph);

        }

        namespace Projection {
            using namespace Base;

            void solve_projection_problem(mve::TriangleMesh::ConstPtr mesh, const mve::MeshInfo &mesh_info,
                                          const acc::BVHTree<unsigned int, math::Vec3f> &bvh_tree,
                                          const std::vector<Base::PlanarAreaFaceGroup> &planar_groups,
                                          Base::LabelGraph &graph, std::vector<Base::TextureView> &texture_views,
                                          const Parameter &param);

            void solve_projection_problem(mve::TriangleMesh::ConstPtr mesh, const mve::MeshInfo &mesh_info,
                                          const acc::BVHTree<unsigned int, math::Vec3f> &bvh_tree,
                                          Base::LabelGraph &graph,
                                          std::vector<Base::TextureView> &texture_views,
                                          const Parameter &param);

            void solve_OptimalPerFace(mve::TriangleMesh::ConstPtr mesh, const mve::MeshInfo &mesh_info,
                                      const acc::BVHTree<unsigned int, math::Vec3f> &bvh_tree,
                                      Base::LabelGraph &graph,
                                      std::vector<Base::TextureView> &texture_views,
                                      const Parameter &param);
        }

        namespace Mrf {
            using DataCosts = Base::SparseTable<std::uint32_t, std::uint16_t, double>;

            void calculate_data_costs(mve::TriangleMesh::ConstPtr mesh,
                                      const acc::BVHTree<unsigned int, math::Vec3f> &bvh_tree,
                                      std::vector<Base::TextureView> &texture_views,
                                      const Parameter &param,
                                      DataCosts *data_costs);

            void solve_mrf_problem(const DataCosts &data_costs, Base::LabelGraph &graph, const Parameter &param);

            void
            solveMultipleMrfProblem(mve::TriangleMesh::Ptr mesh, mve::MeshInfo &meshInfo, const Parameter &param,
                                    const acc::BVHTree<unsigned int, math::Vec3f> &bvhTree, Base::LabelGraph &graph,
                                    std::vector<Base::TextureView> &textureViews,
                                    std::vector<MvsTexturing::Base::PlanarAreaFaceGroup> &areas);
        }
    }
}

#endif //TEXTURINGPIPELINE_VIEWSELECTION_H
