//
// Created by Storm Phoenix on 2021/10/11.
//

#ifndef TEXTURINGPIPELINE_VIEWSELECTION_H
#define TEXTURINGPIPELINE_VIEWSELECTION_H

#include <mve/mesh.h>

#include "Base/View.h"
#include "Utils/Settings.h"

#include <set>
#include <vector>

namespace MvsTexturing {
    namespace ViewSelection {
        void compute_face_camera_photometric(mve::TriangleMesh::ConstPtr mesh,
                                             std::vector<Base::TextureView> &texture_views,
                                             std::vector<std::set<std::size_t>> &face_visibility_sets,
                                             const Settings &settings);
    }
}

#endif //TEXTURINGPIPELINE_VIEWSELECTION_H
