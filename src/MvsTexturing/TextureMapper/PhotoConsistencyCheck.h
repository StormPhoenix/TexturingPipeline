//
// Created by Storm Phoenix on 2021/10/20.
//

#ifndef TEXTURINGPIPELINE_PHOTOCONSISTENCYCHECK_H
#define TEXTURINGPIPELINE_PHOTOCONSISTENCYCHECK_H

#include <set>
#include <vector>

#include "Base/View.h"
#include "Utils/Settings.h"

namespace MvsTexturing {
    namespace PhotoMetric {
        bool photo_consistency_check(std::vector<Base::FaceProjectionInfo> *infos,
                                     std::set<std::size_t> *face_visibility, const Settings &settings);
    }
}

#endif //TEXTURINGPIPELINE_PHOTOCONSISTENCYCHECK_H
