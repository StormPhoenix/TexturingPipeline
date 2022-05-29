//
// Created by Storm Phoenix on 2021/11/5.
//

#ifndef TEXTURINGPIPELINE_FACEOUTLIERDETECTION_H
#define TEXTURINGPIPELINE_FACEOUTLIERDETECTION_H

#include <vector>
#include <Base/View.h>

namespace MvsTexturing {
    namespace OutlierDetection {
        typedef std::vector<Base::FaceProjectionInfo> FaceProjectionInfoArray;

        bool detect_photometric_outliers(FaceProjectionInfoArray &infos, const Parameter &param,
                                         std::set<std::size_t> *outliers = nullptr);

        bool detect_outliers(FaceProjectionInfoArray &infos, const Parameter &param,
                             std::set<std::size_t> *outliers = nullptr);
    }
}

#endif //TEXTURINGPIPELINE_FACEOUTLIERDETECTION_H
