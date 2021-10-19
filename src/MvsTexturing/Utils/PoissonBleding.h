//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_POISSONBLEDING_H
#define TEXTURINGPIPELINE_POISSONBLEDING_H

#include <mve/image.h>

namespace MvsTexturing {
    namespace Utils {
        void poisson_blend(mve::FloatImage::ConstPtr src, mve::ByteImage::ConstPtr mask,
                      mve::FloatImage::Ptr dest, float alpha);
    }
}
#endif //TEXTURINGPIPELINE_POISSONBLEDING_H
