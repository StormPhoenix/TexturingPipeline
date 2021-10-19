//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_RECTBIN_H
#define TEXTURINGPIPELINE_RECTBIN_H

#include <list>
#include <memory>

#include "Math/Rect2D.h"

namespace MvsTexturing {
    namespace Base {
        class RectBin {
        public:
            typedef std::shared_ptr<RectBin> Ptr;

        private:
            unsigned int width;
            unsigned int height;
            std::list<Math::Rect2D<int> > rects;

        public:
            /**
              * Initializes the rectangular binpacking algorithm to fill a rectangle of the given size.
              */
            RectBin(unsigned int width, unsigned int height);

            static RectBin::Ptr create(unsigned int width, unsigned int height);

            /** Returns true and changes the position of the given rect if it fits into the bin. */
            bool insert(Math::Rect2D<int> *rect);
        };

        inline RectBin::Ptr
        RectBin::create(unsigned int width, unsigned int height) {
            return Ptr(new RectBin(width, height));
        }
    }
}

#endif //TEXTURINGPIPELINE_RECTBIN_H
