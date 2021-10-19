//
// Created by Storm Phoenix on 2021/10/15.
//

#ifndef TEXTURINGPIPELINE_RECT2D_H
#define TEXTURINGPIPELINE_RECT2D_H

#include <cassert>

namespace MvsTexturing {
    namespace Math {
        /**
        * Simple class representing a rectangle.
        */
        template<typename T>
        class Rect2D {
        public:
            T min_x;
            T min_y;
            T max_x;
            T max_y;

            Rect2D<T>(void);

            Rect2D<T>(Rect2D<T> *rect);

            Rect2D<T>(T min_x, T min_y, T max_x, T max_y);

            T width(void) const;

            T height(void) const;

            T size(void) const;

            /** Returns true if the rectangle is within or on the border of the given rectangle. */
            bool is_inside(Rect2D const *orect) const;

            /** Returns true if the rectangle intersects with the given rectangle. */
            bool intersect(Rect2D const *orect) const;

            void update(T min_x, T min_y, T max_x, T max_y);

            /** Moves the rectangle and updates its position. */
            void move(T x, T y);
        };

        template<typename T>
        Rect2D<T>::Rect2D(void) {
            update(0, 0, 1, 1);
        }

        template<typename T>
        Rect2D<T>::Rect2D(Rect2D<T> *rect) {
            update(rect->min_x, rect->min_y, rect->max_x, rect->max_y);
        }

        template<typename T>
        Rect2D<T>::Rect2D(T min_x, T min_y, T max_x, T max_y) {
            update(min_x, min_y, max_x, max_y);
        }

        template<typename T>
        inline T
        Rect2D<T>::width(void) const {
            return max_x - min_x;
        }

        template<typename T>
        inline T
        Rect2D<T>::height(void) const {
            return max_y - min_y;
        }

        template<typename T>
        inline T
        Rect2D<T>::size(void) const {
            return width() * height();
        }

        template<typename T>
        inline bool
        Rect2D<T>::is_inside(Rect2D const *rect) const {
            return
                    min_x >= rect->min_x &&
                    max_x <= rect->max_x &&
                    min_y >= rect->min_y &&
                    max_y <= rect->max_y;
        }

        template<typename T>
        inline void
        Rect2D<T>::update(T min_x, T min_y, T max_x, T max_y) {
            this->min_x = min_x;
            this->min_y = min_y;
            this->max_x = max_x;
            this->max_y = max_y;
            assert(min_x <= max_x);
            assert(min_y <= max_y);
        }

        template<typename T>
        inline bool
        Rect2D<T>::intersect(Rect2D<T> const *rect) const {
            return
                    min_x < rect->max_x && max_x > rect->min_x &&
                    min_y < rect->max_y && max_y > rect->min_y;
        }

        template<typename T>
        inline void
        Rect2D<T>::move(T x, T y) {
            update(x, y, x + width(), y + height());
        }
    }
}

#endif //TEXTURINGPIPELINE_RECT2D_H
