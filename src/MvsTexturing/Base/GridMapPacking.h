//
// Created by Storm Phoenix on 2021/12/25.
//

#ifndef TEXTURINGPIPELINE_IRREGULARMAP_H
#define TEXTURINGPIPELINE_IRREGULARMAP_H

#include <mve/image.h>

namespace MvsTexturing {
    namespace Base {
        // GridPatch 向上溢出
        // GridMap 向下溢出
        const unsigned int kGridSize = 20;

        class GridPatch {
        public:
            typedef std::shared_ptr<GridPatch> Ptr;
            typedef std::shared_ptr<const GridPatch> ConstPtr;
        public:
            GridPatch(unsigned int width, unsigned int height, unsigned int grid_size);

            std::size_t grid_width() const {
                return _grid_width;
            }

            std::size_t grid_height() const {
                return _grid_height;
            }

            std::size_t grid_size() const {
                return _grid_size;
            }

            mve::IntImage::ConstPtr map() const {
                return _map;
            }

            int get_val(unsigned int grid_x, unsigned int grid_y) const;

            void update(int val, unsigned int grid_x, unsigned int grid_y);

            static Ptr create(std::size_t width, std::size_t height, std::size_t grid_size);

        private:
            mve::IntImage::Ptr _map;
            std::size_t _grid_width;
            std::size_t _grid_height;
            std::size_t _grid_size;
        };

        class GridMap {
        public:
            typedef std::shared_ptr<GridMap> Ptr;

        public:
            GridMap(unsigned int width, unsigned int height, unsigned int grid_size);

            bool insert(const GridPatch &patch, std::size_t &min_x, std::size_t &min_y);

            void update(const GridPatch &patch, std::size_t x, std::size_t y);

            std::size_t grid_width() const {
                return _grid_width;
            }

            std::size_t grid_height() const {
                return _grid_height;
            }

            mve::IntImage::ConstPtr map() const {
                return _map;
            }

            static Ptr create(std::size_t width, std::size_t height, std::size_t grid_size);

        private:
            void occupy(int grid_x, int grid_y);

        private:
            mve::IntImage::Ptr _map;
            std::size_t _grid_width;
            std::size_t _grid_height;
            std::size_t _grid_size;
            std::size_t _total_width;
            std::size_t _total_height;
        };
    }
}


#endif //TEXTURINGPIPELINE_IRREGULARMAP_H
