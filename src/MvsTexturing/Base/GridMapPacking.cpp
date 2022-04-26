//
// Created by Storm Phoenix on 2021/12/25.
//

#include "GridMapPacking.h"

namespace MvsTexturing {
    namespace Base {
        GridPatch::GridPatch(unsigned int width, unsigned int height, unsigned int grid_size) :
                _grid_size(grid_size) {
            // upper-bound
            _grid_width = (unsigned int) (std::ceil(float(width) / grid_size));
            _grid_height = (unsigned int) (std::ceil(float(height) / grid_size));
            _map = mve::IntImage::create(_grid_width, _grid_height, 1);
            _map->fill(0);
        }

        int GridPatch::get_val(unsigned int grid_x, unsigned int grid_y) const {
            return _map->at(grid_x, grid_y, 0);
        }

        void GridPatch::update(int val, unsigned int grid_x, unsigned int grid_y) {
            _map->at(grid_x, grid_y, 0) = val;
        }

        GridPatch::Ptr GridPatch::create(std::size_t width, std::size_t height, std::size_t grid_size) {
            return std::make_shared<GridPatch>(width, height, grid_size);
        }

        GridMap::Ptr GridMap::create(std::size_t width, std::size_t height, std::size_t grid_size) {
            return std::make_shared<GridMap>(width, height, grid_size);
        }

        GridMap::GridMap(unsigned int width, unsigned int height, unsigned int grid_size) :
                _total_width(width), _total_height(height), _grid_size(grid_size) {

            // upper-bound
            _grid_width = (std::size_t) (std::ceil(float(_total_width) / _grid_size));
            _grid_height = (std::size_t) (std::ceil(float(_total_height) / _grid_size));

            _grid_width = std::max(_grid_width, std::size_t(1));
            _grid_height = std::max(_grid_height, std::size_t(1));

            _map = mve::IntImage::create(_grid_width, _grid_height, 1);
            _map->fill(0);
        }

        std::size_t compare_in_row_direction(
                const GridMap &grid_map, const GridPatch &grid_patch,
                const std::size_t y, const std::size_t x) {
            if (((x + grid_patch.grid_width() - 1) >= grid_map.grid_width()) ||
                ((y + grid_patch.grid_height() - 1) >= grid_map.grid_height())) {
                return 1;
            }

            for (std::size_t r = 0; r < grid_patch.grid_height(); r++) {
                for (std::size_t c = 0; c < grid_patch.grid_width(); c++) {
                    int map_offset = grid_map.map()->at(x + c, y + r, 0);
                    int patch_offset = grid_patch.map()->at(c, r, 0);
                    if ((map_offset != 0) && (patch_offset != 0)) {
                        // conflict
                        int offset = map_offset + patch_offset - 1;
                        return offset;
                    } else {
                        // not conflict
                        continue;
                    }
                }
            }
            return 0;
        }

        bool GridMap::insert(const GridPatch &patch, std::size_t &min_x, std::size_t &min_y) {
            // TODO 这种比较大小的位置，不要使用 unsigned
            for (std::size_t y = 0; y <= (_grid_height - patch.grid_height()); y++) {
                for (std::size_t x = 0; x <= (_grid_width - patch.grid_width());) {
                    std::size_t offset_in_x = compare_in_row_direction((*this), patch, y, x);
                    if (offset_in_x == 0) {
                        min_x = x * _grid_size;
                        min_y = y * _grid_size;
                        return true;
                    } else {
                        x += offset_in_x;
                    }
                }
            }
            return false;
        }

        void GridMap::occupy(int grid_x, int grid_y) {
            if (grid_x < 0 || grid_x >= _grid_width) {
                return;
            }

            if (grid_y < 0 || grid_y >= _grid_height) {
                return;
            }

            int grid_val = _map->at(grid_x, grid_y, 0);
            if (grid_val != 0) {
                return;
            }

            if (grid_x == (_grid_width - 1)) {
                _map->at(grid_x, grid_y, 0) = 1;
            } else {
                _map->at(grid_x, grid_y, 0) = _map->at(grid_x + 1, grid_y, 0) + 1;
            }

            for (int x = grid_x - 1; x >= 0; x--) {
                int val = _map->at(x, grid_y, 0);
                if (val == 0) {
                    break;
                } else {
                    _map->at(x, grid_y, 0) = _map->at(x + 1, grid_y, 0) + 1;
                }
            }
        }

        void GridMap::update(const GridPatch &patch, std::size_t x, std::size_t y) {
            std::size_t start_grid_x = x / _grid_size;
            std::size_t start_grid_y = y / _grid_size;

            std::size_t end_grid_x = std::min(_grid_width, start_grid_x + patch.grid_width());
            std::size_t end_grid_y = std::min(_grid_height, start_grid_y + patch.grid_height());

            for (int grid_x = start_grid_x; grid_x < end_grid_x; grid_x++) {
                for (int grid_y = start_grid_y; grid_y < end_grid_y; grid_y++) {
                    int offset_x_grid = grid_x - start_grid_x;
                    int offset_y_grid = grid_y - start_grid_y;

                    int patch_grid_val = patch.map()->at(offset_x_grid, offset_y_grid, 0);
                    int map_grid_val = _map->at(grid_x, grid_y, 0);

                    if (patch_grid_val != 0) {
                        if (map_grid_val != 0) {
                            continue;
                        } else {
                            occupy(grid_x, grid_y);
                        }
                    } else {
                        continue;
                    }
                }
            }
        }
    }
}