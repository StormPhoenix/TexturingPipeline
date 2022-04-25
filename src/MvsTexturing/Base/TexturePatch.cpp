//
// Created by Storm Phoenix on 2021/10/17.
//

#include <set>

#include <common.h>

#include <math/functions.h>
#include <mve/image_color.h>
#include <mve/image_tools.h>
#include <mve/mesh_io_ply.h>

#include "TexturePatch.h"
#include "Utils/Utils.h"
#include "Utils/PoissonBleding.h"

namespace MvsTexturing {
    namespace Base {
        TexturePatch::TexturePatch(int label, std::vector<std::size_t> const &faces,
                                   std::vector<math::Vec2f> const &texcoords, mve::FloatImage::Ptr image)
                : label(label), faces(faces), texcoords(texcoords), image(image) {

            validity_mask = mve::ByteImage::create(get_width(), get_height(), 1);
            validity_mask->fill(255);
            blending_mask = mve::ByteImage::create(get_width(), get_height(), 1);
        }

        TexturePatch::TexturePatch(TexturePatch const &texture_patch) {
            label = texture_patch.label;
            faces = std::vector<std::size_t>(texture_patch.faces);
            texcoords = std::vector<math::Vec2f>(texture_patch.texcoords);
            image = texture_patch.image->duplicate();
            validity_mask = texture_patch.validity_mask->duplicate();
            if (texture_patch.blending_mask != NULL) {
                blending_mask = texture_patch.blending_mask->duplicate();
            }
        }

        const float sqrt_2 = sqrt(2);

        void
        TexturePatch::adjust_colors(std::vector<math::Vec3f> const &adjust_values) {
            assert(blending_mask != NULL);

            validity_mask->fill(0);

            mve::FloatImage::Ptr iadjust_values = mve::FloatImage::create(get_width(), get_height(), 3);
            for (std::size_t i = 0; i < texcoords.size(); i += 3) {
                math::Vec2f v1 = texcoords[i];
                math::Vec2f v2 = texcoords[i + 1];
                math::Vec2f v3 = texcoords[i + 2];

                Math::Tri2D tri(v1, v2, v3);

                float area = tri.get_area();
                if (area < std::numeric_limits<float>::epsilon()) continue;

                Math::Rect2D<float> aabb = tri.get_aabb();
                int const min_x = static_cast<int>(std::floor(aabb.min_x)) - texture_patch_border;
                int const min_y = static_cast<int>(std::floor(aabb.min_y)) - texture_patch_border;
                int const max_x = static_cast<int>(std::ceil(aabb.max_x)) + texture_patch_border;
                int const max_y = static_cast<int>(std::ceil(aabb.max_y)) + texture_patch_border;

                assert(0 <= min_x && max_x <= get_width());
                assert(0 <= min_y && max_y <= get_height());

                for (int y = min_y; y < max_y; ++y) {
                    for (int x = min_x; x < max_x; ++x) {

                        math::Vec3f bcoords = tri.get_barycentric_coords(x, y);
                        bool inside = bcoords.minimum() >= 0.0f;
                        if (inside) {
                            assert(x != 0 && y != 0);
                            for (int c = 0; c < 3; ++c) {
                                iadjust_values->at(x, y, c) = math::interpolate(
                                        adjust_values[i][c], adjust_values[i + 1][c], adjust_values[i + 2][c],
                                        bcoords[0], bcoords[1], bcoords[2]);
                            }
                            validity_mask->at(x, y, 0) = 255;
                            blending_mask->at(x, y, 0) = 255;
                        } else {

                            if (validity_mask->at(x, y, 0) == 255)
                                continue;

                            /* Check whether the pixels distance from the triangle is more than one pixel. */
                            float ha = 2.0f * -bcoords[0] * area / (v2 - v3).norm();
                            float hb = 2.0f * -bcoords[1] * area / (v1 - v3).norm();
                            float hc = 2.0f * -bcoords[2] * area / (v1 - v2).norm();

                            if (ha > sqrt_2 || hb > sqrt_2 || hc > sqrt_2)
                                continue;

                            for (int c = 0; c < 3; ++c) {
                                iadjust_values->at(x, y, c) = math::interpolate(
                                        adjust_values[i][c], adjust_values[i + 1][c], adjust_values[i + 2][c],
                                        bcoords[0], bcoords[1], bcoords[2]);
                            }
                            validity_mask->at(x, y, 0) = 255;
                            blending_mask->at(x, y, 0) = 64;
                        }
                    }
                }
            }

            for (int i = 0; i < image->get_pixel_amount(); ++i) {
                if (validity_mask->at(i, 0) != 0) {
                    for (int c = 0; c < 3; ++c) {
                        image->at(i, c) += iadjust_values->at(i, c);
                    }
                } else {
                    math::Vec3f color(0.0f, 0.0f, 0.0f);
                    //DEBUG math::Vec3f color(1.0f, 0.0f, 1.0f);
                    std::copy(color.begin(), color.end(), &image->at(i, 0));
                }
            }
        }

        bool TexturePatch::valid_pixel(math::Vec2f pixel) const {
            float x = pixel[0];
            float y = pixel[1];

            float const height = static_cast<float>(get_height());
            float const width = static_cast<float>(get_width());

            bool valid = (0.0f <= x && x < width && 0.0f <= y && y < height);
            if (valid && validity_mask != NULL) {
                /* Only pixel which can be correctly interpolated are valid. */
                float cx = std::max(0.0f, std::min(width - 1.0f, x));
                float cy = std::max(0.0f, std::min(height - 1.0f, y));
                int const floor_x = static_cast<int>(cx);
                int const floor_y = static_cast<int>(cy);
                int const floor_xp1 = std::min(floor_x + 1, get_width() - 1);
                int const floor_yp1 = std::min(floor_y + 1, get_height() - 1);

                float const w1 = cx - static_cast<float>(floor_x);
                float const w0 = 1.0f - w1;
                float const w3 = cy - static_cast<float>(floor_y);
                float const w2 = 1.0f - w3;

                valid = (w0 * w2 == 0.0f || validity_mask->at(floor_x, floor_y, 0) == 255) &&
                        (w1 * w2 == 0.0f || validity_mask->at(floor_xp1, floor_y, 0) == 255) &&
                        (w0 * w3 == 0.0f || validity_mask->at(floor_x, floor_yp1, 0) == 255) &&
                        (w1 * w3 == 0.0f || validity_mask->at(floor_xp1, floor_yp1, 0) == 255);
            }

            return valid;
        }

        bool
        TexturePatch::valid_pixel(math::Vec2i pixel) const {
            int const x = pixel[0];
            int const y = pixel[1];

            bool valid = (0 <= x && x < get_width() && 0 <= y && y < get_height());
            if (valid && validity_mask != NULL) {
                valid = validity_mask->at(x, y, 0) == 255;
            }

            return valid;
        }

        math::Vec3f
        TexturePatch::get_pixel_value(math::Vec2f pixel) const {
            assert(valid_pixel(pixel));

            math::Vec3f color;
            image->linear_at(pixel[0], pixel[1], *color);
            return color;
        }

        void TexturePatch::set_pixel_value(math::Vec2i pixel, math::Vec3f color) {
            assert(blending_mask != NULL);
            assert(valid_pixel(pixel));

            std::copy(color.begin(), color.end(), &image->at(pixel[0], pixel[1], 0));
            blending_mask->at(pixel[0], pixel[1], 0) = 128;
        }

        void TexturePatch::blend(mve::FloatImage::ConstPtr orig) {
            Utils::poisson_blend(orig, blending_mask, image, 1.0f);

            /* Invalidate all pixels outside the boundary. */
            for (int y = 0; y < blending_mask->height(); ++y) {
                for (int x = 0; x < blending_mask->width(); ++x) {
                    if (blending_mask->at(x, y, 0) == 64) {
                        validity_mask->at(x, y, 0) = 0;
                    }
                }
            }
        }

        typedef std::vector<std::pair<int, int> > PixelVector;
        typedef std::set<std::pair<int, int> > PixelSet;

        void TexturePatch::prepare_blending_mask(std::size_t strip_width) {
            int const width = blending_mask->width();
            int const height = blending_mask->height();

            /* Calculate the set of valid pixels at the border of texture patch. */
            PixelSet valid_border_pixels;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    if (validity_mask->at(x, y, 0) == 0) continue;

                    /* Valid border pixels need no invalid neighbours. */
                    if (x == 0 || x == width - 1 || y == 0 || y == height - 1) {
                        valid_border_pixels.insert(std::pair<int, int>(x, y));
                        continue;
                    }

                    /* Check the direct neighbourhood of all invalid pixels. */
                    for (int j = -1; j <= 1; ++j) {
                        for (int i = -1; i <= 1; ++i) {
                            int nx = x + i;
                            int ny = y + j;
                            /* If the valid pixel has a invalid neighbour: */
                            if (validity_mask->at(nx, ny, 0) == 0) {
                                /* Add the pixel to the set of valid border pixels. */
                                valid_border_pixels.insert(std::pair<int, int>(x, y));
                            }
                        }
                    }
                }
            }

            mve::ByteImage::Ptr inner_pixel = validity_mask->duplicate();

            /* Iteratively erode all border pixels. */
            for (std::size_t i = 0; i < strip_width; ++i) {
                PixelVector new_invalid_pixels(valid_border_pixels.begin(), valid_border_pixels.end());
                PixelVector::iterator it;
                valid_border_pixels.clear();

                /* Mark the new invalid pixels invalid in the validity mask. */
                for (it = new_invalid_pixels.begin(); it != new_invalid_pixels.end(); ++it) {
                    int x = it->first;
                    int y = it->second;

                    inner_pixel->at(x, y, 0) = 0;
                }

                /* Calculate the set of valid pixels at the border of the valid area. */
                for (it = new_invalid_pixels.begin(); it != new_invalid_pixels.end(); ++it) {
                    int x = it->first;
                    int y = it->second;

                    for (int j = -1; j <= 1; j++) {
                        for (int i = -1; i <= 1; i++) {
                            int nx = x + i;
                            int ny = y + j;
                            if (0 <= nx && nx < width &&
                                0 <= ny && ny < height &&
                                inner_pixel->at(nx, ny, 0) == 255) {

                                valid_border_pixels.insert(std::pair<int, int>(nx, ny));
                            }
                        }
                    }
                }
            }

            /* Sanitize blending mask. */
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    // 128 代表 patch 的边界
                    if (blending_mask->at(x, y, 0) == 128) {
                        uint8_t n[] = {blending_mask->at(x - 1, y, 0),
                                       blending_mask->at(x + 1, y, 0),
                                       blending_mask->at(x, y - 1, 0),
                                       blending_mask->at(x, y + 1, 0)
                        };
                        bool valid = true;
                        for (uint8_t v: n) {
                            if (v == 255) continue;
                            valid = false;
                        }
                        if (valid) blending_mask->at(x, y, 0) = 255;
                    }
                }
            }

            // blending_mask 代表可绘制的地方
            // 128 代表边界 0 代表内部
            // 猜测边界内部就是可绘制的地方
            /* Mark all remaining pixels invalid in the blending_mask. */
            for (int i = 0; i < inner_pixel->get_pixel_amount(); ++i) {
                if (inner_pixel->at(i) == 255) blending_mask->at(i) = 0;
            }

            /* Mark all border pixels. */
            PixelSet::iterator it;
            for (it = valid_border_pixels.begin(); it != valid_border_pixels.end(); ++it) {
                int x = it->first;
                int y = it->second;

                blending_mask->at(x, y, 0) = 128;
            }
        }

        namespace __inner__ {
            const int kDirectionVertical = 0;
            const int kDirectionHorizontal = 1;

            struct area1d {
                int start = -1;
                int length = -1;
                bool init = false;
            };

            enum order {
                FIRST,
                SECOND
            };

            order choose_belong_area(double *data, int size, int boundary) {
                double max_first_area = -1.0;
                double max_second_area = -1.0;

                for (std::size_t i = 0; i < size; i++) {
                    double coord = data[i];

                    if (coord <= double(boundary)) {
                        // first area
                        max_first_area = std::max(max_first_area, (double(boundary) - coord));
                    } else {
                        // second area
                        max_second_area = std::max(max_second_area, (coord - double(boundary)));
                    }
                }

                if (max_first_area > max_second_area) {
                    return FIRST;
                } else {
                    return SECOND;
                }
            }

            void update_area(double *data, int size, area1d &area) {
                if (size == 0) {
                    return;
                }

                double _min = data[0];
                double _max = data[0];
                for (int i = 1; i < size; i++) {
                    _min = std::min(_min, data[i]);
                    _max = std::max(_max, data[i]);
                }

                if (!area.init) {
                    area.start = static_cast<int>(std::floor(_min));
                    area.length = static_cast<int>(std::ceil(_max)) - area.start + 1;
                    area.init = true;
                    return;
                }

                double area_start = area.start;
                double area_end = area_start + area.length - 1;

                area_end = std::max(area_end, _max);
                area_start = std::min(area_start, _min);

                area.start = static_cast<int>(std::floor(area_start));
                area.length = static_cast<int>(std::ceil(area_end)) - area.start + 1;
            }
        }

        bool TexturePatch::split(std::vector<TexturePatch::Ptr> &results, int direction) {
            int mid = 0;
            int texcoord_channel = 0;
            int other_texcoord_channel = 0;
            std::size_t total_patch_size = 0;
            if (direction == __inner__::kDirectionVertical) {
                mid = get_height() / 2;
                texcoord_channel = 1;
                other_texcoord_channel = 0;
                total_patch_size = get_height();
            } else if (direction == __inner__::kDirectionHorizontal) {
                mid = get_width() / 2;
                texcoord_channel = 0;
                other_texcoord_channel = 1;
                total_patch_size = get_width();
            } else {
                return false;
            }

            __inner__::area1d first_area, second_area;
            std::vector<std::size_t> first_face_indices;
            std::vector<math::Vec2f> first_texcoords;

            std::vector<std::size_t> second_face_indices;
            std::vector<math::Vec2f> second_texcoords;

            for (std::size_t face_index = 0; face_index < faces.size(); face_index++) {
                std::size_t face_id = faces[face_index];

                double coord_data[3];
                for (int i = 0; i < 3; i++) {
                    coord_data[i] = texcoords[face_index * 3 + i][texcoord_channel];
                }

                __inner__::order selected_area = __inner__::choose_belong_area(coord_data, 3, mid);
                if (selected_area == __inner__::FIRST) {
                    first_face_indices.push_back(face_id);
                    for (std::size_t i = 0; i < 3; i++) {
                        first_texcoords.push_back(texcoords[face_index * 3 + i]);
                    }
                    __inner__::update_area(coord_data, 3, first_area);
                } else {
                    second_face_indices.push_back(face_id);
                    for (std::size_t i = 0; i < 3; i++) {
                        second_texcoords.push_back(texcoords[face_index * 3 + i]);
                    }
                    __inner__::update_area(coord_data, 3, second_area);
                }
            }

            int overlap_size = (first_area.start + first_area.length - 1) - (second_area.start) + 1;
            double overlap_rate = double(overlap_size) / double(total_patch_size);
            if (overlap_size >= 0 && overlap_rate > 0.5) {
                return false;
            }

            if (!first_area.init || !second_area.init) {
                return false;
            }

            if ((first_area.init &&
                 (first_area.start < 0 || (first_area.start + first_area.length) > total_patch_size)) ||
                (second_area.init &&
                 (second_area.start < 0 || (second_area.start + second_area.length) > total_patch_size))
                    ) {
                LOG_ERROR(" - split patch out of range");
                return false;
            }

            // compute another direction range
            int first_other_direction_start = std::max(get_width(), get_height());
            int first_other_direction_end = -1;
            int second_other_direction_start = std::max(get_width(), get_height());
            int second_other_direction_end = -1;
            {

                if (first_area.init) {
                    for (std::size_t i = 0; i < first_texcoords.size(); i++) {
                        double coord = first_texcoords[i][other_texcoord_channel];
                        first_other_direction_start = std::min(
                                first_other_direction_start, static_cast<int>(std::floor(coord)));

                        first_other_direction_end = std::max(
                                first_other_direction_end, static_cast<int>(std::ceil(coord)));
                    }
                }

                if (second_area.init) {
                    for (std::size_t i = 0; i < second_texcoords.size(); i++) {
                        double coord = second_texcoords[i][other_texcoord_channel];
                        second_other_direction_start = std::min(
                                second_other_direction_start, static_cast<int>(std::floor(coord)));

                        second_other_direction_end = std::max(
                                second_other_direction_end, static_cast<int>(std::ceil(coord)));
                    }
                }

                int kOtherDirectionMaxSize = 0;
                if (direction == __inner__::kDirectionVertical) {
                    kOtherDirectionMaxSize = get_width() - 1;
                } else if (direction == __inner__::kDirectionHorizontal) {
                    kOtherDirectionMaxSize = get_height() - 1;
                } else {
                    LOG_ERROR(" - split direction error");
                }

                first_other_direction_start = Utils::clamp(first_other_direction_start, 0, kOtherDirectionMaxSize);
                first_other_direction_end = Utils::clamp(first_other_direction_end, 0, kOtherDirectionMaxSize);

                second_other_direction_start = Utils::clamp(second_other_direction_start, 0, kOtherDirectionMaxSize);
                second_other_direction_end = Utils::clamp(second_other_direction_end, 0, kOtherDirectionMaxSize);
            }

            // create first patch image
            TexturePatch::Ptr first_sub_patch = nullptr;
            if (first_area.init) {
                if (direction == __inner__::kDirectionVertical) {
                    first_sub_patch = create_sub_patch(
                            first_other_direction_start, first_other_direction_end,
                            first_area.start, first_area.start + first_area.length - 1,
                            first_face_indices, first_texcoords);
                } else if (direction == __inner__::kDirectionHorizontal) {
                    first_sub_patch = create_sub_patch(
                            first_area.start, first_area.start + first_area.length - 1,
                            first_other_direction_start, first_other_direction_end,
                            first_face_indices, first_texcoords);
                }

            }

            if (first_sub_patch != nullptr) {
                results.push_back(first_sub_patch);
            }

            // create right patch image
            TexturePatch::Ptr second_sub_patch = nullptr;
            if (second_area.init) {
                if (direction == __inner__::kDirectionVertical) {
                    second_sub_patch = create_sub_patch(
                            second_other_direction_start, second_other_direction_end, second_area.start,
                            second_area.start + second_area.length - 1,
                            second_face_indices, second_texcoords);
                } else if (direction == __inner__::kDirectionHorizontal) {
                    second_sub_patch = create_sub_patch(
                            second_area.start, second_area.start + second_area.length - 1,
                            second_other_direction_start, second_other_direction_end,
                            second_face_indices, second_texcoords);
                }
            }

            if (second_sub_patch != nullptr) {
                results.push_back(second_sub_patch);
            }

            if (first_sub_patch == nullptr && second_sub_patch == nullptr) {
                return false;
            } else {
                return true;
            }
        }

        bool TexturePatch::split_vertical(std::vector<TexturePatch::Ptr> &results) {
            return split(results, __inner__::kDirectionVertical);
        }

        bool TexturePatch::split_horizontal(std::vector<TexturePatch::Ptr> &results) {
            return split(results, __inner__::kDirectionHorizontal);
        }

        TexturePatch::Ptr TexturePatch::create_sub_patch(
                int left, int right, int bottom, int top, std::vector<size_t> &face_indices,
                std::vector<math::Vec2f> &texcoords) {
            if (face_indices.size() <= 0 || texcoords.size() <= 0) {
                return nullptr;
            }

            if (left > right || bottom > top) {
                return nullptr;
            }

            int min_x, min_y;
            int max_x, max_y;

            // considering padding
            min_x = std::max((left - kTexturePatchPadding), 0);
            max_x = std::min((right + kTexturePatchPadding), get_width() - 1);

            min_y = std::max((bottom - kTexturePatchPadding), 0);
            max_y = std::min((top + kTexturePatchPadding), get_height() - 1);

            int patch_width, patch_height;
            patch_width = max_x - min_x + 1;
            patch_height = max_y - min_y + 1;

            mve::FloatImage::Ptr crop_image = mve::image::crop(
                    image, patch_width, patch_height, min_x, min_y, *math::Vec3f(1.0, 0, 1.0));

            // modify texture coords
            math::Vec2f _min(min_x, min_y);
            for (std::size_t i = 0; i < texcoords.size(); i++) {
                texcoords[i] = texcoords[i] - _min;
            }

            return create(0, face_indices, texcoords, crop_image);
        }

        void TexturePatch::rescale(float factor) {
            int scale_width = get_width() * factor;
            int scale_height = get_height() * factor;

            // rescale image
            {
                mve::FloatImage::Ptr tmp_image = mve::FloatImage::create(scale_width, scale_height, image->channels());
                mve::image::rescale_linear<float>(image, tmp_image);
                image.reset();
                image = tmp_image;
            }

            // rescale validity mask
            {
                mve::ByteImage::Ptr tmp_image = mve::ByteImage::create(scale_width, scale_height,
                                                                       validity_mask->channels());
                mve::image::rescale_nearest<uint8_t>(validity_mask, tmp_image);
                validity_mask.reset();
                validity_mask = tmp_image;
            }

            // rescale blending mask
            {
                mve::ByteImage::Ptr tmp_image = mve::ByteImage::create(scale_width, scale_height,
                                                                       blending_mask->channels());
                mve::image::rescale_nearest<uint8_t>(blending_mask, tmp_image);
                blending_mask.reset();
                blending_mask = tmp_image;
            }

            // rescale texcoords
            {
                for (int var_i = 0; var_i < texcoords.size(); var_i++) {
                    texcoords[var_i] *= factor;
                }
            }
        }

        void TexturePatch::generate_validity_map() {
            validity_map = GridPatch::create(validity_mask->width(), validity_mask->height(), kGridSize);
            for (int grid_y = 0; grid_y < validity_map->grid_height(); grid_y++) {
                for (int grid_x = 0; grid_x < validity_map->grid_width(); grid_x++) {
                    // calculate offset
                    int start_offset_x = grid_x * kGridSize;
                    int start_offset_y = grid_y * kGridSize;

                    bool is_occupied = false;
                    for (int offset_i = 0; offset_i < kGridSize; offset_i++) {
                        int offset_x = offset_i + start_offset_x;
                        if (offset_x >= validity_mask->width()) {
                            break;
                        }

                        for (int offset_j = 0; offset_j < kGridSize; offset_j++) {
                            int offset_y = offset_j + start_offset_y;
                            if (offset_y >= validity_mask->height()) {
                                break;
                            }

                            unsigned char mask_val = validity_mask->at(offset_x, offset_y, 0);
                            if (mask_val == 255) {
                                is_occupied = true;
                                break;
                            }
                        }

                        if (is_occupied) {
                            break;
                        }
                    }

                    if (is_occupied) {
                        if (grid_x == 0) {
                            validity_map->update(1, grid_x, grid_y);
                        } else {
                            int val = validity_map->get_val(grid_x - 1, grid_y);
                            validity_map->update(val + 1, grid_x, grid_y);
                        }
                    }
                }
            }
        }
    }
}