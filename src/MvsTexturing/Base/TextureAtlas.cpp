//
// Created by Storm Phoenix on 2021/10/17.
//

#include <set>
#include <map>
#include <common.h>

#include <util/file_system.h>
#include <mve/image_tools.h>
#include <mve/image_io.h>

#include "RectBin.h"
#include "TextureAtlas.h"

namespace MvsTexturing {
    namespace Base {
        TextureAtlas::TextureAtlas(unsigned int size) :
//                size(size), padding(size >> 7), finalized(false) {
                size(size), padding(kTextureAtlasPadding), finalized(false) {
            bin = RectBin::create(size, size);
            image = mve::ByteImage::create(size, size, 3);
            validity_mask = mve::ByteImage::create(size, size, 1);
        }

        /**
        * Copies the src image into the dest image at the given position,
        * optionally adding a border.
        * @warning asserts that the given src image fits into the given dest image.
        */
        void copy_into(mve::ByteImage::ConstPtr src, int x, int y,
                       mve::ByteImage::Ptr dest, int border = 0) {

            assert(x >= 0 && x + src->width() + 2 * border <= dest->width());
            assert(y >= 0 && y + src->height() + 2 * border <= dest->height());

            for (int i = 0; i < src->width() + 2 * border; ++i) {
                for (int j = 0; j < src->height() + 2 * border; j++) {
                    int sx = i - border;
                    int sy = j - border;

                    if (sx < 0 || sx >= src->width() || sy < 0 || sy >= src->height())
                        continue;

                    for (int c = 0; c < src->channels(); ++c) {
                        dest->at(x + i, y + j, c) = src->at(sx, sy, c);
                    }
                }
            }
        }

        void copy_into_by_mask(mve::ByteImage::ConstPtr src, int x, int y,
                               const GridPatch &mask, mve::ByteImage::Ptr dest) {
            for (std::size_t grid_x = 0; grid_x < mask.grid_width(); grid_x++) {
                for (std::size_t grid_y = 0; grid_y < mask.grid_height(); grid_y++) {
                    if (mask.get_val(grid_x, grid_y) != 0) {
                        // occupied
                        std::size_t start_offset_x = grid_x * mask.grid_size();
                        std::size_t start_offset_y = grid_y * mask.grid_size();

                        for (int j = 0; j < mask.grid_size(); j++) {
                            int offset_y = j + start_offset_y;
                            if (offset_y >= src->height()) {
                                break;
                            }
                            int dest_y = y + offset_y;
                            if (dest_y >= dest->height()) {
                                break;
                            }

                            for (int i = 0; i < mask.grid_size(); i++) {
                                int offset_x = i + start_offset_x;
                                if (offset_x >= src->width()) {
                                    break;
                                }

                                int dest_x = x + offset_x;
                                if (dest_x >= dest->width()) {
                                    break;
                                }

                                for (int c = 0; c < src->channels(); c++) {
                                    dest->at(dest_x, dest_y, c) = src->at(offset_x, offset_y, c);
                                }
                            }
                        }
                    }
                }
            }
        }

        typedef std::vector<std::pair<int, int> > PixelVector;
        typedef std::set<std::pair<int, int> > PixelSet;

        bool TextureAtlas::insert(TexturePatch::ConstPtr texture_patch) {
            if (finalized) {
                throw util::Exception("No insertion possible, TextureAtlas already finalized");
            }

            assert(bin != NULL);
            assert(validity_mask != NULL);

            int const width = texture_patch->get_width() + 2 * padding;
            int const height = texture_patch->get_height() + 2 * padding;
            Math::Rect2D<int> rect(0, 0, width, height);
            if (!bin->insert(&rect)) return false;

            /* Update texture atlas and its validity mask. */
            mve::ByteImage::Ptr patch_image = mve::image::float_to_byte_image(
                    texture_patch->get_image(), 0.0f, 1.0f);

            copy_into(patch_image, rect.min_x, rect.min_y, image, padding);
            mve::ByteImage::ConstPtr patch_validity_mask = texture_patch->get_validity_mask();
            copy_into(patch_validity_mask, rect.min_x, rect.min_y, validity_mask, padding);

            TexturePatch::Faces const &patch_faces = texture_patch->get_faces();
            TexturePatch::Texcoords const &patch_texcoords = texture_patch->get_texcoords();

            /* Calculate the offset of the texture patches' relative texture coordinates */
            math::Vec2f offset = math::Vec2f(rect.min_x + padding, rect.min_y + padding);

            faces.insert(faces.end(), patch_faces.begin(), patch_faces.end());

            /* Calculate the final textcoords of the faces. */
            for (std::size_t i = 0; i < patch_faces.size(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    math::Vec2f rel_texcoord(patch_texcoords[i * 3 + j]);
                    math::Vec2f texcoord = rel_texcoord + offset;

                    texcoord[0] = texcoord[0] / this->size;
                    texcoord[1] = texcoord[1] / this->size;
                    texcoords.push_back(texcoord);
                }
            }
            return true;
        }

        bool TextureAtlas::insert_grid(TexturePatch::ConstPtr texture_patch) {
            if (finalized) {
                throw util::Exception("No insertion possible, TextureAtlas already finalized");
            }

            assert(bin != NULL);
            assert(validity_mask != NULL);
            assert(validity_map != NULL);

//            int const width = texture_patch->get_width() + 2 * padding;
//            int const height = texture_patch->get_height() + 2 * padding;
//            Math::Rect2D<int> rect(0, 0, width, height);
//            if (!bin->insert(&rect)) return false;

            std::size_t min_x, min_y;
            if (!validity_map->insert(*texture_patch->get_validity_map(), min_x, min_y)) {
                return false;
            }

            /* Update texture atlas and its validity mask. */
            mve::ByteImage::Ptr patch_image = mve::image::float_to_byte_image(
                    texture_patch->get_image(), 0.0f, 1.0f);
            copy_into_by_mask(patch_image, min_x, min_y, *texture_patch->get_validity_map(), image);
            copy_into_by_mask(texture_patch->get_validity_mask(), min_x, min_y, *texture_patch->get_validity_map(),
                              validity_mask);
            validity_map->update(*texture_patch->get_validity_map(), min_x, min_y);


//            copy_into(patch_image, rect.min_x, rect.min_y, image, padding);
//            mve::ByteImage::ConstPtr patch_validity_mask = texture_patch->get_validity_mask();
//            copy_into(patch_validity_mask, rect.min_x, rect.min_y, validity_mask, padding);

            TexturePatch::Faces const &patch_faces = texture_patch->get_faces();
            TexturePatch::Texcoords const &patch_texcoords = texture_patch->get_texcoords();

            /* Calculate the offset of the texture patches' relative texture coordinates */
            math::Vec2f offset = math::Vec2f(min_x, min_y);

            faces.insert(faces.end(), patch_faces.begin(), patch_faces.end());

            /* Calculate the final textcoords of the faces. */
            for (std::size_t i = 0; i < patch_faces.size(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    math::Vec2f rel_texcoord(patch_texcoords[i * 3 + j]);
                    math::Vec2f texcoord = rel_texcoord + offset;

                    texcoord[0] = texcoord[0] / this->size;
                    texcoord[1] = texcoord[1] / this->size;
                    texcoords.push_back(texcoord);
                }
            }
            return true;
        }

        void
        TextureAtlas::apply_edge_padding(void) {
            assert(image != NULL);
            assert(validity_mask != NULL);

            const int width = image->width();
            const int height = image->height();

            math::Matrix<float, 3, 3> gauss;
            gauss[0] = 1.0f;
            gauss[1] = 2.0f;
            gauss[2] = 1.0f;
            gauss[3] = 2.0f;
            gauss[4] = 4.0f;
            gauss[5] = 2.0f;
            gauss[6] = 1.0f;
            gauss[7] = 2.0f;
            gauss[8] = 1.0f;
            gauss /= 16.0f;

            /* Calculate the set of invalid pixels at the border of texture patches. */
            PixelSet invalid_border_pixels;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    if (validity_mask->at(x, y, 0) == 255) continue;

                    /* Check the direct neighbourhood of all invalid pixels. */
                    for (int j = -1; j <= 1; ++j) {
                        for (int i = -1; i <= 1; ++i) {
                            int nx = x + i;
                            int ny = y + j;
                            /* If the invalid pixel has a valid neighbour: */
                            if (0 <= nx && nx < width &&
                                0 <= ny && ny < height &&
                                validity_mask->at(nx, ny, 0) == 255) {

                                /* Add the pixel to the set of invalid border pixels. */
                                invalid_border_pixels.insert(std::pair<int, int>(x, y));
                            }
                        }
                    }
                }
            }

            mve::ByteImage::Ptr new_validity_mask = validity_mask->duplicate();

            /* Iteratively dilate border pixels until padding constants are reached. */
            for (unsigned int n = 0; n <= padding; ++n) {
                PixelVector new_valid_pixels;

                PixelSet::iterator it = invalid_border_pixels.begin();
                for (; it != invalid_border_pixels.end(); it++) {
                    int x = it->first;
                    int y = it->second;

                    bool now_valid = false;
                    /* Calculate new pixel value. */
                    for (int c = 0; c < 3; ++c) {
                        float norm = 0.0f;
                        float value = 0.0f;
                        for (int j = -1; j <= 1; ++j) {
                            for (int i = -1; i <= 1; ++i) {
                                int nx = x + i;
                                int ny = y + j;
                                if (0 <= nx && nx < width &&
                                    0 <= ny && ny < height &&
                                    new_validity_mask->at(nx, ny, 0) == 255) {

                                    float w = gauss[(j + 1) * 3 + (i + 1)];
                                    norm += w;
                                    value += (image->at(nx, ny, c) / 255.0f) * w;
                                }
                            }
                        }

                        if (norm == 0.0f)
                            continue;

                        now_valid = true;
                        image->at(x, y, c) = (value / norm) * 255.0f;
                    }

                    if (now_valid) {
                        new_valid_pixels.push_back(*it);
                    }
                }

                invalid_border_pixels.clear();

                /* Mark the new valid pixels valid in the validity mask. */
                for (std::size_t i = 0; i < new_valid_pixels.size(); ++i) {
                    int x = new_valid_pixels[i].first;
                    int y = new_valid_pixels[i].second;

                    new_validity_mask->at(x, y, 0) = 255;
                }

                /* Calculate the set of invalid pixels at the border of the valid area. */
                for (std::size_t i = 0; i < new_valid_pixels.size(); ++i) {
                    int x = new_valid_pixels[i].first;
                    int y = new_valid_pixels[i].second;

                    for (int j = -1; j <= 1; ++j) {
                        for (int i = -1; i <= 1; ++i) {
                            int nx = x + i;
                            int ny = y + j;
                            if (0 <= nx && nx < width &&
                                0 <= ny && ny < height &&
                                new_validity_mask->at(nx, ny, 0) == 0) {

                                invalid_border_pixels.insert(std::pair<int, int>(nx, ny));
                            }
                        }
                    }
                }
            }
        }

        struct VectorCompare {
            bool operator()(math::Vec2f const &lhs, math::Vec2f const &rhs) const {
                return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
            }
        };

        typedef std::map<math::Vec2f, std::size_t, VectorCompare> TexcoordMap;

        void
        TextureAtlas::merge_texcoords() {
            Texcoords tmp;
            tmp.swap(this->texcoords);

            TexcoordMap texcoord_map;
            for (math::Vec2f const &texcoord: tmp) {
                TexcoordMap::iterator iter = texcoord_map.find(texcoord);
                if (iter == texcoord_map.end()) {
                    std::size_t texcoord_id = this->texcoords.size();
                    texcoord_map[texcoord] = texcoord_id;
                    this->texcoords.push_back(texcoord);
                    this->texcoord_ids.push_back(texcoord_id);
                } else {
                    this->texcoord_ids.push_back(iter->second);
                }
            }

        }

        void
        TextureAtlas::finalize() {
            if (finalized) {
                throw util::Exception("TextureAtlas already finalized");
            }

            this->bin.reset();
            this->apply_edge_padding();
//            this->validity_mask.reset();
            this->merge_texcoords();

            this->finalized = true;
        }

        void TextureAtlas::set_name(const std::string &output_prefix, std::size_t index) {
            std::string prefix = "";
            {
                std::size_t dotpos = output_prefix.find_last_of('.');
                if (dotpos == std::string::npos || dotpos == 0) {
                    prefix = output_prefix;
                } else {
                    prefix = output_prefix.substr(0, dotpos);
                }
            }

            const std::string diffuse_map_postfix = "_material" + util::string::get_filled(index, 4) + "_map_Kd.png";
            const std::string mask_map_postfix = "_mask" + util::string::get_filled(index, 4) + "_map_Kd.png";

            name = util::fs::basename(prefix) + diffuse_map_postfix;
            save_path = prefix + diffuse_map_postfix;
            mask_save_path = prefix + mask_map_postfix;
        }

        void TextureAtlas::save() {
            if (image == nullptr) {
                LOG_ERROR(" - can not save null image: {}", name);
                return;
            }
            mve::image::save_png_file(image, save_path);
        }

        void TextureAtlas::save_mask() {
            if (validity_mask == nullptr) {
                LOG_ERROR(" - can not save null mask image: {}", name);
                return;
            }
            mve::image::save_png_file(validity_mask, mask_save_path);
        }

        void TextureAtlas::release_image() {
            if (image == nullptr) {
                LOG_ERROR(" - can not release null image: {}", name);
                return;
            }
            image.reset();
        }

        void TextureAtlas::release_mask() {
            if (validity_mask == nullptr) {
                LOG_ERROR(" - can not release null mask image: {}", name);
                return;
            }
            validity_mask.reset();
        }

        void TextureAtlas::generate_validity_map() {
            validity_map = GridMap::create(validity_mask->width(), validity_mask->height(), kGridSize);
        }
    }
}