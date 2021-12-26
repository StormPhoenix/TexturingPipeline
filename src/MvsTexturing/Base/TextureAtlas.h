//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_TEXTUREATLAS_H
#define TEXTURINGPIPELINE_TEXTUREATLAS_H

#include <vector>

#include <util/exception.h>
#include <math/vector.h>
#include <mve/mesh.h>
#include <mve/image.h>

#include <Base/RectBin.h>
#include <Base/TexturePatch.h>
#include <Base/GridMapPacking.h>

namespace MvsTexturing {
    namespace Base {


        class TextureAtlas {
        public:
            static const int kTextureAtlasPadding = 4;
            typedef std::shared_ptr<TextureAtlas> Ptr;
            typedef std::vector<std::size_t> Faces;
            typedef std::vector<std::size_t> TexcoordIds;
            typedef std::vector<math::Vec2f> Texcoords;

        public:
            TextureAtlas(unsigned int size);

            static TextureAtlas::Ptr create(unsigned int size);

            Faces const &get_faces(void) const;

            TexcoordIds const &get_texcoord_ids(void) const;

            Texcoords const &get_texcoords(void) const;

            mve::ByteImage::ConstPtr get_image(void) const;

            void generate_validity_map();

            bool insert(TexturePatch::ConstPtr texture_patch);

            bool insert_grid(TexturePatch::ConstPtr texture_patch);

            void finalize(void);

            void set_name(const std::string &output_prefix, std::size_t index);

            const std::string &get_name() const {
                return name;
            }


            const std::string &get_save_path() const {
                return save_path;
            }

            const std::string &get_mask_save_path() const {
                return mask_save_path;
            }

            void save();

            void save_mask();

            void release_image();

            void release_mask();

        private:
            unsigned int const size;
            unsigned int const padding;
            bool finalized;

            Faces faces;
            Texcoords texcoords;
            TexcoordIds texcoord_ids;

            mve::ByteImage::Ptr image;
            mve::ByteImage::Ptr validity_mask;
            GridMap::Ptr validity_map;

            std::string name;
            std::string save_path;
            std::string mask_save_path;

            RectBin::Ptr bin;

            void apply_edge_padding(void);

            void merge_texcoords(void);
        };

        inline TextureAtlas::Ptr
        TextureAtlas::create(unsigned int size) {
            return Ptr(new TextureAtlas(size));
        }

        inline TextureAtlas::Faces const &
        TextureAtlas::get_faces(void) const {
            return faces;
        }

        inline TextureAtlas::TexcoordIds const &
        TextureAtlas::get_texcoord_ids(void) const {
            return texcoord_ids;
        }

        inline TextureAtlas::Texcoords const &
        TextureAtlas::get_texcoords(void) const {
            return texcoords;
        }

        inline mve::ByteImage::ConstPtr
        TextureAtlas::get_image(void) const {
            if (!finalized) {
                throw util::Exception("Texture atlas not finalized");
            }
            return image;
        }
    }
}

#endif //TEXTURINGPIPELINE_TEXTUREATLAS_H
