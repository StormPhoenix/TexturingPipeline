//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_ATLASMAPPER_H
#define TEXTURINGPIPELINE_ATLASMAPPER_H

#include <Base/TexturePatch.h>
#include <Base/TextureAtlas.h>

namespace MvsTexturing {
    namespace AtlasMapper {
        void generate_texture_atlases(std::vector<Base::TexturePatch::Ptr> *orig_texture_patches,
                                      std::vector<Base::TextureAtlas::Ptr> *texture_atlases,
                                      bool tone_mapping = false);
    }
}

#endif //TEXTURINGPIPELINE_ATLASMAPPER_H
